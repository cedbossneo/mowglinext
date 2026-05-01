package pairing

import (
	"crypto/rand"
	"encoding/binary"
	"encoding/hex"
	"errors"
	"fmt"
	"sync"
	"time"
)

// Status represents the current pairing state machine phase.
type Status int

const (
	// Unstarted is the default state: no pairing attempt has begun.
	Unstarted Status = iota
	// Pending means Start has been called successfully; the user must confirm
	// the 4-digit code shown on both the wizard and the mobile app.
	Pending
	// Confirmed means Confirm has been called; the OnPaired callback has fired.
	Confirmed
	// Failed means an error occurred and Reset must be called before retrying.
	Failed
	// Paired means cloud_sync has acknowledged the pair (set by the Service
	// after the OnPaired callback completes successfully).
	Paired
)

// setupTokenTTL is how long a setup token remains valid after StartFirstBoot.
const setupTokenTTL = 30 * time.Minute

// StartRequest carries the fields the mobile app POSTs to /api/pair/start.
type StartRequest struct {
	UID         string `json:"uid"`
	IDToken     string `json:"idToken"`
	DisplayName string `json:"displayName"`
	SetupToken  string `json:"setupToken"`
}

// StartResponse is returned by Start on success.
type StartResponse struct {
	// ConfirmCode is a 4-digit string (leading zeros preserved) shown on the
	// mobile app so the user can compare it with the web wizard.
	ConfirmCode string    `json:"confirmCode"`
	ExpiresAt   time.Time `json:"expiresAt"`
}

// StatusSnapshot is a point-in-time snapshot of the pairing state returned by
// the Status method. It is a plain value type so callers can read it without
// holding any lock.
type StatusSnapshot struct {
	State       Status `json:"state"`
	RobotID     string `json:"robotID"`
	OwnerUID    string `json:"ownerUID,omitempty"`
	OwnerName   string `json:"ownerName,omitempty"`
	ConfirmCode string `json:"confirmCode,omitempty"`
	Error       string `json:"error,omitempty"`
}

// Service is the in-memory pairing state machine. All methods are safe for
// concurrent use. A single instance lives for the lifetime of the process;
// Reset prepares it for a new pairing attempt.
type Service struct {
	mu sync.Mutex

	robotID string
	pub     []byte

	setupToken  string
	tokenExpiry time.Time

	pendingUID  string
	pendingName string
	confirmCode string
	state       Status
	stateErr    string

	// lanAddr is the host:port the mobile app should reach during pairing,
	// e.g. "192.168.1.42:4006" or "mowgli.local:4006". Embedded in the QR
	// payload's `lan` field so the app knows where to POST /api/pair/start.
	// Empty when not configured.
	lanAddr string
	// tunnelHostname is the predicted Cloudflare Tunnel hostname for this
	// robot, e.g. "r-a1b2c3.tunnel.mowgli.garden". Embedded in the QR
	// payload's `cf` field so the app can connect after pairing completes.
	// Empty when not configured.
	tunnelHostname string

	// OnPaired is called by Confirm once the state transitions to Confirmed.
	// The caller (main.go) wires this to cloud_sync. If it returns an error
	// the pairing transitions to Failed.
	OnPaired func(uid, displayName, robotID string) error
}

// SetEndpoints records the LAN host:port and predicted Cloudflare Tunnel
// hostname for this robot. Both are embedded in the QR payload returned by
// the GET /api/pair/qr endpoints. Either may be empty.
func (s *Service) SetEndpoints(lanAddr, tunnelHostname string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.lanAddr = lanAddr
	s.tunnelHostname = tunnelHostname
}

// Endpoints returns the configured LAN and tunnel addresses for embedding
// in the QR payload. Empty strings when SetEndpoints has not been called.
func (s *Service) Endpoints() (lanAddr, tunnelHostname string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.lanAddr, s.tunnelHostname
}

// NewService constructs a Service. robotID is the stable robot identifier
// (from RobotIDFromPub). pub is the Curve25519 public key (32 bytes).
func NewService(robotID string, pub []byte) *Service {
	return &Service{
		robotID: robotID,
		pub:     pub,
		state:   Unstarted,
	}
}

// StartFirstBoot generates a fresh one-time setup token (32 random bytes,
// hex-encoded) with a 30-minute TTL and resets state to Unstarted.
// It should be called once at process start and again after each Reset.
func (s *Service) StartFirstBoot() {
	token := make([]byte, 32)
	if _, err := rand.Read(token); err != nil {
		// rand.Read only fails on catastrophic OS entropy failure; the robot
		// cannot operate safely without entropy, so panic is appropriate here.
		panic(fmt.Sprintf("pairing: StartFirstBoot: rand.Read: %v", err))
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	s.setupToken = hex.EncodeToString(token)
	s.tokenExpiry = time.Now().Add(setupTokenTTL)
	s.state = Unstarted
	s.pendingUID = ""
	s.pendingName = ""
	s.confirmCode = ""
	s.stateErr = ""
}

// SetupToken returns the current setup token, or an empty string if the token
// has expired or no token has been generated yet.
func (s *Service) SetupToken() string {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.setupToken == "" || time.Now().After(s.tokenExpiry) {
		return ""
	}
	return s.setupToken
}

// Reset clears all pairing state and issues a fresh setup token via
// StartFirstBoot. Use this to allow a new pairing attempt after failure or
// when the owner wants to re-pair.
func (s *Service) Reset() {
	// StartFirstBoot acquires the lock itself; do not hold it here.
	s.StartFirstBoot()
}

// Start validates the setup token, records the pending pair, and generates a
// cryptographically random 4-digit confirm code. The token is consumed
// immediately on success so a second call with the same token is rejected.
func (s *Service) Start(req StartRequest) (StartResponse, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	if req.SetupToken == "" {
		return StartResponse{}, errors.New("pairing: missing setupToken")
	}
	if s.setupToken == "" {
		return StartResponse{}, errors.New("pairing: no active setup token — call StartFirstBoot first")
	}
	if time.Now().After(s.tokenExpiry) {
		return StartResponse{}, errors.New("pairing: setup token expired")
	}
	if req.SetupToken != s.setupToken {
		return StartResponse{}, errors.New("pairing: invalid setup token")
	}
	if req.UID == "" {
		return StartResponse{}, errors.New("pairing: missing uid")
	}

	// Consume the token immediately — single use.
	s.setupToken = ""

	code, err := randomFourDigit()
	if err != nil {
		return StartResponse{}, fmt.Errorf("pairing: generate confirm code: %w", err)
	}

	s.pendingUID = req.UID
	s.pendingName = req.DisplayName
	s.confirmCode = code
	s.state = Pending

	return StartResponse{
		ConfirmCode: code,
		ExpiresAt:   s.tokenExpiry,
	}, nil
}

// Confirm finalizes the pairing. State must be Pending. The setupToken
// parameter is accepted for API symmetry but the actual gate is the Pending
// state (the token was consumed by Start). On success OnPaired is fired;
// if it returns an error the state transitions to Failed.
func (s *Service) Confirm(setupToken string) error {
	s.mu.Lock()
	if s.state != Pending {
		s.mu.Unlock()
		return fmt.Errorf("pairing: confirm called in state %d, want Pending", s.state)
	}
	uid := s.pendingUID
	name := s.pendingName
	robotID := s.robotID
	s.state = Confirmed
	s.mu.Unlock()

	// Fire the callback outside the lock so cloud_sync can call Status or
	// other Service methods without deadlocking.
	if s.OnPaired != nil {
		if err := s.OnPaired(uid, name, robotID); err != nil {
			s.mu.Lock()
			s.state = Failed
			s.stateErr = err.Error()
			s.mu.Unlock()
			return fmt.Errorf("pairing: OnPaired: %w", err)
		}
	}

	s.mu.Lock()
	s.state = Paired
	s.mu.Unlock()
	return nil
}

// Status returns a point-in-time snapshot of the pairing state. ConfirmCode
// is only populated when State == Pending so it is not leaked once confirmed.
func (s *Service) Status() StatusSnapshot {
	s.mu.Lock()
	defer s.mu.Unlock()

	snap := StatusSnapshot{
		State:     s.state,
		RobotID:   s.robotID,
		OwnerUID:  s.pendingUID,
		OwnerName: s.pendingName,
		Error:     s.stateErr,
	}
	if s.state == Pending {
		snap.ConfirmCode = s.confirmCode
	}
	return snap
}

// randomFourDigit returns a cryptographically random decimal string in the
// range "0000"–"9999" with leading zeros preserved.
func randomFourDigit() (string, error) {
	var buf [2]byte
	if _, err := rand.Read(buf[:]); err != nil {
		return "", err
	}
	n := binary.BigEndian.Uint16(buf[:]) % 10000
	return fmt.Sprintf("%04d", n), nil
}
