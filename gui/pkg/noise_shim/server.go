package noise_shim

import (
	"context"
	"encoding/base64"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"net/http"
	"sync"
	"time"

	"github.com/flynn/noise"
	"github.com/fxamacker/cbor/v2"
	"github.com/gorilla/websocket"
	"github.com/sirupsen/logrus"
)

// AllowList is the read-only set of Firebase UIDs allowed to connect to this
// robot. The cloud_sync package owns the underlying data and refreshes it
// from Firestore; the shim only reads.
type AllowList interface {
	Contains(uid string) bool
}

// TokenVerifier validates a Firebase ID token offline and returns its uid.
type TokenVerifier interface {
	Verify(ctx context.Context, idToken string) (uid string, err error)
}

// Proxy targets the local Go API server. The shim never instantiates HTTP
// clients itself; all upstream calls go through this Proxy. Tests use a fake.
type Proxy interface {
	HTTP(ctx context.Context, method, path string, headers map[string]string, body []byte) (status int, respHeaders map[string]string, respBody []byte, err error)
	OpenWS(ctx context.Context, path string, headers map[string]string) (WSStream, error)
}

// WSStream is one upstream WebSocket. ReadFrame blocks until a message
// arrives or the stream closes.
type WSStream interface {
	WriteFrame(binary bool, data []byte) error
	ReadFrame() (binary bool, data []byte, err error)
	Close(code int, reason string) error
}

// Config bundles everything Server needs.
type Config struct {
	// StaticPriv is the Curve25519 private key (32 bytes). Generated and
	// persisted by the pairing package on first boot.
	StaticPriv []byte
	// RobotID, RobotName surfaced in HandshakeReply.
	RobotID   string
	RobotName string
	// Version surfaced in HandshakeReply.
	Version string

	Allow    AllowList
	Verifier TokenVerifier
	Proxy    Proxy

	Logger *logrus.Entry
}

// NonceWindow is how long we remember handshake nonces to defeat replay.
const NonceWindow = 10 * time.Minute

// HandshakeTimeout caps the time a session can spend on the IK exchange.
const HandshakeTimeout = 10 * time.Second

// MaxFrameSize bounds an inner-protocol frame (after AEAD removal). Keeps a
// hostile client from exhausting memory.
const MaxFrameSize = 4 * 1024 * 1024 // 4 MiB

// Server is the long-lived holder. One Server handles many concurrent
// sessions; each WS upgrade spawns a handshake + dispatch goroutine.
type Server struct {
	cfg          Config
	upgrader     websocket.Upgrader
	noncesMu     sync.Mutex
	recentNonces map[string]time.Time
}

// New constructs a Server with sane defaults. Caller must mount Handler.
func New(cfg Config) *Server {
	if cfg.Logger == nil {
		cfg.Logger = logrus.NewEntry(logrus.StandardLogger()).WithField("c", "noise_shim")
	}
	return &Server{
		cfg: cfg,
		upgrader: websocket.Upgrader{
			CheckOrigin: func(r *http.Request) bool { return true }, // cloudflared origin
		},
		recentNonces: make(map[string]time.Time),
	}
}

// Handler returns the HTTP handler that cloudflared targets. It upgrades to
// WebSocket and runs the Noise handshake + dispatch.
func (s *Server) Handler() http.Handler {
	return http.HandlerFunc(s.serveHTTP)
}

func (s *Server) serveHTTP(w http.ResponseWriter, r *http.Request) {
	conn, err := s.upgrader.Upgrade(w, r, nil)
	if err != nil {
		s.cfg.Logger.WithError(err).Warn("ws upgrade failed")
		return
	}
	defer conn.Close()

	ctx, cancel := context.WithCancel(r.Context())
	defer cancel()

	if err := s.handle(ctx, conn); err != nil {
		s.cfg.Logger.WithError(err).Debug("session ended")
	}
}

func (s *Server) handle(ctx context.Context, conn *websocket.Conn) error {
	hsCtx, cancelHs := context.WithTimeout(ctx, HandshakeTimeout)
	defer cancelHs()

	hs, _, err := s.runHandshake(hsCtx, conn)
	if err != nil {
		return fmt.Errorf("handshake: %w", err)
	}

	enc, dec := hs.cipherStates()
	sess := &session{
		log:    s.cfg.Logger.WithField("kind", "session"),
		ctx:    ctx,
		cancel: func() { _ = conn.Close() },
		conn:   conn,
		enc:    enc,
		dec:    dec,
		proxy:  s.cfg.Proxy,
		ws:     make(map[uint32]WSStream),
	}
	return sess.run()
}

// handshakeState wraps the underlying noise.HandshakeState plus the
// post-handshake cipher pair.
type handshakeState struct {
	hs  *noise.HandshakeState
	cs1 *noise.CipherState
	cs2 *noise.CipherState
}

// cipherStates returns (encrypt, decrypt) from the responder's perspective.
//
// Noise spec: after split, cs1 is the cipher used by the initiator to send
// (and the responder to receive); cs2 is the cipher used by the responder to
// send (and the initiator to receive). We are the responder, so:
//   - cs2 (responder-out) = our encrypt
//   - cs1 (initiator-out) = our decrypt
func (h *handshakeState) cipherStates() (encrypt, decrypt *noise.CipherState) {
	return h.cs2, h.cs1
}

func (s *Server) runHandshake(ctx context.Context, conn *websocket.Conn) (*handshakeState, string, error) {
	cs := noise.NewCipherSuite(noise.DH25519, noise.CipherChaChaPoly, noise.HashBLAKE2s)
	kp := derivePub(s.cfg.StaticPriv)
	hs, err := noise.NewHandshakeState(noise.Config{
		CipherSuite:   cs,
		Pattern:       noise.HandshakeIK,
		Initiator:     false,
		StaticKeypair: kp,
	})
	if err != nil {
		return nil, "", err
	}

	// 1) Read initiator -> responder
	if err := setReadDeadline(conn, ctx); err != nil {
		return nil, "", err
	}
	mt, msg1, err := conn.ReadMessage()
	if err != nil {
		return nil, "", fmt.Errorf("read msg1: %w", err)
	}
	if mt != websocket.BinaryMessage {
		return nil, "", errors.New("msg1 not binary")
	}
	if len(msg1) < 2 {
		return nil, "", errors.New("msg1 too short")
	}
	declaredLen := binary.BigEndian.Uint16(msg1[:2])
	if int(declaredLen) != len(msg1)-2 {
		return nil, "", errors.New("msg1 length mismatch")
	}
	plain, _, _, err := hs.ReadMessage(nil, msg1[2:])
	if err != nil {
		return nil, "", fmt.Errorf("noise read msg1: %w", err)
	}

	var inPayload HandshakePayload
	if err := cbor.Unmarshal(plain, &inPayload); err != nil {
		return nil, "", fmt.Errorf("cbor msg1 payload: %w", err)
	}
	uid, err := s.cfg.Verifier.Verify(ctx, inPayload.FirebaseIDToken)
	if err != nil {
		return nil, "", fmt.Errorf("verify token: %w", err)
	}
	if !s.cfg.Allow.Contains(uid) {
		return nil, "", fmt.Errorf("uid %q not in allow-list", uid)
	}
	if err := s.checkNonce(inPayload.Nonce); err != nil {
		return nil, "", err
	}

	// 2) Build responder -> initiator
	reply := HandshakeReply{
		RobotID:   s.cfg.RobotID,
		RobotName: s.cfg.RobotName,
		Version:   s.cfg.Version,
		TS:        time.Now().Unix(),
	}
	replyBytes, err := cbor.Marshal(reply)
	if err != nil {
		return nil, "", err
	}
	out, cs1, cs2, err := hs.WriteMessage(nil, replyBytes)
	if err != nil {
		return nil, "", fmt.Errorf("noise write msg2: %w", err)
	}
	if cs1 == nil || cs2 == nil {
		return nil, "", errors.New("handshake did not complete after msg2")
	}
	framed := make([]byte, 2+len(out))
	binary.BigEndian.PutUint16(framed[:2], uint16(len(out)))
	copy(framed[2:], out)
	if err := setWriteDeadline(conn, ctx); err != nil {
		return nil, "", err
	}
	if err := conn.WriteMessage(websocket.BinaryMessage, framed); err != nil {
		return nil, "", fmt.Errorf("write msg2: %w", err)
	}
	_ = conn.SetReadDeadline(time.Time{})
	_ = conn.SetWriteDeadline(time.Time{})

	s.cfg.Logger.WithField("uid", uid).Info("session opened")
	return &handshakeState{hs: hs, cs1: cs1, cs2: cs2}, uid, nil
}

func (s *Server) checkNonce(n []byte) error {
	if len(n) < 16 {
		return errors.New("nonce too short")
	}
	key := base64.StdEncoding.EncodeToString(n)
	now := time.Now()
	s.noncesMu.Lock()
	defer s.noncesMu.Unlock()
	for k, t := range s.recentNonces {
		if now.Sub(t) > NonceWindow {
			delete(s.recentNonces, k)
		}
	}
	if _, seen := s.recentNonces[key]; seen {
		return errors.New("replayed nonce")
	}
	s.recentNonces[key] = now
	return nil
}

// derivePub computes a noise.DHKey from a raw 32-byte Curve25519 private key.
// flynn/noise's GenerateKeypair reads 32 bytes as the scalar, clamps it,
// and computes the public via the standard X25519 base-point multiplication.
func derivePub(priv []byte) noise.DHKey {
	if len(priv) != 32 {
		panic(fmt.Sprintf("noise_shim: static priv must be 32 bytes, got %d", len(priv)))
	}
	r := &fixedReader{b: append([]byte(nil), priv...)}
	kp, err := noise.DH25519.GenerateKeypair(r)
	if err != nil {
		panic(fmt.Sprintf("noise_shim: derivePub: %v", err))
	}
	return kp
}

type fixedReader struct {
	b    []byte
	used bool
}

func (r *fixedReader) Read(p []byte) (int, error) {
	if r.used {
		return 0, io.EOF
	}
	n := copy(p, r.b)
	r.used = true
	return n, nil
}

func setReadDeadline(conn *websocket.Conn, ctx context.Context) error {
	dl, ok := ctx.Deadline()
	if !ok {
		return conn.SetReadDeadline(time.Time{})
	}
	return conn.SetReadDeadline(dl)
}

func setWriteDeadline(conn *websocket.Conn, ctx context.Context) error {
	dl, ok := ctx.Deadline()
	if !ok {
		return conn.SetWriteDeadline(time.Time{})
	}
	return conn.SetWriteDeadline(dl)
}
