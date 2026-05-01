package pairing

import (
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func init() {
	gin.SetMode(gin.TestMode)
}

// newTestService returns a Service with a fresh keypair and StartFirstBoot
// already called, suitable for use in any test.
func newTestService(t *testing.T) *Service {
	t.Helper()
	_, pub, err := LoadOrCreate(t.TempDir())
	require.NoError(t, err)
	svc := NewService(RobotIDFromPub(pub), pub)
	svc.StartFirstBoot()
	return svc
}

// ─── Status transitions ───────────────────────────────────────────────────────

func TestStatusTransitions_UnstartedToPendingToPaired(t *testing.T) {
	svc := newTestService(t)
	tok := svc.SetupToken()
	require.NotEmpty(t, tok, "setup token must be non-empty after StartFirstBoot")

	assert.Equal(t, Unstarted, svc.Status().State, "initial state must be Unstarted")

	resp, err := svc.Start(StartRequest{
		UID:         "uid-alice",
		DisplayName: "Alice",
		SetupToken:  tok,
	})
	require.NoError(t, err)
	assert.Equal(t, Pending, svc.Status().State)
	assert.NotEmpty(t, resp.ConfirmCode)

	require.NoError(t, svc.Confirm(tok))
	assert.Equal(t, Paired, svc.Status().State)
}

func TestStatusTransitions_OnPairedCallback(t *testing.T) {
	svc := newTestService(t)
	tok := svc.SetupToken()

	var gotUID, gotName, gotRobotID string
	svc.OnPaired = func(uid, name, robotID string) error {
		gotUID, gotName, gotRobotID = uid, name, robotID
		return nil
	}

	_, err := svc.Start(StartRequest{UID: "uid-bob", DisplayName: "Bob", SetupToken: tok})
	require.NoError(t, err)
	require.NoError(t, svc.Confirm(tok))

	assert.Equal(t, Paired, svc.Status().State)
	assert.Equal(t, "uid-bob", gotUID)
	assert.Equal(t, "Bob", gotName)
	assert.NotEmpty(t, gotRobotID)
}

func TestStatusTransitions_OnPairedError_SetsFailedState(t *testing.T) {
	svc := newTestService(t)
	tok := svc.SetupToken()
	svc.OnPaired = func(uid, name, robotID string) error {
		return assert.AnError
	}

	_, err := svc.Start(StartRequest{UID: "uid-err", SetupToken: tok})
	require.NoError(t, err)

	err = svc.Confirm(tok)
	require.Error(t, err)
	assert.Equal(t, Failed, svc.Status().State)
}

// ─── Confirm without prior Start ─────────────────────────────────────────────

func TestConfirmWithoutStart_Fails(t *testing.T) {
	svc := newTestService(t)
	err := svc.Confirm("")
	require.Error(t, err)
	assert.Contains(t, err.Error(), "want Pending")
	assert.Equal(t, Unstarted, svc.Status().State)
}

// ─── Setup token single-use ───────────────────────────────────────────────────

func TestSetupToken_SingleUse(t *testing.T) {
	svc := newTestService(t)
	tok := svc.SetupToken()
	require.NotEmpty(t, tok)

	// First Start succeeds and consumes the token.
	_, err := svc.Start(StartRequest{UID: "uid-1", DisplayName: "User1", SetupToken: tok})
	require.NoError(t, err)

	// Second Start with the same token must fail. After the token is consumed
	// the service clears it, so the error may say "no active setup token" or
	// "invalid setup token" depending on timing — either way it must error.
	_, err = svc.Start(StartRequest{UID: "uid-2", DisplayName: "User2", SetupToken: tok})
	require.Error(t, err)
	assert.Contains(t, err.Error(), "setup token")
}

// ─── Setup token expiry ───────────────────────────────────────────────────────

func TestSetupToken_ExpiryAfterTTL(t *testing.T) {
	svc := newTestService(t)

	// Back-date the expiry to simulate a 31-minute-old token.
	svc.mu.Lock()
	expiredTok := svc.setupToken
	svc.tokenExpiry = time.Now().Add(-31 * time.Minute)
	svc.mu.Unlock()

	assert.Empty(t, svc.SetupToken(), "SetupToken must return empty when token is expired")

	_, err := svc.Start(StartRequest{UID: "uid-x", SetupToken: expiredTok})
	require.Error(t, err)
	assert.Contains(t, err.Error(), "expired")
}

// ─── 4-digit confirm code ─────────────────────────────────────────────────────

func TestConfirmCode_ExactlyFourDecimalDigits(t *testing.T) {
	// Run many iterations to exercise leading-zero edge cases.
	for i := 0; i < 200; i++ {
		svc := newTestService(t)
		tok := svc.SetupToken()
		resp, err := svc.Start(StartRequest{UID: "uid-test", SetupToken: tok})
		require.NoError(t, err)

		code := resp.ConfirmCode
		assert.Len(t, code, 4, "confirm code must be exactly 4 characters, got %q", code)
		for _, ch := range code {
			assert.True(t, ch >= '0' && ch <= '9',
				"confirm code must contain only decimal digits, got %q", code)
		}
	}
}

func TestRandomFourDigit_FormatAndRange(t *testing.T) {
	seen := map[string]bool{}
	for i := 0; i < 2000; i++ {
		code, err := randomFourDigit()
		require.NoError(t, err)
		require.Len(t, code, 4, "randomFourDigit must always return 4 chars")
		for _, ch := range code {
			assert.True(t, ch >= '0' && ch <= '9')
		}
		seen[code] = true
	}
	// With 2000 samples from a uniform 10000-wide distribution we expect
	// well over 50 distinct values.
	assert.Greater(t, len(seen), 50, "randomFourDigit must produce varied output")
}

// ─── ConfirmCode visibility ───────────────────────────────────────────────────

func TestConfirmCode_NotLeakedAfterConfirm(t *testing.T) {
	svc := newTestService(t)
	tok := svc.SetupToken()

	_, err := svc.Start(StartRequest{UID: "uid-c", SetupToken: tok})
	require.NoError(t, err)

	snap := svc.Status()
	assert.Equal(t, Pending, snap.State)
	assert.NotEmpty(t, snap.ConfirmCode, "code must be present while Pending")

	require.NoError(t, svc.Confirm(tok))

	snap = svc.Status()
	assert.Equal(t, Paired, snap.State)
	assert.Empty(t, snap.ConfirmCode, "code must be absent after Confirm")
}

// ─── Reset ────────────────────────────────────────────────────────────────────

func TestReset_ClearsStateAndIssuedFreshToken(t *testing.T) {
	svc := newTestService(t)
	tok := svc.SetupToken()

	_, err := svc.Start(StartRequest{UID: "uid-r", SetupToken: tok})
	require.NoError(t, err)
	assert.Equal(t, Pending, svc.Status().State)

	svc.Reset()

	snap := svc.Status()
	assert.Equal(t, Unstarted, snap.State)
	assert.Empty(t, snap.OwnerUID)
	assert.Empty(t, snap.ConfirmCode)

	newTok := svc.SetupToken()
	assert.NotEmpty(t, newTok)
	assert.NotEqual(t, tok, newTok, "Reset must issue a new token")
}

// ─── LAN-only middleware ──────────────────────────────────────────────────────

func newTestRouter(svc *Service) *gin.Engine {
	r := gin.New()
	RegisterRoutes(r.Group("/"), svc)
	return r
}

var lanIPCases = []struct {
	name      string
	remoteAddr string
	xff       string
	wantAllow bool
}{
	{"loopback v4", "127.0.0.1:12345", "", true},
	{"loopback v6", "[::1]:12345", "", true},
	{"private 10/8", "10.0.1.5:9999", "", true},
	{"private 172.16/12", "172.20.0.1:9999", "", true},
	{"private 192.168/16", "192.168.1.100:9999", "", true},
	{"link-local fe80 via XFF", "127.0.0.1:1", "fe80::1", true},
	{"public IPv4 via RemoteAddr", "8.8.8.8:12345", "", false},
	{"public IPv4 via XFF", "127.0.0.1:1", "8.8.8.8", false},
	{"public IPv4 XFF multi", "127.0.0.1:1", "8.8.8.8, 10.0.0.1", false},
}

func TestLANOnlyMiddleware(t *testing.T) {
	for _, tc := range lanIPCases {
		tc := tc
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			svc := newTestService(t)
			router := newTestRouter(svc)

			req := httptest.NewRequest(http.MethodGet, "/api/pair/status", nil)
			req.RemoteAddr = tc.remoteAddr
			if tc.xff != "" {
				req.Header.Set("X-Forwarded-For", tc.xff)
			}
			w := httptest.NewRecorder()
			router.ServeHTTP(w, req)

			if tc.wantAllow {
				assert.NotEqual(t, http.StatusForbidden, w.Code,
					"expected LAN IP %q to be allowed, got %d", tc.remoteAddr, w.Code)
			} else {
				assert.Equal(t, http.StatusForbidden, w.Code,
					"expected non-LAN IP %q to be blocked, got %d", tc.remoteAddr, w.Code)
			}
		})
	}
}

// ─── Full API integration ─────────────────────────────────────────────────────

func TestAPI_StartConfirmStatusFlow(t *testing.T) {
	svc := newTestService(t)
	router := newTestRouter(svc)
	tok := svc.SetupToken()
	require.NotEmpty(t, tok)

	// POST /api/pair/start from a LAN IP.
	body := `{"uid":"uid-flow","idToken":"tok","displayName":"Flow User","setupToken":"` + tok + `"}`
	req := httptest.NewRequest(http.MethodPost, "/api/pair/start", strings.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	req.RemoteAddr = "192.168.1.50:9000"
	w := httptest.NewRecorder()
	router.ServeHTTP(w, req)
	assert.Equal(t, http.StatusOK, w.Code, "start should succeed: %s", w.Body.String())

	var startResp StartResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &startResp))
	assert.Len(t, startResp.ConfirmCode, 4)

	// GET /api/pair/status → Pending.
	req = httptest.NewRequest(http.MethodGet, "/api/pair/status", nil)
	req.RemoteAddr = "127.0.0.1:1"
	w = httptest.NewRecorder()
	router.ServeHTTP(w, req)
	assert.Equal(t, http.StatusOK, w.Code)
	var snap StatusSnapshot
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &snap))
	assert.Equal(t, Pending, snap.State)
	assert.Equal(t, startResp.ConfirmCode, snap.ConfirmCode)

	// POST /api/pair/confirm.
	req = httptest.NewRequest(http.MethodPost, "/api/pair/confirm", strings.NewReader(`{"setupToken":""}`))
	req.Header.Set("Content-Type", "application/json")
	req.RemoteAddr = "192.168.1.50:9000"
	w = httptest.NewRecorder()
	router.ServeHTTP(w, req)
	assert.Equal(t, http.StatusOK, w.Code, "confirm should succeed: %s", w.Body.String())

	// GET /api/pair/status → Paired, no code.
	// Use a fresh snapshot variable so that fields absent from the JSON
	// (omitempty) are not inherited from the previous Pending snapshot.
	req = httptest.NewRequest(http.MethodGet, "/api/pair/status", nil)
	req.RemoteAddr = "127.0.0.1:1"
	w = httptest.NewRecorder()
	router.ServeHTTP(w, req)
	var pairedSnap StatusSnapshot
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &pairedSnap))
	assert.Equal(t, Paired, pairedSnap.State)
	assert.Empty(t, pairedSnap.ConfirmCode)
}

func TestAPI_StartRejectedFromPublicIP(t *testing.T) {
	svc := newTestService(t)
	router := newTestRouter(svc)
	tok := svc.SetupToken()

	body := `{"uid":"uid-x","idToken":"tok","displayName":"X","setupToken":"` + tok + `"}`
	req := httptest.NewRequest(http.MethodPost, "/api/pair/start", strings.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	req.RemoteAddr = "8.8.8.8:1234"
	w := httptest.NewRecorder()
	router.ServeHTTP(w, req)
	assert.Equal(t, http.StatusForbidden, w.Code)
}

func TestAPI_QRPNGReturnsImage(t *testing.T) {
	svc := newTestService(t)
	router := newTestRouter(svc)

	req := httptest.NewRequest(http.MethodGet, "/api/pair/qr.png", nil)
	req.RemoteAddr = "127.0.0.1:1"
	w := httptest.NewRecorder()
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)
	assert.Equal(t, "image/png", w.Header().Get("Content-Type"))
	assert.Greater(t, w.Body.Len(), 100, "PNG should have non-trivial size")
}

func TestAPI_QRJSONContainsPayload(t *testing.T) {
	svc := newTestService(t)
	router := newTestRouter(svc)

	req := httptest.NewRequest(http.MethodGet, "/api/pair/qr", nil)
	req.RemoteAddr = "127.0.0.1:1"
	w := httptest.NewRecorder()
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)
	var resp map[string]string
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.NotEmpty(t, resp["payload"])
	assert.NotEmpty(t, resp["robotID"])
	assert.NotEmpty(t, resp["setupToken"])
	assert.True(t, strings.HasPrefix(resp["payload"], "mowgli://pair?"),
		"payload should be a mowgli:// URL, got %q", resp["payload"])
}

func TestAPI_QRUnavailableWhenNoToken(t *testing.T) {
	_, pub, err := LoadOrCreate(t.TempDir())
	require.NoError(t, err)
	// Deliberately do NOT call StartFirstBoot — no token available.
	svc := NewService(RobotIDFromPub(pub), pub)
	router := newTestRouter(svc)

	for _, path := range []string{"/api/pair/qr.png", "/api/pair/qr"} {
		req := httptest.NewRequest(http.MethodGet, path, nil)
		req.RemoteAddr = "127.0.0.1:1"
		w := httptest.NewRecorder()
		router.ServeHTTP(w, req)
		assert.Equal(t, http.StatusServiceUnavailable, w.Code,
			"expected 503 for %s when no token available", path)
	}
}
