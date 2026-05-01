package pairing

import (
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// TestResetEndpoint verifies that POST /api/pair/reset cancels the in-flight
// pairing attempt, returns ok, and the next /api/pair/qr returns a fresh
// setup token.
func TestResetEndpoint(t *testing.T) {
	svc := newTestService(t)
	r := newTestRouter(svc)

	req := httptest.NewRequest(http.MethodGet, "/api/pair/qr", nil)
	req.RemoteAddr = "192.168.1.10:1234"
	rr := httptest.NewRecorder()
	r.ServeHTTP(rr, req)
	require.Equal(t, http.StatusOK, rr.Code)
	originalBody := rr.Body.String()
	require.Contains(t, originalBody, `"setupToken"`)

	req2 := httptest.NewRequest(http.MethodPost, "/api/pair/reset", strings.NewReader(""))
	req2.RemoteAddr = "192.168.1.10:1234"
	rr2 := httptest.NewRecorder()
	r.ServeHTTP(rr2, req2)
	require.Equal(t, http.StatusOK, rr2.Code)
	assert.JSONEq(t, `{"ok":true}`, rr2.Body.String())

	req3 := httptest.NewRequest(http.MethodGet, "/api/pair/qr", nil)
	req3.RemoteAddr = "192.168.1.10:1234"
	rr3 := httptest.NewRecorder()
	r.ServeHTTP(rr3, req3)
	require.Equal(t, http.StatusOK, rr3.Code)
	assert.NotEqual(t, originalBody, rr3.Body.String(),
		"setup token should rotate after Reset")
}

// TestResetEndpointLANOnly verifies that the LAN-only middleware rejects a
// reset attempt from a non-private IP.
func TestResetEndpointLANOnly(t *testing.T) {
	svc := newTestService(t)
	r := newTestRouter(svc)

	req := httptest.NewRequest(http.MethodPost, "/api/pair/reset", strings.NewReader(""))
	req.RemoteAddr = "8.8.8.8:1234"
	rr := httptest.NewRecorder()
	r.ServeHTTP(rr, req)
	assert.Equal(t, http.StatusForbidden, rr.Code)
}

// TestSetEndpointsInjectsLanAndCfIntoQR verifies that values configured via
// SetEndpoints reach the QR payload.
func TestSetEndpointsInjectsLanAndCfIntoQR(t *testing.T) {
	svc := newTestService(t)
	svc.SetEndpoints("192.168.42.7:4006", "r-abc123.tunnel.mowgli.garden")
	r := newTestRouter(svc)

	req := httptest.NewRequest(http.MethodGet, "/api/pair/qr", nil)
	req.RemoteAddr = "192.168.1.10:1234"
	rr := httptest.NewRecorder()
	r.ServeHTTP(rr, req)
	require.Equal(t, http.StatusOK, rr.Code)
	body := rr.Body.String()
	assert.Contains(t, body, "192.168.42.7", "lan address must appear in QR payload")
	assert.Contains(t, body, "r-abc123.tunnel.mowgli.garden")
}
