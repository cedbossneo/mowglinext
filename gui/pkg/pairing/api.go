package pairing

import (
	"net"
	"net/http"

	"github.com/gin-gonic/gin"
	"github.com/sirupsen/logrus"
)

// lanCIDRs lists all private / loopback / link-local IPv4 and IPv6 ranges.
// Requests whose effective source IP does not fall inside one of these blocks
// are rejected by the lanOnly middleware.
var lanCIDRs = func() []*net.IPNet {
	blocks := []string{
		"10.0.0.0/8",
		"172.16.0.0/12",
		"192.168.0.0/16",
		"127.0.0.0/8",
		"::1/128",
		"fc00::/7",
		"fe80::/10",
	}
	nets := make([]*net.IPNet, 0, len(blocks))
	for _, b := range blocks {
		_, ipNet, err := net.ParseCIDR(b)
		if err != nil {
			panic("pairing: invalid LAN CIDR " + b + ": " + err.Error())
		}
		nets = append(nets, ipNet)
	}
	return nets
}()

// isLANIP returns true when ip falls within any private/loopback/link-local
// range defined in lanCIDRs.
func isLANIP(ip net.IP) bool {
	for _, block := range lanCIDRs {
		if block.Contains(ip) {
			return true
		}
	}
	return false
}

// effectiveIP extracts the effective source IP for a request. It prefers the
// first address in X-Forwarded-For (set by nginx/cloudflared on LAN), then
// X-Real-IP, then falls back to RemoteAddr.
func effectiveIP(r *http.Request) net.IP {
	if xff := r.Header.Get("X-Forwarded-For"); xff != "" {
		// XFF may be a comma-separated list; use the leftmost (original client).
		for i := 0; i < len(xff); i++ {
			if xff[i] == ',' {
				xff = xff[:i]
				break
			}
		}
		if ip := net.ParseIP(trimSpace(xff)); ip != nil {
			return ip
		}
	}
	if xri := r.Header.Get("X-Real-IP"); xri != "" {
		if ip := net.ParseIP(trimSpace(xri)); ip != nil {
			return ip
		}
	}
	host, _, err := net.SplitHostPort(r.RemoteAddr)
	if err != nil {
		host = r.RemoteAddr
	}
	return net.ParseIP(host)
}

// trimSpace removes leading and trailing ASCII space/tab from s without
// importing strings, keeping the dependency surface minimal.
func trimSpace(s string) string {
	start, end := 0, len(s)
	for start < end && (s[start] == ' ' || s[start] == '\t') {
		start++
	}
	for end > start && (s[end-1] == ' ' || s[end-1] == '\t') {
		end--
	}
	return s[start:end]
}

// lanOnly is a Gin middleware that aborts with 403 Forbidden when the
// effective source IP is not within a private/loopback/link-local range.
func lanOnly(log *logrus.Entry) gin.HandlerFunc {
	return func(c *gin.Context) {
		ip := effectiveIP(c.Request)
		if ip == nil || !isLANIP(ip) {
			if log != nil {
				log.WithField("ip", ip).Warn("pairing: rejected non-LAN request")
			}
			c.AbortWithStatusJSON(http.StatusForbidden, gin.H{
				"error": "pairing endpoints are only accessible on the local network",
			})
			return
		}
		c.Next()
	}
}

// RegisterRoutes mounts all pairing API routes under the provided Gin router
// group. Call this from main.go as:
//
//	pairing.RegisterRoutes(router.Group("/"), svc)
//
// The LAN-only middleware is applied to every pairing route; requests from
// non-private IPs receive 403 Forbidden.
func RegisterRoutes(rg *gin.RouterGroup, svc *Service) {
	log := logrus.WithField("c", "pairing")

	pair := rg.Group("/api/pair")
	pair.Use(lanOnly(log))

	// POST /api/pair/start
	// Body: { uid, idToken, displayName, setupToken }
	// Response: { confirmCode, expiresAt }
	pair.POST("/start", func(c *gin.Context) {
		var req StartRequest
		if err := c.ShouldBindJSON(&req); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": "invalid request body: " + err.Error()})
			return
		}
		resp, err := svc.Start(req)
		if err != nil {
			log.WithError(err).Warn("pair/start failed")
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		c.JSON(http.StatusOK, resp)
	})

	// POST /api/pair/confirm
	// Body: { setupToken }
	// Response: { ok: true }
	pair.POST("/confirm", func(c *gin.Context) {
		var body struct {
			SetupToken string `json:"setupToken"`
		}
		if err := c.ShouldBindJSON(&body); err != nil {
			c.JSON(http.StatusBadRequest, gin.H{"error": "invalid request body: " + err.Error()})
			return
		}
		if err := svc.Confirm(body.SetupToken); err != nil {
			log.WithError(err).Warn("pair/confirm failed")
			c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
			return
		}
		c.JSON(http.StatusOK, gin.H{"ok": true})
	})

	// GET /api/pair/status
	// Response: { state, robotID, ownerName, confirmCode }
	// confirmCode is only present when state == Pending (1).
	pair.GET("/status", func(c *gin.Context) {
		snap := svc.Status()
		c.JSON(http.StatusOK, snap)
	})

	// GET /api/pair/qr.png
	// Returns a 256×256 PNG of the pairing QR code.
	pair.GET("/qr.png", func(c *gin.Context) {
		tok := svc.SetupToken()
		if tok == "" {
			c.JSON(http.StatusServiceUnavailable, gin.H{"error": "no active setup token"})
			return
		}
		lan, cf := svc.Endpoints()
		// Fall back to the request's Host header when the operator did not
		// pre-configure the LAN address — handles the typical
		// http://mowgli.local case without extra config.
		if lan == "" {
			lan = c.Request.Host
		}
		payload := BuildPayload(svc.robotID, svc.pub, tok, lan, cf)
		png, err := RenderPNG(payload, 256)
		if err != nil {
			log.WithError(err).Error("pair/qr.png render failed")
			c.JSON(http.StatusInternalServerError, gin.H{"error": "QR render failed"})
			return
		}
		c.Data(http.StatusOK, "image/png", png)
	})

	// GET /api/pair/qr
	// Returns JSON with the QR payload for clients that render their own QR.
	// Response: { payload, robotID, setupToken }
	pair.GET("/qr", func(c *gin.Context) {
		tok := svc.SetupToken()
		if tok == "" {
			c.JSON(http.StatusServiceUnavailable, gin.H{"error": "no active setup token"})
			return
		}
		lan, cf := svc.Endpoints()
		if lan == "" {
			lan = c.Request.Host
		}
		payload := BuildPayload(svc.robotID, svc.pub, tok, lan, cf)
		c.JSON(http.StatusOK, gin.H{
			"payload":    payload,
			"robotID":    svc.robotID,
			"setupToken": tok,
		})
	})

	// POST /api/pair/reset
	// Body: ignored
	// Response: { ok: true }
	// Cancels the in-flight pairing attempt and issues a fresh setup token.
	// Used by both the web wizard's "Cancel" button and the mobile app when
	// the user backs out of the pairing flow.
	pair.POST("/reset", func(c *gin.Context) {
		svc.Reset()
		log.Info("pairing reset by client")
		c.JSON(http.StatusOK, gin.H{"ok": true})
	})
}
