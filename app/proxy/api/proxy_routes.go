package api

import (
	"encoding/base64"
	"io"
	"net/http"
	"strings"
	"sync"
	"time"

	"github.com/gin-gonic/gin"
)

// API whitelist — safety-critical
var allowedCommands = map[string]bool{
	"high_level_control": true,
	"emergency":          true,
	"mow_enabled":        true,
	"start_in_area":      true,
	"mower_logic":        true,
}

// Rate limits per command
var commandRateLimits = map[string]time.Duration{
	"emergency":          0, // unlimited
	"mow_enabled":        5 * time.Second,
	"high_level_control": 2 * time.Second,
}

var rateLimitMu sync.Mutex
var lastCommandTime = make(map[string]time.Time)

func (s *Server) handleProxyCall(c *gin.Context) {
	command := c.Param("command")
	robotID := c.Param("id")

	// Whitelist check
	if !allowedCommands[command] {
		c.JSON(http.StatusForbidden, gin.H{"error": "command not allowed via cloud proxy"})
		return
	}

	// Rate limit check
	if limit, ok := commandRateLimits[command]; ok && limit > 0 {
		key := robotID + ":" + command
		rateLimitMu.Lock()
		if last, exists := lastCommandTime[key]; exists && time.Since(last) < limit {
			rateLimitMu.Unlock()
			c.JSON(http.StatusTooManyRequests, gin.H{"error": "rate limited"})
			return
		}
		lastCommandTime[key] = time.Now()
		rateLimitMu.Unlock()
	}

	// Read request body
	body, err := io.ReadAll(c.Request.Body)
	if err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": "failed to read body"})
		return
	}

	// Get tunnel connection
	conn, err := s.tm.Get(robotID)
	if err != nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{"error": "robot not connected"})
		return
	}

	// Send through tunnel
	resp, err := conn.SendRequest(
		"POST",
		"/api/openmower/call/"+command,
		base64.StdEncoding.EncodeToString(body),
		map[string]string{"Content-Type": "application/json"},
	)
	if err != nil {
		c.JSON(http.StatusGatewayTimeout, gin.H{"error": err.Error()})
		return
	}

	// Decode and forward response
	respBody, _ := base64.StdEncoding.DecodeString(resp.Body)
	c.Data(resp.Status, "application/json", respBody)
}

// handleProxyHTTP proxies whitelisted HTTP endpoints
func (s *Server) handleProxyHTTP(c *gin.Context) {
	robotID := c.Param("id")

	// Build the local path (strip /api/robots/:id prefix)
	path := c.Request.URL.Path
	prefix := "/api/robots/" + robotID
	localPath := "/api" + strings.TrimPrefix(path, prefix)

	// Whitelist check
	if !isAllowedPath(localPath) {
		c.JSON(http.StatusForbidden, gin.H{"error": "path not allowed via cloud proxy"})
		return
	}

	conn, err := s.tm.Get(robotID)
	if err != nil {
		c.JSON(http.StatusServiceUnavailable, gin.H{"error": "robot not connected"})
		return
	}

	body, _ := io.ReadAll(c.Request.Body)
	resp, err := conn.SendRequest(
		c.Request.Method,
		localPath,
		base64.StdEncoding.EncodeToString(body),
		map[string]string{"Content-Type": c.ContentType()},
	)
	if err != nil {
		c.JSON(http.StatusGatewayTimeout, gin.H{"error": err.Error()})
		return
	}

	respBody, _ := base64.StdEncoding.DecodeString(resp.Body)
	c.Data(resp.Status, "application/json", respBody)
}

var allowedPathPrefixes = []string{
	"/api/schedules",
	"/api/settings/status",
	"/api/system/info",
}

func isAllowedPath(path string) bool {
	for _, prefix := range allowedPathPrefixes {
		if strings.HasPrefix(path, prefix) {
			return true
		}
	}
	return false
}
