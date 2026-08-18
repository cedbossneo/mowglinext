package api

import (
	"mime"
	"net/http"
	"os"
	"path/filepath"
	"strconv"
	"strings"

	"github.com/gin-contrib/static"
	"github.com/gin-gonic/gin"
)

const (
	noCacheHTMLControl         = "no-cache, no-store, must-revalidate"
	immutableAssetCacheControl = "public, max-age=31536000, immutable"
)

func registerWebUI(router *gin.Engine, webDir string) {
	router.Use(webAssetDelivery(webDir))
	router.Use(static.Serve("/", static.LocalFile(webDir, false)))
	router.NoRoute(func(c *gin.Context) {
		c.Header("Cache-Control", noCacheHTMLControl)
		c.File(filepath.Join(webDir, "index.html"))
	})
}

func webAssetDelivery(webDir string) gin.HandlerFunc {
	return func(c *gin.Context) {
		requestPath := c.Request.URL.Path
		if requestPath == "/" || requestPath == "/index.html" {
			c.Header("Cache-Control", noCacheHTMLControl)
		}

		if !strings.HasPrefix(requestPath, "/assets/") {
			c.Next()
			return
		}

		c.Header("Cache-Control", immutableAssetCacheControl)
		appendVary(c.Writer.Header(), "Accept-Encoding")

		if (c.Request.Method != http.MethodGet && c.Request.Method != http.MethodHead) ||
			!acceptsGzip(c.GetHeader("Accept-Encoding")) ||
			!isCompressibleAsset(requestPath) ||
			!hasGzipSidecar(webDir, requestPath) {
			c.Next()
			return
		}

		originalPath := c.Request.URL.Path
		c.Request.URL.Path += ".gz"
		c.Header("Content-Encoding", "gzip")
		if contentType := mime.TypeByExtension(filepath.Ext(originalPath)); contentType != "" {
			c.Header("Content-Type", contentType)
		}
		c.Next()
		c.Request.URL.Path = originalPath
	}
}

func hasGzipSidecar(webDir, requestPath string) bool {
	relativePath := filepath.Clean(strings.TrimPrefix(requestPath, "/"))
	assetsPrefix := "assets" + string(filepath.Separator)
	if !strings.HasPrefix(relativePath, assetsPrefix) {
		return false
	}

	info, err := os.Stat(filepath.Join(webDir, relativePath) + ".gz")
	return err == nil && !info.IsDir()
}

func isCompressibleAsset(requestPath string) bool {
	switch strings.ToLower(filepath.Ext(requestPath)) {
	case ".css", ".js", ".json", ".svg", ".wasm":
		return true
	default:
		return false
	}
}

func acceptsGzip(header string) bool {
	var wildcardQuality *float64
	for _, value := range strings.Split(header, ",") {
		parts := strings.Split(value, ";")
		encoding := strings.ToLower(strings.TrimSpace(parts[0]))
		quality := 1.0
		for _, parameter := range parts[1:] {
			name, rawValue, found := strings.Cut(strings.TrimSpace(parameter), "=")
			if !found || !strings.EqualFold(strings.TrimSpace(name), "q") {
				continue
			}
			parsed, err := strconv.ParseFloat(strings.TrimSpace(rawValue), 64)
			if err != nil {
				quality = 0
			} else {
				quality = parsed
			}
		}

		switch encoding {
		case "gzip":
			return quality > 0
		case "*":
			qualityCopy := quality
			wildcardQuality = &qualityCopy
		}
	}

	return wildcardQuality != nil && *wildcardQuality > 0
}

func appendVary(header http.Header, value string) {
	for _, existing := range header.Values("Vary") {
		for _, field := range strings.Split(existing, ",") {
			if strings.EqualFold(strings.TrimSpace(field), value) {
				return
			}
		}
	}
	header.Add("Vary", value)
}
