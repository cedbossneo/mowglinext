package api

import (
	"bytes"
	"compress/gzip"
	"io"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/gin-contrib/static"
	"github.com/gin-gonic/gin"
	"github.com/stretchr/testify/require"
)

const testAssetPath = "/assets/index-ABC123.js"

func writeWebFixture(tb testing.TB) (string, []byte) {
	tb.Helper()
	webDir := tb.TempDir()
	require.NoError(tb, os.MkdirAll(filepath.Join(webDir, "assets"), 0o755))
	require.NoError(tb, os.WriteFile(filepath.Join(webDir, "index.html"), []byte("<html>shell</html>"), 0o644))

	// Mix repeated JavaScript-like content with deterministic high-entropy text.
	// This keeps the fixture large while avoiding an unrealistically tiny gzip.
	raw := bytes.Repeat([]byte("const mowerStatus = updateTelemetry(frame);\n"), 7_000)
	noise := make([]byte, 132_000)
	const alphabet = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#$%^&*()[]{}"
	seed := uint32(0x9e3779b9)
	for i := range noise {
		seed ^= seed << 13
		seed ^= seed >> 17
		seed ^= seed << 5
		noise[i] = alphabet[int(seed)%len(alphabet)]
	}
	raw = append(raw, noise...)
	require.NoError(tb, os.WriteFile(filepath.Join(webDir, testAssetPath), raw, 0o644))

	compressedFile, err := os.Create(filepath.Join(webDir, testAssetPath+".gz"))
	require.NoError(tb, err)
	gzipWriter, err := gzip.NewWriterLevel(compressedFile, gzip.BestCompression)
	require.NoError(tb, err)
	_, err = gzipWriter.Write(raw)
	require.NoError(tb, err)
	require.NoError(tb, gzipWriter.Close())
	require.NoError(tb, compressedFile.Close())
	return webDir, raw
}

func newBaselineStaticRouter(webDir string) *gin.Engine {
	router := gin.New()
	router.Use(static.Serve("/", static.LocalFile(webDir, false)))
	router.NoRoute(func(c *gin.Context) {
		c.File(filepath.Join(webDir, "index.html"))
	})
	return router
}

func performRequest(router http.Handler, path, acceptEncoding string) *httptest.ResponseRecorder {
	request := httptest.NewRequest(http.MethodGet, path, nil)
	if acceptEncoding != "" {
		request.Header.Set("Accept-Encoding", acceptEncoding)
	}
	response := httptest.NewRecorder()
	router.ServeHTTP(response, request)
	return response
}

func TestRegisterWebUIUsesPrecompressedHashedAssets(t *testing.T) {
	gin.SetMode(gin.TestMode)
	webDir, raw := writeWebFixture(t)
	router := gin.New()
	registerWebUI(router, webDir)

	response := performRequest(router, testAssetPath, "br, gzip")
	require.Equal(t, http.StatusOK, response.Code)
	require.Equal(t, "gzip", response.Header().Get("Content-Encoding"))
	require.Equal(t, "Accept-Encoding", response.Header().Get("Vary"))
	require.Equal(t, immutableAssetCacheControl, response.Header().Get("Cache-Control"))
	require.Contains(t, response.Header().Get("Content-Type"), "javascript")

	reader, err := gzip.NewReader(response.Body)
	require.NoError(t, err)
	decoded, err := io.ReadAll(reader)
	require.NoError(t, err)
	require.NoError(t, reader.Close())
	require.Equal(t, raw, decoded)
}

func TestRegisterWebUIFallsBackWhenGzipIsRejected(t *testing.T) {
	gin.SetMode(gin.TestMode)
	webDir, raw := writeWebFixture(t)
	router := gin.New()
	registerWebUI(router, webDir)

	response := performRequest(router, testAssetPath, "gzip;q=0, identity")
	require.Equal(t, http.StatusOK, response.Code)
	require.Empty(t, response.Header().Get("Content-Encoding"))
	require.Equal(t, immutableAssetCacheControl, response.Header().Get("Cache-Control"))
	require.Equal(t, raw, response.Body.Bytes())
}

func TestRegisterWebUIFallsBackWhenSidecarIsMissing(t *testing.T) {
	gin.SetMode(gin.TestMode)
	webDir, raw := writeWebFixture(t)
	require.NoError(t, os.Remove(filepath.Join(webDir, testAssetPath+".gz")))
	router := gin.New()
	registerWebUI(router, webDir)

	response := performRequest(router, testAssetPath, "gzip")
	require.Equal(t, http.StatusOK, response.Code)
	require.Empty(t, response.Header().Get("Content-Encoding"))
	require.Equal(t, immutableAssetCacheControl, response.Header().Get("Cache-Control"))
	require.Equal(t, raw, response.Body.Bytes())
}

func TestRegisterWebUIDoesNotCacheHTMLShell(t *testing.T) {
	gin.SetMode(gin.TestMode)
	webDir, _ := writeWebFixture(t)
	router := gin.New()
	registerWebUI(router, webDir)

	for _, test := range []struct {
		path       string
		statusCode int
		body       string
	}{
		{path: "/", statusCode: http.StatusOK, body: "<html>shell</html>"},
		{path: "/index.html", statusCode: http.StatusMovedPermanently},
		{path: "/settings", statusCode: http.StatusOK, body: "<html>shell</html>"},
	} {
		response := performRequest(router, test.path, "gzip")
		require.Equal(t, test.statusCode, response.Code, test.path)
		require.Equal(t, noCacheHTMLControl, response.Header().Get("Cache-Control"), test.path)
		require.Empty(t, response.Header().Get("Content-Encoding"), test.path)
		require.Equal(t, test.body, response.Body.String(), test.path)
	}
}

func BenchmarkStaticAssetDelivery(b *testing.B) {
	gin.SetMode(gin.TestMode)
	webDir, _ := writeWebFixture(b)

	benchmarks := []struct {
		name           string
		router         http.Handler
		acceptEncoding string
	}{
		{name: "baseline_raw", router: newBaselineStaticRouter(webDir)},
		{name: "precompressed_gzip", router: func() http.Handler {
			router := gin.New()
			registerWebUI(router, webDir)
			return router
		}(), acceptEncoding: "gzip"},
	}

	for _, benchmark := range benchmarks {
		b.Run(benchmark.name, func(b *testing.B) {
			request := httptest.NewRequest(http.MethodGet, testAssetPath, nil)
			request.Header.Set("Accept-Encoding", benchmark.acceptEncoding)
			var wireBytes int
			b.ReportAllocs()
			b.ResetTimer()
			for i := 0; i < b.N; i++ {
				response := httptest.NewRecorder()
				benchmark.router.ServeHTTP(response, request)
				require.Equal(b, http.StatusOK, response.Code)
				wireBytes = response.Body.Len()
			}
			b.ReportMetric(float64(wireBytes), "wire-B/op")
		})
	}
}

func TestAcceptsGzip(t *testing.T) {
	for _, test := range []struct {
		header string
		want   bool
	}{
		{header: "gzip", want: true},
		{header: "br, gzip", want: true},
		{header: "gzip;q=0.5", want: true},
		{header: "gzip;q=0", want: false},
		{header: "*;q=0.5", want: true},
		{header: "*;q=1, gzip;q=0", want: false},
		{header: "gzip;q=bogus", want: false},
		{header: "br", want: false},
		{header: "", want: false},
	} {
		t.Run(strings.ReplaceAll(test.header, ";", "_"), func(t *testing.T) {
			require.Equal(t, test.want, acceptsGzip(test.header))
		})
	}
}
