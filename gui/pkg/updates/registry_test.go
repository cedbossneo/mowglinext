package updates

import (
	"context"
	"io"
	"net/http"
	"strings"
	"testing"
)

type transportFunc func(*http.Request) (*http.Response, error)

func (f transportFunc) RoundTrip(r *http.Request) (*http.Response, error) { return f(r) }
func response(body string) *http.Response {
	return &http.Response{StatusCode: 200, Body: io.NopCloser(strings.NewReader(body)), Header: http.Header{}}
}

func TestRegistryResolvesPlatformIdentityWithoutLayers(t *testing.T) {
	config := `{"architecture":"arm64","os":"linux","config":{"Labels":{"org.opencontainers.image.revision":"` + strings.Repeat("a", 40) + `"}}}`
	manifest := `{"config":{"digest":"` + Hash([]byte(config)) + `"},"layers":[{"digest":"never-download"}]}`
	index := `{"manifests":[{"digest":"` + Hash([]byte(manifest)) + `","platform":{"os":"linux","architecture":"arm64","variant":"v8"}},{"digest":"ignored","platform":{"os":"unknown","architecture":"unknown"}}]}`
	requests := 0
	tamperConfig := false
	r := Registry{Client: &http.Client{Transport: transportFunc(func(req *http.Request) (*http.Response, error) {
		requests++
		if req.URL.Host != "ghcr.io" {
			t.Fatal("unexpected host")
		}
		switch {
		case req.URL.Path == "/token":
			return response(`{"token":"read-only-token"}`), nil
		case strings.HasSuffix(req.URL.Path, "/manifests/dev"):
			return response(index), nil
		case strings.HasSuffix(req.URL.Path, "/manifests/"+Hash([]byte(manifest))):
			return response(manifest), nil
		case strings.HasSuffix(req.URL.Path, "/blobs/"+Hash([]byte(config))):
			if tamperConfig {
				return response(config + " "), nil
			}
			return response(config), nil
		default:
			t.Fatalf("unexpected registry request %s", req.URL)
			return nil, nil
		}
	})}}
	image, err := r.Resolve(context.Background(), "ghcr.io/owner/repo/gui", "dev")
	if err != nil {
		t.Fatal(err)
	}
	p := image.Platforms["linux/arm64"]
	if image.Digest != Hash([]byte(index)) || p.Manifest != Hash([]byte(manifest)) || p.Config != Hash([]byte(config)) || requests != 4 {
		t.Fatalf("wrong platform resolution: %+v (%d requests)", image, requests)
	}
	if _, err = r.Resolve(context.Background(), "http://127.0.0.1/private", "dev"); err == nil || requests != 4 {
		t.Fatal("accepted arbitrary URL")
	}
	tamperConfig = true
	if _, err = r.Resolve(context.Background(), "ghcr.io/owner/repo/gui", "dev"); err == nil {
		t.Fatal("accepted configuration bytes that do not match their digest")
	}
}

func TestImageComparisonNeverUsesSourceRevisionAsIdentity(t *testing.T) {
	i := Image{Repository: "ghcr.io/owner/repo/gps", Digest: "sha256:" + strings.Repeat("a", 64), Platforms: map[string]Platform{"linux/arm64": {Manifest: "sha256:" + strings.Repeat("b", 64), Config: "sha256:" + strings.Repeat("c", 64), Revision: strings.Repeat("d", 40)}}}
	if !Matches(i, "linux/arm64", i.Platforms["linux/arm64"].Config, nil) {
		t.Fatal("config identity did not match")
	}
	if !Matches(i, "linux/arm64", "", []string{i.Repository + "@" + i.Digest}) {
		t.Fatal("index identity did not match")
	}
	if Matches(i, "linux/arm64", i.Platforms["linux/arm64"].Revision, nil) || Matches(i, "linux/arm/v7", i.Digest, nil) {
		t.Fatal("compared unrelated identities/platforms")
	}
}
