package updates

import (
	"context"
	"encoding/json"
	"io"
	"net/http"
	"strings"
	"testing"
	"time"
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
}

func fixtureManifest() Manifest {
	m := Manifest{SchemaVersion: 1, Channel: "dev", SourceRepository: "owner/repo", SourceRevision: strings.Repeat("a", 40), Version: "dev", PublishedAt: time.Now().UTC().Format(time.RFC3339), NotesURL: "https://github.com/owner/repo/commits/dev", Compatibility: Compatibility{HardwareBackend: "mowgli", FirmwareProtocol: 6, ComposeSchema: 1, ConfigSchema: 1, Migration: "manual-review", Rollback: "manual-review"}, Images: map[string]Image{}}
	for _, name := range ImageNames {
		m.Images[name] = Image{Repository: "ghcr.io/owner/repo/" + name, Digest: "sha256:" + strings.Repeat("b", 64), Platforms: map[string]Platform{"linux/arm64": {Manifest: "sha256:" + strings.Repeat("c", 64), Config: "sha256:" + strings.Repeat("d", 64), Revision: strings.Repeat("a", 40)}, "linux/amd64": {Manifest: "sha256:" + strings.Repeat("e", 64), Config: "sha256:" + strings.Repeat("f", 64), Revision: strings.Repeat("a", 40)}}}
	}
	return m
}

func TestCatalogChecksChecksumCompletenessAndPlatform(t *testing.T) {
	m := fixtureManifest()
	body, _ := json.Marshal(m)
	pointer, _ := json.Marshal(Pointer{SchemaVersion: 1, Path: "manifests/" + strings.TrimPrefix(Hash(body), "sha256:") + ".json", Digest: Hash(body)})
	client := &http.Client{Transport: transportFunc(func(r *http.Request) (*http.Response, error) {
		if strings.Contains(r.URL.Path, "/channels/") {
			return response(string(pointer)), nil
		}
		return response(string(body)), nil
	})}
	if _, err := LoadCatalog(context.Background(), client, "owner/repo", "dev"); err != nil {
		t.Fatal(err)
	}
	body = append(body, ' ')
	if _, err := LoadCatalog(context.Background(), client, "owner/repo", "dev"); err == nil {
		t.Fatal("accepted modified catalog bytes")
	}
	delete(m.Images["gps"].Platforms, "linux/arm64")
	if err := m.Validate("owner/repo", "dev"); err == nil {
		t.Fatal("accepted incomplete platform set")
	}
}

func TestImageComparisonNeverUsesSourceRevisionAsIdentity(t *testing.T) {
	i := fixtureManifest().Images["gps"]
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
