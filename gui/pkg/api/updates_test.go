package api

import (
	"context"
	"errors"
	"net/http/httptest"
	"strings"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/updates"
)

type fakeUpdates struct {
	manifest   *updates.Manifest
	catalogErr error
	image      updates.Image
	old        updates.Image
	resolveErr error
	calls      int
	head       string
}

func (f *fakeUpdates) Catalog(context.Context, string, string) (*updates.Manifest, error) {
	f.calls++
	return f.manifest, f.catalogErr
}
func (f *fakeUpdates) Stable(context.Context, string) (string, string, error) {
	return "1.1.0", "https://github.com/owner/repo/releases/tag/v1.1.0", nil
}
func (f *fakeUpdates) Head(context.Context, string) (string, error) { return f.head, nil }
func (f *fakeUpdates) Resolve(_ context.Context, _ string, ref string) (updates.Image, error) {
	f.calls++
	if strings.HasPrefix(ref, "sha256:") {
		return f.old, f.resolveErr
	}
	return f.image, f.resolveErr
}

func updateFixture() (VersionsResponse, *fakeUpdates) {
	image := updates.Image{Repository: "ghcr.io/owner/repo/gps", Digest: "sha256:" + strings.Repeat("a", 64), Platforms: map[string]updates.Platform{"linux/arm64": {Manifest: "sha256:" + strings.Repeat("b", 64), Config: "sha256:" + strings.Repeat("c", 64), Revision: strings.Repeat("d", 40)}}}
	return VersionsResponse{DockerAvailable: true, Components: []InstalledComponent{{Name: "mowgli-gps", Image: "ghcr.io/owner/repo/gps:dev", ImageID: image.Platforms["linux/arm64"].Config, Architecture: "arm64", Revision: strings.Repeat("d", 40), MetadataAvailable: true}}}, &fakeUpdates{image: image, old: image, head: strings.Repeat("e", 40), manifest: &updates.Manifest{Channel: "dev", SourceRevision: strings.Repeat("e", 40), Version: "dev", Images: map[string]updates.Image{"gps": image}}}
}

func TestUpdateCheckCatalogAndSameCommitRebuild(t *testing.T) {
	inv, source := updateFixture()
	if got := checkUpdates(context.Background(), inv, "owner/repo", "dev", source); got.State != "current" {
		t.Fatalf("%+v", got)
	}
	inv.Components[0].ImageID = "sha256:" + strings.Repeat("f", 64)
	got := checkUpdates(context.Background(), inv, "owner/repo", "dev", source)
	if got.State != "available" || got.Components[0].State != "changed" {
		t.Fatalf("same-SHA rebuild hidden: %+v", got)
	}
	inv.Components[0].Digests = []string{"ghcr.io/owner/repo/gps@sha256:" + strings.Repeat("f", 64)}
	if got = checkUpdates(context.Background(), inv, "owner/repo", "dev", source); got.State != "current" {
		t.Fatalf("different index with same platform: %+v", got)
	}
}

func TestUpdateCheckMissingCatalogIsExplicitlyUnverified(t *testing.T) {
	inv, source := updateFixture()
	source.manifest = nil
	source.catalogErr = updates.RemoteError{Status: 404}
	got := checkUpdates(context.Background(), inv, "owner/repo", "dev", source)
	if got.State != "unverified" || got.Source != "registry" || got.LastSuccessfulAt == "" {
		t.Fatalf("%+v", got)
	}
	source.resolveErr = updates.RemoteError{Status: 429}
	got = checkUpdates(context.Background(), inv, "owner/repo", "dev", source)
	if got.State != "incomplete" || got.LastSuccessfulAt != "" {
		t.Fatalf("failed check marked current: %+v", got)
	}
	source.catalogErr = errors.New("bad catalog checksum")
	if got = checkUpdates(context.Background(), inv, "owner/repo", "dev", source); got.State != "unavailable" {
		t.Fatal("invalid catalog silently bypassed")
	}
}

func TestUpdateCheckPreparingMissingPlatformAndUnknownIdentity(t *testing.T) {
	inv, source := updateFixture()
	source.head = strings.Repeat("f", 40)
	if got := checkUpdates(context.Background(), inv, "owner/repo", "dev", source); got.State != "preparing" {
		t.Fatalf("%+v", got)
	}
	inv.Components[0].Architecture = "arm"
	if got := checkUpdates(context.Background(), inv, "owner/repo", "dev", source); got.State != "incomplete" || got.Components[0].State != "missing_platform" {
		t.Fatalf("%+v", got)
	}
	inv.Components[0].Architecture = "arm64"
	inv.Components[0].ImageID = ""
	inv.Components[0].MetadataAvailable = false
	if got := checkUpdates(context.Background(), inv, "owner/repo", "dev", source); got.Components[0].State != "unknown" {
		t.Fatalf("%+v", got)
	}
}

func TestUpdatesEndpointIsReadOnlyAndCacheDoesNotContactRegistry(t *testing.T) {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	_, source := updateFixture()
	registerUpdatesRoutes(r.Group("/api"), &versionDocker{}, "owner/repo", source)
	for _, channel := range []string{"dev", "stable"} {
		w := httptest.NewRecorder()
		r.ServeHTTP(w, httptest.NewRequest("GET", "/api/system/updates?channel="+channel, nil))
		if w.Code != 200 || w.Header().Get("Cache-Control") != "no-store" || !strings.Contains(w.Body.String(), "not_checked") {
			t.Fatalf("%d %s", w.Code, w.Body.String())
		}
	}
	if source.calls != 0 {
		t.Fatal("cached read contacted registry")
	}
	for _, request := range []struct {
		method, path string
		code         int
	}{{"GET", "/api/system/updates?channel=http://localhost", 400}, {"POST", "/api/system/updates", 404}} {
		w := httptest.NewRecorder()
		r.ServeHTTP(w, httptest.NewRequest(request.method, request.path, nil))
		if w.Code != request.code {
			t.Fatalf("%s returned %d", request.path, w.Code)
		}
	}
}
