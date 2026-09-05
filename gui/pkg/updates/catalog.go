package updates

import (
	"context"
	"encoding/json"
	"fmt"
	"net/http"
	"strings"
)

func LoadCatalog(ctx context.Context, client *http.Client, repository, channel string) (*Manifest, error) {
	if !RepositoryPattern.MatchString(repository) || (channel != "stable" && channel != "dev") {
		return nil, fmt.Errorf("invalid catalog source")
	}
	base := "https://raw.githubusercontent.com/" + repository + "/update-manifests/"
	body, err := Read(ctx, client, base+"channels/"+channel+".json", "")
	if err != nil {
		return nil, err
	}
	var pointer Pointer
	if err = json.Unmarshal(body, &pointer); err != nil {
		return nil, err
	}
	if pointer.SchemaVersion != 1 || !DigestPattern.MatchString(pointer.Digest) || pointer.Path != "manifests/"+strings.TrimPrefix(pointer.Digest, "sha256:")+".json" {
		return nil, fmt.Errorf("invalid catalog pointer")
	}
	body, err = Read(ctx, client, base+pointer.Path, "")
	if err != nil {
		return nil, err
	}
	if Hash(body) != pointer.Digest {
		return nil, fmt.Errorf("catalog checksum mismatch")
	}
	var manifest Manifest
	if err = json.Unmarshal(body, &manifest); err != nil {
		return nil, err
	}
	if err = manifest.Validate(repository, channel); err != nil {
		return nil, err
	}
	return &manifest, nil
}

// Matches compares like-for-like immutable identities. A source revision alone
// is never sufficient: rebuilding the same commit can change the image bytes.
func Matches(image Image, platform, imageID string, digests []string) bool {
	p, ok := image.Platforms[platform]
	if !ok {
		return false
	}
	if imageID == p.Config || imageID == p.Manifest || imageID == image.Digest {
		return true
	}
	for _, d := range digests {
		parts := strings.Split(d, "@")
		if len(parts) == 2 && (parts[1] == image.Digest || parts[1] == p.Manifest) {
			return true
		}
	}
	return false
}
