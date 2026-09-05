// Package updates implements read-only deployment catalogs and registry discovery.
package updates

import (
	"fmt"
	"regexp"
	"strings"
	"time"
)

var RepositoryPattern = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9_.-]*/[A-Za-z0-9][A-Za-z0-9_.-]*$`)
var DigestPattern = regexp.MustCompile(`^sha256:[a-f0-9]{64}$`)
var RevisionPattern = regexp.MustCompile(`^[a-f0-9]{40}$`)

type Platform struct {
	Manifest string `json:"manifest"`
	Config   string `json:"config"`
	Revision string `json:"revision"`
	Version  string `json:"version,omitempty"`
	BuiltAt  string `json:"built_at,omitempty"`
}

type Image struct {
	Repository string              `json:"repository"`
	Digest     string              `json:"digest"`
	Platforms  map[string]Platform `json:"platforms"`
}

type Compatibility struct {
	HardwareBackend  string `json:"hardware_backend"`
	FirmwareProtocol int    `json:"firmware_protocol"`
	ComposeSchema    int    `json:"compose_schema"`
	ConfigSchema     int    `json:"config_schema"`
	Migration        string `json:"migration"`
	Rollback         string `json:"rollback"`
}

type Manifest struct {
	SchemaVersion    int              `json:"schema_version"`
	Channel          string           `json:"channel"`
	SourceRepository string           `json:"source_repository"`
	SourceRevision   string           `json:"source_revision"`
	Version          string           `json:"version"`
	PublishedAt      string           `json:"published_at"`
	NotesURL         string           `json:"notes_url"`
	Compatibility    Compatibility    `json:"compatibility"`
	Images           map[string]Image `json:"images"`
}

type Pointer struct {
	SchemaVersion int    `json:"schema_version"`
	Path          string `json:"path"`
	Digest        string `json:"digest"`
}

var ImageNames = []string{"mowgli-ros2", "mowglinext-gui", "gps", "lidar-ldlidar", "lidar-rplidar", "lidar-stl27l"}

func (m Manifest) Validate(repository, channel string) error {
	if m.SchemaVersion != 1 || m.Channel != channel || m.SourceRepository != repository || !RevisionPattern.MatchString(m.SourceRevision) || m.Version == "" {
		return fmt.Errorf("invalid deployment identity")
	}
	if _, err := time.Parse(time.RFC3339, m.PublishedAt); err != nil {
		return fmt.Errorf("invalid publication time")
	}
	if !strings.HasPrefix(m.NotesURL, "https://github.com/"+repository+"/") {
		return fmt.Errorf("invalid release notes link")
	}
	c := m.Compatibility
	if c.HardwareBackend != "mowgli" || c.FirmwareProtocol < 1 || c.ComposeSchema != 1 || c.ConfigSchema != 1 || c.Migration != "manual-review" || c.Rollback != "manual-review" {
		return fmt.Errorf("unsupported compatibility declaration")
	}
	for _, name := range ImageNames {
		i, ok := m.Images[name]
		if !ok || i.Repository != "ghcr.io/"+strings.ToLower(repository)+"/"+name || !DigestPattern.MatchString(i.Digest) {
			return fmt.Errorf("missing or invalid image %s", name)
		}
		for _, arch := range []string{"linux/amd64", "linux/arm64"} {
			p, ok := i.Platforms[arch]
			if !ok || !DigestPattern.MatchString(p.Manifest) || !DigestPattern.MatchString(p.Config) || !RevisionPattern.MatchString(p.Revision) {
				return fmt.Errorf("missing platform or provenance for %s/%s", name, arch)
			}
		}
	}
	return nil
}
