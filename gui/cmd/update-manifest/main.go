// update-manifest creates a catalog candidate after checking image provenance,
// unchanged build inputs, platform availability and successful CI evidence.
// It never publishes by itself. The workflow promotes its output atomically.
package main

import (
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"regexp"
	"strings"
	"time"

	"github.com/mowglinext/mowglinext/pkg/updates"
)

type inputs struct {
	workflow string
	paths    []string
}

var components = map[string]inputs{
	"mowgli-ros2":    {"ros2-docker.yml", []string{"ros2", "tools/motor", ".github/workflows/ros2-docker.yml"}},
	"mowglinext-gui": {"gui-docker.yml", []string{"gui", ".github/workflows/gui-docker.yml"}},
	"gps":            {"sensors-gps.yml", []string{"sensors/gps", "sensors/README.md", "ros2/src/mowgli_interfaces", "ros2/src/external/universal-gnss", ".github/workflows/sensors-gps.yml", ".github/workflows/_sensor-docker.yml"}},
	"lidar-ldlidar":  {"sensors-lidar-ldlidar.yml", []string{"sensors/lidar-ldlidar", ".github/workflows/sensors-lidar-ldlidar.yml", ".github/workflows/_sensor-docker.yml"}},
	"lidar-rplidar":  {"sensors-lidar-rplidar.yml", []string{"sensors/lidar-rplidar", ".github/workflows/sensors-lidar-rplidar.yml", ".github/workflows/_sensor-docker.yml"}},
	"lidar-stl27l":   {"sensors-lidar-stl27l.yml", []string{"sensors/lidar-stl27l", ".github/workflows/sensors-lidar-stl27l.yml", ".github/workflows/_sensor-docker.yml"}},
}

var repositoryRoot string

func git(args ...string) (string, error) {
	if repositoryRoot != "" {
		args = append([]string{"-C", repositoryRoot}, args...)
	}
	out, err := exec.Command("git", args...).Output()
	return strings.TrimSpace(string(out)), err
}
func unchanged(revision, target string, paths []string) bool {
	if !updates.RevisionPattern.MatchString(revision) {
		return false
	}
	if _, err := git("merge-base", "--is-ancestor", revision, target); err != nil {
		return false
	}
	args := append([]string{"diff", "--quiet", revision, target, "--"}, paths...)
	_, err := git(args...)
	return err == nil
}

type run struct {
	SHA        string `json:"head_sha"`
	Conclusion string `json:"conclusion"`
	Event      string `json:"event"`
	URL        string `json:"html_url"`
	Repository struct {
		Name string `json:"full_name"`
	} `json:"head_repository"`
}

func ci(ctx context.Context, repo, workflow, revision, target string, paths []string) (string, error) {
	address := "https://api.github.com/repos/" + repo + "/actions/workflows/" + workflow + "/runs?per_page=100"
	if revision != "" {
		address += "&head_sha=" + revision
	}
	body, err := updates.Read(ctx, &http.Client{Timeout: 20 * time.Second}, address, os.Getenv("GH_TOKEN"))
	if err != nil {
		return "", err
	}
	var response struct {
		Runs []run `json:"workflow_runs"`
	}
	if err = json.Unmarshal(body, &response); err != nil {
		return "", err
	}
	for _, r := range response.Runs {
		if !strings.EqualFold(r.Repository.Name, repo) || (r.Event != "push" && r.Event != "workflow_dispatch") {
			continue
		}
		if revision != "" && r.SHA != revision {
			continue
		}
		if !unchanged(r.SHA, target, paths) {
			continue
		}
		if r.Conclusion != "success" {
			return "", fmt.Errorf("%s has not passed for candidate inputs", workflow)
		}
		return r.URL, nil
	}
	return "", fmt.Errorf("no successful %s run covers candidate inputs", workflow)
}

func main() {
	if err := build(); err != nil {
		fmt.Fprintln(os.Stderr, "Candidate not published:", err)
		os.Exit(1)
	}
}
func build() error {
	repositoryRoot, _ = git("rev-parse", "--show-toplevel")
	repo := flag.String("repository", "mowglinext/mowglinext", "GitHub/image repository")
	channel := flag.String("channel", "dev", "stable or dev")
	ref := flag.String("ref", "HEAD", "local source revision to validate")
	version := flag.String("version", "dev", "dev or published release tag")
	out := flag.String("out", "../update-candidate", "output directory")
	flag.Parse()
	if !updates.RepositoryPattern.MatchString(*repo) || (*channel != "dev" && *channel != "stable") {
		return fmt.Errorf("invalid repository/channel")
	}
	target, err := git("rev-parse", *ref+"^{commit}")
	if err != nil || !updates.RevisionPattern.MatchString(target) {
		return fmt.Errorf("invalid source revision")
	}
	ctx, cancel := context.WithTimeout(context.Background(), 8*time.Minute)
	defer cancel()
	tag := "dev"
	notes := "https://github.com/" + *repo + "/commit/" + target
	if *channel == "stable" {
		if !regexp.MustCompile(`^v[0-9]+\.[0-9]+\.[0-9]+$`).MatchString(*version) {
			return fmt.Errorf("stable requires a release tag")
		}
		body, e := updates.Read(ctx, &http.Client{Timeout: 20 * time.Second}, "https://api.github.com/repos/"+*repo+"/releases/latest", os.Getenv("GH_TOKEN"))
		if e != nil {
			return e
		}
		var release struct {
			Tag        string `json:"tag_name"`
			Draft      bool   `json:"draft"`
			Prerelease bool   `json:"prerelease"`
		}
		if e = json.Unmarshal(body, &release); e != nil {
			return e
		}
		tagSHA, e := git("rev-parse", "refs/tags/"+*version+"^{commit}")
		if release.Tag != *version || release.Draft || release.Prerelease || e != nil || tagSHA != target {
			return fmt.Errorf("stable candidate must be the latest published release")
		}
		tag = strings.TrimPrefix(*version, "v")
		notes = "https://github.com/" + *repo + "/releases/tag/" + *version
	} else if *version != "dev" {
		return fmt.Errorf("development version must be dev")
	}
	protocol, err := git("show", target+":firmware/stm32/ros_usbnode/include/mowgli_protocol.h")
	if err != nil {
		return err
	}
	match := regexp.MustCompile(`#define MOWGLI_PROTOCOL_VERSION ([0-9]+)u`).FindStringSubmatch(protocol)
	if len(match) != 2 {
		return fmt.Errorf("firmware protocol not declared")
	}
	var protocolNumber int
	fmt.Sscanf(match[1], "%d", &protocolNumber)
	m := updates.Manifest{SchemaVersion: 1, Channel: *channel, SourceRepository: *repo, SourceRevision: target, Version: *version, PublishedAt: time.Now().UTC().Format(time.RFC3339), NotesURL: notes, Compatibility: updates.Compatibility{HardwareBackend: "mowgli", FirmwareProtocol: protocolNumber, ComposeSchema: 1, ConfigSchema: 1, Migration: "manual-review", Rollback: "manual-review"}, Images: map[string]updates.Image{}}
	evidence := map[string]string{}
	registry := updates.NewRegistry()
	for _, name := range updates.ImageNames {
		input := components[name]
		image, e := registry.Resolve(ctx, "ghcr.io/"+strings.ToLower(*repo)+"/"+name, tag)
		if e != nil {
			return fmt.Errorf("%s: %w", name, e)
		}
		for _, arch := range []string{"linux/amd64", "linux/arm64"} {
			p, ok := image.Platforms[arch]
			if !ok {
				return fmt.Errorf("%s missing %s", name, arch)
			}
			if !unchanged(p.Revision, target, input.paths) {
				return fmt.Errorf("%s/%s source inputs do not match candidate", name, arch)
			}
			url, e := ci(ctx, *repo, input.workflow, p.Revision, target, input.paths)
			if e != nil {
				return e
			}
			evidence[name+"/"+arch] = url
		}
		m.Images[name] = image
	}
	for _, gate := range []inputs{
		{"gui-ci.yml", []string{"gui", ".github/workflows/gui-ci.yml"}},
		{"ros2-ci.yml", []string{"ros2", "tools/motor", "install/config/mowgli", ".github/workflows/ros2-ci.yml"}},
		{"protocol-version-drift.yml", []string{"firmware/stm32/ros_usbnode/include/mowgli_protocol.h", "ros2/src/mowgli_hardware/include/mowgli_hardware/ll_datatypes.hpp", "firmware/scripts/protocol_version_guard.py", "firmware/scripts/protocol_baseline.json"}},
	} {
		url, e := ci(ctx, *repo, gate.workflow, "", target, gate.paths)
		if e != nil {
			return e
		}
		evidence[gate.workflow] = url
	}
	if err = m.Validate(*repo, *channel); err != nil {
		return err
	}
	body, err := json.MarshalIndent(struct {
		updates.Manifest
		Validation map[string]string `json:"validation"`
	}{m, evidence}, "", "  ")
	if err != nil {
		return err
	}
	body = append(body, '\n')
	digest := updates.Hash(body)
	path := "manifests/" + strings.TrimPrefix(digest, "sha256:") + ".json"
	if err = os.MkdirAll(filepath.Join(*out, "manifests"), 0755); err != nil {
		return err
	}
	if err = os.MkdirAll(filepath.Join(*out, "channels"), 0755); err != nil {
		return err
	}
	if err = os.WriteFile(filepath.Join(*out, path), body, 0644); err != nil {
		return err
	}
	pointer, _ := json.MarshalIndent(updates.Pointer{SchemaVersion: 1, Path: path, Digest: digest}, "", "  ")
	if err = os.WriteFile(filepath.Join(*out, "channels", *channel+".json"), append(pointer, '\n'), 0644); err != nil {
		return err
	}
	fmt.Println("Validated candidate:", target, digest)
	return nil
}
