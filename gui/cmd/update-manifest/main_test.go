package main

import (
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

func TestCIEvidenceExcludesPublisherAndRequiresRealTests(t *testing.T) {
	gui := []job{{"Unit Tests (vitest + tsc)", "success"}, {"Go Tests (gui backend)", "success"}, {"Publish update catalog / candidate", ""}}
	if passed, skipped := jobsPassed("gui-ci.yml", gui); !passed || skipped {
		t.Fatal("running publisher must not block successful tests")
	}
	gui[1].Conclusion = "failure"
	if passed, _ := jobsPassed("gui-ci.yml", gui); passed {
		t.Fatal("failed Go tests certified a deployment")
	}
	gui[1].Conclusion = ""
	if passed, _ := jobsPassed("gui-ci.yml", gui); passed {
		t.Fatal("pending Go tests certified a deployment")
	}
	if passed, _ := jobsPassed("gui-ci.yml", gui[:1]); passed {
		t.Fatal("missing Go job certified a deployment")
	}
	gui[1] = job{"Go backend tests", "success"}
	if passed, _ := jobsPassed("gui-ci.yml", gui); !passed {
		t.Fatal("previous Go job name was not recognized")
	}
	ros := []job{{"Build & Test (ROS2 kilted)", "skipped"}, {"Config Drift (mowgli_robot.yaml)", "skipped"}, {"Formatting (clang-format)", "skipped"}, {"Static Analysis (cppcheck)", "skipped"}}
	if passed, skipped := jobsPassed("ros2-ci.yml", ros); passed || !skipped {
		t.Fatal("skipped ROS2 checks must require earlier matching evidence")
	}
	if passed, _ := jobsPassed("sensors-gps.yml", []job{{"build / Merge manifest (gps)", "success"}}); !passed {
		t.Fatal("nested sensor merge was not recognized")
	}
	if passed, _ := jobsPassed("ros2-docker.yml", []job{{"Merge manifest (dev)", "success"}}); passed {
		t.Fatal("development image alone cannot certify the runtime image")
	}
	if passed, _ := jobsPassed("unknown.yml", gui); passed {
		t.Fatal("unknown workflow certified a deployment")
	}
}

func TestImageReuseRequiresAncestorAndUnchangedBuildInputs(t *testing.T) {
	dir := t.TempDir()
	run := func(args ...string) string {
		t.Helper()
		c := exec.Command("git", append([]string{"-C", dir}, args...)...)
		out, e := c.CombinedOutput()
		if e != nil {
			t.Fatalf("git %v: %s", args, out)
		}
		return strings.TrimSpace(string(out))
	}
	run("init")
	run("config", "user.name", "Test")
	run("config", "user.email", "test@example.invalid")
	write := func(path, body string) {
		t.Helper()
		if e := os.MkdirAll(filepath.Dir(filepath.Join(dir, path)), 0755); e != nil {
			t.Fatal(e)
		}
		if e := os.WriteFile(filepath.Join(dir, path), []byte(body), 0644); e != nil {
			t.Fatal(e)
		}
	}
	write("gui/server.go", "first")
	run("add", ".")
	run("commit", "-m", "first")
	base := run("rev-parse", "HEAD")
	write("docs/notes.md", "documentation")
	run("add", ".")
	run("commit", "-m", "docs")
	docs := run("rev-parse", "HEAD")
	previous := repositoryRoot
	repositoryRoot = dir
	t.Cleanup(func() { repositoryRoot = previous })
	if !unchanged(base, docs, []string{"gui"}) {
		t.Fatal("unchanged GUI image could not be reused")
	}
	write("gui/server.go", "changed")
	run("add", ".")
	run("commit", "-m", "GUI")
	changed := run("rev-parse", "HEAD")
	if unchanged(base, changed, []string{"gui"}) {
		t.Fatal("reused image after build input changed")
	}
	if unchanged(changed, base, []string{"docs"}) {
		t.Fatal("accepted image from a future/non-ancestor revision")
	}
}
