package main

import (
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

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
