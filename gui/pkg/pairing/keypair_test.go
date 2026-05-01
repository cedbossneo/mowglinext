package pairing

import (
	"os"
	"path/filepath"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestLoadOrCreate_GeneratesKeyOnFirstCall(t *testing.T) {
	dir := t.TempDir()

	priv, pub, err := LoadOrCreate(dir)
	require.NoError(t, err)
	assert.Len(t, priv, 32, "private key must be 32 bytes")
	assert.Len(t, pub, 32, "public key must be 32 bytes")
	assert.NotEqual(t, make([]byte, 32), priv, "private key must not be all-zero")
	assert.NotEqual(t, make([]byte, 32), pub, "public key must not be all-zero")
}

func TestLoadOrCreate_RoundTrip_SameKeyReturned(t *testing.T) {
	dir := t.TempDir()

	priv1, pub1, err := LoadOrCreate(dir)
	require.NoError(t, err)

	priv2, pub2, err := LoadOrCreate(dir)
	require.NoError(t, err)

	assert.Equal(t, priv1, priv2, "private key must be identical on second load")
	assert.Equal(t, pub1, pub2, "public key must be identical on second load")
}

func TestLoadOrCreate_KeyFileMode0600(t *testing.T) {
	dir := t.TempDir()

	_, _, err := LoadOrCreate(dir)
	require.NoError(t, err)

	info, err := os.Stat(filepath.Join(dir, keyFile))
	require.NoError(t, err)
	mode := info.Mode().Perm()
	assert.Equal(t, os.FileMode(0o600), mode,
		"noise.key must be mode 0600, got %04o", mode)
}

func TestLoadOrCreate_ExistingKeyFileIsReused(t *testing.T) {
	dir := t.TempDir()

	priv1, pub1, err := LoadOrCreate(dir)
	require.NoError(t, err)

	// Verify the file contains exactly the private key.
	raw, err := os.ReadFile(filepath.Join(dir, keyFile))
	require.NoError(t, err)
	assert.Equal(t, priv1, raw, "key file must contain exactly the private key bytes")

	// Second call must reuse the file, not regenerate.
	priv2, pub2, err := LoadOrCreate(dir)
	require.NoError(t, err)
	assert.Equal(t, priv1, priv2)
	assert.Equal(t, pub1, pub2)
}

func TestLoadOrCreate_InvalidLengthReturnsError(t *testing.T) {
	dir := t.TempDir()

	err := os.WriteFile(filepath.Join(dir, keyFile), []byte("tooshort"), 0o600)
	require.NoError(t, err)

	_, _, err = LoadOrCreate(dir)
	require.Error(t, err)
	assert.Contains(t, err.Error(), "wrong length")
}

func TestLoadOrCreate_CreatesDirIfMissing(t *testing.T) {
	base := t.TempDir()
	dir := filepath.Join(base, "nested", "state")

	_, _, err := LoadOrCreate(dir)
	require.NoError(t, err)

	_, statErr := os.Stat(dir)
	assert.NoError(t, statErr, "state directory must have been created")
}

func TestRobotIDFromPub_SixLowercaseHexChars(t *testing.T) {
	_, pub, err := LoadOrCreate(t.TempDir())
	require.NoError(t, err)

	id := RobotIDFromPub(pub)
	assert.Len(t, id, 6, "robot ID must be 6 characters")
	for _, ch := range id {
		assert.True(t,
			(ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f'),
			"robot ID must be lowercase hex, got char %q in %q", ch, id)
	}
}

func TestRobotIDFromPub_Deterministic(t *testing.T) {
	_, pub, err := LoadOrCreate(t.TempDir())
	require.NoError(t, err)

	id1 := RobotIDFromPub(pub)
	id2 := RobotIDFromPub(pub)
	assert.Equal(t, id1, id2, "RobotIDFromPub must be deterministic for the same input")
}

func TestRobotIDFromPub_DifferentForDifferentPub(t *testing.T) {
	_, pub1, err := LoadOrCreate(t.TempDir())
	require.NoError(t, err)
	_, pub2, err := LoadOrCreate(t.TempDir())
	require.NoError(t, err)

	// Two independently generated keys should not collide on the first 3 bytes
	// of their SHA-256 hashes (probability ~1 in 16 million).
	assert.NotEqual(t, RobotIDFromPub(pub1), RobotIDFromPub(pub2),
		"different public keys should produce different robot IDs")
}
