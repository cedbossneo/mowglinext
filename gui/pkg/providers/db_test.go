package providers

import (
	"os"
	"path/filepath"
	"testing"

	"git.mills.io/prologic/bitcask"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestDBProvider_SetAndGet(t *testing.T) {
	dir := t.TempDir()
	os.Setenv("DB_PATH", dir)
	defer os.Unsetenv("DB_PATH")

	db := NewDBProvider()

	err := db.Set("test.key", []byte("hello"))
	require.NoError(t, err)

	val, err := db.Get("test.key")
	require.NoError(t, err)
	assert.Equal(t, "hello", string(val))
}

func TestDBProvider_GetNotFound(t *testing.T) {
	dir := t.TempDir()
	os.Setenv("DB_PATH", dir)
	defer os.Unsetenv("DB_PATH")

	db := NewDBProvider()

	_, err := db.Get("nonexistent.key")
	assert.Error(t, err)
	assert.Contains(t, err.Error(), "not found")
}

func TestDBProvider_GetWithEnvFallback(t *testing.T) {
	dir := t.TempDir()
	os.Setenv("DB_PATH", dir)
	defer os.Unsetenv("DB_PATH")

	db := NewDBProvider()

	// Test env fallback
	os.Setenv("ROS_MASTER_URI", "http://test:11311")
	defer os.Unsetenv("ROS_MASTER_URI")

	val, err := db.Get("system.ros.masterUri")
	require.NoError(t, err)
	assert.Equal(t, "http://test:11311", string(val))
}

func TestDBProvider_GetWithDefault(t *testing.T) {
	dir := t.TempDir()
	os.Setenv("DB_PATH", dir)
	defer os.Unsetenv("DB_PATH")

	db := NewDBProvider()

	// No env var set, should return default
	os.Unsetenv("ROS_MASTER_URI")

	val, err := db.Get("system.ros.masterUri")
	require.NoError(t, err)
	assert.Equal(t, "http://localhost:11311", string(val))
}

func TestDBProvider_SetOverridesEnvAndDefault(t *testing.T) {
	dir := t.TempDir()
	os.Setenv("DB_PATH", dir)
	defer os.Unsetenv("DB_PATH")

	db := NewDBProvider()

	os.Setenv("ROS_MASTER_URI", "http://env:11311")
	defer os.Unsetenv("ROS_MASTER_URI")

	// Set in DB should override env fallback
	err := db.Set("system.ros.masterUri", []byte("http://db:11311"))
	require.NoError(t, err)

	val, err := db.Get("system.ros.masterUri")
	require.NoError(t, err)
	assert.Equal(t, "http://db:11311", string(val))
}

func TestDBProvider_Delete(t *testing.T) {
	dir := t.TempDir()
	os.Setenv("DB_PATH", dir)
	defer os.Unsetenv("DB_PATH")

	db := NewDBProvider()

	err := db.Set("delete.me", []byte("value"))
	require.NoError(t, err)

	err = db.Delete("delete.me")
	require.NoError(t, err)

	_, err = db.Get("delete.me")
	assert.Error(t, err)
}

func TestDBProvider_GetWithEnvFallbackMethod(t *testing.T) {
	dir := t.TempDir()
	os.Setenv("DB_PATH", dir)
	defer os.Unsetenv("DB_PATH")

	db := NewDBProvider()

	// No env, no db -> returns default
	result := db.GetWithEnvFallback("nonexistent", "NONEXISTENT_ENV", "my-default")
	assert.Equal(t, "my-default", result)

	// Env set -> returns env
	os.Setenv("MY_TEST_ENV", "env-value")
	defer os.Unsetenv("MY_TEST_ENV")

	result = db.GetWithEnvFallback("nonexistent", "MY_TEST_ENV", "my-default")
	assert.Equal(t, "env-value", result)

	// DB set -> returns DB value
	err := db.Set("nonexistent", []byte("db-value"))
	require.NoError(t, err)

	result = db.GetWithEnvFallback("nonexistent", "MY_TEST_ENV", "my-default")
	assert.Equal(t, "db-value", result)
}

// seedStore opens a bitcask store at dir, writes the given key/values, and
// closes it (releasing the lock and flushing to the datafile) so the caller
// can corrupt the on-disk files.
func seedStore(t *testing.T, dir string, kv map[string]string) {
	t.Helper()
	db, err := bitcask.Open(dir)
	require.NoError(t, err)
	for k, v := range kv {
		require.NoError(t, db.Put([]byte(k), []byte(v)))
	}
	require.NoError(t, db.Close())
}

// TestDBProvider_RecoversCorruptedDatafileTail reproduces issue #288: an
// unclean shutdown mid-write leaves the tail of the datafile corrupted, so
// decoding it yields "key/value size is invalid". NewDBProvider must recover
// (truncate the corrupted tail) instead of panicking, preserving intact keys.
func TestDBProvider_RecoversCorruptedDatafileTail(t *testing.T) {
	dir := t.TempDir()
	seedStore(t, dir, map[string]string{"onboarding.completed": "true"})

	// Append garbage to the datafile: a 0xFF header decodes to an oversized
	// key size, which is exactly the "key/value size is invalid" corruption.
	matches, err := filepath.Glob(filepath.Join(dir, "*.data"))
	require.NoError(t, err)
	require.NotEmpty(t, matches)
	f, err := os.OpenFile(matches[0], os.O_APPEND|os.O_WRONLY, 0o644)
	require.NoError(t, err)
	_, err = f.Write([]byte{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF})
	require.NoError(t, err)
	require.NoError(t, f.Close())

	t.Setenv("DB_PATH", dir)

	// Must not panic.
	var db *DBProvider
	require.NotPanics(t, func() { db = NewDBProvider() })

	// The intact key written before the corruption is preserved.
	val, err := db.Get("onboarding.completed")
	require.NoError(t, err)
	assert.Equal(t, "true", string(val))
}

// TestDBProvider_QuarantinesUnrecoverableStore covers corruption that
// auto-recovery cannot fix (a corrupted meta.json). NewDBProvider must move the
// broken store aside and come up on a fresh, writable store instead of panicking.
func TestDBProvider_QuarantinesUnrecoverableStore(t *testing.T) {
	dir := t.TempDir()
	seedStore(t, dir, map[string]string{"onboarding.completed": "true"})

	// Corrupt meta.json with invalid JSON — Open fails with ErrBadMetadata,
	// which bitcask's auto-recovery does not handle.
	require.NoError(t, os.WriteFile(filepath.Join(dir, "meta.json"), []byte("{not json"), 0o644))

	t.Setenv("DB_PATH", dir)

	var db *DBProvider
	require.NotPanics(t, func() { db = NewDBProvider() })

	// A backup directory holds the quarantined store.
	backups, err := filepath.Glob(filepath.Join(dir, quarantinePrefix+"*"))
	require.NoError(t, err)
	require.NotEmpty(t, backups, "expected a quarantine backup directory")

	// The fresh store is writable and readable.
	require.NoError(t, db.Set("fresh.key", []byte("ok")))
	val, err := db.Get("fresh.key")
	require.NoError(t, err)
	assert.Equal(t, "ok", string(val))
}

func TestDBProvider_DefaultValues(t *testing.T) {
	dir := t.TempDir()
	os.Setenv("DB_PATH", dir)
	defer os.Unsetenv("DB_PATH")

	db := NewDBProvider()

	tests := []struct {
		key      string
		expected string
	}{
		{"system.api.addr", ":4006"},
		{"system.api.webDirectory", "/app/web"},
		{"system.map.enabled", "false"},
		{"system.mower.configFile", "/config/mower_config.sh"},
		{"system.mower.runtimeEnvFile", "/runtime_config/.env"},
		{"system.ros.nodeName", "mowglinext"},
		{"system.ros.nodeHost", "localhost"},
		{"system.mqtt.enabled", "false"},
		{"system.mqtt.host", ":1883"},
		{"system.mqtt.prefix", "/gui"},
		{"system.homekit.enabled", "false"},
		{"system.homekit.pincode", "00102003"},
	}

	for _, tt := range tests {
		t.Run(tt.key, func(t *testing.T) {
			// Clear any env vars that might interfere
			if envVar, ok := EnvFallbacks[tt.key]; ok {
				os.Unsetenv(envVar)
			}

			val, err := db.Get(tt.key)
			require.NoError(t, err)
			assert.Equal(t, tt.expected, string(val))
		})
	}
}
