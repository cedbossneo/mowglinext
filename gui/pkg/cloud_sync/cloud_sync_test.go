package cloud_sync

import (
	"context"
	"os"
	"testing"
	"time"

	"cloud.google.com/go/firestore"
	firebase "firebase.google.com/go/v4"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"

	"github.com/cedbossneo/mowglinext/pkg/noise_shim"
)

// emulatorHost skips the calling test when FIRESTORE_EMULATOR_HOST is unset.
func emulatorHost(t *testing.T) {
	t.Helper()
	if os.Getenv("FIRESTORE_EMULATOR_HOST") == "" {
		t.Skip("set FIRESTORE_EMULATOR_HOST to run Firestore emulator tests (e.g. localhost:8080)")
	}
}

// newEmulatorClient returns a Firestore client pointed at the emulator.
func newEmulatorClient(t *testing.T, projectID string) *firestore.Client {
	t.Helper()
	ctx := context.Background()
	app, err := firebase.NewApp(ctx, &firebase.Config{ProjectID: projectID})
	require.NoError(t, err)
	client, err := app.Firestore(ctx)
	require.NoError(t, err)
	t.Cleanup(func() { _ = client.Close() })
	return client
}

// TestSnapshotWatcherUpdatesAllowList verifies that Run() picks up an
// allowedUids change in Firestore and calls MemoryAllowList.Replace.
func TestSnapshotWatcherUpdatesAllowList(t *testing.T) {
	emulatorHost(t)

	const projectID = "test-project"
	const robotID = "robot-abc"

	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()

	// Seed the robot document before starting the watcher.
	raw := newEmulatorClient(t, projectID)
	_, err := raw.Collection("robots").Doc(robotID).Set(ctx, map[string]interface{}{
		"allowedUids": []string{"uid-alice"},
	})
	require.NoError(t, err)

	allow := noise_shim.NewMemoryAllowList(nil)
	s, err := NewSync(ctx, SyncOptions{
		ProjectID: projectID,
		RobotID:   robotID,
		AllowList: allow,
	})
	require.NoError(t, err)
	t.Cleanup(func() { _ = s.Close() })

	runCtx, runCancel := context.WithCancel(ctx)
	defer runCancel()

	started := make(chan struct{})
	go func() {
		close(started)
		_ = s.Run(runCtx)
	}()
	<-started

	// Give the initial snapshot time to arrive.
	assert.Eventually(t, func() bool {
		return allow.Contains("uid-alice")
	}, 5*time.Second, 100*time.Millisecond, "initial snapshot should populate allow-list")

	// Update the document — watcher should call Replace again.
	_, err = raw.Collection("robots").Doc(robotID).Set(ctx, map[string]interface{}{
		"allowedUids": []string{"uid-alice", "uid-bob"},
	})
	require.NoError(t, err)

	assert.Eventually(t, func() bool {
		return allow.Contains("uid-bob")
	}, 5*time.Second, 100*time.Millisecond, "updated snapshot should add uid-bob")
}

// TestHeartbeatUpdatesLastSeen verifies that Heartbeat writes lastSeen and
// status into the Firestore document.
func TestHeartbeatUpdatesLastSeen(t *testing.T) {
	emulatorHost(t)

	const projectID = "test-project"
	const robotID = "robot-hb"

	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()

	allow := noise_shim.NewMemoryAllowList(nil)
	s, err := NewSync(ctx, SyncOptions{
		ProjectID: projectID,
		RobotID:   robotID,
		AllowList: allow,
	})
	require.NoError(t, err)
	defer func() { _ = s.Close() }()

	require.NoError(t, s.Heartbeat(ctx))

	// Read back directly via a separate client.
	raw := newEmulatorClient(t, projectID)
	snap, err := raw.Collection("robots").Doc(robotID).Get(ctx)
	require.NoError(t, err)

	status, _ := snap.Data()["status"].(string)
	assert.Equal(t, "online", status, "status should be 'online' after Heartbeat")

	_, hasLastSeen := snap.Data()["lastSeen"]
	assert.True(t, hasLastSeen, "lastSeen field should be present after Heartbeat")
}

// TestFCMSoftDeleteOnUnregistered verifies that softDeleteDevice sets
// deletedAt on a device document. (Full SendToRobot with real FCM requires a
// live project and is covered by integration tests; here we test the Firestore
// write path only.)
func TestFCMSoftDeleteOnUnregistered(t *testing.T) {
	emulatorHost(t)

	const projectID = "test-project"
	const robotID = "robot-fcm"
	const deviceID = "device-stale"

	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()

	n, err := NewNotifier(ctx, NotifierOptions{
		ProjectID: projectID,
	})
	require.NoError(t, err)
	defer func() { _ = n.Close() }()

	// Seed a device document without deletedAt.
	raw := newEmulatorClient(t, projectID)
	_, err = raw.Collection("robots").Doc(robotID).
		Collection("devices").Doc(deviceID).
		Set(ctx, map[string]interface{}{
			"token": "fake-fcm-token",
		})
	require.NoError(t, err)

	// Invoke softDeleteDevice directly — simulates what SendToRobot does when
	// FCM returns UNREGISTERED for this token.
	require.NoError(t, n.softDeleteDevice(ctx, robotID, deviceID))

	snap, err := raw.Collection("robots").Doc(robotID).
		Collection("devices").Doc(deviceID).Get(ctx)
	require.NoError(t, err)

	deletedAt, exists := snap.Data()["deletedAt"]
	assert.True(t, exists, "deletedAt should be present after soft-delete")
	assert.NotNil(t, deletedAt, "deletedAt should not be nil after soft-delete")
}

// TestLoadActiveTokensSkipsSoftDeleted verifies that loadActiveTokens omits
// documents that carry a non-nil deletedAt field.
func TestLoadActiveTokensSkipsSoftDeleted(t *testing.T) {
	emulatorHost(t)

	const projectID = "test-project"
	const robotID = "robot-tokens"

	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()

	n, err := NewNotifier(ctx, NotifierOptions{
		ProjectID: projectID,
	})
	require.NoError(t, err)
	defer func() { _ = n.Close() }()

	raw := newEmulatorClient(t, projectID)
	col := raw.Collection("robots").Doc(robotID).Collection("devices")

	_, err = col.Doc("dev-active").Set(ctx, map[string]interface{}{
		"token": "active-token",
	})
	require.NoError(t, err)

	_, err = col.Doc("dev-deleted").Set(ctx, map[string]interface{}{
		"token":     "deleted-token",
		"deletedAt": firestore.ServerTimestamp,
	})
	require.NoError(t, err)

	tokens, _, err := n.loadActiveTokens(ctx, robotID)
	require.NoError(t, err)

	assert.Contains(t, tokens, "active-token")
	assert.NotContains(t, tokens, "deleted-token")
}
