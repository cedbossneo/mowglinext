package api

import (
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestCalibrationJobStore_LifecycleHappyPath(t *testing.T) {
	store := newCalibrationJobStore()

	job := store.create()
	require.NotEmpty(t, job.ID)
	assert.Equal(t, CalibrationJobRunning, job.State)
	assert.False(t, job.StartedAt.IsZero())
	assert.Nil(t, job.EndedAt)

	got, ok := store.get(job.ID)
	require.True(t, ok)
	assert.Equal(t, job.ID, got.ID)

	store.markDone(job.ID, &CalibrateImuYawResponse{Success: true, ImuYawDeg: -4.86})
	finished, _ := store.get(job.ID)
	assert.Equal(t, CalibrationJobDone, finished.State)
	require.NotNil(t, finished.Result)
	assert.InDelta(t, -4.86, finished.Result.ImuYawDeg, 1e-9)
	require.NotNil(t, finished.EndedAt)
}

func TestCalibrationJobStore_FailurePath(t *testing.T) {
	store := newCalibrationJobStore()
	job := store.create()
	store.markFailed(job.ID, "service not advertised")

	got, _ := store.get(job.ID)
	assert.Equal(t, CalibrationJobFailed, got.State)
	assert.Equal(t, "service not advertised", got.Error)
	assert.Nil(t, got.Result)
}

func TestCalibrationJobStore_PrunesCompletedJobsPastRetention(t *testing.T) {
	store := newCalibrationJobStore()
	store.retentionWindow = 50 * time.Millisecond

	job := store.create()
	store.markDone(job.ID, &CalibrateImuYawResponse{Success: true})

	// Still present immediately after completion.
	_, ok := store.get(job.ID)
	require.True(t, ok)

	// Backdate EndedAt past the retention window.
	jobs := storeSnapshot(t, store)
	jobs[0].EndedAt = ptrTime(time.Now().Add(-1 * time.Second))

	// Next access prunes it.
	_, ok = store.get(job.ID)
	assert.False(t, ok, "expired job should be pruned on next access")
}

func TestCalibrationJobStore_RunningJobsAreNeverPruned(t *testing.T) {
	store := newCalibrationJobStore()
	store.retentionWindow = 1 * time.Millisecond

	job := store.create()
	time.Sleep(5 * time.Millisecond)

	got, ok := store.get(job.ID)
	require.True(t, ok, "running job must not be pruned by retention window")
	assert.Equal(t, CalibrationJobRunning, got.State)
}

func TestCalibrationJobStore_GetUnknownJobReturnsFalse(t *testing.T) {
	store := newCalibrationJobStore()
	_, ok := store.get("nope-not-a-real-id")
	assert.False(t, ok)
}

func TestNewJobID_IsHexAndReasonablyLong(t *testing.T) {
	a := newJobID()
	b := newJobID()
	assert.NotEqual(t, a, b)
	assert.GreaterOrEqual(t, len(a), 20)
}

func TestClampDuration_Bounds(t *testing.T) {
	assert.InDelta(t, 30.0, clampDuration(0), 1e-9)
	assert.InDelta(t, 30.0, clampDuration(-5), 1e-9)
	assert.InDelta(t, 120.0, clampDuration(500), 1e-9)
	assert.InDelta(t, 45.5, clampDuration(45.5), 1e-9)
}

// helpers

func storeSnapshot(t *testing.T, s *calibrationJobStore) []*CalibrationJob {
	t.Helper()
	s.mu.Lock()
	defer s.mu.Unlock()
	out := make([]*CalibrationJob, 0, len(s.jobs))
	for _, j := range s.jobs {
		out = append(out, j)
	}
	return out
}

func ptrTime(t time.Time) *time.Time { return &t }
