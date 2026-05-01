package cloud_sync

import (
	"encoding/json"
	"sync"
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// fakeROS is a test double for ROSSubscriber. Callbacks registered via
// Subscribe are stored and can be driven by calling deliver.
type fakeROS struct {
	mu   sync.Mutex
	subs map[string]func([]byte) // topic -> latest cb
}

func newFakeROS() *fakeROS {
	return &fakeROS{subs: make(map[string]func([]byte))}
}

func (f *fakeROS) Subscribe(topic, _ string, cb func([]byte)) error {
	f.mu.Lock()
	f.subs[topic] = cb
	f.mu.Unlock()
	return nil
}

func (f *fakeROS) UnSubscribe(topic, _ string) {
	f.mu.Lock()
	delete(f.subs, topic)
	f.mu.Unlock()
}

// deliver invokes the registered callback for topic with msg synchronously.
func (f *fakeROS) deliver(topic string, msg interface{}) {
	f.mu.Lock()
	cb := f.subs[topic]
	f.mu.Unlock()
	if cb == nil {
		return
	}
	b, _ := json.Marshal(msg)
	cb(b)
}

// mustDetector constructs an EventDetector and registers its internal handlers
// directly on the fakeROS without spawning a goroutine (avoids context
// plumbing in pure-unit tests).
func mustDetector(t *testing.T, ros *fakeROS, notify func(Payload) error) *EventDetector {
	t.Helper()
	d := NewEventDetector(DetectorOptions{ROS: ros, Notify: notify})
	require.NoError(t, ros.Subscribe("power", "cloud_sync_power", d.onPower))
	require.NoError(t, ros.Subscribe("highLevelStatus", "cloud_sync_hls", d.onHighLevelStatus))
	require.NoError(t, ros.Subscribe("emergency", "cloud_sync_emg", d.onEmergency))
	return d
}

// ---------------------------------------------------------------------------
// Battery tests
// ---------------------------------------------------------------------------

func TestBatteryLowFires(t *testing.T) {
	ros := newFakeROS()
	var got []Payload
	var mu sync.Mutex

	mustDetector(t, ros, func(p Payload) error {
		mu.Lock()
		got = append(got, p)
		mu.Unlock()
		return nil
	})

	ros.deliver("power", map[string]interface{}{"v_battery": 21.0})

	mu.Lock()
	defer mu.Unlock()
	require.Len(t, got, 1)
	assert.Equal(t, "low-battery", got[0].Channel)
	assert.Equal(t, "Battery low", got[0].Title)
}

func TestBatteryAboveThresholdNoFire(t *testing.T) {
	ros := newFakeROS()
	var count int
	var mu sync.Mutex

	mustDetector(t, ros, func(p Payload) error {
		mu.Lock()
		count++
		mu.Unlock()
		return nil
	})

	ros.deliver("power", map[string]interface{}{"v_battery": 25.5})

	mu.Lock()
	defer mu.Unlock()
	assert.Equal(t, 0, count, "no alert when battery is healthy")
}

func TestBatteryLowCooldown(t *testing.T) {
	ros := newFakeROS()
	var count int
	var mu sync.Mutex

	d := mustDetector(t, ros, func(p Payload) error {
		mu.Lock()
		count++
		mu.Unlock()
		return nil
	})

	// Ensure the first delivery fires by clearing any existing cooldown entry.
	d.mu.Lock()
	d.lastAlert["low-battery"] = time.Time{}
	d.mu.Unlock()

	ros.deliver("power", map[string]interface{}{"v_battery": 20.0})
	// Second delivery is within the cooldown window.
	ros.deliver("power", map[string]interface{}{"v_battery": 20.0})

	mu.Lock()
	defer mu.Unlock()
	assert.Equal(t, 1, count, "second delivery within cooldown must be suppressed")
}

// ---------------------------------------------------------------------------
// High-level state transition tests
// ---------------------------------------------------------------------------

func TestMowingStartedTransition(t *testing.T) {
	ros := newFakeROS()
	var got []Payload
	var mu sync.Mutex

	mustDetector(t, ros, func(p Payload) error {
		mu.Lock()
		got = append(got, p)
		mu.Unlock()
		return nil
	})

	// Establish baseline state: IDLE.
	ros.deliver("highLevelStatus", map[string]interface{}{
		"state": hlStateIdle, "is_charging": false,
	})
	// Transition to AUTONOMOUS — should fire "Mowing started".
	ros.deliver("highLevelStatus", map[string]interface{}{
		"state": hlStateAutonomous, "is_charging": false,
	})

	mu.Lock()
	defer mu.Unlock()
	require.Len(t, got, 1)
	assert.Equal(t, "info", got[0].Channel)
	assert.Equal(t, "Mowing started", got[0].Title)
}

func TestMowingCompleteTransitionDocked(t *testing.T) {
	ros := newFakeROS()
	var got []Payload
	var mu sync.Mutex

	mustDetector(t, ros, func(p Payload) error {
		mu.Lock()
		got = append(got, p)
		mu.Unlock()
		return nil
	})

	// Establish AUTONOMOUS baseline.
	ros.deliver("highLevelStatus", map[string]interface{}{
		"state": hlStateAutonomous, "is_charging": false,
	})
	// AUTONOMOUS→IDLE with is_charging=true means robot docked successfully.
	ros.deliver("highLevelStatus", map[string]interface{}{
		"state": hlStateIdle, "is_charging": true,
	})

	mu.Lock()
	defer mu.Unlock()
	require.Len(t, got, 1)
	assert.Equal(t, "info", got[0].Channel)
	assert.Equal(t, "Mowing complete", got[0].Title)
}

func TestMowingStoppedTransitionNotDocked(t *testing.T) {
	ros := newFakeROS()
	var got []Payload
	var mu sync.Mutex

	mustDetector(t, ros, func(p Payload) error {
		mu.Lock()
		got = append(got, p)
		mu.Unlock()
		return nil
	})

	ros.deliver("highLevelStatus", map[string]interface{}{
		"state": hlStateAutonomous, "is_charging": false,
	})
	// AUTONOMOUS→IDLE without charging: unexpected stop.
	ros.deliver("highLevelStatus", map[string]interface{}{
		"state": hlStateIdle, "is_charging": false,
	})

	mu.Lock()
	defer mu.Unlock()
	require.Len(t, got, 1)
	assert.Equal(t, "alerts", got[0].Channel)
	assert.Equal(t, "Mowing stopped", got[0].Title)
}

func TestNoTransitionOnFirstMessage(t *testing.T) {
	ros := newFakeROS()
	var count int
	var mu sync.Mutex

	mustDetector(t, ros, func(p Payload) error {
		mu.Lock()
		count++
		mu.Unlock()
		return nil
	})

	// Single message with no prior state — no transition, no notification.
	ros.deliver("highLevelStatus", map[string]interface{}{
		"state": hlStateIdle, "is_charging": false,
	})

	mu.Lock()
	defer mu.Unlock()
	assert.Equal(t, 0, count, "no notification on first message (no previous state)")
}

// ---------------------------------------------------------------------------
// Emergency tests
// ---------------------------------------------------------------------------

func TestEmergencyFiresOnActivation(t *testing.T) {
	ros := newFakeROS()
	var got []Payload
	var mu sync.Mutex

	mustDetector(t, ros, func(p Payload) error {
		mu.Lock()
		got = append(got, p)
		mu.Unlock()
		return nil
	})

	ros.deliver("emergency", map[string]interface{}{
		"active_emergency": true, "latched_emergency": false,
	})

	mu.Lock()
	defer mu.Unlock()
	require.Len(t, got, 1)
	assert.Equal(t, "emergency", got[0].Channel)
	assert.Equal(t, "Emergency stop", got[0].Title)
}

func TestEmergencyLatchNoDuplicateAlert(t *testing.T) {
	ros := newFakeROS()
	var count int
	var mu sync.Mutex

	mustDetector(t, ros, func(p Payload) error {
		mu.Lock()
		count++
		mu.Unlock()
		return nil
	})

	// First activation fires.
	ros.deliver("emergency", map[string]interface{}{
		"active_emergency": true, "latched_emergency": false,
	})
	// Repeated message while still active must not fire again.
	ros.deliver("emergency", map[string]interface{}{
		"active_emergency": true, "latched_emergency": false,
	})

	mu.Lock()
	defer mu.Unlock()
	assert.Equal(t, 1, count, "duplicate emergency message must not fire a second alert")
}

func TestEmergencyRefiresAfterClear(t *testing.T) {
	ros := newFakeROS()
	var count int
	var mu sync.Mutex

	d := mustDetector(t, ros, func(p Payload) error {
		mu.Lock()
		count++
		mu.Unlock()
		return nil
	})

	// First activation.
	ros.deliver("emergency", map[string]interface{}{
		"active_emergency": true, "latched_emergency": false,
	})

	// Reset the channel cooldown so the second activation is not suppressed by
	// the time-based gate (the emergency latch is the behaviour under test here).
	d.mu.Lock()
	d.lastAlert["emergency"] = time.Time{}
	d.mu.Unlock()

	// Clear the emergency.
	ros.deliver("emergency", map[string]interface{}{
		"active_emergency": false, "latched_emergency": false,
	})
	// New activation after clear must fire again.
	ros.deliver("emergency", map[string]interface{}{
		"active_emergency": true, "latched_emergency": false,
	})

	mu.Lock()
	defer mu.Unlock()
	assert.Equal(t, 2, count, "emergency should refire after it cleared and activated again")
}
