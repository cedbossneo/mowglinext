package cloud_sync

import (
	"context"
	"encoding/json"
	"sync"
	"time"
)

// ROSSubscriber is the minimal interface EventDetector needs from the ROS
// provider. The real implementation is types.IRosProvider; the interface is
// defined here so this package stays decoupled from the providers package.
// The caller in main.go wires the concrete type.
//
// Subscribe uses the logical topic keys understood by RosProvider (e.g.
// "power", "highLevelStatus", "emergency"). cb is called from a dedicated
// goroutine; implementations must be safe for concurrent use.
type ROSSubscriber interface {
	Subscribe(topic string, id string, cb func(msg []byte)) error
	UnSubscribe(topic string, id string)
}

// HighLevelState constants mirror HighLevelStatus.msg integer values.
const (
	hlStateNull       uint8 = 0
	hlStateIdle       uint8 = 1
	hlStateAutonomous uint8 = 2
	hlStateRecording  uint8 = 3
	hlStateManual     uint8 = 4
)

// alertCooldown is the minimum time between two alerts on the same channel.
const alertCooldown = 5 * time.Minute

// lowBatteryThreshold is the v_battery value (volts) below which the
// low-battery alert fires.
const lowBatteryThreshold = 22.0

// DetectorOptions carries all constructor parameters for EventDetector.
type DetectorOptions struct {
	// ROS is the subscriber interface used to receive topic messages.
	ROS ROSSubscriber
	// Notify is called whenever a notable event is detected. It should be
	// wired to Notifier.SendToRobot (partially applied with robotID) by the
	// caller. A nil Notify disables all outbound notifications (useful in
	// tests that only verify state transitions).
	Notify func(p Payload) error
}

// EventDetector subscribes to a small set of ROS topics and fires push
// notifications on notable state changes. All exported methods are safe for
// concurrent use after construction.
type EventDetector struct {
	opts DetectorOptions

	mu sync.Mutex

	// lastAlert tracks the last time a notification was sent per channel for
	// cooldown enforcement.
	lastAlert map[string]time.Time

	// prevHLState is the most recently observed high-level state integer.
	prevHLState uint8
	// prevHLSet is false until the first high-level status message arrives.
	prevHLSet bool

	// emergencyActive prevents duplicate emergency alerts for the same event.
	emergencyActive bool
}

// NewEventDetector constructs an EventDetector. Run must be called to start
// subscriptions.
func NewEventDetector(opts DetectorOptions) *EventDetector {
	return &EventDetector{
		opts:      opts,
		lastAlert: make(map[string]time.Time),
	}
}

// Run subscribes to ROS topics and blocks until ctx is cancelled. It returns
// ctx.Err() on clean shutdown or a subscription error on failure. All
// subscriptions are cleaned up before returning.
func (d *EventDetector) Run(ctx context.Context) error {
	type sub struct{ topic, id string }
	subs := []sub{
		{"power", "cloud_sync_power"},
		{"highLevelStatus", "cloud_sync_hls"},
		{"emergency", "cloud_sync_emg"},
	}

	for i, s := range subs {
		topic, id := s.topic, s.id
		var cb func([]byte)
		switch topic {
		case "power":
			cb = d.onPower
		case "highLevelStatus":
			cb = d.onHighLevelStatus
		case "emergency":
			cb = d.onEmergency
		}
		if err := d.opts.ROS.Subscribe(topic, id, cb); err != nil {
			// Clean up already-registered subscriptions before returning.
			for _, s2 := range subs[:i] {
				d.opts.ROS.UnSubscribe(s2.topic, s2.id)
			}
			return err
		}
	}

	<-ctx.Done()

	for _, s := range subs {
		d.opts.ROS.UnSubscribe(s.topic, s.id)
	}
	return ctx.Err()
}

// onPower handles messages from the "power" logical topic
// (/hardware_bridge/power). Fires a low-battery alert when v_battery drops
// below lowBatteryThreshold, subject to the per-channel cooldown.
func (d *EventDetector) onPower(msg []byte) {
	var p struct {
		VBattery float32 `json:"v_battery"`
	}
	if err := json.Unmarshal(msg, &p); err != nil {
		return
	}
	if float64(p.VBattery) < lowBatteryThreshold {
		d.maybeNotify("low-battery", Payload{
			Title:   "Battery low",
			Body:    "Voltage is low — robot may stop soon",
			Channel: "low-battery",
		})
	}
}

// onHighLevelStatus handles messages from the "highLevelStatus" logical topic
// (/behavior_tree_node/high_level_status). Detects IDLE→AUTONOMOUS (mowing
// started), AUTONOMOUS→IDLE+charging (mowing complete), and
// AUTONOMOUS→IDLE+not-charging (unexpected stop) transitions.
func (d *EventDetector) onHighLevelStatus(msg []byte) {
	var s struct {
		State      uint8 `json:"state"`
		IsCharging bool  `json:"is_charging"`
	}
	if err := json.Unmarshal(msg, &s); err != nil {
		return
	}

	d.mu.Lock()
	prev := d.prevHLState
	prevSet := d.prevHLSet
	d.prevHLState = s.State
	d.prevHLSet = true
	d.mu.Unlock()

	if !prevSet {
		// First message: record baseline state, no transition to report.
		return
	}

	switch {
	case prev == hlStateIdle && s.State == hlStateAutonomous:
		d.maybeNotify("info", Payload{
			Title:   "Mowing started",
			Body:    "The robot has left the dock and started mowing.",
			Channel: "info",
		})

	case prev == hlStateAutonomous && s.State == hlStateIdle && s.IsCharging:
		d.maybeNotify("info", Payload{
			Title:   "Mowing complete",
			Body:    "The robot has finished mowing and returned to the dock.",
			Channel: "info",
		})

	case prev == hlStateAutonomous && s.State == hlStateIdle && !s.IsCharging:
		d.maybeNotify("alerts", Payload{
			Title:   "Mowing stopped",
			Body:    "The robot stopped autonomously and is not on the dock.",
			Channel: "alerts",
		})
	}
}

// onEmergency handles messages from the "emergency" logical topic
// (/hardware_bridge/emergency). Fires once per emergency activation; resets
// the latch when the emergency clears so a subsequent activation fires again.
func (d *EventDetector) onEmergency(msg []byte) {
	var e struct {
		ActiveEmergency  bool `json:"active_emergency"`
		LatchedEmergency bool `json:"latched_emergency"`
	}
	if err := json.Unmarshal(msg, &e); err != nil {
		return
	}

	active := e.ActiveEmergency || e.LatchedEmergency

	d.mu.Lock()
	wasActive := d.emergencyActive
	d.emergencyActive = active
	d.mu.Unlock()

	if active && !wasActive {
		d.maybeNotify("emergency", Payload{
			Title:   "Emergency stop",
			Body:    "The robot has triggered an emergency stop.",
			Channel: "emergency",
		})
	}
}

// maybeNotify fires p via the Notify callback only when the per-channel
// cooldown has elapsed. Safe for concurrent use.
func (d *EventDetector) maybeNotify(channel string, p Payload) {
	d.mu.Lock()
	last := d.lastAlert[channel]
	if time.Since(last) < alertCooldown {
		d.mu.Unlock()
		return
	}
	d.lastAlert[channel] = time.Now()
	d.mu.Unlock()

	if d.opts.Notify == nil {
		return
	}
	// Call outside the lock — the callback may block on network I/O.
	_ = d.opts.Notify(p)
}
