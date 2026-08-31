package providers

import (
	"encoding/json"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/foxglove"
)

func gnssAuthorityMessage(t *testing.T, sequence uint64, sec int64, nanosec uint32) []byte {
	t.Helper()
	message := map[string]any{
		"header": map[string]any{
			"stamp": map[string]any{"sec": sec, "nanosec": nanosec},
		},
		"backend":                       "universal",
		"fix_type":                      3,
		"fix_valid":                     true,
		"position_observation_sequence": sequence,
	}
	encoded, err := json.Marshal(message)
	if err != nil {
		t.Fatalf("json.Marshal: %v", err)
	}
	return encoded
}

func TestGnssAuthorityDisconnectPreservesWatermarkAgainstReplay(t *testing.T) {
	tracker := newGnssAuthorityTracker(5 * time.Second)
	now := time.Unix(1_000, 0)
	if tracker.ConnectionChanged(true, 1) {
		t.Fatal("initial connection unexpectedly invalidated authority")
	}
	preLoss := gnssAuthorityMessage(t, 10, 100, 0)
	if got := tracker.Observe(preLoss, now); got != gnssAuthorityPublish {
		t.Fatalf("first observation action = %v, want publish", got)
	}

	if !tracker.ConnectionChanged(false, 1) {
		t.Fatal("disconnect did not invalidate current authority")
	}
	if tracker.Current(now) {
		t.Fatal("authority remained current after disconnect")
	}
	if tracker.ConnectionChanged(true, 2) {
		t.Fatal("reconnect emitted a second invalidation")
	}
	if got := tracker.Observe(preLoss, now.Add(time.Second)); got != gnssAuthorityIgnore {
		t.Fatalf("pre-loss replay action = %v, want ignore", got)
	}
	if tracker.Current(now.Add(time.Second)) {
		t.Fatal("pre-loss replay restored authority")
	}

	fresh := gnssAuthorityMessage(t, 11, 101, 0)
	if got := tracker.Observe(fresh, now.Add(2*time.Second)); got != gnssAuthorityPublish {
		t.Fatalf("fresh post-reconnect action = %v, want publish", got)
	}
}

func TestGnssAuthorityCachedPublicationDoesNotExtendObservationDeadline(t *testing.T) {
	tracker := newGnssAuthorityTracker(5 * time.Second)
	now := time.Unix(2_000, 0)
	tracker.ConnectionChanged(true, 1)
	message := gnssAuthorityMessage(t, 20, 200, 0)
	if got := tracker.Observe(message, now); got != gnssAuthorityPublish {
		t.Fatalf("first action = %v, want publish", got)
	}
	if got := tracker.Observe(message, now.Add(4*time.Second)); got != gnssAuthorityPublish {
		t.Fatalf("current cached publication action = %v, want publish", got)
	}
	if !tracker.Expire(now.Add(5 * time.Second)) {
		t.Fatal("same provenance did not expire at its original deadline")
	}
	if got := tracker.Observe(message, now.Add(5*time.Second)); got != gnssAuthorityIgnore {
		t.Fatalf("expired cached publication action = %v, want ignore", got)
	}
	if tracker.Current(now.Add(5 * time.Second)) {
		t.Fatal("expired cached publication restored authority")
	}
}

func TestGnssAuthorityAdvancingObservationSupersedesOldDeadline(t *testing.T) {
	tracker := newGnssAuthorityTracker(5 * time.Second)
	now := time.Unix(3_000, 0)
	tracker.ConnectionChanged(true, 1)
	tracker.Observe(gnssAuthorityMessage(t, 30, 300, 0), now)
	tracker.Observe(gnssAuthorityMessage(t, 31, 301, 0), now.Add(4*time.Second))

	if tracker.Expire(now.Add(5 * time.Second)) {
		t.Fatal("obsolete first-observation deadline invalidated newer authority")
	}
	if !tracker.Current(now.Add(5 * time.Second)) {
		t.Fatal("newer observation was not current")
	}
	if !tracker.Expire(now.Add(9 * time.Second)) {
		t.Fatal("newer observation did not expire at its own deadline")
	}
}

func TestGnssAuthorityIdentityRules(t *testing.T) {
	t.Run("source restart uses lower sequence and later receipt", func(t *testing.T) {
		tracker := newGnssAuthorityTracker(5 * time.Second)
		now := time.Unix(4_000, 0)
		tracker.ConnectionChanged(true, 1)
		tracker.Observe(gnssAuthorityMessage(t, 80, 800, 0), now)
		if got := tracker.Observe(gnssAuthorityMessage(t, 1, 801, 0), now.Add(time.Second)); got != gnssAuthorityPublish {
			t.Fatalf("source restart action = %v, want publish", got)
		}
	})

	t.Run("delayed sequence cannot replace current authority", func(t *testing.T) {
		tracker := newGnssAuthorityTracker(5 * time.Second)
		now := time.Unix(5_000, 0)
		tracker.ConnectionChanged(true, 1)
		tracker.Observe(gnssAuthorityMessage(t, 80, 800, 0), now)
		if got := tracker.Observe(gnssAuthorityMessage(t, 79, 799, 0), now.Add(time.Second)); got != gnssAuthorityIgnore {
			t.Fatalf("delayed action = %v, want ignore", got)
		}
		if !tracker.Current(now.Add(time.Second)) {
			t.Fatal("delayed packet cleared newer current authority")
		}
	})

	t.Run("equal sequence with changed receipt invalidates", func(t *testing.T) {
		tracker := newGnssAuthorityTracker(5 * time.Second)
		now := time.Unix(6_000, 0)
		tracker.ConnectionChanged(true, 1)
		tracker.Observe(gnssAuthorityMessage(t, 80, 800, 0), now)
		if got := tracker.Observe(gnssAuthorityMessage(t, 80, 801, 0), now.Add(time.Second)); got != gnssAuthorityInvalidate {
			t.Fatalf("contradictory action = %v, want invalidate", got)
		}
	})

	t.Run("zero receipt is invalid", func(t *testing.T) {
		tracker := newGnssAuthorityTracker(5 * time.Second)
		now := time.Unix(7_000, 0)
		tracker.ConnectionChanged(true, 1)
		tracker.Observe(gnssAuthorityMessage(t, 1, 700, 0), now)
		if got := tracker.Observe(gnssAuthorityMessage(t, 1, 0, 0), now.Add(time.Second)); got != gnssAuthorityInvalidate {
			t.Fatalf("zero receipt action = %v, want invalidate", got)
		}
	})
}

func TestGnssAuthorityDeliverySilenceExpires(t *testing.T) {
	tracker := newGnssAuthorityTracker(5 * time.Second)
	now := time.Unix(8_000, 0)
	tracker.ConnectionChanged(true, 1)
	tracker.Observe(gnssAuthorityMessage(t, 90, 900, 0), now)
	if tracker.Expire(now.Add(4*time.Second + 999*time.Millisecond)) {
		t.Fatal("authority expired before delivery deadline")
	}
	if !tracker.Expire(now.Add(5 * time.Second)) {
		t.Fatal("delivery silence did not expire authority")
	}
}

func newAuthorityTestRosProvider() *RosProvider {
	return &RosProvider{
		client:             foxglove.NewClient("ws://unused"),
		subscribers:        make(map[string]map[string]*RosSubscriber),
		lastMessage:        make(map[string][]byte),
		foxgloveSubscribed: make(map[string]bool),
		gnssAuthority:      newGnssAuthorityTracker(5 * time.Second),
	}
}

func pendingSubscriberMessage(subscriber *RosSubscriber) []byte {
	subscriber.mtx.Lock()
	defer subscriber.mtx.Unlock()
	return append([]byte(nil), subscriber.nextMessage...)
}

func TestRosProviderGnssLossDeletesCacheAndRejectsPreLossReplay(t *testing.T) {
	provider := newAuthorityTestRosProvider()
	provider.onFoxgloveConnectionState(foxglove.ConnectionState{Connected: true, Generation: 1})
	preLoss := gnssAuthorityMessage(t, 100, 1_000, 0)
	provider.fanOut("gnssStatus", preLoss)
	if got := string(provider.lastMessage["gnssStatus"]); got != string(preLoss) {
		t.Fatalf("cached GNSS = %q, want pre-loss sample", got)
	}

	subscriber := &RosSubscriber{}
	provider.subscribers["gnssStatus"] = map[string]*RosSubscriber{"browser": subscriber}
	provider.onFoxgloveConnectionState(foxglove.ConnectionState{Connected: false, Generation: 1})
	if _, ok := provider.lastMessage["gnssStatus"]; ok {
		t.Fatal("GNSS cache survived upstream loss")
	}
	if got := string(pendingSubscriberMessage(subscriber)); got != "{}" {
		t.Fatalf("loss message = %q, want tombstone", got)
	}

	provider.onFoxgloveConnectionState(foxglove.ConnectionState{Connected: true, Generation: 2})
	provider.fanOut("gnssStatus", preLoss)
	if _, ok := provider.lastMessage["gnssStatus"]; ok {
		t.Fatal("pre-loss replay regained cache authority")
	}

	fresh := gnssAuthorityMessage(t, 101, 1_001, 0)
	provider.fanOut("gnssStatus", fresh)
	if got := string(provider.lastMessage["gnssStatus"]); got != string(fresh) {
		t.Fatalf("fresh post-reconnect cache = %q, want fresh sample", got)
	}
	provider.mtx.Lock()
	provider.cancelGnssDeadlineLocked()
	provider.mtx.Unlock()
}

func TestRosProviderGnssSubscribeDoesNotReplayInvalidCache(t *testing.T) {
	provider := newAuthorityTestRosProvider()
	provider.lastMessage["gnssStatus"] = gnssAuthorityMessage(t, 200, 2_000, 0)
	subscriber := &RosSubscriber{}
	provider.subscribers["gnssStatus"] = map[string]*RosSubscriber{"browser": subscriber}

	if err := provider.Subscribe("gnssStatus", "browser", 0, func([]byte) {}); err != nil {
		t.Fatalf("Subscribe: %v", err)
	}
	if got := string(pendingSubscriberMessage(subscriber)); got != "{}" {
		t.Fatalf("invalid-cache subscribe message = %q, want tombstone", got)
	}
	if _, ok := provider.lastMessage["gnssStatus"]; ok {
		t.Fatal("invalid GNSS cache survived Subscribe")
	}
}

func TestRosProviderGnssSubscribeReplaysCurrentCache(t *testing.T) {
	provider := newAuthorityTestRosProvider()
	provider.onFoxgloveConnectionState(foxglove.ConnectionState{Connected: true, Generation: 1})
	current := gnssAuthorityMessage(t, 250, 2_500, 0)
	provider.fanOut("gnssStatus", current)
	subscriber := &RosSubscriber{}
	provider.subscribers["gnssStatus"] = map[string]*RosSubscriber{"browser": subscriber}

	if err := provider.Subscribe("gnssStatus", "browser", 0, func([]byte) {}); err != nil {
		t.Fatalf("Subscribe: %v", err)
	}
	if got := string(pendingSubscriberMessage(subscriber)); got != string(current) {
		t.Fatalf("current GNSS replay = %q, want %q", got, current)
	}
	provider.mtx.Lock()
	provider.cancelGnssDeadlineLocked()
	provider.mtx.Unlock()
}

func TestRosProviderNonGnssReplayRemainsUnchanged(t *testing.T) {
	provider := newAuthorityTestRosProvider()
	payload := []byte(`{"state":"MOWING"}`)
	provider.lastMessage["highLevelStatus"] = payload
	subscriber := &RosSubscriber{}
	provider.subscribers["highLevelStatus"] = map[string]*RosSubscriber{"browser": subscriber}

	if err := provider.Subscribe("highLevelStatus", "browser", 0, func([]byte) {}); err != nil {
		t.Fatalf("Subscribe: %v", err)
	}
	if got := string(pendingSubscriberMessage(subscriber)); got != string(payload) {
		t.Fatalf("non-GNSS replay = %q, want %q", got, payload)
	}
	provider.onFoxgloveConnectionState(foxglove.ConnectionState{Connected: false, Generation: 1})
	if got := string(provider.lastMessage["highLevelStatus"]); got != string(payload) {
		t.Fatalf("non-GNSS cache changed on Foxglove loss: %q", got)
	}
}
