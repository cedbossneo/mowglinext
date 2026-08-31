package providers

import (
	"encoding/json"
	"time"
)

const defaultGnssAuthorityTimeout = 5 * time.Second

type gnssAuthorityAction uint8

const (
	gnssAuthorityIgnore gnssAuthorityAction = iota
	gnssAuthorityPublish
	gnssAuthorityInvalidate
)

type gnssReceiptStamp struct {
	sec     int64
	nanosec uint32
}

func (stamp gnssReceiptStamp) valid() bool {
	return stamp.nanosec < 1_000_000_000 &&
		(stamp.sec > 0 || (stamp.sec == 0 && stamp.nanosec > 0))
}

func (stamp gnssReceiptStamp) compare(other gnssReceiptStamp) int {
	if stamp.sec < other.sec {
		return -1
	}
	if stamp.sec > other.sec {
		return 1
	}
	if stamp.nanosec < other.nanosec {
		return -1
	}
	if stamp.nanosec > other.nanosec {
		return 1
	}
	return 0
}

type gnssAuthorityEnvelope struct {
	Header struct {
		Stamp struct {
			Sec     int64  `json:"sec"`
			Nanosec uint32 `json:"nanosec"`
		} `json:"stamp"`
	} `json:"header"`
	PositionObservationSequence uint64 `json:"position_observation_sequence"`
}

type gnssAuthorityTracker struct {
	timeout time.Duration

	connected  bool
	generation uint64

	hasIdentity bool
	sequence    uint64
	stamp       gnssReceiptStamp

	authoritative   bool
	lastDelivery    time.Time
	lastObservation time.Time
}

func newGnssAuthorityTracker(timeout time.Duration) *gnssAuthorityTracker {
	return &gnssAuthorityTracker{timeout: timeout}
}

// ConnectionChanged updates transport incarnation without discarding the last
// accepted observation watermark. Preserving that watermark is what prevents a
// pre-loss cached sample from regaining authority after reconnect.
func (tracker *gnssAuthorityTracker) ConnectionChanged(connected bool, generation uint64) bool {
	if tracker.connected == connected && tracker.generation == generation {
		return false
	}

	invalidated := tracker.authoritative
	tracker.connected = connected
	tracker.generation = generation
	tracker.authoritative = false
	tracker.lastDelivery = time.Time{}
	tracker.lastObservation = time.Time{}
	return invalidated
}

func (tracker *gnssAuthorityTracker) Observe(message []byte, now time.Time) gnssAuthorityAction {
	if !tracker.connected {
		return gnssAuthorityIgnore
	}

	var envelope gnssAuthorityEnvelope
	if err := json.Unmarshal(message, &envelope); err != nil {
		return tracker.invalidate()
	}
	stamp := gnssReceiptStamp{
		sec:     envelope.Header.Stamp.Sec,
		nanosec: envelope.Header.Stamp.Nanosec,
	}
	if !stamp.valid() {
		return tracker.invalidate()
	}
	if (!tracker.lastDelivery.IsZero() && now.Before(tracker.lastDelivery)) ||
		(!tracker.lastObservation.IsZero() && now.Before(tracker.lastObservation)) {
		return tracker.invalidate()
	}
	tracker.lastDelivery = now

	if !tracker.hasIdentity {
		tracker.accept(envelope.PositionObservationSequence, stamp, now)
		return gnssAuthorityPublish
	}

	sequence := envelope.PositionObservationSequence
	stampOrder := stamp.compare(tracker.stamp)
	if sequence != 0 && tracker.sequence != 0 {
		switch {
		case sequence == tracker.sequence && stampOrder == 0:
			return tracker.publishCachedIfCurrent(now)
		case sequence == tracker.sequence:
			return tracker.invalidate()
		case sequence > tracker.sequence && stampOrder >= 0:
			tracker.accept(sequence, stamp, now)
			return gnssAuthorityPublish
		case sequence < tracker.sequence && stampOrder > 0:
			tracker.accept(sequence, stamp, now)
			return gnssAuthorityPublish
		default:
			return tracker.ignoreOutOfOrder(now)
		}
	}

	if stampOrder == 0 {
		if sequence != 0 && tracker.sequence == 0 {
			tracker.accept(sequence, stamp, now)
			return gnssAuthorityPublish
		}
		return tracker.publishCachedIfCurrent(now)
	}
	if stampOrder < 0 {
		if sequence == 0 {
			return tracker.invalidate()
		}
		return tracker.ignoreOutOfOrder(now)
	}

	tracker.accept(sequence, stamp, now)
	return gnssAuthorityPublish
}

func (tracker *gnssAuthorityTracker) Current(now time.Time) bool {
	if !tracker.connected || !tracker.authoritative ||
		tracker.timeout < 0 || tracker.lastDelivery.IsZero() ||
		tracker.lastObservation.IsZero() || now.Before(tracker.lastDelivery) ||
		now.Before(tracker.lastObservation) {
		return false
	}
	return now.Sub(tracker.lastDelivery) < tracker.timeout &&
		now.Sub(tracker.lastObservation) < tracker.timeout
}

func (tracker *gnssAuthorityTracker) Expire(now time.Time) bool {
	if !tracker.authoritative || tracker.Current(now) {
		return false
	}
	tracker.authoritative = false
	return true
}

func (tracker *gnssAuthorityTracker) NextDeadline() (time.Time, bool) {
	if !tracker.authoritative || tracker.lastDelivery.IsZero() ||
		tracker.lastObservation.IsZero() {
		return time.Time{}, false
	}
	deliveryDeadline := tracker.lastDelivery.Add(tracker.timeout)
	observationDeadline := tracker.lastObservation.Add(tracker.timeout)
	if observationDeadline.Before(deliveryDeadline) {
		return observationDeadline, true
	}
	return deliveryDeadline, true
}

func (tracker *gnssAuthorityTracker) accept(sequence uint64, stamp gnssReceiptStamp, now time.Time) {
	tracker.hasIdentity = true
	tracker.sequence = sequence
	tracker.stamp = stamp
	tracker.authoritative = true
	tracker.lastObservation = now
}

func (tracker *gnssAuthorityTracker) invalidate() gnssAuthorityAction {
	if !tracker.authoritative {
		return gnssAuthorityIgnore
	}
	tracker.authoritative = false
	return gnssAuthorityInvalidate
}

func (tracker *gnssAuthorityTracker) publishCachedIfCurrent(now time.Time) gnssAuthorityAction {
	if tracker.Current(now) {
		return gnssAuthorityPublish
	}
	return tracker.invalidate()
}

func (tracker *gnssAuthorityTracker) ignoreOutOfOrder(now time.Time) gnssAuthorityAction {
	if tracker.Current(now) {
		return gnssAuthorityIgnore
	}
	return tracker.invalidate()
}
