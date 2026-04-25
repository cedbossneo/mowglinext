package foxglove

import (
	"encoding/binary"
	"math"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// SetDockingPoint.srv request schema as foxglove_bridge advertises it: the
// concatenated msg definitions separated by the standard "=" divider lines.
const setDockingPointReqSchema = `geometry_msgs/Pose docking_pose
================================================================================
MSG: geometry_msgs/Pose
Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
`

// TestSerializeCDR_PoseFloat64Alignment is the regression test for the
// /map_server_node/set_docking_point garbage-pose bug.
//
// On real hardware (Pi5, ROS 2 Kilted, Cyclone DDS) we observed that values
// sent through this serializer landed at byte offset 8 in the wire payload
// while map_server_node read them from offset 4 — yielding either zeros (when
// the misread bytes happened to be the encap padding) or huge garbage like
// −9.26e+58 (when the misread bytes spanned padding + actual mantissa bytes).
//
// The fix in this PR sets maxAlign=4 and switches align() to be relative to
// the start of the payload, so float64 fields land where rmw expects them.
func TestSerializeCDR_PoseFloat64Alignment(t *testing.T) {
	schema, err := ParseSchema(setDockingPointReqSchema)
	require.NoError(t, err)

	// Picked to make alignment failures obvious: every byte of every double
	// is non-zero so a misalignment of even one byte changes the value
	// dramatically.
	jsonReq := []byte(`{"docking_pose":{"position":{"x":4.2,"y":7.7,"z":1.5},"orientation":{"x":0.1,"y":0.2,"z":0.5,"w":0.866}}}`)

	out, err := SerializeCDR(jsonReq, schema)
	require.NoError(t, err)

	// 4-byte encap header + 7 doubles + 4 bytes of trailing pad (rmw checks
	// total wire size for a multiple of 8 — see SerializeCDR's end-of-buffer
	// pad).
	require.Len(t, out, 64)
	assert.Equal(t, []byte{0x00, 0x01, 0x00, 0x00}, out[:4], "encap header")

	readDouble := func(off int) float64 {
		return math.Float64frombits(binary.LittleEndian.Uint64(out[off : off+8]))
	}

	// position.x must be the FIRST field after the encap header, at byte 4 —
	// not byte 8. This is the actual regression: with the old absolute-from-
	// buffer-start alignment the writer padded 4 bytes here and the C++
	// receiver read garbage.
	assert.Equal(t, 4.2, readDouble(4), "position.x at byte 4")
	assert.Equal(t, 7.7, readDouble(12), "position.y at byte 12")
	assert.Equal(t, 1.5, readDouble(20), "position.z at byte 20")
	assert.Equal(t, 0.1, readDouble(28), "orientation.x at byte 28")
	assert.Equal(t, 0.2, readDouble(36), "orientation.y at byte 36")
	assert.Equal(t, 0.5, readDouble(44), "orientation.z at byte 44")
	assert.Equal(t, 0.866, readDouble(52), "orientation.w at byte 52")
}

// TestSerializeCDR_RoundTripsThroughReader cross-checks the writer against the
// existing cdrReader so the two stay in lock-step.
func TestSerializeCDR_RoundTripsThroughReader(t *testing.T) {
	schema, err := ParseSchema(setDockingPointReqSchema)
	require.NoError(t, err)

	jsonReq := []byte(`{"docking_pose":{"position":{"x":-1.25,"y":3.5e9,"z":0},"orientation":{"x":0,"y":0,"z":0,"w":1}}}`)

	wire, err := SerializeCDR(jsonReq, schema)
	require.NoError(t, err)

	decoded, err := DeserializeCDR(wire, schema)
	require.NoError(t, err)

	pose := decoded["docking_pose"].(map[string]interface{})
	pos := pose["position"].(map[string]interface{})
	orient := pose["orientation"].(map[string]interface{})

	assert.InDelta(t, -1.25, pos["x"], 1e-12)
	assert.InDelta(t, 3.5e9, pos["y"], 1e-3)
	assert.InDelta(t, 0.0, pos["z"], 1e-12)
	assert.InDelta(t, 1.0, orient["w"], 1e-12)
}

// CalibrateImuYaw.srv request schema as advertised by foxglove_bridge:
// a single primitive float64 field with no nested structs.
const calibrateImuYawReqSchema = `float64 duration_sec
`

// TestSerializeCDR_SingleFloat64PadsToMultipleOfEight is the regression test
// for the CalibrateImuYaw rejection path. Without the end-of-buffer pad in
// SerializeCDR, the wire payload is 12 bytes (4 header + 8 data), which
// foxglove_bridge's rmw layer rejects with `rmw_serialize: invalid data
// size`. Nested-struct messages like SetDockingPoint (60 bytes) happened to
// be over the threshold and were waved through, so the bug only surfaced
// for services with very small request bodies — like the IMU-yaw
// calibration service that takes a single float64 duration.
func TestSerializeCDR_SingleFloat64PadsToMultipleOfEight(t *testing.T) {
	schema, err := ParseSchema(calibrateImuYawReqSchema)
	require.NoError(t, err)

	out, err := SerializeCDR([]byte(`{"duration_sec": 30.0}`), schema)
	require.NoError(t, err)

	// 4 header + 8 data + 4 trailing pad = 16 bytes (multiple of 8).
	require.Len(t, out, 16)
	assert.Equal(t, []byte{0x00, 0x01, 0x00, 0x00}, out[:4], "encap header")

	val := math.Float64frombits(binary.LittleEndian.Uint64(out[4:12]))
	assert.Equal(t, 30.0, val, "duration_sec at byte 4")

	// Trailing four bytes must be zero pad, not random memory.
	assert.Equal(t, []byte{0, 0, 0, 0}, out[12:16], "trailing pad zeros")
}
