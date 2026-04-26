package foxglove

import (
	"encoding/binary"
	"encoding/hex"
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

// CalibrateImuYawStatus.msg schema as advertised by foxglove_bridge for
// /calibrate_imu_yaw_node/calibrate_status — mixed primitives that exercise
// every alignment edge in the reader.
const calibrateImuYawStatusSchema = `string job_id
bool done
bool success
string message
float64 imu_yaw_rad
float64 imu_yaw_deg
int32 samples_used
float64 std_dev_deg
float64 imu_pitch_rad
float64 imu_pitch_deg
float64 imu_roll_rad
float64 imu_roll_deg
int32 stationary_samples_used
float64 gravity_mag_mps2
`

// TestSerializeCDR_CalibrateImuYawStatusRoundTrip exercises the CDR reader
// against the EXACT shape that triggered the GUI's "samples_used = -1071219282"
// garbage display. Values are taken from a live Pi5 capture of
// /calibrate_imu_yaw_node/calibrate_status (verified clean via ros2 topic echo)
// where the GUI then mis-displayed them.
//
// If this round-trip succeeds, the cdr.go reader handles the shape correctly
// and the bug is downstream (wire encoding from foxglove_bridge differs from
// what the writer produces). If it fails, the reader has a bug for this shape.
func TestSerializeCDR_CalibrateImuYawStatusRoundTrip(t *testing.T) {
	schema, err := ParseSchema(calibrateImuYawStatusSchema)
	require.NoError(t, err)

	jsonReq := []byte(`{
		"job_id": "47fe01d54c94a7d2a7ec9240",
		"done": true,
		"success": true,
		"message": "imu_yaw=-11.25 deg from 263 samples",
		"imu_yaw_rad": -0.1962704701472458,
		"imu_yaw_deg": -11.245469582485601,
		"samples_used": 263,
		"std_dev_deg": 49.59494910238911,
		"imu_pitch_rad": 0.016132494450135544,
		"imu_pitch_deg": 0.9243238450109904,
		"imu_roll_rad": -0.06137728957413168,
		"imu_roll_deg": -3.5166596505500554,
		"stationary_samples_used": 378,
		"gravity_mag_mps2": 11.305172532189346
	}`)

	wire, err := SerializeCDR(jsonReq, schema)
	require.NoError(t, err)
	t.Logf("wire payload size: %d bytes", len(wire))

	decoded, err := DeserializeCDR(wire, schema)
	require.NoError(t, err)

	assert.Equal(t, "47fe01d54c94a7d2a7ec9240", decoded["job_id"])
	assert.Equal(t, true, decoded["done"])
	assert.Equal(t, true, decoded["success"])
	assert.Equal(t, "imu_yaw=-11.25 deg from 263 samples", decoded["message"])
	assert.InDelta(t, -0.1962704701472458, decoded["imu_yaw_rad"], 1e-12)
	assert.InDelta(t, -11.245469582485601, decoded["imu_yaw_deg"], 1e-12)
	assert.Equal(t, int32(263), decoded["samples_used"])
	assert.InDelta(t, 49.59494910238911, decoded["std_dev_deg"], 1e-12)
	assert.InDelta(t, 0.016132494450135544, decoded["imu_pitch_rad"], 1e-12)
	assert.InDelta(t, 0.9243238450109904, decoded["imu_pitch_deg"], 1e-12)
	assert.InDelta(t, -0.06137728957413168, decoded["imu_roll_rad"], 1e-12)
	assert.InDelta(t, -3.5166596505500554, decoded["imu_roll_deg"], 1e-12)
	assert.Equal(t, int32(378), decoded["stationary_samples_used"])
	assert.InDelta(t, 11.305172532189346, decoded["gravity_mag_mps2"], 1e-12)
}

// TestDeserializeCDR_CalibrateImuYawStatusGoldenWire decodes a captured wire
// payload of /calibrate_imu_yaw_node/calibrate_status from a live Pi5
// (rmw_cyclonedds, ROS 2 Kilted) and asserts every field matches what
// `ros2 topic echo` reported on the publisher side.
//
// Before the cdr.go align fix, the reader (with maxAlign=4 and absolute-from-
// buffer alignment) read garbage starting at imu_yaw_rad — the GUI's surface
// symptom was "samples_used = -1071219282".
func TestDeserializeCDR_CalibrateImuYawStatusGoldenWire(t *testing.T) {
	schema, err := ParseSchema(calibrateImuYawStatusSchema)
	require.NoError(t, err)

	wire, err := hex.DecodeString("0001000019000000343766653031643534633934613764326137656339323430000101008f000000696d755f7961773d2d31312e3235c2b02066726f6d203236332073616d706c65732028766563746f7220523d302e36392c207065722d73616d706c6520523d302e3539292e2070697463683d302e3932c2b020726f6c6c3d2d332e3532c2b02066726f6d203337382073746174696f6e6172792073616d706c657320287c677c3d31312e3331206d2f73c2b2292e000000000000c2fa3909641fc9bf7ae06930ae7d26c00701000000000000dcc5cc4a27cc4840c2d243600985903f9186a7990f94ed3fcf43e016d86cafbfc43372741e220cc07a0100000000000010c8fa923f9c2640")
	require.NoError(t, err)

	decoded, err := DeserializeCDR(wire, schema)
	require.NoError(t, err)

	assert.Equal(t, "47fe01d54c94a7d2a7ec9240", decoded["job_id"])
	assert.Equal(t, true, decoded["done"])
	assert.Equal(t, true, decoded["success"])
	assert.InDelta(t, -0.1962704701472458, decoded["imu_yaw_rad"], 1e-12)
	assert.InDelta(t, -11.245469582485601, decoded["imu_yaw_deg"], 1e-12)
	assert.Equal(t, int32(263), decoded["samples_used"])
	assert.InDelta(t, 49.59494910238911, decoded["std_dev_deg"], 1e-12)
	assert.InDelta(t, 0.016132494450135544, decoded["imu_pitch_rad"], 1e-12)
	assert.InDelta(t, 0.9243238450109904, decoded["imu_pitch_deg"], 1e-12)
	assert.InDelta(t, -0.06137728957413168, decoded["imu_roll_rad"], 1e-12)
	assert.InDelta(t, -3.5166596505500554, decoded["imu_roll_deg"], 1e-12)
	assert.Equal(t, int32(378), decoded["stationary_samples_used"])
	assert.InDelta(t, 11.305172532189346, decoded["gravity_mag_mps2"], 1e-12)
}
