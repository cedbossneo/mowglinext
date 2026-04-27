package foxglove

import (
	"encoding/binary"
	"encoding/json"
	"math"
	"testing"
)

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

func mustParseSchema(t *testing.T, text string) *msgSchema {
	t.Helper()
	s, err := ParseSchema(text)
	if err != nil {
		t.Fatalf("ParseSchema: %v", err)
	}
	return s
}

func mustSerialize(t *testing.T, v interface{}, schema *msgSchema) []byte {
	t.Helper()
	b, err := json.Marshal(v)
	if err != nil {
		t.Fatalf("json.Marshal: %v", err)
	}
	cdr, err := SerializeCDR(b, schema)
	if err != nil {
		t.Fatalf("SerializeCDR: %v", err)
	}
	return cdr
}

func mustDeserialize(t *testing.T, data []byte, schema *msgSchema) map[string]interface{} {
	t.Helper()
	m, err := DeserializeCDR(data, schema)
	if err != nil {
		t.Fatalf("DeserializeCDR: %v", err)
	}
	return m
}

// assertFloat64 checks that m[key] is a float64 bit-equal to want.
func assertFloat64(t *testing.T, m map[string]interface{}, key string, want float64) {
	t.Helper()
	v, ok := m[key]
	if !ok {
		t.Errorf("%s: key missing", key)
		return
	}
	got, ok := v.(float64)
	if !ok {
		t.Errorf("%s: want float64, got %T", key, v)
		return
	}
	if math.Float64bits(got) != math.Float64bits(want) {
		t.Errorf("%s: got %v (%016x), want %v (%016x)",
			key, got, math.Float64bits(got), want, math.Float64bits(want))
	}
}

func assertBool(t *testing.T, m map[string]interface{}, key string, want bool) {
	t.Helper()
	v, ok := m[key]
	if !ok {
		t.Errorf("%s: key missing", key)
		return
	}
	got, ok := v.(bool)
	if !ok {
		t.Errorf("%s: want bool, got %T", key, v)
		return
	}
	if got != want {
		t.Errorf("%s: got %v, want %v", key, got, want)
	}
}

func assertInt32(t *testing.T, m map[string]interface{}, key string, want int32) {
	t.Helper()
	v, ok := m[key]
	if !ok {
		t.Errorf("%s: key missing", key)
		return
	}
	got, ok := v.(int32)
	if !ok {
		t.Errorf("%s: want int32, got %T (%v)", key, v, v)
		return
	}
	if got != want {
		t.Errorf("%s: got %v, want %v", key, got, want)
	}
}

func assertString(t *testing.T, m map[string]interface{}, key string, want string) {
	t.Helper()
	v, ok := m[key]
	if !ok {
		t.Errorf("%s: key missing", key)
		return
	}
	got, ok := v.(string)
	if !ok {
		t.Errorf("%s: want string, got %T", key, v)
		return
	}
	if got != want {
		t.Errorf("%s: got %q, want %q", key, got, want)
	}
}

func leFloat64(f float64) []byte {
	b := make([]byte, 8)
	binary.LittleEndian.PutUint64(b, math.Float64bits(f))
	return b
}

func beFloat64(f float64) []byte {
	b := make([]byte, 8)
	binary.BigEndian.PutUint64(b, math.Float64bits(f))
	return b
}

// ---------------------------------------------------------------------------
// Schema definitions used across tests
// ---------------------------------------------------------------------------

const schemaSingleFloat64 = `float64 value`

const schemaCalibrateImuYawRes = `bool success
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
bool dock_valid
float64 dock_pose_x
float64 dock_pose_y
float64 dock_pose_yaw_rad
float64 dock_pose_yaw_deg
float64 dock_yaw_sigma_deg
float64 dock_undock_displacement_m`

// schemaInterleaved: int32 + float64 interleaving verifies alignment across field
// boundaries. Under maxAlign=8, float64 must align to 8; under maxAlign=4 it aligns
// to 4. After an int32 (4 bytes), offset is a multiple of 4 and also 8 if the start
// was 8-aligned — round-trip tests here verify no silent corruption.
const schemaInterleaved = `int32 a
float64 b
int32 c
float64 d`

const schemaStringThenFloat64 = `string label
float64 value`

// schemaInt32BoolFloat64 is the canonical schema that produces DIFFERENT byte
// layouts under maxAlign=4 vs maxAlign=8: after int32 (4 bytes) + bool (1 byte)
// the offset is 9, which needs 7 padding bytes to reach the next multiple of 8
// (XCDR1) but only 3 to reach the next multiple of 4 (CDR2).
const schemaInt32BoolFloat64 = `int32 a
bool b
float64 c`

// ---------------------------------------------------------------------------
// 1. Round-trip tests (writer→reader, XCDR1 0x0001 header produced by SerializeCDR)
// ---------------------------------------------------------------------------

// TestRoundTripSingleFloat64 serializes and deserializes a struct with one float64.
func TestRoundTripSingleFloat64(t *testing.T) {
	schema := mustParseSchema(t, schemaSingleFloat64)
	in := map[string]interface{}{"value": 3.14159265358979}
	cdr := mustSerialize(t, in, schema)
	out := mustDeserialize(t, cdr, schema)
	assertFloat64(t, out, "value", 3.14159265358979)
}

// TestRoundTripCalibrateImuYawRes round-trips the full CalibrateImuYaw response shape
// with 13 float64 fields, 2 bool fields, 2 int32 fields, and 1 string field.
func TestRoundTripCalibrateImuYawRes(t *testing.T) {
	schema := mustParseSchema(t, schemaCalibrateImuYawRes)

	in := map[string]interface{}{
		"success":                    true,
		"message":                    "calibration ok",
		"imu_yaw_rad":                1.2345678901234567,
		"imu_yaw_deg":                70.7355,
		"samples_used":               float64(42),
		"std_dev_deg":                0.123,
		"imu_pitch_rad":              0.01,
		"imu_pitch_deg":              0.573,
		"imu_roll_rad":               -0.005,
		"imu_roll_deg":               -0.286,
		"stationary_samples_used":    float64(200),
		"gravity_mag_mps2":           9.81,
		"dock_valid":                 true,
		"dock_pose_x":                1.5,
		"dock_pose_y":                -0.3,
		"dock_pose_yaw_rad":          2.2,
		"dock_pose_yaw_deg":          126.05,
		"dock_yaw_sigma_deg":         0.4,
		"dock_undock_displacement_m": 2.001,
	}

	cdr := mustSerialize(t, in, schema)
	out := mustDeserialize(t, cdr, schema)

	assertBool(t, out, "success", true)
	assertString(t, out, "message", "calibration ok")
	assertFloat64(t, out, "imu_yaw_rad", 1.2345678901234567)
	assertFloat64(t, out, "imu_yaw_deg", 70.7355)
	assertInt32(t, out, "samples_used", 42)
	assertFloat64(t, out, "std_dev_deg", 0.123)
	assertFloat64(t, out, "imu_pitch_rad", 0.01)
	assertFloat64(t, out, "imu_pitch_deg", 0.573)
	assertFloat64(t, out, "imu_roll_rad", -0.005)
	assertFloat64(t, out, "imu_roll_deg", -0.286)
	assertInt32(t, out, "stationary_samples_used", 200)
	assertFloat64(t, out, "gravity_mag_mps2", 9.81)
	assertBool(t, out, "dock_valid", true)
	assertFloat64(t, out, "dock_pose_x", 1.5)
	assertFloat64(t, out, "dock_pose_y", -0.3)
	assertFloat64(t, out, "dock_pose_yaw_rad", 2.2)
	assertFloat64(t, out, "dock_pose_yaw_deg", 126.05)
	assertFloat64(t, out, "dock_yaw_sigma_deg", 0.4)
	assertFloat64(t, out, "dock_undock_displacement_m", 2.001)
}

// TestRoundTripInterleaved round-trips int32+float64 interleaving.
func TestRoundTripInterleaved(t *testing.T) {
	schema := mustParseSchema(t, schemaInterleaved)
	in := map[string]interface{}{
		"a": float64(1),
		"b": math.Pi,
		"c": float64(-7),
		"d": math.E,
	}
	cdr := mustSerialize(t, in, schema)
	out := mustDeserialize(t, cdr, schema)
	assertInt32(t, out, "a", 1)
	assertFloat64(t, out, "b", math.Pi)
	assertInt32(t, out, "c", -7)
	assertFloat64(t, out, "d", math.E)
}

// TestRoundTripStringThenFloat64 round-trips string+float64; string length
// shifts the offset in a way that exercises 8-byte padding.
func TestRoundTripStringThenFloat64(t *testing.T) {
	schema := mustParseSchema(t, schemaStringThenFloat64)
	in := map[string]interface{}{
		"label": "hi",
		"value": 2.718281828459045,
	}
	cdr := mustSerialize(t, in, schema)
	out := mustDeserialize(t, cdr, schema)
	assertString(t, out, "label", "hi")
	assertFloat64(t, out, "value", 2.718281828459045)
}

// ---------------------------------------------------------------------------
// 2. Header-driven tests — hand-crafted payloads verify alignment selection
// ---------------------------------------------------------------------------

// The canonical differentiator: schema "float64 value" (single field).
//
// Layout after the 4-byte header:
//   XCDR1 (maxAlign=8): offset=4 → align(8,8) → 4%8=4 → pad 4 → float64 at offset 8
//   CDR2  (maxAlign=4): offset=4 → align(8,4) → 4%4=0 → no pad → float64 at offset 4
//
// The writer always writes XCDR1 (0x0001) so serialized payloads put float64 at 8.
// Hand-crafting lets us also test the CDR2 path that the reader must handle.

// buildXCDR1SingleFloat64 builds a minimal XCDR1_LE payload for "float64 value".
// float64 is at offset 8 (4 pad bytes after the 4-byte header).
func buildXCDR1SingleFloat64(f float64) []byte {
	// header 0x0001 = XCDR1_LE, then 4 padding bytes, then the float64 (LE)
	buf := []byte{0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
	return append(buf, leFloat64(f)...)
}

// buildCDR2SingleFloat64 builds a minimal PLAIN_CDR2_LE payload for "float64 value".
// float64 is at offset 4 (no padding needed; 4%4==0).
func buildCDR2SingleFloat64(f float64) []byte {
	buf := []byte{0x00, 0x07, 0x00, 0x00}
	return append(buf, leFloat64(f)...)
}

// TestHeaderXCDR1SingleFloat64 verifies that header 0x0001 (XCDR1_LE) triggers
// maxAlign=8 so the float64 is read from offset 8.
func TestHeaderXCDR1SingleFloat64(t *testing.T) {
	schema := mustParseSchema(t, schemaSingleFloat64)
	const wantF = 1.23456789e10
	payload := buildXCDR1SingleFloat64(wantF)

	if len(payload) != 8+8 {
		t.Fatalf("unexpected payload length %d, want 16", len(payload))
	}

	out := mustDeserialize(t, payload, schema)
	assertFloat64(t, out, "value", wantF)
}

// TestHeaderCDR2SingleFloat64 verifies that header 0x0007 (PLAIN_CDR2_LE) triggers
// maxAlign=4 so the float64 is read from offset 4 (no padding).
func TestHeaderCDR2SingleFloat64(t *testing.T) {
	schema := mustParseSchema(t, schemaSingleFloat64)
	const wantF = 9.87654321e-5
	payload := buildCDR2SingleFloat64(wantF)

	if len(payload) != 4+8 {
		t.Fatalf("unexpected payload length %d, want 12", len(payload))
	}

	out := mustDeserialize(t, payload, schema)
	assertFloat64(t, out, "value", wantF)
}

// TestHeaderXCDR1WrongAlignDetected is the regression guard: if maxAlign were
// still hardcoded to 4, reading the XCDR1 single-float64 payload (float64 at
// offset 8) would instead read from offset 4 and return garbage. We confirm
// the old broken reader misreads the value so the test logic is sound.
func TestHeaderXCDR1WrongAlignDetected(t *testing.T) {
	schema := mustParseSchema(t, schemaSingleFloat64)
	const wantF = 1.23456789e10
	payload := buildXCDR1SingleFloat64(wantF)

	// Simulate old broken reader: maxAlign=4 forces float64 to offset 4.
	r := &cdrReader{data: payload, offset: 4, le: true, maxAlign: 4}
	out, err := r.readMessage(schema.Fields)
	if err != nil {
		t.Fatalf("readMessage: %v", err)
	}
	got := out["value"].(float64)
	if math.Float64bits(got) == math.Float64bits(wantF) {
		t.Fatal("expected misread with maxAlign=4 on XCDR1 payload — got correct value, test logic error")
	}
}

// TestHeaderXCDR1Int32BoolFloat64 tests a schema where XCDR1 and CDR2 produce
// genuinely different padding:
//
//	int32 (4 bytes) + bool (1 byte) → offset 9
//	XCDR1 (maxAlign=8): 9%8=1 → pad 7 → float64 at offset 16
//	CDR2  (maxAlign=4): 9%4=1 → pad 3 → float64 at offset 12
func TestHeaderXCDR1Int32BoolFloat64(t *testing.T) {
	schema := mustParseSchema(t, schemaInt32BoolFloat64)
	const wantA int32 = 42
	const wantB = true
	const wantC = math.Pi

	// Build XCDR1_LE payload manually.
	buf := []byte{0x00, 0x01, 0x00, 0x00} // header
	// int32 at offset 4 (no padding needed — 4%4=0)
	var i32b [4]byte
	binary.LittleEndian.PutUint32(i32b[:], uint32(wantA))
	buf = append(buf, i32b[:]...)
	// bool at offset 8
	buf = append(buf, 0x01)
	// offset now 9; align(8,8): 9%8=1 → pad 7 bytes
	buf = append(buf, 0, 0, 0, 0, 0, 0, 0)
	// float64 at offset 16
	buf = append(buf, leFloat64(wantC)...)

	if len(buf) != 16+8 {
		t.Fatalf("unexpected XCDR1 payload length %d, want 24", len(buf))
	}

	out := mustDeserialize(t, buf, schema)
	assertInt32(t, out, "a", wantA)
	assertBool(t, out, "b", wantB)
	assertFloat64(t, out, "c", wantC)
}

// TestHeaderCDR2Int32BoolFloat64 is the CDR2 counterpart of the above test.
// float64 is at offset 12 (only 3 padding bytes needed for 4-byte alignment).
func TestHeaderCDR2Int32BoolFloat64(t *testing.T) {
	schema := mustParseSchema(t, schemaInt32BoolFloat64)
	const wantA int32 = 99
	const wantB = false
	const wantC = math.E

	// Build CDR2_LE payload manually.
	buf := []byte{0x00, 0x07, 0x00, 0x00} // header PLAIN_CDR2_LE
	// int32 at offset 4
	var i32b [4]byte
	binary.LittleEndian.PutUint32(i32b[:], uint32(wantA))
	buf = append(buf, i32b[:]...)
	// bool at offset 8
	buf = append(buf, 0x00)
	// offset now 9; align(8,4): 9%4=1 → pad 3 bytes
	buf = append(buf, 0, 0, 0)
	// float64 at offset 12
	buf = append(buf, leFloat64(wantC)...)

	if len(buf) != 12+8 {
		t.Fatalf("unexpected CDR2 payload length %d, want 20", len(buf))
	}

	out := mustDeserialize(t, buf, schema)
	assertInt32(t, out, "a", wantA)
	assertBool(t, out, "b", wantB)
	assertFloat64(t, out, "c", wantC)
}

// ---------------------------------------------------------------------------
// 3. Endianness tests
// ---------------------------------------------------------------------------

// TestEndiannessLECDR2 verifies that header 0x0007 (PLAIN_CDR2_LE) is decoded
// as little-endian. A big-endian misread would byte-swap the float and produce
// a different value.
func TestEndiannessLECDR2(t *testing.T) {
	schema := mustParseSchema(t, schemaSingleFloat64)
	const wantF = 12345.6789

	// CDR2_LE: float64 starts at offset 4 (no padding required — 4%4=0).
	payload := append([]byte{0x00, 0x07, 0x00, 0x00}, leFloat64(wantF)...)
	out := mustDeserialize(t, payload, schema)
	assertFloat64(t, out, "value", wantF)

	// Sanity-check: a big-endian interpretation of the same bytes gives a different value.
	beBits := binary.BigEndian.Uint64(leFloat64(wantF))
	if math.Float64bits(math.Float64frombits(beBits)) == math.Float64bits(wantF) {
		t.Fatal("BE and LE byte patterns match — test is not exercising endianness")
	}
}

// TestEndiannessXCDR1LE verifies that header 0x0001 (XCDR1_LE) is decoded as
// little-endian. float64 sits at offset 8 (4 padding bytes after header).
func TestEndiannessXCDR1LE(t *testing.T) {
	schema := mustParseSchema(t, schemaSingleFloat64)
	const wantF = 98765.4321

	// XCDR1_LE: 4-byte header + 4 pad + 8-byte float64 (LE).
	payload := append([]byte{0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, leFloat64(wantF)...)
	out := mustDeserialize(t, payload, schema)
	assertFloat64(t, out, "value", wantF)
}

// TestEndiannessXCDR1BE verifies that header 0x0000 (CDR_BE / XCDR1 big-endian)
// is decoded as big-endian. float64 sits at offset 8 (4 padding bytes after header).
func TestEndiannessXCDR1BE(t *testing.T) {
	schema := mustParseSchema(t, schemaSingleFloat64)
	const wantF = 11111.2222

	// CDR_BE: 4-byte header + 4 pad + 8-byte float64 (BE).
	payload := append([]byte{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, beFloat64(wantF)...)
	out := mustDeserialize(t, payload, schema)
	assertFloat64(t, out, "value", wantF)
}

// ---------------------------------------------------------------------------
// 4. ROS2 message type round-trip tests
//    These guard against the CDR reader bug where maxAlign was hardcoded to 4
//    (XCDR2) instead of the correct 8 (XCDR1), causing float64 misalignment
//    that produced the "map stuck at 0,0" and "orientation -180/90/-14" symptoms.
// ---------------------------------------------------------------------------

// schemaNavOdometry is the foxglove_bridge schema text for nav_msgs/Odometry.
// The nested-type separator ("===...===") + "MSG: " header format matches what
// foxglove_bridge emits on the wire.
const schemaNavOdometry = `std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
================================================================================
MSG: std_msgs/Header
uint32 sec
uint32 nanosec
string frame_id
================================================================================
MSG: geometry_msgs/PoseWithCovariance
geometry_msgs/Pose pose
float64[36] covariance
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
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
================================================================================
MSG: geometry_msgs/TwistWithCovariance
geometry_msgs/Twist twist
float64[36] covariance
================================================================================
MSG: geometry_msgs/Twist
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z`

// schemaNavSatFix is the foxglove_bridge schema text for sensor_msgs/NavSatFix.
const schemaNavSatFix = `std_msgs/Header header
sensor_msgs/NavSatStatus status
float64 latitude
float64 longitude
float64 altitude
float64[9] position_covariance
uint8 position_covariance_type
================================================================================
MSG: std_msgs/Header
uint32 sec
uint32 nanosec
string frame_id
================================================================================
MSG: sensor_msgs/NavSatStatus
int8 status
uint16 service`

// schemaSensorImu is the foxglove_bridge schema text for sensor_msgs/Imu.
const schemaSensorImu = `std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
================================================================================
MSG: std_msgs/Header
uint32 sec
uint32 nanosec
string frame_id
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z`

// TestRoundTripNavOdometry round-trips a fully populated nav_msgs/Odometry through
// SerializeCDR → DeserializeCDR. Guards the "map marker stuck at 0,0" bug: if float64
// alignment is wrong (maxAlign=4 on an XCDR1 payload) position.x, position.y and the
// quaternion components all read as garbage.
func TestRoundTripNavOdometry(t *testing.T) {
	schema := mustParseSchema(t, schemaNavOdometry)

	// yaw = π/3 unit quaternion: w=cos(π/6)≈0.866025, z=sin(π/6)=0.5
	const yawW = 0.8660254037844387
	const yawZ = 0.5

	cov36 := make([]interface{}, 36)
	for i := range cov36 {
		cov36[i] = float64(i) * 0.001
	}

	in := map[string]interface{}{
		"header": map[string]interface{}{
			"stamp":    map[string]interface{}{"sec": float64(1700000000), "nanosec": float64(123456789)},
			"frame_id": "map",
		},
		"child_frame_id": "base_footprint",
		"pose": map[string]interface{}{
			"pose": map[string]interface{}{
				"position":    map[string]interface{}{"x": 12.345, "y": -67.89, "z": 0.0},
				"orientation": map[string]interface{}{"x": 0.0, "y": 0.0, "z": yawZ, "w": yawW},
			},
			"covariance": cov36,
		},
		"twist": map[string]interface{}{
			"twist": map[string]interface{}{
				"linear":  map[string]interface{}{"x": 0.25, "y": 0.0, "z": 0.0},
				"angular": map[string]interface{}{"x": 0.0, "y": 0.0, "z": 0.15},
			},
			"covariance": cov36,
		},
	}

	cdr := mustSerialize(t, in, schema)
	out := mustDeserialize(t, cdr, schema)

	poseMsg := out["pose"].(map[string]interface{})
	innerPose := poseMsg["pose"].(map[string]interface{})
	pos := innerPose["position"].(map[string]interface{})
	ori := innerPose["orientation"].(map[string]interface{})

	assertFloat64(t, pos, "x", 12.345)
	assertFloat64(t, pos, "y", -67.89)
	assertFloat64(t, pos, "z", 0.0)
	assertFloat64(t, ori, "x", 0.0)
	assertFloat64(t, ori, "y", 0.0)
	assertFloat64(t, ori, "z", yawZ)
	assertFloat64(t, ori, "w", yawW)

	twistMsg := out["twist"].(map[string]interface{})
	innerTwist := twistMsg["twist"].(map[string]interface{})
	linear := innerTwist["linear"].(map[string]interface{})
	angular := innerTwist["angular"].(map[string]interface{})
	assertFloat64(t, linear, "x", 0.25)
	assertFloat64(t, angular, "z", 0.15)

	assertString(t, out["header"].(map[string]interface{}), "frame_id", "map")
	assertString(t, out, "child_frame_id", "base_footprint")

	// Verify the first few diagonal covariance elements survive round-trip.
	poseCov := poseMsg["covariance"].([]float64)
	if math.Float64bits(poseCov[0]) != math.Float64bits(0.0) {
		t.Errorf("pose.covariance[0]: got %v, want 0.0", poseCov[0])
	}
	if math.Float64bits(poseCov[1]) != math.Float64bits(0.001) {
		t.Errorf("pose.covariance[1]: got %v, want 0.001", poseCov[1])
	}
}

// TestRoundTripSensorImu round-trips sensor_msgs/Imu through SerializeCDR →
// DeserializeCDR. Guards the "orientation shows -180/90/-14" diagnostics bug:
// the quaternion components land after covariance arrays whose byte count shifts
// the reader off the correct 8-byte boundaries when maxAlign=4.
func TestRoundTripSensorImu(t *testing.T) {
	schema := mustParseSchema(t, schemaSensorImu)

	// yaw = -π/4 unit quaternion: w=cos(-π/8)≈0.92388, z=sin(-π/8)≈-0.38268
	const oriW = 0.9238795325112867
	const oriZ = -0.3826834323650898

	cov9 := func(diag float64) []interface{} {
		arr := make([]interface{}, 9)
		for i := range arr {
			arr[i] = float64(0)
		}
		arr[0] = diag
		arr[4] = diag
		arr[8] = diag
		return arr
	}

	in := map[string]interface{}{
		"header": map[string]interface{}{
			"stamp":    map[string]interface{}{"sec": float64(1700000001), "nanosec": float64(0)},
			"frame_id": "imu_link",
		},
		"orientation":                    map[string]interface{}{"x": 0.0, "y": 0.0, "z": oriZ, "w": oriW},
		"orientation_covariance":         cov9(0.001),
		"angular_velocity":               map[string]interface{}{"x": 0.01, "y": 0.02, "z": 0.5},
		"angular_velocity_covariance":    cov9(0.0001),
		"linear_acceleration":            map[string]interface{}{"x": 0.1, "y": -0.05, "z": 9.81},
		"linear_acceleration_covariance": cov9(0.01),
	}

	cdr := mustSerialize(t, in, schema)
	out := mustDeserialize(t, cdr, schema)

	ori := out["orientation"].(map[string]interface{})
	assertFloat64(t, ori, "x", 0.0)
	assertFloat64(t, ori, "y", 0.0)
	assertFloat64(t, ori, "z", oriZ)
	assertFloat64(t, ori, "w", oriW)

	// Quaternion must be a unit quaternion: magnitude ≈ 1.
	qx := ori["x"].(float64)
	qy := ori["y"].(float64)
	qz := ori["z"].(float64)
	qw := ori["w"].(float64)
	mag := math.Sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
	if math.Abs(mag-1.0) > 1e-9 {
		t.Errorf("quaternion magnitude: got %v, want 1.0 (misalignment would produce values outside [-1,1])", mag)
	}

	av := out["angular_velocity"].(map[string]interface{})
	assertFloat64(t, av, "x", 0.01)
	assertFloat64(t, av, "y", 0.02)
	assertFloat64(t, av, "z", 0.5)

	la := out["linear_acceleration"].(map[string]interface{})
	assertFloat64(t, la, "x", 0.1)
	assertFloat64(t, la, "y", -0.05)
	assertFloat64(t, la, "z", 9.81)
}

// TestRoundTripNavSatFix round-trips sensor_msgs/NavSatFix through SerializeCDR →
// DeserializeCDR with realistic RTK-Fixed coordinates.
func TestRoundTripNavSatFix(t *testing.T) {
	schema := mustParseSchema(t, schemaNavSatFix)

	cov9 := make([]interface{}, 9)
	for i := range cov9 {
		cov9[i] = float64(0)
	}
	cov9[0] = 9e-6 // ~3mm RTK-Fixed σ²
	cov9[4] = 9e-6
	cov9[8] = 0.04

	in := map[string]interface{}{
		"header": map[string]interface{}{
			"stamp":    map[string]interface{}{"sec": float64(1700000002), "nanosec": float64(500000000)},
			"frame_id": "gps",
		},
		"status": map[string]interface{}{
			"status":  float64(2), // STATUS_GBAS_FIX (RTK-Fixed)
			"service": float64(1),
		},
		"latitude":                 48.8566,
		"longitude":                2.3522,
		"altitude":                 35.0,
		"position_covariance":      cov9,
		"position_covariance_type": float64(3), // COVARIANCE_TYPE_KNOWN
	}

	cdr := mustSerialize(t, in, schema)
	out := mustDeserialize(t, cdr, schema)

	assertFloat64(t, out, "latitude", 48.8566)
	assertFloat64(t, out, "longitude", 2.3522)
	assertFloat64(t, out, "altitude", 35.0)

	statusMsg := out["status"].(map[string]interface{})
	// status is int8 → int in the reader
	if v, ok := statusMsg["status"]; !ok || int(v.(int)) != 2 {
		t.Errorf("status.status: got %v, want 2", statusMsg["status"])
	}

	assertString(t, out["header"].(map[string]interface{}), "frame_id", "gps")

	// position_covariance[0] must round-trip exactly.
	pcov := out["position_covariance"].([]float64)
	if math.Float64bits(pcov[0]) != math.Float64bits(9e-6) {
		t.Errorf("position_covariance[0]: got %v, want 9e-6", pcov[0])
	}
}

// ---------------------------------------------------------------------------
// 5. XCDR1 wire-frame alignment tests — hand-crafted payloads that prove the
//    reader honours the encapsulation header's alignment contract.
//    These use a minimal schema to isolate the alignment arithmetic.
// ---------------------------------------------------------------------------

// schemaFrameIdAndFloat is a minimal schema matching the Header-then-float
// pattern: uint32 + uint32 + string (simulating a Header) followed by a float64
// sentinel. This matches the alignment shape of the first float64 field that
// foxglove_bridge puts on the wire for Odometry/IMU messages.
//
// Byte layout (XCDR1_LE, starting at CDR offset 4 after header):
//
//	offset 4:  sec (uint32, 4 bytes) → offset 8
//	offset 8:  nanosec (uint32, 4 bytes) → offset 12
//	offset 12: frame_id string-length (uint32=8) → offset 16; "abcdefg\0" → offset 24
//	offset 24: float64 sentinel
//	           XCDR1 align(8): 24%8=0 → no pad → offset 24  ✓
//	           broken align(4): 24%4=0 → same offset 24 (coincidence — need different string)
//
// With frame_id = "abc" (length=4, 3 chars + null):
//
//	string-length uint32 at 12 → offset 16; "abc\0" (4 bytes) → offset 20
//	float64 sentinel:
//	   XCDR1 align(8): 20%8=4 → pad 4 → float64 at offset 24
//	   broken align(4): 20%4=0 → no pad → float64 at offset 20  ← DIFFERENT
const schemaFrameIdAndFloat = `uint32 sec
uint32 nanosec
string frame_id
float64 sentinel`

// buildXCDR1FramePayload builds an XCDR1_LE payload for schemaFrameIdAndFloat.
// frame_id="abc" (3 chars) is chosen so the float64 sits at offset 24 under
// XCDR1 (maxAlign=8) and would be read from offset 20 under a broken maxAlign=4.
func buildXCDR1FramePayload(sentinel float64) []byte {
	buf := []byte{0x00, 0x01, 0x00, 0x00} // XCDR1_LE header
	// sec uint32 at offset 4 (aligned)
	sec := make([]byte, 4)
	binary.LittleEndian.PutUint32(sec, 1700000000)
	buf = append(buf, sec...)
	// nanosec uint32 at offset 8
	ns := make([]byte, 4)
	binary.LittleEndian.PutUint32(ns, 0)
	buf = append(buf, ns...)
	// frame_id string: length=4 ("abc\0") at offset 12
	strlen := make([]byte, 4)
	binary.LittleEndian.PutUint32(strlen, 4)
	buf = append(buf, strlen...)
	buf = append(buf, 'a', 'b', 'c', 0x00) // offset now 20
	// XCDR1: align(8) at offset 20 → 20%8=4 → pad 4 → float64 at offset 24
	buf = append(buf, 0, 0, 0, 0)
	buf = append(buf, leFloat64(sentinel)...)
	return buf
}

// buildCDR2FramePayload builds a CDR2_LE payload for schemaFrameIdAndFloat.
// Under CDR2 (maxAlign=4), float64 aligns to 4: offset 20 % 4 = 0, so the
// float64 sits directly at offset 20 — different from the XCDR1 layout above.
func buildCDR2FramePayload(sentinel float64) []byte {
	buf := []byte{0x00, 0x07, 0x00, 0x00} // CDR2_LE header
	sec := make([]byte, 4)
	binary.LittleEndian.PutUint32(sec, 1700000000)
	buf = append(buf, sec...)
	ns := make([]byte, 4)
	binary.LittleEndian.PutUint32(ns, 0)
	buf = append(buf, ns...)
	strlen := make([]byte, 4)
	binary.LittleEndian.PutUint32(strlen, 4)
	buf = append(buf, strlen...)
	buf = append(buf, 'a', 'b', 'c', 0x00) // offset now 20
	// CDR2: align(8,4) at offset 20 → 20%4=0 → no pad → float64 at offset 20
	buf = append(buf, leFloat64(sentinel)...)
	return buf
}

// TestXCDR1OdometryWireFrame proves the fixed reader extracts a sentinel float64
// from an XCDR1 wire payload where the float lives at offset 24 (8-byte aligned).
// Running this test against the old reader (maxAlign=4 hardcoded) would fail:
// it would read from offset 20 and return garbage bytes instead of the sentinel.
func TestXCDR1OdometryWireFrame(t *testing.T) {
	schema := mustParseSchema(t, schemaFrameIdAndFloat)
	const sentinel = math.Pi

	payload := buildXCDR1FramePayload(sentinel)
	// Verify float64 is at byte offset 24 in the payload (the 8-aligned slot).
	gotBits := binary.LittleEndian.Uint64(payload[24:])
	if gotBits != math.Float64bits(sentinel) {
		t.Fatalf("test setup: sentinel not at offset 24 in XCDR1 payload")
	}

	out := mustDeserialize(t, payload, schema)
	assertFloat64(t, out, "sentinel", sentinel)
}

// TestCDR2OdometryWireFrame proves the fixed reader correctly handles CDR2_LE
// (header 0x0007, maxAlign=4) where the float64 lands at offset 20 (4-byte aligned,
// no padding after the string). This is the complementary case to XCDR1.
func TestCDR2OdometryWireFrame(t *testing.T) {
	schema := mustParseSchema(t, schemaFrameIdAndFloat)
	const sentinel = math.E

	payload := buildCDR2FramePayload(sentinel)
	// Verify float64 is at byte offset 20 (no padding from CDR2 alignment).
	gotBits := binary.LittleEndian.Uint64(payload[20:])
	if gotBits != math.Float64bits(sentinel) {
		t.Fatalf("test setup: sentinel not at offset 20 in CDR2 payload")
	}

	out := mustDeserialize(t, payload, schema)
	assertFloat64(t, out, "sentinel", sentinel)
}

// schemaQuatAfterString is a minimal schema mirroring the shape of the IMU
// orientation quaternion that follows a Header (string field) on the wire.
// The string "imu_link" has 9 chars including null → length=9; after the 4-byte
// length uint32 and 9 bytes: offset = 4 (header) + 4(sec) + 4(ns) + 4(len) + 9 = 25.
//
// XCDR1 float64 align(8): 25%8=1 → pad 7 → first float64 at offset 32.
// Broken maxAlign=4:       25%4=1 → pad 3 → first float64 at offset 28. ← GARBLED
const schemaQuatAfterString = `uint32 sec
uint32 nanosec
string frame_id
float64 qx
float64 qy
float64 qz
float64 qw`

// TestImuOrientationGarbledOnWrongAlign is the regression guard for the diagnostics
// "orientation -180/90/-14" symptom. It builds an XCDR1_LE wire frame where
// the quaternion floats sit at 8-byte-aligned offsets. The fixed reader extracts
// a unit quaternion (magnitude ≈ 1). A reader with maxAlign=4 reads from the wrong
// offsets and produces large-magnitude garbage — we demonstrate both outcomes to
// prove the fix is load-bearing.
func TestImuOrientationGarbledOnWrongAlign(t *testing.T) {
	schema := mustParseSchema(t, schemaQuatAfterString)

	// yaw = -π/4 unit quaternion
	const qz = -0.3826834323650898
	const qw = 0.9238795325112867

	// Build XCDR1_LE payload by hand.
	buf := []byte{0x00, 0x01, 0x00, 0x00} // XCDR1_LE header; CDR body starts at offset 4
	// sec uint32 at offset 4
	s := make([]byte, 4)
	binary.LittleEndian.PutUint32(s, 1700000003)
	buf = append(buf, s...)
	// nanosec uint32 at offset 8
	ns := make([]byte, 4)
	binary.LittleEndian.PutUint32(ns, 0)
	buf = append(buf, ns...)
	// frame_id string: length = 9 ("imu_link\0") at offset 12 → offset 16; string 9 bytes → offset 25
	slen := make([]byte, 4)
	binary.LittleEndian.PutUint32(slen, 9)
	buf = append(buf, slen...)
	buf = append(buf, []byte("imu_link\x00")...)
	// offset is now 25
	// XCDR1 align(8): 25%8=1 → pad 7 → first float64 at offset 32
	buf = append(buf, 0, 0, 0, 0, 0, 0, 0)
	// qx=0 at offset 32
	buf = append(buf, leFloat64(0.0)...)
	// qy=0 at offset 40 (8-byte aligned, no pad needed)
	buf = append(buf, leFloat64(0.0)...)
	// qz at offset 48
	buf = append(buf, leFloat64(qz)...)
	// qw at offset 56
	buf = append(buf, leFloat64(qw)...)

	// Sanity check: verify the sentinel floats are where we expect them.
	if binary.LittleEndian.Uint64(buf[48:]) != math.Float64bits(qz) {
		t.Fatalf("test setup: qz not at offset 48")
	}
	if binary.LittleEndian.Uint64(buf[56:]) != math.Float64bits(qw) {
		t.Fatalf("test setup: qw not at offset 56")
	}

	// Fixed reader (XCDR1, maxAlign=8) — must decode a unit quaternion.
	out := mustDeserialize(t, buf, schema)
	gotQx := out["qx"].(float64)
	gotQy := out["qy"].(float64)
	gotQz := out["qz"].(float64)
	gotQw := out["qw"].(float64)
	mag := math.Sqrt(gotQx*gotQx + gotQy*gotQy + gotQz*gotQz + gotQw*gotQw)
	if math.Abs(mag-1.0) > 1e-9 {
		t.Errorf("fixed reader: quaternion magnitude %v, want 1.0 — reader is mis-aligned", mag)
	}
	if math.Float64bits(gotQz) != math.Float64bits(qz) {
		t.Errorf("fixed reader: qz got %v, want %v", gotQz, qz)
	}
	if math.Float64bits(gotQw) != math.Float64bits(qw) {
		t.Errorf("fixed reader: qw got %v, want %v", gotQw, qw)
	}

	// Broken reader (maxAlign=4 hardcoded) — demonstrates the garbled symptom.
	// With maxAlign=4, the reader aligns to offset 28 (25%4=1 → pad 3) instead
	// of offset 32, so it reads 8 bytes starting from a padding/string byte region.
	rBroken := &cdrReader{data: buf, offset: 4, le: true, maxAlign: 4}
	broken, err := rBroken.readMessage(schema.Fields)
	if err != nil {
		// A broken reader may also return an error (short read) — that's also a failure.
		t.Logf("broken reader returned error (also evidence of misalignment): %v", err)
		return
	}
	brokenQz := broken["qz"].(float64)
	brokenQw := broken["qw"].(float64)
	brokenMag := math.Sqrt(
		broken["qx"].(float64)*broken["qx"].(float64) +
			broken["qy"].(float64)*broken["qy"].(float64) +
			brokenQz*brokenQz +
			brokenQw*brokenQw,
	)
	// The broken reader must NOT produce a unit quaternion — it reads garbage bytes.
	if math.Abs(brokenMag-1.0) <= 1e-9 {
		t.Errorf("broken reader (maxAlign=4) still produced a unit quaternion (magnitude %v) — "+
			"test is not exercising the alignment bug; re-check wire layout", brokenMag)
	}
	t.Logf("broken reader produces quaternion magnitude %v (expected far from 1.0) — bug confirmed", brokenMag)
}
