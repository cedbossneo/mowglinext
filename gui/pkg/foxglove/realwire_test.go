package foxglove

import (
	"encoding/hex"
	"math"
	"testing"
)

// realImuSchema is the exact schema text foxglove_bridge sends for
// sensor_msgs/msg/Imu (captured live from the robot). It includes comments,
// blank lines, default-value tokens (e.g. "float64 x 0"), and "MSG:" headers.
const realImuSchema = `# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an
# orientation estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each
# covariance matrix, and disregard the associated estimate.

std_msgs/Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data
# in a particular coordinate frame.

# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp

# Transform frame with which this data is associated.
string frame_id

================================================================================
MSG: builtin_interfaces/Time
# This message communicates ROS Time defined here:
# https://design.ros2.org/articles/clock_and_time.html

# The seconds component, valid over all int32 values.
int32 sec

# The nanoseconds component, valid in the range [0, 1e9), to be added to the seconds component.
# e.g.
# The time -1.7 seconds is represented as {sec: -2, nanosec: 3e8}
# The time 1.7 seconds is represented as {sec: 1, nanosec: 7e8}
uint32 nanosec
`

// realImuPayloadHex is one /imu/data message captured live (hardware bridge,
// XCDR1 little-endian, frame_id="imu_link", quaternion=(0,0,0,1)).
const realImuPayloadHex = "00010000092cf16903635b0509000000696d755f6c696e6b00000000000000000000000000000000000000000000000000000000000000000000f03ffca9f1d24d62503f000000000000000000000000000000000000000000000000fca9f1d24d62503f0000000000000000000000000000000000000000000000000000000000c05840e0d0227be3a17a3f20ae4761ebab63bfc420b01e19f783bffca9f1d24d62503f000000000000000000000000000000000000000000000000fca9f1d24d62503f0000000000000000000000000000000000000000000000007b14ae47e17a843f06817510fa3dc03fe0fba99168d98cbf00000000ae7d2440005a9b41705f6f3f000000000000000000000000000000000000000000000000c56c78edff0c733f0000000000000000000000000000000000000000000000007b14ae47e17a843f"

func TestRealImuWireFrameDeserializes(t *testing.T) {
	schema, err := ParseSchema(realImuSchema)
	if err != nil {
		t.Fatalf("ParseSchema: %v", err)
	}

	data, err := hex.DecodeString(realImuPayloadHex)
	if err != nil {
		t.Fatalf("hex decode: %v", err)
	}

	out, err := DeserializeCDR(data, schema)
	if err != nil {
		t.Fatalf("DeserializeCDR: %v", err)
	}

	// Header → frame_id == "imu_link"
	hdr := out["header"].(map[string]interface{})
	if got := hdr["frame_id"].(string); got != "imu_link" {
		t.Errorf("frame_id: got %q want %q", got, "imu_link")
	}

	// orientation == (0, 0, 0, 1) for an IMU that only reports gyro+accel
	ori := out["orientation"].(map[string]interface{})
	for _, k := range []string{"x", "y", "z"} {
		v := ori[k].(float64)
		if v != 0 {
			t.Errorf("orientation.%s: got %g want 0", k, v)
		}
	}
	if w := ori["w"].(float64); math.Abs(w-1.0) > 1e-12 {
		t.Errorf("orientation.w: got %g want 1.0", w)
	}

	// orientation_covariance must be a length-9 array.
	if cov, ok := out["orientation_covariance"].([]float64); ok {
		if len(cov) != 9 {
			t.Errorf("orientation_covariance: len=%d want 9", len(cov))
		}
	} else {
		t.Errorf("orientation_covariance is not []float64: %T", out["orientation_covariance"])
	}

	// angular_velocity must NOT contain denormal garbage. Values from a stationary
	// IMU on a robot's chassis should be small (|v| < 10 rad/s), but the bug we're
	// hunting produces values like 5e-315 (denormals from misaligned reads).
	angV := out["angular_velocity"].(map[string]interface{})
	for _, k := range []string{"x", "y", "z"} {
		v := angV[k].(float64)
		if v != 0 && math.Abs(v) < 1e-100 {
			t.Errorf("angular_velocity.%s = %g is a denormal — misaligned read", k, v)
		}
		if math.Abs(v) > 1e10 {
			t.Errorf("angular_velocity.%s = %g is implausibly large — misaligned read", k, v)
		}
	}

	// linear_acceleration on a stationary robot: |z| ≈ 9.81, |x|,|y| < 1.
	linA := out["linear_acceleration"].(map[string]interface{})
	for _, k := range []string{"x", "y", "z"} {
		v := linA[k].(float64)
		if v != 0 && math.Abs(v) < 1e-100 {
			t.Errorf("linear_acceleration.%s = %g is a denormal — misaligned read", k, v)
		}
		if math.Abs(v) > 1e10 {
			t.Errorf("linear_acceleration.%s = %g is implausibly large — misaligned read", k, v)
		}
	}
}
