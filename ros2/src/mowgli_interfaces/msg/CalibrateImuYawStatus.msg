# Status / final result published by calibrate_imu_yaw_node on its
# `~/calibrate_status` topic. One message per state change; the LAST message
# (the one with `done=true`) is the durable result of the run.
#
# Published QoS: reliable + transient_local (depth 1) so a subscriber that
# attaches AFTER the run finishes still receives the final result.
#
# This split exists because foxglove_bridge ↔ rmw_cyclonedds GenericClient
# can't dispatch service responses with the original 12-field shape. Topic
# delivery doesn't go through that dispatch path. (closes #19)

# Caller-issued identifier echoed from the CalibrateImuYaw.srv request that
# triggered this run. Lets a `transient_local` subscriber discard stale
# cached statuses from previous runs.
string job_id

# True when this is the FINAL message of a run (success or failure).
# False while a run is in progress (no progress messages are emitted today,
# but the field is reserved for that).
bool done

# Whether the calibration finished cleanly. Only meaningful when done=true.
# Reasons it can be false: preflight (charging / emergency / autonomous) or
# the numerical solve falling under MIN_SAMPLES.
bool success

# Human-readable single-line summary, useful for the GUI to surface and for
# log inspection. Same format as the legacy in-band response.
string message

# Yaw solve.
float64 imu_yaw_rad
float64 imu_yaw_deg
int32 samples_used
float64 std_dev_deg

# Pitch / roll from stationary samples (baseline + inter-cycle pauses).
float64 imu_pitch_rad
float64 imu_pitch_deg
float64 imu_roll_rad
float64 imu_roll_deg
int32 stationary_samples_used
float64 gravity_mag_mps2
