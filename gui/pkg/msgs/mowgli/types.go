package mowgli

import "github.com/cedbossneo/mowglinext/pkg/msgs/geometry"

// Status matches mowgli_interfaces/Status
type Status struct {
	Stamp                 geometry.Stamp `json:"stamp"`
	MowerStatus           uint8          `json:"mower_status"`
	RaspberryPiPower      bool           `json:"raspberry_pi_power"`
	IsCharging            bool           `json:"is_charging"`
	EscPower              bool           `json:"esc_power"`
	RainDetected          bool           `json:"rain_detected"`
	SoundModuleAvailable  bool           `json:"sound_module_available"`
	SoundModuleBusy       bool           `json:"sound_module_busy"`
	UiBoardAvailable      bool           `json:"ui_board_available"`
	MowEnabled            bool           `json:"mow_enabled"`
	MowerEscStatus        uint8          `json:"mower_esc_status"`
	MowerEscTemperature   float32        `json:"mower_esc_temperature"`
	MowerEscCurrent       float32        `json:"mower_esc_current"`
	MowerMotorTemperature float32        `json:"mower_motor_temperature"`
	MowerMotorRpm         float32        `json:"mower_motor_rpm"`
}

// HighLevelStatus matches mowgli_interfaces/HighLevelStatus
type HighLevelStatus struct {
	State             uint8   `json:"state"`
	StateName         string  `json:"state_name"`
	SubStateName      string  `json:"sub_state_name"`
	CurrentArea       int16   `json:"current_area"`
	CurrentPath       int16   `json:"current_path"`
	CurrentPathIndex  int16   `json:"current_path_index"`
	GpsQualityPercent float32 `json:"gps_quality_percent"`
	BatteryPercent    float32 `json:"battery_percent"`
	IsCharging        bool    `json:"is_charging"`
	Emergency         bool    `json:"emergency"`
}

// Power matches mowgli_interfaces/Power
type Power struct {
	Stamp          geometry.Stamp `json:"stamp"`
	VCharge        float32        `json:"v_charge"`
	VBattery       float32        `json:"v_battery"`
	ChargeCurrent  float32        `json:"charge_current"`
	ChargerEnabled bool           `json:"charger_enabled"`
	ChargerStatus  string         `json:"charger_status"`
}

// Emergency matches mowgli_interfaces/Emergency
type Emergency struct {
	Stamp            geometry.Stamp `json:"stamp"`
	ActiveEmergency  bool           `json:"active_emergency"`
	LatchedEmergency bool           `json:"latched_emergency"`
	Reason           string         `json:"reason"`
}

// AbsolutePose matches mowgli_interfaces/AbsolutePose
type AbsolutePose struct {
	Header              geometry.Header              `json:"header"`
	SensorStamp         uint32                       `json:"sensor_stamp"`
	ReceivedStamp       uint32                       `json:"received_stamp"`
	Source              uint8                        `json:"source"`
	Flags               uint16                       `json:"flags"`
	OrientationValid    uint8                        `json:"orientation_valid"`
	MotionVectorValid   uint8                        `json:"motion_vector_valid"`
	PositionAccuracy    float32                      `json:"position_accuracy"`
	OrientationAccuracy float32                      `json:"orientation_accuracy"`
	Pose                geometry.PoseWithCovariance  `json:"pose"`
	MotionVector        geometry.Vector3             `json:"motion_vector"`
	VehicleHeading      float64                      `json:"vehicle_heading"`
	MotionHeading       float64                      `json:"motion_heading"`
}

// WheelTick matches mowgli_interfaces/WheelTick
type WheelTick struct {
	Stamp            geometry.Stamp `json:"stamp"`
	WheelTickFactor  uint32         `json:"wheel_tick_factor"`
	ValidWheels      uint8          `json:"valid_wheels"`
	WheelDirectionFl uint8          `json:"wheel_direction_fl"`
	WheelTicksFl     uint32         `json:"wheel_ticks_fl"`
	WheelDirectionFr uint8          `json:"wheel_direction_fr"`
	WheelTicksFr     uint32         `json:"wheel_ticks_fr"`
	WheelDirectionRl uint8          `json:"wheel_direction_rl"`
	WheelTicksRl     uint32         `json:"wheel_ticks_rl"`
	WheelDirectionRr uint8          `json:"wheel_direction_rr"`
	WheelTicksRr     uint32         `json:"wheel_ticks_rr"`
}

// Map is the internal map structure sent to the frontend.
type Map struct {
	MapWidth        float64   `json:"map_width"`
	MapHeight       float64   `json:"map_height"`
	MapCenterX      float64   `json:"map_center_x"`
	MapCenterY      float64   `json:"map_center_y"`
	NavigationAreas []MapArea `json:"navigation_areas"`
	WorkingArea     []MapArea `json:"working_area"`
	DockX           float64   `json:"dock_x"`
	DockY           float64   `json:"dock_y"`
	DockHeading     float64   `json:"dock_heading"`
}

// MapArea matches the old xbot_msgs.MapArea JSON format.
type MapArea struct {
	Name      string             `json:"name"`
	Area      geometry.Polygon   `json:"area"`
	Obstacles []geometry.Polygon `json:"obstacles"`
}

// TrackedObstacle matches mowgli_interfaces/TrackedObstacle
type TrackedObstacle struct {
	Id               uint32           `json:"id"`
	Polygon          geometry.Polygon `json:"polygon"`
	Centroid         geometry.Point   `json:"centroid"`
	Radius           float64          `json:"radius"`
	FirstSeen        geometry.Stamp   `json:"first_seen"`
	ObservationCount uint32           `json:"observation_count"`
	Status           uint8            `json:"status"`
}

// ObstacleArray matches mowgli_interfaces/ObstacleArray
type ObstacleArray struct {
	Header    geometry.Header   `json:"header"`
	Obstacles []TrackedObstacle `json:"obstacles"`
}

// DockingSensor - placeholder, may not exist in ROS2 mowgli
type DockingSensor struct {
	DockPresent  bool    `json:"dock_present"`
	DockDistance float32 `json:"dock_distance"`
}
