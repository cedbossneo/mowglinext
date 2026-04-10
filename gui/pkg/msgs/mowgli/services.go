package mowgli

import "github.com/cedbossneo/mowglinext/pkg/msgs/geometry"

// HighLevelControlReq for /behavior_tree_node/high_level_control
type HighLevelControlReq struct {
	Command uint8 `json:"command"`
}

type HighLevelControlRes struct {
	Success bool `json:"success"`
}

// EmergencyStopReq for /hardware_bridge/emergency_stop
type EmergencyStopReq struct {
	Emergency uint8 `json:"emergency"`
}

type EmergencyStopRes struct {
	Success bool `json:"success"`
}

// MowerControlReq for /hardware_bridge/mower_control
type MowerControlReq struct {
	MowEnabled   uint8 `json:"mow_enabled"`
	MowDirection uint8 `json:"mow_direction"`
}

type MowerControlRes struct {
	Success bool `json:"success"`
}

// StartInAreaReq - for scheduled mowing
type StartInAreaReq struct {
	Area uint8 `json:"area"`
}

type StartInAreaRes struct {
	Success bool `json:"success"`
}

// GetMowingAreaReq for /map_server_node/get_mowing_area
type GetMowingAreaReq struct {
	Index uint32 `json:"index"`
}

type GetMowingAreaRes struct {
	Area             MapArea `json:"area"`
	IsNavigationArea bool    `json:"is_navigation_area"`
	Success          bool    `json:"success"`
}

// AddMowingAreaReq for /map_server_node/add_area
type AddMowingAreaReq struct {
	Area             MapArea `json:"area"`
	IsNavigationArea bool    `json:"is_navigation_area"`
}

type AddMowingAreaRes struct {
	Success bool `json:"success"`
}

// SetDockingPointReq for map_server dock service
type SetDockingPointReq struct {
	DockingPose geometry.Pose `json:"docking_pose"`
}

type SetDockingPointRes struct {
	Success bool `json:"success"`
}

// ClearMapReq for /map_server_node/clear_map (std_srvs/Trigger)
type ClearMapReq struct{}

type ClearMapRes struct {
	Success bool   `json:"success"`
	Message string `json:"message"`
}

// AreaRecordingReq for /behavior_tree_node/area_recording (convenience alias
// for sending COMMAND_RECORD_FINISH / COMMAND_RECORD_CANCEL via high_level_control)
// Not a separate service — reuses HighLevelControlReq with command 5 (finish) or 6 (cancel).

// ReplaceMapReq - used by the frontend to clear+add all areas
type ReplaceMapArea struct {
	Area             MapArea `json:"area"`
	IsNavigationArea bool    `json:"is_navigation_area"`
}

type ReplaceMapReq struct {
	Areas []ReplaceMapArea `json:"areas"`
}
