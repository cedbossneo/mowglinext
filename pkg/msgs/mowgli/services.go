package mowgli

import "github.com/cedbossneo/openmower-gui/pkg/msgs/geometry"

// HighLevelControlReq for /behavior_tree_node/high_level_control
type HighLevelControlReq struct {
	Command uint8
}

type HighLevelControlRes struct {
	Success bool
}

// EmergencyStopReq for /hardware_bridge/emergency_stop
type EmergencyStopReq struct {
	Emergency uint8
}

type EmergencyStopRes struct {
	Success bool
}

// MowerControlReq for /hardware_bridge/mower_control
type MowerControlReq struct {
	MowEnabled   uint8
	MowDirection uint8
}

type MowerControlRes struct {
	Success bool
}

// StartInAreaReq - for scheduled mowing
type StartInAreaReq struct {
	Area uint8
}

type StartInAreaRes struct {
	Success bool
}

// AddMowingAreaReq for /map_server_node/add_area
type AddMowingAreaReq struct {
	Area             MapArea
	IsNavigationArea bool
}

type AddMowingAreaRes struct {
	Success bool
}

// SetDockingPointReq for map_server dock service
type SetDockingPointReq struct {
	DockingPose geometry.Pose
}

type SetDockingPointRes struct {
	Success bool
}

// ClearMapReq for /map_server_node/clear_map (std_srvs/Trigger)
type ClearMapReq struct{}

type ClearMapRes struct {
	Success bool
	Message string
}

// ReplaceMapReq - used by the frontend to clear+add all areas
type ReplaceMapArea struct {
	Area             MapArea
	IsNavigationArea bool
}

type ReplaceMapReq struct {
	Areas []ReplaceMapArea
}
