package mowgli

import "github.com/cedbossneo/mowglinext/app/server/pkg/msgs/geometry"

// HighLevelControlReq for /mowgli/behavior/command
type HighLevelControlReq struct {
	Command uint8 `json:"command"`
}

type HighLevelControlRes struct {
	Success bool `json:"success"`
}

// EmergencyStopReq for /mowgli/hardware/emergency_stop
type EmergencyStopReq struct {
	Emergency uint8 `json:"emergency"`
}

type EmergencyStopRes struct {
	Success bool `json:"success"`
}

// MowerControlReq for /mowgli/hardware/mower_control
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

// GetMowingAreaReq for /mowgli/map/get_area
type GetMowingAreaReq struct {
	Index uint32 `json:"index"`
}

type GetMowingAreaRes struct {
	Area             MapArea `json:"area"`
	IsNavigationArea bool    `json:"is_navigation_area"`
	Success          bool    `json:"success"`
}

// AddMowingAreaReq for /mowgli/map/add_area
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

// ClearMapReq for /mowgli/map/clear (std_srvs/Trigger)
type ClearMapReq struct{}

type ClearMapRes struct {
	Success bool   `json:"success"`
	Message string `json:"message"`
}

// ReplaceMapReq - used by the frontend to clear+add all areas
type ReplaceMapArea struct {
	Area             MapArea `json:"area"`
	IsNavigationArea bool    `json:"is_navigation_area"`
}

type ReplaceMapReq struct {
	Areas []ReplaceMapArea `json:"areas"`
}
