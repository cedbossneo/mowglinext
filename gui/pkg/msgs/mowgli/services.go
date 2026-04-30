package mowgli

// Types in this file are GUI-internal (not generated from ROS2 .srv files).
// For ROS2 service types, see services_generated.go.

// ReplaceMapReq - used by the frontend to clear+add all areas in a single HTTP call.
// Not a ROS2 service — the backend splits this into clear_map + add_area calls.
type ReplaceMapArea struct {
	Area             MapArea `json:"area"`
	IsNavigationArea bool    `json:"is_navigation_area"`
}

type ReplaceMapReq struct {
	Areas []ReplaceMapArea `json:"areas"`
}
