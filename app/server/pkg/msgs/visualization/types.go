package visualization

import "github.com/cedbossneo/mowglinext/app/server/pkg/msgs/geometry"

type Marker struct {
	Header   geometry.Header `json:"header"`
	Ns       string          `json:"ns"`
	Id       int32           `json:"id"`
	Type     int32           `json:"type"`
	Action   int32           `json:"action"`
	Pose     geometry.Pose   `json:"pose"`
	Scale    geometry.Vector3 `json:"scale"`
	Color    ColorRGBA        `json:"color"`
	Lifetime Duration         `json:"lifetime"`
	Points   []geometry.Point `json:"points"`
}

type ColorRGBA struct {
	R float32 `json:"r"`
	G float32 `json:"g"`
	B float32 `json:"b"`
	A float32 `json:"a"`
}

type Duration struct {
	Sec     int32  `json:"sec"`
	Nanosec uint32 `json:"nanosec"`
}

type MarkerArray struct {
	Markers []Marker `json:"markers"`
}
