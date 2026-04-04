package nav

import "github.com/cedbossneo/mowglinext/pkg/msgs/geometry"

type Path struct {
	Header geometry.Header       `json:"header"`
	Poses  []geometry.PoseStamped `json:"poses"`
}

type Odometry struct {
	Header       geometry.Header              `json:"header"`
	ChildFrameId string                       `json:"child_frame_id"`
	Pose         geometry.PoseWithCovariance  `json:"pose"`
	Twist        geometry.TwistWithCovariance `json:"twist"`
}
