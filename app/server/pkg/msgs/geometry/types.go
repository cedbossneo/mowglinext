package geometry

type Point struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

type Point32 struct {
	X float32 `json:"x"`
	Y float32 `json:"y"`
	Z float32 `json:"z"`
}

type Vector3 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

type Quaternion struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
	W float64 `json:"w"`
}

type Pose struct {
	Position    Point      `json:"position"`
	Orientation Quaternion `json:"orientation"`
}

type PoseWithCovariance struct {
	Pose       Pose        `json:"pose"`
	Covariance [36]float64 `json:"covariance"`
}

type PoseStamped struct {
	Header Header `json:"header"`
	Pose   Pose   `json:"pose"`
}

type Polygon struct {
	Points []Point32 `json:"points"`
}

type Twist struct {
	Linear  Vector3 `json:"linear"`
	Angular Vector3 `json:"angular"`
}

type TwistWithCovariance struct {
	Twist      Twist       `json:"twist"`
	Covariance [36]float64 `json:"covariance"`
}

// Header is std_msgs/Header
type Header struct {
	Stamp   Stamp  `json:"stamp"`
	FrameId string `json:"frame_id"`
}

type Stamp struct {
	Sec     uint32 `json:"sec"`
	Nanosec uint32 `json:"nanosec"`
}
