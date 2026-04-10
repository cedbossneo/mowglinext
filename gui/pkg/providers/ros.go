package providers

import (
	"context"
	"encoding/json"
	"math"
	"sync"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/msgs/mowgli"
	"github.com/cedbossneo/mowglinext/pkg/rosbridge"
	types2 "github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/sirupsen/logrus"
)

// topicDef maps a logical subscribe key to a ROS2 topic name and message type.
// The frontend and internal routes always use logical keys; the ROS2 topic name
// is only used when sending the rosbridge subscribe op.
// ThrottleMs sets the minimum interval (ms) between messages that rosbridge
// will forward for this subscription. High-frequency or large topics should
// use a non-zero value to avoid saturating the CPU with JSON serialization.
type topicDef struct {
	ROS2Topic  string
	MsgType    string
	ThrottleMs int // 0 = no throttle
}

// topicMap maps logical keys (used by SubscriberRoute and internal code) to
// their corresponding ROS2 topics and message types.
// Virtual topics (map) have an empty MsgType and are never sent to
// rosbridge; they are populated by internal logic instead.
//
// Throttle rates are chosen to balance GUI responsiveness against CPU:
//   - Status/control topics: 200-500 ms (low bandwidth, GUI polls visually)
//   - Sensor topics (IMU, odom): 200-500 ms (GUI only needs display rate)
//   - Heavy topics (LaserScan, OccupancyGrid): 1000-2000 ms
//   - Event topics (path, plan, emergency): 0 (infrequent, need prompt delivery)
var topicMap = map[string]topicDef{
	"status":          {"/hardware_bridge/status", "mowgli_interfaces/msg/Status", 200},
	"highLevelStatus": {"/behavior_tree_node/high_level_status", "mowgli_interfaces/msg/HighLevelStatus", 500},
	"gps":             {"/gps/absolute_pose", "mowgli_interfaces/msg/AbsolutePose", 500},
	"pose":            {"/odometry/filtered_map", "nav_msgs/msg/Odometry", 200},
	"imu":             {"/imu/data", "sensor_msgs/msg/Imu", 500},
	"ticks":           {"/wheel_odom", "nav_msgs/msg/Odometry", 500},
	"map":             {"", "", 0},                                                                       // virtual – populated via map_server services
	"path":            {"/FollowCoveragePath/global_plan", "nav_msgs/msg/Path", 0},                       // infrequent event
	"plan":            {"/plan", "nav_msgs/msg/Path", 0},                                                 // infrequent event
	"coverageCells":   {"/map_server_node/coverage_cells", "nav_msgs/msg/OccupancyGrid", 2000},           // large message
	"power":           {"/hardware_bridge/power", "mowgli_interfaces/msg/Power", 1000},
	"emergency":       {"/hardware_bridge/emergency", "mowgli_interfaces/msg/Emergency", 0},              // safety-critical, no throttle
	// NOTE: DockingSensor.msg does not exist in mowgli_interfaces yet; omitted to avoid rosbridge errors.
	"lidar":            {"/scan", "sensor_msgs/msg/LaserScan", 1000},                                     // large message
	"diagnostics":      {"/diagnostics", "diagnostic_msgs/msg/DiagnosticArray", 2000},
	"obstacles":        {"/obstacle_tracker/obstacles", "mowgli_interfaces/msg/ObstacleArray", 1000},
	"robotDescription":     {"/robot_description", "std_msgs/msg/String", 0},                                // published once
	"recordingTrajectory": {"/behavior_tree_node/recording_trajectory", "nav_msgs/msg/Path", 500},        // area recording preview
}

// ---------------------------------------------------------------------------
// RosSubscriber – single fan-out worker for one (topic, id) pair
// ---------------------------------------------------------------------------

// RosSubscriber delivers messages from a ROS2 topic to one registered callback.
// It runs a background goroutine that drains a single-slot mailbox so that a
// slow consumer cannot stall the rosbridge read pump or other subscribers.
type RosSubscriber struct {
	Topic string
	Id    string

	mtx         sync.Mutex
	cb          func(msg []byte)
	nextMessage []byte
	close       chan struct{}
}

// NewRosSubscriber creates and starts a RosSubscriber. The caller must eventually
// call Close to release the background goroutine.
func NewRosSubscriber(topic, id string, cb func(msg []byte)) *RosSubscriber {
	r := &RosSubscriber{
		Topic: topic,
		Id:    id,
		cb:    cb,
		close: make(chan struct{}),
	}
	go r.run()
	return r
}

// Publish stores msg as the next message to be delivered. If a previous message
// has not yet been consumed it is silently overwritten (drop-oldest semantics).
func (r *RosSubscriber) Publish(msg []byte) {
	r.mtx.Lock()
	r.nextMessage = msg
	r.mtx.Unlock()
}

// Close stops the background goroutine. It is safe to call from any goroutine.
func (r *RosSubscriber) Close() {
	close(r.close)
}

// run is the delivery loop. It polls the mailbox at 100 ms intervals when idle.
func (r *RosSubscriber) run() {
	for {
		select {
		case <-r.close:
			return
		default:
		}

		r.mtx.Lock()
		msg := r.nextMessage
		r.nextMessage = nil
		r.mtx.Unlock()

		if msg != nil {
			r.cb(msg)
		} else {
			time.Sleep(100 * time.Millisecond)
		}
	}
}

// ---------------------------------------------------------------------------
// RosProvider – IRosProvider implementation backed by rosbridge WebSocket
// ---------------------------------------------------------------------------

// RosProvider implements types2.IRosProvider using a rosbridge v2 WebSocket
// client. All topic access uses logical keys defined in topicMap; the actual
// ROS2 topic names are an internal concern.
type RosProvider struct {
	client *rosbridge.Client

	mtx         sync.Mutex
	subscribers map[string]map[string]*RosSubscriber // logicalKey -> id -> subscriber
	lastMessage map[string][]byte                     // logicalKey -> last JSON bytes

	// Cached docking pose from map_server_node (guarded by mtx)
	dockPoseSet bool
	dockX       float64
	dockY       float64
	dockHeading float64

	dbProvider types2.IDBProvider
}

// NewRosProvider constructs a RosProvider, reads the rosbridge URL from the
// database (falling back to ws://localhost:9090), and connects asynchronously.
// The returned value is ready to use immediately; Subscribe calls made before
// the connection is established will be fulfilled once the connection comes up
// via the rosbridge client's reconnect loop.
func NewRosProvider(dbProvider types2.IDBProvider) types2.IRosProvider {
	rosbridgeURL := "ws://localhost:9090"
	if url, err := dbProvider.Get("system.ros.rosbridgeUrl"); err == nil && len(url) > 0 {
		rosbridgeURL = string(url)
	}

	r := &RosProvider{
		client:      rosbridge.NewClient(rosbridgeURL),
		subscribers: make(map[string]map[string]*RosSubscriber),
		lastMessage: make(map[string][]byte),
		dbProvider:  dbProvider,
	}

	go func() {
		if err := r.client.Connect(context.Background()); err != nil {
			logrus.Errorf("RosProvider: rosbridge initial connect failed: %v", err)
			// The rosbridge client's reconnect loop will keep retrying; we
			// still proceed so that subscriptions are registered and will be
			// re-sent on reconnect.
		}
		r.initRosbridgeSubscriptions()
			r.initDockPoseSubscription()
			r.initMapPolling()
	}()

	return r
}

// initRosbridgeSubscriptions sends subscribe ops to rosbridge for every
// real (non-virtual) topic in topicMap. Shape-changing topics are passed
// through dedicated adapter functions (defined in transform.go); all other
// topics are forwarded as-is (rosbridge snake_case JSON).
func (r *RosProvider) initRosbridgeSubscriptions() {
	adapters := map[string]func([]byte) ([]byte, error){
		"pose":  adaptPose,
		"ticks": adaptTicks,
	}

	for logicalKey, def := range topicMap {
		if def.MsgType == "" {
			continue
		}
		key := logicalKey     // capture for closure
		throttle := def.ThrottleMs // capture

		if adapt, ok := adapters[key]; ok {
			fn := adapt // capture
			err := r.client.Subscribe(def.ROS2Topic, def.MsgType, "gui-"+key, func(msg json.RawMessage) {
				adapted, err := fn([]byte(msg))
				if err != nil {
					logrus.Errorf("RosProvider: adapt %s: %v", key, err)
					return
				}
				r.fanOut(key, adapted)
			}, throttle)
			if err != nil {
				logrus.Errorf("RosProvider: subscribe %s (%s): %v", def.ROS2Topic, key, err)
			} else {
				logrus.Infof("RosProvider: subscribed to %s as '%s' (throttle %dms)", def.ROS2Topic, key, throttle)
			}
		} else {
			err := r.client.Subscribe(def.ROS2Topic, def.MsgType, "gui-"+key, func(msg json.RawMessage) {
				r.fanOut(key, []byte(msg))
			}, throttle)
			if err != nil {
				logrus.Errorf("RosProvider: subscribe %s (%s): %v", def.ROS2Topic, key, err)
			} else {
				logrus.Infof("RosProvider: subscribed to %s as '%s' (throttle %dms)", def.ROS2Topic, key, throttle)
			}
		}
	}
}

// fanOut stores msg as the latest value for logicalKey and delivers it to all
// registered RosSubscribers for that key. The caller must not hold mtx.
func (r *RosProvider) fanOut(logicalKey string, msg []byte) {
	r.mtx.Lock()
	defer r.mtx.Unlock()
	r.lastMessage[logicalKey] = msg
	for _, sub := range r.subscribers[logicalKey] {
		sub.Publish(msg)
	}
}

// initDockPoseSubscription subscribes to the map_server_node's docking_pose
// topic (transient_local QoS) and caches the latest dock position/heading.
// The cached values are included in the virtual "map" topic by pollMap().
func (r *RosProvider) initDockPoseSubscription() {
	type rawPoseStamped struct {
		Pose struct {
			Position    struct{ X, Y, Z float64 } `json:"position"`
			Orientation struct{ X, Y, Z, W float64 } `json:"orientation"`
		} `json:"pose"`
	}

	err := r.client.Subscribe(
		"/map_server_node/docking_pose",
		"geometry_msgs/msg/PoseStamped",
		"gui-docking-pose",
		func(msg json.RawMessage) {
			var ps rawPoseStamped
			if err := json.Unmarshal([]byte(msg), &ps); err != nil {
				logrus.Errorf("RosProvider: unmarshal docking_pose: %v", err)
				return
			}
			q := ps.Pose.Orientation
			heading := math.Atan2(2*(q.W*q.Z+q.X*q.Y), 1-2*(q.Y*q.Y+q.Z*q.Z))

			r.mtx.Lock()
			r.dockPoseSet = true
			r.dockX = ps.Pose.Position.X
			r.dockY = ps.Pose.Position.Y
			r.dockHeading = heading
			r.mtx.Unlock()

			logrus.Infof("RosProvider: docking pose updated (%.3f, %.3f) heading=%.3f",
				ps.Pose.Position.X, ps.Pose.Position.Y, heading)
		},
		0,
	)
	if err != nil {
		logrus.Errorf("RosProvider: subscribe docking_pose: %v", err)
	} else {
		logrus.Info("RosProvider: subscribed to /map_server_node/docking_pose")
	}
}

// initMapPolling periodically fetches mowing areas from the map_server_node
// and publishes the result to the virtual "map" topic for the GUI.
func (r *RosProvider) initMapPolling() {
	go func() {
		// Wait for rosbridge to be ready
		time.Sleep(5 * time.Second)

		ticker := time.NewTicker(5 * time.Second)
		defer ticker.Stop()

		for range ticker.C {
			r.pollMap()
		}
	}()
}

func (r *RosProvider) pollMap() {
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()

	var workingAreas []mowgli.MapArea
	var navAreas []mowgli.MapArea

	// Fetch all areas (index 0..N until success=false)
	for i := uint32(0); i < 100; i++ {
		req := mowgli.GetMowingAreaReq{Index: i}
		var res mowgli.GetMowingAreaRes
		err := r.CallService(ctx, "/map_server_node/get_mowing_area", &req, &res, "mowgli_interfaces/srv/GetMowingArea")
		if err != nil {
			if i == 0 {
				logrus.WithError(err).WithField("index", i).Warn("pollMap: get_mowing_area failed — map_server_node may not be ready")
			} else {
				logrus.WithError(err).WithField("index", i).Warn("pollMap: get_mowing_area failed mid-iteration")
			}
			break
		}
		if !res.Success {
			break
		}
		if res.Area.IsNavigationArea {
			navAreas = append(navAreas, res.Area)
		} else {
			workingAreas = append(workingAreas, res.Area)
		}
	}

	if workingAreas == nil {
		workingAreas = []mowgli.MapArea{}
	}
	if navAreas == nil {
		navAreas = []mowgli.MapArea{}
	}

	// Read cached docking pose (written by initDockPoseSubscription)
	r.mtx.Lock()
	dockX := r.dockX
	dockY := r.dockY
	dockHeading := r.dockHeading
	r.mtx.Unlock()

	mapData := mowgli.Map{
		MapWidth:        20.0,
		MapHeight:       20.0,
		MapCenterX:      0.0,
		MapCenterY:      0.0,
		NavigationAreas: navAreas,
		WorkingArea:     workingAreas,
		DockX:           dockX,
		DockY:           dockY,
		DockHeading:     dockHeading,
	}

	data, err := json.Marshal(mapData)
	if err != nil {
		return
	}

	r.fanOut("map", data)
}


// ---------------------------------------------------------------------------
// IRosProvider implementation
// ---------------------------------------------------------------------------

// CallService calls a ROS2 service via rosbridge and unmarshals the response
// into res (ignored when res is nil). An optional serviceType (e.g.
// "mowgli_interfaces/srv/GetMowingArea") helps rosbridge discover the service.
func (r *RosProvider) CallService(ctx context.Context, service string, req any, res any, serviceType ...string) error {
	result, err := r.client.CallService(ctx, service, req, serviceType...)
	if err != nil {
		return err
	}
	if res != nil {
		if err := json.Unmarshal(result, res); err != nil {
			return err
		}
	}
	return nil
}

// Subscribe registers cb to receive JSON messages on the given logical topic
// key. If a message was already received for this key, cb is invoked
// immediately with the cached value.
func (r *RosProvider) Subscribe(topic string, id string, cb func(msg []byte)) error {
	r.mtx.Lock()
	defer r.mtx.Unlock()

	if r.subscribers[topic] == nil {
		r.subscribers[topic] = make(map[string]*RosSubscriber)
	}
	if _, exists := r.subscribers[topic][id]; !exists {
		r.subscribers[topic][id] = NewRosSubscriber(topic, id, cb)
	}

	// Replay the most recent message so the subscriber is immediately usable.
	if last, ok := r.lastMessage[topic]; ok {
		r.subscribers[topic][id].Publish(last)
	}
	return nil
}

// UnSubscribe stops and removes the subscriber identified by (topic, id).
func (r *RosProvider) UnSubscribe(topic string, id string) {
	r.mtx.Lock()
	defer r.mtx.Unlock()

	subs, ok := r.subscribers[topic]
	if !ok {
		return
	}
	sub, exists := subs[id]
	if !exists {
		return
	}
	sub.Close()
	delete(subs, id)
}

// Publish sends msg to the named ROS2 topic via rosbridge. msgType is
// forwarded to rosbridge's advertise op (handled internally by the client).
func (r *RosProvider) Publish(topic string, msgType string, msg interface{}) error {
	if err := r.client.Advertise(topic, msgType); err != nil {
		// Log but do not abort – the client may not be connected yet; the
		// message will be dropped silently by the client's Publish path.
		logrus.Warnf("RosProvider: advertise %s: %v", topic, err)
	}
	return r.client.Publish(topic, msg)
}
