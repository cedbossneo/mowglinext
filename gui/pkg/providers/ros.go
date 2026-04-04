package providers

import (
	"context"
	"encoding/json"
	"sync"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/msgs/geometry"
	"github.com/cedbossneo/mowglinext/pkg/msgs/mowgli"
	"github.com/cedbossneo/mowglinext/pkg/msgs/nav"
	"github.com/cedbossneo/mowglinext/pkg/rosbridge"
	types2 "github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/paulmach/orb"
	"github.com/paulmach/orb/simplify"
	"github.com/samber/lo"
	"github.com/sirupsen/logrus"
)

// topicDef maps a logical subscribe key to a ROS2 topic name and message type.
// The frontend and internal routes always use logical keys; the ROS2 topic name
// is only used when sending the rosbridge subscribe op.
type topicDef struct {
	ROS2Topic string
	MsgType   string
}

// topicMap maps logical keys (used by SubscriberRoute and internal code) to
// their corresponding ROS2 topics and message types.
// Virtual topics (map, mowingPath) have an empty MsgType and are never sent to
// rosbridge; they are populated by internal logic instead.
var topicMap = map[string]topicDef{
	"status":           {"/mowgli/hardware/status", "mowgli_interfaces/msg/Status"},
	"highLevelStatus":  {"/mowgli/behavior/status", "mowgli_interfaces/msg/HighLevelStatus"},
	"gps":              {"/mowgli/gps/absolute_pose", "mowgli_interfaces/msg/AbsolutePose"},
	"pose":             {"/mowgli/localization/odom_map", "nav_msgs/msg/Odometry"},
	"imu":              {"/mowgli/hardware/imu", "sensor_msgs/msg/Imu"},
	"ticks":            {"/mowgli/hardware/wheel_odom", "nav_msgs/msg/Odometry"},
	"map":              {"", ""},                                                        // virtual – populated via map_server services
	"path":             {"/mowgli/coverage/path", "nav_msgs/msg/Path"},
	"plan":             {"/plan", "nav_msgs/msg/Path"},                                  // Nav2 global plan (standard name)
	"mowingPath":       {"", ""},                                                        // virtual – populated by initMowingPathTracking
	"power":            {"/mowgli/hardware/power", "mowgli_interfaces/msg/Power"},
	"emergency":        {"/mowgli/hardware/emergency", "mowgli_interfaces/msg/Emergency"},
	"lidar":            {"/scan", "sensor_msgs/msg/LaserScan"},                          // Nav2 standard name
	"diagnostics":      {"/mowgli/diagnostics", "diagnostic_msgs/msg/DiagnosticArray"},
	"obstacles":        {"/mowgli/obstacles/tracked", "mowgli_interfaces/msg/ObstacleArray"},
	"robotDescription": {"/robot_description", "std_msgs/msg/String"},                   // ROS2 standard name
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

	// Mowing path state (guarded by mtx)
	mowingPaths      []*nav.Path
	mowingPath       *nav.Path
	mowingPathOrigin orb.LineString

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
		r.initMowingPathTracking()
		r.initMapPolling()
	}()

	return r
}

// initRosbridgeSubscriptions sends subscribe ops to rosbridge for every
// real (non-virtual) topic in topicMap. Shape-changing topics are passed
// through dedicated adapter functions (defined in transform.go); all other
// topics receive a generic snake_case → PascalCase key rename.
func (r *RosProvider) initRosbridgeSubscriptions() {
	adapters := map[string]func([]byte) ([]byte, error){
		"pose":  adaptPose,
		"ticks": adaptTicks,
	}

	for logicalKey, def := range topicMap {
		if def.MsgType == "" {
			continue
		}
		key := logicalKey // capture for closure

		if adapt, ok := adapters[key]; ok {
			fn := adapt // capture
			err := r.client.Subscribe(def.ROS2Topic, def.MsgType, "gui-"+key, func(msg json.RawMessage) {
				adapted, err := fn([]byte(msg))
				if err != nil {
					logrus.Errorf("RosProvider: adapt %s: %v", key, err)
					return
				}
				// Adapter output uses Go structs with json:"snake_case" tags;
				// the frontend expects PascalCase keys, so convert.
				converted, err := snakeToPascalJSON(adapted)
				if err != nil {
					logrus.Errorf("RosProvider: transform adapted %s: %v", key, err)
					r.fanOut(key, adapted) // fallback
					return
				}
				r.fanOut(key, converted)
			})
			if err != nil {
				logrus.Errorf("RosProvider: subscribe %s (%s): %v", def.ROS2Topic, key, err)
			} else {
				logrus.Infof("RosProvider: subscribed to %s as '%s'", def.ROS2Topic, key)
			}
		} else {
			err := r.client.Subscribe(def.ROS2Topic, def.MsgType, "gui-"+key, func(msg json.RawMessage) {
				converted, err := snakeToPascalJSON([]byte(msg))
				if err != nil {
					logrus.Errorf("RosProvider: transform %s: %v", key, err)
					r.fanOut(key, []byte(msg)) // fallback
					return
				}
				r.fanOut(key, converted)
			})
			if err != nil {
				logrus.Errorf("RosProvider: subscribe %s (%s): %v", def.ROS2Topic, key, err)
			} else {
				logrus.Infof("RosProvider: subscribed to %s as '%s'", def.ROS2Topic, key)
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

// initMowingPathTracking registers an internal subscriber on the "pose"
// logical key. It records the mower's position while the blade motor is
// running (MOWING state) and publishes the accumulated path list to the
// virtual "mowingPath" key.
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
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()

	var workingAreas []mowgli.MapArea
	var navAreas []mowgli.MapArea

	// Fetch all areas (index 0..N until success=false)
	for i := uint32(0); i < 100; i++ {
		req := mowgli.GetMowingAreaReq{Index: i}
		var res mowgli.GetMowingAreaRes
		err := r.CallService(ctx, "/mowgli/map/get_area", &req, &res)
		if err != nil || !res.Success {
			break
		}
		if res.IsNavigationArea {
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

	mapData := mowgli.Map{
		MapWidth:        20.0,
		MapHeight:       20.0,
		MapCenterX:      0.0,
		MapCenterY:      0.0,
		NavigationAreas: navAreas,
		WorkingArea:     workingAreas,
	}

	data, err := json.Marshal(mapData)
	if err != nil {
		return
	}

	// Convert to PascalCase for frontend
	converted, err := snakeToPascalJSON(data)
	if err != nil {
		r.fanOut("map", data)
		return
	}
	r.fanOut("map", converted)
}

func (r *RosProvider) initMowingPathTracking() {
	err := r.Subscribe("pose", "gui-mowing-path-tracker", func(msg []byte) {
		r.mtx.Lock()
		defer r.mtx.Unlock()
		r.processMowingPathUpdate(msg)
	})
	if err != nil {
		logrus.Errorf("RosProvider: failed to init mowing path tracking: %v", err)
	}
}

// processMowingPathUpdate is called for every odometry message while the
// internal subscriber goroutine holds mtx. It updates mowingPaths and fans
// out the result to "mowingPath" subscribers.
//
// Callers must hold r.mtx.
func (r *RosProvider) processMowingPathUpdate(msg []byte) {
	// Require high-level state to be MOWING.
	hlsData, ok := r.lastMessage["highLevelStatus"]
	if !ok {
		return
	}
	var hls mowgli.HighLevelStatus
	if err := json.Unmarshal(hlsData, &hls); err != nil {
		logrus.Errorf("RosProvider: unmarshal HighLevelStatus: %v", err)
		return
	}

	if hls.StateName != "MOWING" {
		// Any non-MOWING state resets the accumulated path.
		r.mowingPaths = []*nav.Path{}
		r.mowingPath = nil
		r.mowingPathOrigin = nil
		return
	}

	// Require blade motor to be spinning.
	statusData, ok := r.lastMessage["status"]
	if !ok {
		return
	}
	var status mowgli.Status
	if err := json.Unmarshal(statusData, &status); err != nil {
		logrus.Errorf("RosProvider: unmarshal Status: %v", err)
		return
	}

	if status.MowerMotorRpm <= 0 {
		// Motor stopped – end the current segment but keep previous segments.
		r.mowingPath = nil
		r.mowingPathOrigin = nil
		return
	}

	// Parse the pose message for the current position. The "pose" key is now
	// stored as AbsolutePose PascalCase JSON (produced by adaptPose).
	var pose mowgli.AbsolutePose
	if err := json.Unmarshal(msg, &pose); err != nil {
		logrus.Errorf("RosProvider: unmarshal AbsolutePose: %v", err)
		return
	}

	// Start a new path segment if needed.
	if r.mowingPath == nil {
		r.mowingPath = &nav.Path{}
		r.mowingPathOrigin = orb.LineString{}
		r.mowingPaths = append(r.mowingPaths, r.mowingPath)
	}

	r.mowingPathOrigin = append(r.mowingPathOrigin, orb.Point{
		pose.Pose.Pose.Position.X,
		pose.Pose.Pose.Position.Y,
	})

	// Simplify every 5 points to keep the payload compact.
	if len(r.mowingPathOrigin)%5 == 0 {
		reduced := simplify.DouglasPeucker(0.03).LineString(r.mowingPathOrigin.Clone())
		r.mowingPath.Poses = lo.Map(reduced, func(p orb.Point, _ int) geometry.PoseStamped {
			return geometry.PoseStamped{
				Pose: geometry.Pose{
					Position: geometry.Point{X: p[0], Y: p[1]},
				},
			}
		})
	}

	pathJSON, err := json.Marshal(r.mowingPaths)
	if err != nil {
		logrus.Errorf("RosProvider: marshal mowingPaths: %v", err)
		return
	}
	r.lastMessage["mowingPath"] = pathJSON
	for _, sub := range r.subscribers["mowingPath"] {
		sub.Publish(pathJSON)
	}
}

// ---------------------------------------------------------------------------
// IRosProvider implementation
// ---------------------------------------------------------------------------

// CallService calls a ROS2 service via rosbridge and unmarshals the response
// into res (ignored when res is nil).
func (r *RosProvider) CallService(ctx context.Context, service string, req any, res any) error {
	result, err := r.client.CallService(ctx, service, req)
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
