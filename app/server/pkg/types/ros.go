package types

import "context"

// IRosProvider is the abstraction layer for all ROS2 communication.
// The implementation uses a rosbridge WebSocket client; callers should never
// depend on goroslib or any ROS1 transport.
type IRosProvider interface {
	// CallService calls a ROS2 service via rosbridge.
	// service is the full service name (e.g. "/behavior_tree_node/high_level_control").
	// req is marshalled as the JSON args payload.
	// res, if non-nil, receives the unmarshalled service response values.
	CallService(ctx context.Context, service string, req any, res any) error

	// Subscribe registers cb to receive JSON-encoded messages on a logical
	// topic key (e.g. "status", "gps", "pose"). The cb is invoked from a
	// dedicated goroutine for each (topic, id) pair, so individual slow
	// subscribers do not block others. If a message was already received for
	// this topic, cb is called immediately with the last cached message.
	// Multiple callers may register distinct ids for the same logical key.
	Subscribe(topic string, id string, cb func(msg []byte)) error

	// UnSubscribe removes the subscriber identified by (topic, id) and stops
	// its goroutine. It is a no-op when the subscriber does not exist.
	UnSubscribe(topic string, id string)

	// Publish sends msg to the named ROS2 topic via rosbridge.
	// msgType is the ROS2 message type string (e.g. "geometry_msgs/msg/Twist").
	// The topic must have been advertised before publishing; implementations
	// may handle advertisement internally.
	Publish(topic string, msgType string, msg interface{}) error
}
