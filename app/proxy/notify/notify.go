package notify

import "context"

// NotificationEvent represents a notification to send
type NotificationEvent struct {
	RobotID string
	Title   string
	Body    string
	Data    map[string]string
}

// Dispatcher sends notifications to registered devices
type Dispatcher interface {
	Send(ctx context.Context, event *NotificationEvent, tokens []string) error
}
