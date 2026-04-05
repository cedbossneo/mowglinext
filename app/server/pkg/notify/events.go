package notify

// NotificationEvent represents an event detected on the robot that should
// trigger a push notification via the proxy
type NotificationEvent struct {
	Type    string `json:"type"`
	Title   string `json:"title"`
	Body    string `json:"body"`
	Urgency string `json:"urgency"` // "high", "normal", "low"
}

// Event types
const (
	EventMowingComplete = "mowing_complete"
	EventError          = "error"
	EventLowBattery     = "low_battery"
	EventGPSLost        = "gps_lost"
	EventEmergency      = "emergency"
	EventOffline        = "offline" // detected proxy-side
)
