package tunnel

// TunnelFrame is the JSON envelope for multiplexed tunnel messages
type TunnelFrame struct {
	Type      string            `json:"type"`
	StreamID  string            `json:"stream_id,omitempty"`
	Topic     string            `json:"topic,omitempty"`
	Method    string            `json:"method,omitempty"`
	Path      string            `json:"path,omitempty"`
	Status    int               `json:"status,omitempty"`
	Body      string            `json:"body,omitempty"`
	RequestID string            `json:"request_id,omitempty"`
	Headers   map[string]string `json:"headers,omitempty"`
}

const (
	FrameSubscribe   = "subscribe"
	FrameUnsubscribe = "unsubscribe"
	FrameMessage     = "message"
	FrameRequest     = "request"
	FrameResponse    = "response"
	FrameHeartbeat   = "heartbeat"
)

// FrameSender can send frames through the tunnel
type FrameSender interface {
	SendFrame(frame *TunnelFrame) error
}
