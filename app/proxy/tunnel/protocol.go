package tunnel

// TunnelFrame is the JSON envelope for all messages over the multiplexed tunnel WS
type TunnelFrame struct {
	Type      string            `json:"type"`                // subscribe, unsubscribe, message, request, response, heartbeat
	StreamID  string            `json:"stream_id,omitempty"` // UUID per logical stream
	Topic     string            `json:"topic,omitempty"`
	Method    string            `json:"method,omitempty"`
	Path      string            `json:"path,omitempty"`
	Status    int               `json:"status,omitempty"`
	Body      string            `json:"body,omitempty"` // base64 payload
	RequestID string            `json:"request_id,omitempty"`
	Headers   map[string]string `json:"headers,omitempty"`
}

// Frame types
const (
	FrameSubscribe   = "subscribe"
	FrameUnsubscribe = "unsubscribe"
	FrameMessage     = "message"
	FrameRequest     = "request"
	FrameResponse    = "response"
	FrameHeartbeat   = "heartbeat"
)
