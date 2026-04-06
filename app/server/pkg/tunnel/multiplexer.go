package tunnel

import (
	"encoding/base64"
	"io"
	"log"
	"net/http"
	"strings"
	"sync"

	"github.com/gorilla/websocket"
)

// Agent-side allowlist — mirrors proxy whitelist for defense in depth
var allowedPaths = []string{
	"/api/openmower/call/high_level_control",
	"/api/openmower/call/emergency",
	"/api/openmower/call/mow_enabled",
	"/api/openmower/call/start_in_area",
	"/api/openmower/call/mower_logic",
	"/api/openmower/subscribe/",
	"/api/openmower/publish/",
	"/api/schedules",
	"/api/settings/status",
	"/api/system/info",
}

// blockedPaths are NEVER allowed even if they match a prefix
var blockedPaths = []string{
	"/api/system/reboot",
	"/api/system/shutdown",
	"/api/containers",
	"/api/settings/yaml",
	"/api/openmower/map",
	"/api/setup",
}

// Multiplexer routes tunnel frames to local HTTP/WS connections
type Multiplexer struct {
	localURL string
	sender   FrameSender
	streams  map[string]*localWSStream
	mu       sync.RWMutex
}

type localWSStream struct {
	conn     *websocket.Conn
	streamID string
	done     chan struct{}
}

func NewMultiplexer(localURL string, sender FrameSender) *Multiplexer {
	return &Multiplexer{
		localURL: localURL,
		sender:   sender,
		streams:  make(map[string]*localWSStream),
	}
}

func isAllowedPath(path string) bool {
	// Check blocked paths first
	for _, blocked := range blockedPaths {
		if strings.HasPrefix(path, blocked) {
			return false
		}
	}
	for _, allowed := range allowedPaths {
		if strings.HasPrefix(path, allowed) {
			return true
		}
	}
	return false
}

// HandleRequest processes an HTTP request frame from the proxy
func (m *Multiplexer) HandleRequest(frame *TunnelFrame) {
	if !isAllowedPath(frame.Path) {
		log.Printf("Blocked disallowed path: %s", frame.Path)
		m.sender.SendFrame(&TunnelFrame{
			Type:      FrameResponse,
			StreamID:  frame.StreamID,
			RequestID: frame.RequestID,
			Status:    403,
			Body:      base64.StdEncoding.EncodeToString([]byte(`{"error":"path not allowed"}`)),
		})
		return
	}

	// Decode request body
	var bodyReader io.Reader
	if frame.Body != "" {
		decoded, err := base64.StdEncoding.DecodeString(frame.Body)
		if err != nil {
			log.Printf("Failed to decode request body: %v", err)
			return
		}
		bodyReader = strings.NewReader(string(decoded))
	}

	// Make local HTTP request
	url := m.localURL + frame.Path
	req, err := http.NewRequest(frame.Method, url, bodyReader)
	if err != nil {
		log.Printf("Failed to create request: %v", err)
		return
	}

	for k, v := range frame.Headers {
		req.Header.Set(k, v)
	}

	resp, err := http.DefaultClient.Do(req)
	if err != nil {
		log.Printf("Local request failed: %v", err)
		m.sender.SendFrame(&TunnelFrame{
			Type:      FrameResponse,
			StreamID:  frame.StreamID,
			RequestID: frame.RequestID,
			Status:    502,
			Body:      base64.StdEncoding.EncodeToString([]byte(`{"error":"local request failed"}`)),
		})
		return
	}
	defer resp.Body.Close()

	respBody, _ := io.ReadAll(resp.Body)

	m.sender.SendFrame(&TunnelFrame{
		Type:      FrameResponse,
		StreamID:  frame.StreamID,
		RequestID: frame.RequestID,
		Status:    resp.StatusCode,
		Body:      base64.StdEncoding.EncodeToString(respBody),
	})
}

// HandleSubscribe opens a local WS subscription and forwards messages back through the tunnel
func (m *Multiplexer) HandleSubscribe(frame *TunnelFrame) {
	topic := frame.Topic
	streamID := frame.StreamID

	// Build local WS URL
	wsURL := strings.Replace(m.localURL, "http://", "ws://", 1)
	wsURL = strings.Replace(wsURL, "https://", "wss://", 1)
	wsURL += "/api/openmower/subscribe/" + topic

	conn, _, err := websocket.DefaultDialer.Dial(wsURL, nil)
	if err != nil {
		log.Printf("Failed to subscribe to local topic %s: %v", topic, err)
		return
	}

	stream := &localWSStream{
		conn:     conn,
		streamID: streamID,
		done:     make(chan struct{}),
	}

	m.mu.Lock()
	m.streams[streamID] = stream
	m.mu.Unlock()

	log.Printf("Subscribed to local topic: %s (stream %s)", topic, streamID)

	// Forward messages from local WS to tunnel
	go func() {
		defer func() {
			conn.Close()
			m.mu.Lock()
			delete(m.streams, streamID)
			m.mu.Unlock()
		}()

		for {
			select {
			case <-stream.done:
				return
			default:
			}

			_, msg, err := conn.ReadMessage()
			if err != nil {
				log.Printf("Local WS read error for topic %s: %v", topic, err)
				return
			}

			m.sender.SendFrame(&TunnelFrame{
				Type:     FrameMessage,
				StreamID: streamID,
				Topic:    topic,
				Body:     string(msg), // Already base64 from the GUI
			})
		}
	}()
}

// HandleUnsubscribe closes a local WS subscription
func (m *Multiplexer) HandleUnsubscribe(frame *TunnelFrame) {
	m.mu.RLock()
	stream, ok := m.streams[frame.StreamID]
	m.mu.RUnlock()
	if ok {
		close(stream.done)
		stream.conn.Close()
	}
}

// CloseAll closes all local WS streams
func (m *Multiplexer) CloseAll() {
	m.mu.Lock()
	defer m.mu.Unlock()
	for _, stream := range m.streams {
		close(stream.done)
		stream.conn.Close()
	}
	m.streams = make(map[string]*localWSStream)
}
