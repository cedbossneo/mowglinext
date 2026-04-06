package tunnel

import (
	"encoding/json"
	"log"
	"sync"
	"time"

	"github.com/google/uuid"
	"github.com/gorilla/websocket"
)

// Connection represents a single robot's tunnel
type Connection struct {
	RobotID   string
	conn      *websocket.Conn
	mu        sync.Mutex
	streams   map[string]chan *TunnelFrame // StreamID -> response channel
	streamsMu sync.RWMutex
	lastSeen  time.Time
	done      chan struct{}
}

func NewConnection(robotID string, conn *websocket.Conn) *Connection {
	c := &Connection{
		RobotID:  robotID,
		conn:     conn,
		streams:  make(map[string]chan *TunnelFrame),
		lastSeen: time.Now(),
		done:     make(chan struct{}),
	}
	go c.readLoop()
	go c.heartbeatLoop()
	return c
}

func (c *Connection) readLoop() {
	defer close(c.done)
	for {
		_, msg, err := c.conn.ReadMessage()
		if err != nil {
			log.Printf("Tunnel read error for robot %s: %v", c.RobotID, err)
			return
		}
		c.lastSeen = time.Now()

		var frame TunnelFrame
		if err := json.Unmarshal(msg, &frame); err != nil {
			log.Printf("Tunnel frame parse error: %v", err)
			continue
		}

		switch frame.Type {
		case FrameHeartbeat:
			// Already updated lastSeen
		case FrameResponse, FrameMessage:
			c.streamsMu.RLock()
			ch, ok := c.streams[frame.StreamID]
			c.streamsMu.RUnlock()
			if ok {
				select {
				case ch <- &frame:
				default:
					// Channel full, drop frame
				}
			}
		}
	}
}

func (c *Connection) heartbeatLoop() {
	ticker := time.NewTicker(30 * time.Second)
	defer ticker.Stop()
	for {
		select {
		case <-ticker.C:
			c.mu.Lock()
			err := c.conn.WriteJSON(&TunnelFrame{Type: FrameHeartbeat})
			c.mu.Unlock()
			if err != nil {
				return
			}
		case <-c.done:
			return
		}
	}
}

// SendRequest sends an HTTP request through the tunnel and waits for response
func (c *Connection) SendRequest(method, path string, body string, headers map[string]string) (*TunnelFrame, error) {
	reqID := uuid.New().String()
	streamID := uuid.New().String()

	// Create response channel
	ch := make(chan *TunnelFrame, 1)
	c.streamsMu.Lock()
	c.streams[streamID] = ch
	c.streamsMu.Unlock()
	defer func() {
		c.streamsMu.Lock()
		delete(c.streams, streamID)
		c.streamsMu.Unlock()
	}()

	// Send request frame
	frame := &TunnelFrame{
		Type:      FrameRequest,
		StreamID:  streamID,
		RequestID: reqID,
		Method:    method,
		Path:      path,
		Body:      body,
		Headers:   headers,
	}
	c.mu.Lock()
	err := c.conn.WriteJSON(frame)
	c.mu.Unlock()
	if err != nil {
		return nil, err
	}

	// Wait for response (30s timeout)
	select {
	case resp := <-ch:
		return resp, nil
	case <-time.After(30 * time.Second):
		return nil, ErrTimeout
	case <-c.done:
		return nil, ErrDisconnected
	}
}

// Subscribe creates a stream for topic subscription
func (c *Connection) Subscribe(topic string) (string, <-chan *TunnelFrame, error) {
	streamID := uuid.New().String()
	ch := make(chan *TunnelFrame, 64)

	c.streamsMu.Lock()
	c.streams[streamID] = ch
	c.streamsMu.Unlock()

	frame := &TunnelFrame{
		Type:     FrameSubscribe,
		StreamID: streamID,
		Topic:    topic,
	}
	c.mu.Lock()
	err := c.conn.WriteJSON(frame)
	c.mu.Unlock()
	if err != nil {
		c.streamsMu.Lock()
		delete(c.streams, streamID)
		c.streamsMu.Unlock()
		return "", nil, err
	}

	return streamID, ch, nil
}

// Unsubscribe removes a topic subscription
func (c *Connection) Unsubscribe(streamID string) {
	frame := &TunnelFrame{
		Type:     FrameUnsubscribe,
		StreamID: streamID,
	}
	c.mu.Lock()
	c.conn.WriteJSON(frame)
	c.mu.Unlock()

	c.streamsMu.Lock()
	delete(c.streams, streamID)
	c.streamsMu.Unlock()
}

// Publish sends a message to a topic through the tunnel
func (c *Connection) Publish(topic string, body string) error {
	frame := &TunnelFrame{
		Type:     FrameMessage,
		StreamID: uuid.New().String(),
		Topic:    topic,
		Body:     body,
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.conn.WriteJSON(frame)
}

// IsAlive checks if the connection is still active
func (c *Connection) IsAlive() bool {
	select {
	case <-c.done:
		return false
	default:
		return time.Since(c.lastSeen) < 90*time.Second
	}
}

// Done returns a channel that is closed when the connection terminates
func (c *Connection) Done() <-chan struct{} {
	return c.done
}

// Close shuts down the connection
func (c *Connection) Close() {
	c.conn.Close()
}
