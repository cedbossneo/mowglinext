package tunnel

import (
	"encoding/json"
	"errors"
	"log"
	"math"
	"net/url"
	"sync"
	"time"

	"github.com/cedbossneo/mowglinext/app/agent/config"
	"github.com/gorilla/websocket"
)

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

// ErrNotConnected is returned when sending on a closed tunnel
var ErrNotConnected = errors.New("not connected to proxy")

// Client manages the outbound tunnel connection to the proxy
type Client struct {
	cfg    *config.Config
	conn   *websocket.Conn
	mu     sync.Mutex
	mux    *Multiplexer
	done   chan struct{}
	closed bool
}

func NewClient(cfg *config.Config) *Client {
	return &Client{
		cfg:  cfg,
		done: make(chan struct{}),
	}
}

// Connect establishes and maintains the tunnel connection with exponential backoff
func (c *Client) Connect() {
	backoff := 1 * time.Second
	maxBackoff := 60 * time.Second

	for {
		select {
		case <-c.done:
			return
		default:
		}

		err := c.connectOnce()
		if err != nil {
			log.Printf("Tunnel connection failed: %v, retrying in %v", err, backoff)
			time.Sleep(backoff)
			backoff = time.Duration(math.Min(float64(backoff*2), float64(maxBackoff)))
			continue
		}

		// Reset backoff on successful connection
		backoff = 1 * time.Second
	}
}

func (c *Client) connectOnce() error {
	u, err := url.Parse(c.cfg.ProxyURL)
	if err != nil {
		return err
	}

	// Build tunnel URL
	scheme := "wss"
	if u.Scheme == "http" {
		scheme = "ws"
	}
	tunnelURL := scheme + "://" + u.Host + "/tunnel/connect?token=" + c.cfg.RobotToken + "&robot_id=" + c.cfg.RobotID

	log.Printf("Connecting to tunnel: %s", u.Host)

	conn, _, err := websocket.DefaultDialer.Dial(tunnelURL, nil)
	if err != nil {
		return err
	}

	c.mu.Lock()
	c.conn = conn
	c.mux = NewMultiplexer(c.cfg.LocalGUIURL, c)
	c.mu.Unlock()

	log.Printf("Tunnel connected to %s", u.Host)

	// Start heartbeat
	go c.heartbeatLoop()

	// Read loop (blocks until disconnect)
	return c.readLoop()
}

func (c *Client) readLoop() error {
	for {
		_, msg, err := c.conn.ReadMessage()
		if err != nil {
			log.Printf("Tunnel read error: %v", err)
			return err
		}

		var frame TunnelFrame
		if err := json.Unmarshal(msg, &frame); err != nil {
			log.Printf("Frame parse error: %v", err)
			continue
		}

		switch frame.Type {
		case FrameHeartbeat:
			// Proxy heartbeat received, no action needed
		case FrameRequest:
			go c.mux.HandleRequest(&frame)
		case FrameSubscribe:
			go c.mux.HandleSubscribe(&frame)
		case FrameUnsubscribe:
			c.mux.HandleUnsubscribe(&frame)
		}
	}
}

func (c *Client) heartbeatLoop() {
	ticker := time.NewTicker(30 * time.Second)
	defer ticker.Stop()
	for {
		select {
		case <-ticker.C:
			c.mu.Lock()
			if c.conn != nil {
				c.conn.WriteJSON(&TunnelFrame{Type: FrameHeartbeat})
			}
			c.mu.Unlock()
		case <-c.done:
			return
		}
	}
}

// SendFrame sends a frame through the tunnel
func (c *Client) SendFrame(frame *TunnelFrame) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.conn == nil {
		return ErrNotConnected
	}
	return c.conn.WriteJSON(frame)
}

// Close shuts down the client
func (c *Client) Close() {
	c.mu.Lock()
	defer c.mu.Unlock()
	if !c.closed {
		c.closed = true
		close(c.done)
	}
	if c.conn != nil {
		c.conn.Close()
	}
	if c.mux != nil {
		c.mux.CloseAll()
	}
}
