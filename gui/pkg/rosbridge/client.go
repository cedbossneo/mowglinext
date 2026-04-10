// Package rosbridge implements a WebSocket client for the rosbridge v2 JSON protocol.
// It connects to a rosbridge_server (ROS2) to subscribe to topics, call services,
// and publish messages using the standard rosbridge v2 JSON envelope format.
package rosbridge

import (
	"context"
	"encoding/json"
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	"github.com/gorilla/websocket"
	"github.com/sirupsen/logrus"
)

const (
	defaultReconnectDelay = 1 * time.Second
	maxReconnectDelay     = 30 * time.Second
	callServiceTimeout    = 10 * time.Second
)

// subscriberEntry associates a caller-supplied ID with its callback so that
// multiple independent callers can listen on the same topic, and individual
// subscriptions can be removed without disturbing the others.
type subscriberEntry struct {
	id       string
	callback func(json.RawMessage)
}

// serviceResponse mirrors the rosbridge "service_response" op payload.
type serviceResponse struct {
	Result bool            `json:"result"`
	Values json.RawMessage `json:"values"`
}

// inboundMsg is the minimal envelope used when dispatching inbound frames.
type inboundMsg struct {
	Op      string          `json:"op"`
	ID      string          `json:"id"`
	Topic   string          `json:"topic"`
	Msg     json.RawMessage `json:"msg"`
	Result  bool            `json:"result"`
	Values  json.RawMessage `json:"values"`
}

// Client connects to a rosbridge_server via WebSocket and exposes a
// topic-oriented API for subscribing, publishing, and calling services.
//
// A Client automatically reconnects with exponential back-off when the
// underlying WebSocket is interrupted, re-registering all active subscriptions
// on every successful reconnect.
//
// All exported methods are safe for concurrent use.
type Client struct {
	url string

	// conn is the active WebSocket connection. connMu serialises all writes
	// (gorilla/websocket connections are not write-safe for concurrent use).
	conn   *websocket.Conn
	connMu sync.Mutex

	// subscribers maps topic -> ordered list of (id, callback) pairs.
	// subMu is a RWMutex: the readPump dispatch path acquires a read lock,
	// while Subscribe/Unsubscribe acquire the write lock.
	subscribers    map[string][]subscriberEntry
	topicTypes     map[string]string // topic -> msgType for resubscribe
	throttleRates  map[string]int    // topic -> throttle_rate ms for resubscribe
	subMu       sync.RWMutex

	// pendingSvc maps a unique call ID to the channel that CallService is
	// blocking on. svcMu serialises map access.
	pendingSvc map[string]chan serviceResponse
	svcMu      sync.Mutex

	// advertised tracks which topics have already been advertised so that
	// Advertise is idempotent within a single connection lifetime.
	advertised map[string]bool
	advMu      sync.Mutex

	// connected reflects whether a live WebSocket connection currently exists.
	connected atomic.Bool

	// done is closed by Close to signal the reconnect loop to exit.
	done chan struct{}

	// reconnect timing parameters (set once by NewClient, read-only afterward).
	reconnectDelay time.Duration
	maxReconnect   time.Duration

	// idCounter is atomically incremented to produce unique message IDs.
	idCounter atomic.Uint64
}

// NewClient creates a Client that will connect to the rosbridge WebSocket at
// url (e.g. "ws://localhost:9090"). Connect must be called to open the
// connection. Reconnection parameters default to a 1 s initial delay capped
// at 30 s.
func NewClient(url string) *Client {
	return &Client{
		url:            url,
		subscribers:    make(map[string][]subscriberEntry),
		topicTypes:     make(map[string]string),
		throttleRates:  make(map[string]int),
		pendingSvc:     make(map[string]chan serviceResponse),
		advertised:     make(map[string]bool),
		done:           make(chan struct{}),
		reconnectDelay: defaultReconnectDelay,
		maxReconnect:   maxReconnectDelay,
	}
}

// nextID returns a unique string identifier for outbound messages.
func (c *Client) nextID() string {
	return fmt.Sprintf("gui-%d", c.idCounter.Add(1))
}

// Connected reports whether the client currently has an active WebSocket
// connection to rosbridge.
func (c *Client) Connected() bool {
	return c.connected.Load()
}

// Connect starts the reconnect loop which will keep trying to reach
// rosbridge until ctx is cancelled. If the initial dial succeeds the read
// pump is started immediately; otherwise the reconnect loop handles
// subsequent attempts with exponential back-off. Connect never returns an
// error – callers can watch Connected() for state changes.
func (c *Client) Connect(ctx context.Context) error {
	conn, _, err := websocket.DefaultDialer.DialContext(ctx, c.url, nil)
	if err != nil {
		logrus.WithError(err).WithField("url", c.url).
			Warn("rosbridge: initial dial failed, will keep retrying")
		// Start the reconnect loop so it retries in the background.
		go c.reconnectLoop(ctx)
		return nil
	}

	c.connMu.Lock()
	c.conn = conn
	c.connMu.Unlock()
	c.connected.Store(true)

	logrus.WithField("url", c.url).Info("rosbridge: connected")

	go c.readPump()
	go c.reconnectLoop(ctx)

	return nil
}

// Close shuts down the WebSocket connection and stops all background
// goroutines. Subsequent calls to Close are safe but no-ops.
func (c *Client) Close() error {
	select {
	case <-c.done:
		// already closed
		return nil
	default:
		close(c.done)
	}

	c.connMu.Lock()
	defer c.connMu.Unlock()

	if c.conn == nil {
		return nil
	}

	err := c.conn.WriteMessage(
		websocket.CloseMessage,
		websocket.FormatCloseMessage(websocket.CloseNormalClosure, ""),
	)
	closeErr := c.conn.Close()
	c.connected.Store(false)

	if err != nil && closeErr != nil {
		return fmt.Errorf("rosbridge: close: %w", closeErr)
	}
	if closeErr != nil {
		return fmt.Errorf("rosbridge: close: %w", closeErr)
	}
	return err
}

// Subscribe registers cb to receive every message published on topic by
// rosbridge. id is a caller-supplied identifier that allows the same topic to
// have multiple independent subscribers (e.g. different API handlers). If this
// is the first callback for topic, a subscribe message is sent to rosbridge.
// msgType is the ROS2 message type string (e.g. "sensor_msgs/NavSatFix").
// opts accepts an optional throttle rate in milliseconds as the first element;
// use 0 or omit for no throttling. This tells rosbridge to send messages for
// this topic no faster than the given interval, reducing CPU on the server.
//
// If the client is not yet connected the callback is registered locally and
// the wire subscribe will be sent automatically when a connection is
// established (via resubscribeAll).
func (c *Client) Subscribe(topic, msgType, id string, cb func(json.RawMessage), opts ...int) error {
	throttleMs := 0
	if len(opts) > 0 {
		throttleMs = opts[0]
	}

	c.subMu.Lock()
	defer c.subMu.Unlock()

	entries := c.subscribers[topic]
	// Deduplicate: replace an existing entry with the same id.
	for i, e := range entries {
		if e.id == id {
			entries[i].callback = cb
			c.subscribers[topic] = entries
			return nil
		}
	}

	firstSubscriber := len(entries) == 0

	c.subscribers[topic] = append(entries, subscriberEntry{id: id, callback: cb})

	// Remember the message type and throttle rate for resubscribeAll after reconnect.
	if msgType != "" {
		c.topicTypes[topic] = msgType
	}
	if throttleMs > 0 {
		c.throttleRates[topic] = throttleMs
	}

	// Only send one subscribe message to rosbridge per topic.
	// If not connected, skip – resubscribeAll will send it on connect.
	if firstSubscriber && c.connected.Load() {
		return c.sendSubscribe(topic, msgType, throttleMs)
	}
	return nil
}

// sendSubscribe sends the rosbridge subscribe op. It must be called with subMu
// at least read-locked (to prevent topic removal racing), and the write lock
// held when called from Subscribe. connMu is acquired internally.
// throttleMs sets the minimum interval (ms) between messages for this topic;
// 0 means no throttling.
func (c *Client) sendSubscribe(topic, msgType string, throttleMs int) error {
	msg := map[string]interface{}{
		"op":            "subscribe",
		"id":            c.nextID(),
		"topic":         topic,
		"type":          msgType,
		"compression":   "none",
		"throttle_rate": throttleMs,
	}
	return c.writeJSON(msg)
}

// Unsubscribe removes the callback identified by id from topic. If it was the
// last callback for that topic, an unsubscribe message is sent to rosbridge.
// Unsubscribe is a no-op when id is not registered.
func (c *Client) Unsubscribe(topic, id string) {
	c.subMu.Lock()
	defer c.subMu.Unlock()

	entries, ok := c.subscribers[topic]
	if !ok {
		return
	}

	filtered := entries[:0]
	for _, e := range entries {
		if e.id != id {
			filtered = append(filtered, e)
		}
	}

	if len(filtered) == 0 {
		delete(c.subscribers, topic)
		if c.connected.Load() {
			msg := map[string]interface{}{
				"op":    "unsubscribe",
				"id":    c.nextID(),
				"topic": topic,
			}
			if err := c.writeJSON(msg); err != nil {
				logrus.WithError(err).WithField("topic", topic).
					Warn("rosbridge: failed to send unsubscribe")
			}
		}
	} else {
		c.subscribers[topic] = filtered
	}
}

// Advertise informs rosbridge that this client intends to publish on topic
// with the given ROS2 msgType. Advertise is idempotent; repeated calls for
// the same topic are silently ignored.
func (c *Client) Advertise(topic, msgType string) error {
	if !c.connected.Load() {
		return fmt.Errorf("rosbridge: Advertise %s: not connected", topic)
	}

	c.advMu.Lock()
	defer c.advMu.Unlock()

	if c.advertised[topic] {
		return nil
	}

	msg := map[string]interface{}{
		"op":    "advertise",
		"id":    c.nextID(),
		"topic": topic,
		"type":  msgType,
	}
	if err := c.writeJSON(msg); err != nil {
		return fmt.Errorf("rosbridge: Advertise %s: %w", topic, err)
	}

	c.advertised[topic] = true
	return nil
}

// unadvertise sends the unadvertise op and removes the topic from the
// advertised map. advMu must be held by the caller.
func (c *Client) unadvertise(topic string) {
	if !c.advertised[topic] {
		return
	}
	msg := map[string]interface{}{
		"op":    "unadvertise",
		"id":    c.nextID(),
		"topic": topic,
	}
	if err := c.writeJSON(msg); err != nil {
		logrus.WithError(err).WithField("topic", topic).
			Warn("rosbridge: failed to send unadvertise")
	}
	delete(c.advertised, topic)
}

// Publish sends a message on topic. The caller should call Advertise first so
// that rosbridge knows the message type. If the topic has not been advertised,
// Publish sends the message anyway but rosbridge may reject or ignore it.
func (c *Client) Publish(topic string, msg interface{}) error {
	if !c.connected.Load() {
		return fmt.Errorf("rosbridge: Publish %s: not connected", topic)
	}

	envelope := map[string]interface{}{
		"op":    "publish",
		"topic": topic,
		"msg":   msg,
	}
	if err := c.writeJSON(envelope); err != nil {
		return fmt.Errorf("rosbridge: Publish %s: %w", topic, err)
	}
	return nil
}

// CallService invokes a ROS2 service and blocks until a response arrives or
// ctx is cancelled. args is marshalled as the "args" field of the
// call_service message. The raw "values" payload from the service response is
// returned on success; a non-nil result field set to false is treated as a
// service-level error.
//
// An optional serviceType (e.g. "mowgli_interfaces/srv/GetMowingArea") can be
// passed to include the "type" field in the rosbridge call_service message.
// This helps rosbridge resolve the service without runtime type discovery.
func (c *Client) CallService(ctx context.Context, service string, args interface{}, serviceType ...string) (json.RawMessage, error) {
	if !c.connected.Load() {
		return nil, fmt.Errorf("rosbridge: CallService %s: not connected", service)
	}

	id := c.nextID()
	respCh := make(chan serviceResponse, 1)

	c.svcMu.Lock()
	c.pendingSvc[id] = respCh
	c.svcMu.Unlock()

	defer func() {
		c.svcMu.Lock()
		delete(c.pendingSvc, id)
		c.svcMu.Unlock()
	}()

	callMsg := map[string]interface{}{
		"op":      "call_service",
		"id":      id,
		"service": service,
		"args":    args,
	}
	if len(serviceType) > 0 && serviceType[0] != "" {
		callMsg["type"] = serviceType[0]
	}
	if err := c.writeJSON(callMsg); err != nil {
		return nil, fmt.Errorf("rosbridge: CallService %s: send: %w", service, err)
	}

	select {
	case <-ctx.Done():
		return nil, fmt.Errorf("rosbridge: CallService %s: %w", service, ctx.Err())
	case resp := <-respCh:
		if !resp.Result {
			return nil, fmt.Errorf("rosbridge: CallService %s: service returned failure: %s", service, string(resp.Values))
		}
		return resp.Values, nil
	}
}

// writeJSON marshals v as JSON and sends it as a single WebSocket text frame.
// connMu serialises concurrent writes.
func (c *Client) writeJSON(v interface{}) error {
	data, err := json.Marshal(v)
	if err != nil {
		return fmt.Errorf("rosbridge: marshal: %w", err)
	}

	c.connMu.Lock()
	defer c.connMu.Unlock()

	if c.conn == nil {
		return fmt.Errorf("rosbridge: writeJSON: no active connection")
	}

	if err := c.conn.WriteMessage(websocket.TextMessage, data); err != nil {
		return fmt.Errorf("rosbridge: write: %w", err)
	}
	return nil
}

// readPump reads JSON frames from the WebSocket and dispatches them. It
// terminates when the connection is closed or an unrecoverable read error
// occurs, setting connected=false so the reconnect loop can take over.
func (c *Client) readPump() {
	defer func() {
		c.connected.Store(false)

		c.connMu.Lock()
		if c.conn != nil {
			_ = c.conn.Close()
			c.conn = nil
		}
		c.connMu.Unlock()

		logrus.Info("rosbridge: readPump exiting")
	}()

	// Capture the connection reference once. readPump owns a single connection
	// for its lifetime; when the connection is replaced after a reconnect a
	// new readPump goroutine is started for the new conn.
	c.connMu.Lock()
	conn := c.conn
	c.connMu.Unlock()

	if conn == nil {
		return
	}

	for {
		// Check for intentional shutdown before each blocking read.
		select {
		case <-c.done:
			return
		default:
		}

		// ReadMessage blocks until a frame arrives or the connection is closed.
		// We must NOT hold connMu here – that would deadlock writeJSON.
		_, data, err := conn.ReadMessage()
		if err != nil {
			select {
			case <-c.done:
				// Intentional shutdown; the close path already closed the conn.
				return
			default:
				logrus.WithError(err).Warn("rosbridge: read error, triggering reconnect")
				return
			}
		}

		c.dispatch(data)
	}
}

// dispatch parses a single inbound JSON frame and routes it to the
// appropriate handler based on the "op" field.
func (c *Client) dispatch(data []byte) {
	var frame inboundMsg
	if err := json.Unmarshal(data, &frame); err != nil {
		logrus.WithError(err).Warn("rosbridge: failed to unmarshal inbound frame")
		return
	}

	switch frame.Op {
	case "publish":
		c.dispatchPublish(frame.Topic, frame.Msg)

	case "service_response":
		c.dispatchServiceResponse(frame.ID, serviceResponse{
			Result: frame.Result,
			Values: frame.Values,
		})

	default:
		logrus.WithField("op", frame.Op).Debug("rosbridge: unhandled op")
	}
}

// dispatchPublish delivers a topic message to all registered callbacks.
// A read lock on subMu is sufficient because callbacks are invoked
// concurrently – each in its own goroutine to avoid blocking the read pump.
func (c *Client) dispatchPublish(topic string, msg json.RawMessage) {
	c.subMu.RLock()
	entries := make([]subscriberEntry, len(c.subscribers[topic]))
	copy(entries, c.subscribers[topic])
	c.subMu.RUnlock()

	for _, e := range entries {
		cb := e.callback
		go cb(msg)
	}
}

// dispatchServiceResponse delivers a service response to the waiting
// CallService call identified by id.
func (c *Client) dispatchServiceResponse(id string, resp serviceResponse) {
	c.svcMu.Lock()
	ch, ok := c.pendingSvc[id]
	c.svcMu.Unlock()

	if !ok {
		logrus.WithField("id", id).Warn("rosbridge: received service_response for unknown id")
		return
	}

	select {
	case ch <- resp:
	default:
		logrus.WithField("id", id).Warn("rosbridge: service_response channel full, dropping")
	}
}

// reconnectLoop watches for connection loss and re-dials with exponential
// back-off. On every successful reconnect it re-subscribes all topics that
// were active before the disconnect. It exits when ctx is cancelled or
// c.done is closed.
func (c *Client) reconnectLoop(ctx context.Context) {
	delay := c.reconnectDelay

	for {
		// Wait until disconnected or shutdown.
		for c.connected.Load() {
			select {
			case <-c.done:
				return
			case <-ctx.Done():
				return
			case <-time.After(200 * time.Millisecond):
			}
		}

		// Check for intentional shutdown before attempting to reconnect.
		select {
		case <-c.done:
			return
		case <-ctx.Done():
			return
		default:
		}

		logrus.WithFields(logrus.Fields{
			"url":   c.url,
			"delay": delay,
		}).Info("rosbridge: attempting reconnect")

		conn, _, err := websocket.DefaultDialer.DialContext(ctx, c.url, nil)
		if err != nil {
			logrus.WithError(err).WithField("retry_in", delay).
				Warn("rosbridge: reconnect failed")

			select {
			case <-c.done:
				return
			case <-ctx.Done():
				return
			case <-time.After(delay):
			}

			// Exponential back-off capped at maxReconnect.
			delay *= 2
			if delay > c.maxReconnect {
				delay = c.maxReconnect
			}
			continue
		}

		// Successful reconnect.
		c.connMu.Lock()
		c.conn = conn
		c.connMu.Unlock()
		c.connected.Store(true)

		// Clear the advertised map so topics get re-advertised on next Publish.
		c.advMu.Lock()
		c.advertised = make(map[string]bool)
		c.advMu.Unlock()

		logrus.WithField("url", c.url).Info("rosbridge: reconnected")

		// Re-subscribe all previously active topics.
		c.resubscribeAll()

		// Restart the read pump for the new connection.
		go c.readPump()

		// Reset back-off on success.
		delay = c.reconnectDelay
	}
}

// resubscribeAll sends a subscribe op for every topic that currently has at
// least one registered callback. This is called after a successful reconnect
// so that rosbridge resumes forwarding messages.
func (c *Client) resubscribeAll() {
	c.subMu.RLock()
	defer c.subMu.RUnlock()

	for topic, entries := range c.subscribers {
		if len(entries) == 0 {
			continue
		}
		msgType := c.topicTypes[topic]     // may be "" if never stored
		throttle := c.throttleRates[topic] // 0 if not set
		msg := map[string]interface{}{
			"op":            "subscribe",
			"id":            c.nextID(),
			"topic":         topic,
			"type":          msgType,
			"compression":   "none",
			"throttle_rate": throttle,
		}
		if err := c.writeJSON(msg); err != nil {
			logrus.WithError(err).WithField("topic", topic).
				Warn("rosbridge: failed to re-subscribe after reconnect")
		}
	}
}
