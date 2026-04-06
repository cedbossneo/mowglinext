package tunnel

import (
	"errors"
	"sync"
)

var (
	ErrTimeout      = errors.New("tunnel request timeout")
	ErrDisconnected = errors.New("tunnel disconnected")
	ErrNotConnected = errors.New("robot not connected")
)

// Manager tracks all active robot tunnel connections
type Manager struct {
	connections map[string]*Connection
	mu          sync.RWMutex
}

func NewManager() *Manager {
	return &Manager{
		connections: make(map[string]*Connection),
	}
}

// Register adds a new robot connection, closing any existing one
func (m *Manager) Register(robotID string, conn *Connection) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if existing, ok := m.connections[robotID]; ok {
		existing.Close()
	}
	m.connections[robotID] = conn
}

// Get returns the connection for a robot, or nil if not connected
func (m *Manager) Get(robotID string) (*Connection, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	conn, ok := m.connections[robotID]
	if !ok || !conn.IsAlive() {
		return nil, ErrNotConnected
	}
	return conn, nil
}

// Remove removes a robot connection
func (m *Manager) Remove(robotID string) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if conn, ok := m.connections[robotID]; ok {
		conn.Close()
		delete(m.connections, robotID)
	}
}

// IsOnline checks if a robot has an active tunnel
func (m *Manager) IsOnline(robotID string) bool {
	m.mu.RLock()
	defer m.mu.RUnlock()
	conn, ok := m.connections[robotID]
	return ok && conn.IsAlive()
}

// OnlineRobots returns a list of currently connected robot IDs
func (m *Manager) OnlineRobots() []string {
	m.mu.RLock()
	defer m.mu.RUnlock()
	var ids []string
	for id, conn := range m.connections {
		if conn.IsAlive() {
			ids = append(ids, id)
		}
	}
	return ids
}
