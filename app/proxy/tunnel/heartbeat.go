package tunnel

import (
	"log"
	"time"
)

// StartCleanup periodically removes stale connections
func (m *Manager) StartCleanup(interval time.Duration) {
	go func() {
		ticker := time.NewTicker(interval)
		defer ticker.Stop()
		for range ticker.C {
			m.mu.Lock()
			for id, conn := range m.connections {
				if !conn.IsAlive() {
					log.Printf("Removing stale tunnel for robot %s", id)
					conn.Close()
					delete(m.connections, id)
				}
			}
			m.mu.Unlock()
		}
	}()
}
