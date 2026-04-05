package tunnel

// Heartbeat logic is integrated into Client.heartbeatLoop()
// This file exists for organizational parity with the plan.
//
// The client sends a heartbeat frame every 30s.
// The proxy marks the robot offline if no heartbeat is received within 90s.
