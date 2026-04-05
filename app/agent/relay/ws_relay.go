package relay

// WebSocket relay logic is integrated into tunnel.Multiplexer.HandleSubscribe()
// This file documents the relay design:
//
// When a TunnelFrame{type:"subscribe"} arrives:
// 1. Open a WebSocket to ws://localhost:80/api/openmower/subscribe/:topic
// 2. For each message received on the local WS, wrap it in a
//    TunnelFrame{type:"message"} and send through the tunnel
// 3. When a TunnelFrame{type:"unsubscribe"} arrives, close the local WS
