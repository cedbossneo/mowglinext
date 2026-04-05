package relay

// HTTP relay logic is integrated into tunnel.Multiplexer.HandleRequest()
// This file documents the relay design:
//
// When a TunnelFrame{type:"request"} arrives:
// 1. Validate the path against the agent-side allowlist
// 2. Decode the base64 body
// 3. Make an HTTP request to the local GUI (localhost:80)
// 4. Encode the response body as base64
// 5. Send a TunnelFrame{type:"response"} back through the tunnel
