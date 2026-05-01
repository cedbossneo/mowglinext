// Package noise_shim implements a Noise IK terminator that lets remote mobile
// clients reach the local mowgli HTTP+WS API through a Cloudflare Tunnel
// without the cloud relay being able to read or modify any traffic.
//
// Wire format
//
// Outer transport: a single WebSocket connection (provided by cloudflared).
// Each WebSocket binary message carries one Noise transport frame.
//
//	┌────────────────┬───────────────────────────────────────────────────┐
//	│ u16 length(BE) │ AEAD ciphertext                                    │
//	└────────────────┴───────────────────────────────────────────────────┘
//
// The first two messages are the Noise IK handshake (initiator → responder
// then responder → initiator). The initiator's handshake payload carries:
//
//	{ "fb_id_token": "<firebase id token>", "nonce": "<32B base64>" }
//
// The responder validates the token (offline, against cached Google JWK),
// checks the uid is in the allow-list for this robot, and replies with:
//
//	{ "robot_id": "...", "robot_name": "...", "version": "...", "ts": ... }
//
// After the handshake, every Noise transport frame carries one inner-protocol
// message. Inner-protocol messages are CBOR-encoded for compactness. Each
// message has a top-level "t" (type), "id" (stream id, u32), and per-type
// fields.
//
//	Type     | Direction | Payload
//	---------+-----------+----------------------------------------------------
//	"req"    | C → R     | { method, path, headers (map), body (bytes) }
//	"res"    | R → C     | { status, headers (map), body (bytes) }
//	"wso"    | C → R     | { path, headers (map) }   // open WS subscription
//	"wsa"    | R → C     | {}                         // open accepted
//	"wse"    | R → C     | { reason }                 // open rejected
//	"wsd"    | both      | { data, binary }           // ws frame
//	"wsc"    | both      | { code, reason }           // ws close
//	"ping"   | both      | { ts }                     // keep-alive
//	"pong"   | both      | { ts }
//
// The shim is a strict reverse proxy to "http://localhost:<api_port>" — it
// MUST NOT execute any logic of its own beyond auth + framing.
package noise_shim

// FrameType is the inner-protocol message discriminator.
type FrameType string

const (
	FrameReq     FrameType = "req"
	FrameRes     FrameType = "res"
	FrameWSOpen  FrameType = "wso"
	FrameWSAck   FrameType = "wsa"
	FrameWSErr   FrameType = "wse"
	FrameWSData  FrameType = "wsd"
	FrameWSClose FrameType = "wsc"
	FramePing    FrameType = "ping"
	FramePong    FrameType = "pong"
)

// Frame is the decoded inner-protocol message. Only the fields relevant to a
// given Type are populated. CBOR encoding uses the struct tags below.
type Frame struct {
	Type     FrameType         `cbor:"t"`
	ID       uint32            `cbor:"id"`
	Method   string            `cbor:"method,omitempty"`
	Path     string            `cbor:"path,omitempty"`
	Headers  map[string]string `cbor:"headers,omitempty"`
	Body     []byte            `cbor:"body,omitempty"`
	Status   int               `cbor:"status,omitempty"`
	Data     []byte            `cbor:"data,omitempty"`
	Binary   bool              `cbor:"binary,omitempty"`
	Code     int               `cbor:"code,omitempty"`
	Reason   string            `cbor:"reason,omitempty"`
	TS       int64             `cbor:"ts,omitempty"`
}

// HandshakePayload is what the initiator (mobile client) sends inside the
// first IK handshake message.
type HandshakePayload struct {
	FirebaseIDToken string `cbor:"fb_id_token"`
	// Nonce is 32 random bytes; the responder rejects repeats within the
	// last NonceWindow.
	Nonce []byte `cbor:"nonce"`
	// AppVersion lets the robot know what client schema to expect.
	AppVersion string `cbor:"app_version,omitempty"`
}

// HandshakeReply is the responder's payload (from robot to phone).
type HandshakeReply struct {
	RobotID   string `cbor:"robot_id"`
	RobotName string `cbor:"robot_name,omitempty"`
	Version   string `cbor:"version"`
	TS        int64  `cbor:"ts"`
}
