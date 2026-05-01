package pairing

import (
	"encoding/hex"
	"fmt"
	"net/url"

	qrcode "github.com/skip2/go-qrcode"
)

// BuildPayload returns the text content of the pairing QR code as a deep-link
// URL. The mobile app parses this URL to initiate the pairing flow.
//
// Format: mowgli://pair?v=1&rid=<robotID>&pub=<pubHex>&tok=<setupToken>&lan=<lanHostPort>&cf=<tunnelHostname>
//
// Fields:
//   - v:   protocol version, currently "1"
//   - rid: robot ID (6 uppercase hex chars from RobotIDFromPub)
//   - pub: robot's Curve25519 public key, hex-encoded (64 chars)
//   - tok: one-time setup token, hex-encoded
//   - lan: LAN base address, e.g. "mowgli.local:8080"
//   - cf:  Cloudflare Tunnel hostname, e.g. "mowgli-abc123.cfargotunnel.com"
func BuildPayload(robotID string, pub []byte, setupToken string, lanHostPort string, tunnelHostname string) string {
	q := url.Values{}
	q.Set("v", "1")
	q.Set("rid", robotID)
	q.Set("pub", hex.EncodeToString(pub))
	q.Set("tok", setupToken)
	q.Set("lan", lanHostPort)
	q.Set("cf", tunnelHostname)
	return fmt.Sprintf("mowgli://pair?%s", q.Encode())
}

// RenderPNG encodes text as a QR code and returns the PNG image bytes at the
// requested pixel size. size is the image dimension in pixels (e.g. 256).
// The QR code uses medium error correction level (recovers up to 15% damage).
func RenderPNG(text string, size int) ([]byte, error) {
	png, err := qrcode.Encode(text, qrcode.Medium, size)
	if err != nil {
		return nil, fmt.Errorf("pairing: render QR PNG: %w", err)
	}
	return png, nil
}
