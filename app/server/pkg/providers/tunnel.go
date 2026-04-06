package providers

import (
	"crypto/rand"
	"encoding/hex"
	"log"

	"github.com/cedbossneo/mowglinext/app/server/pkg/notify"
	"github.com/cedbossneo/mowglinext/app/server/pkg/tunnel"
	"github.com/cedbossneo/mowglinext/app/server/pkg/types"
)

type TunnelProvider struct {
	client   *tunnel.Client
	detector *notify.Detector
}

func NewTunnelProvider(db types.IDBProvider) *TunnelProvider {
	proxyURL := dbGetString(db, "system.tunnel.proxy_url")
	robotToken := dbGetString(db, "system.tunnel.robot_token")
	robotID := dbGetString(db, "system.tunnel.robot_id")

	// Auto-generate robot_token if empty
	if robotToken == "" {
		robotToken = generateToken(32)
		db.Set("system.tunnel.robot_token", []byte(robotToken))
		log.Printf("Tunnel: generated new robot token")
	}

	// Auto-generate robot_id if empty
	if robotID == "" {
		robotID = generateToken(8)
		db.Set("system.tunnel.robot_id", []byte(robotID))
		log.Printf("Tunnel: generated new robot ID: %s", robotID)
	}

	if proxyURL == "" {
		log.Println("Tunnel: proxy_url not configured, skipping")
		return nil
	}

	cfg := &tunnel.Config{
		ProxyURL:    proxyURL,
		RobotToken:  robotToken,
		RobotID:     robotID,
		LocalGUIURL: "http://localhost:80",
	}

	client := tunnel.NewClient(cfg)

	// Start notification detector
	detector := notify.NewDetector(cfg.LocalGUIURL, client)
	go detector.Start()

	// Connect to proxy (reconnects automatically)
	go client.Connect()

	log.Printf("Tunnel: connecting to %s (robot %s)", proxyURL, robotID)

	return &TunnelProvider{
		client:   client,
		detector: detector,
	}
}

func (t *TunnelProvider) Close() {
	if t != nil && t.client != nil {
		t.client.Close()
	}
}

func dbGetString(db types.IDBProvider, key string) string {
	val, err := db.Get(key)
	if err != nil || len(val) == 0 {
		return ""
	}
	return string(val)
}

func generateToken(bytes int) string {
	b := make([]byte, bytes)
	rand.Read(b)
	return hex.EncodeToString(b)
}
