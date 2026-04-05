package main

import (
	"log"
	"os"
	"os/signal"
	"syscall"

	"github.com/cedbossneo/mowglinext/app/agent/config"
	"github.com/cedbossneo/mowglinext/app/agent/notify"
	"github.com/cedbossneo/mowglinext/app/agent/tunnel"
)

func main() {
	cfg := config.Load()

	if cfg.ProxyURL == "" || cfg.RobotToken == "" {
		log.Fatal("PROXY_URL and ROBOT_TOKEN must be set")
	}

	client := tunnel.NewClient(cfg)

	// Start notification detector
	detector := notify.NewDetector(cfg.LocalGUIURL, client)
	go detector.Start()

	// Connect to proxy (blocking, reconnects automatically)
	go client.Connect()

	log.Printf("Tunnel agent started, connecting to %s", cfg.ProxyURL)

	// Wait for shutdown
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit
	log.Println("Shutting down tunnel agent...")
	client.Close()
}
