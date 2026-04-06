package main

import (
	"context"
	"log"
	"os"
	"os/signal"
	"syscall"

	"github.com/cedbossneo/mowglinext/app/proxy/api"
	"github.com/cedbossneo/mowglinext/app/proxy/config"
	"github.com/cedbossneo/mowglinext/app/proxy/store"
	"github.com/cedbossneo/mowglinext/app/proxy/tunnel"
)

func main() {
	cfg := config.Load()

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	// Connect to MongoDB
	mongoStore, err := store.NewMongoStore(ctx, cfg.MongoURI)
	if err != nil {
		log.Fatalf("Failed to connect to MongoDB: %v", err)
	}
	defer mongoStore.Close(ctx)

	// Create indexes
	if err := mongoStore.EnsureIndexes(ctx); err != nil {
		log.Fatalf("Failed to create indexes: %v", err)
	}

	// Tunnel manager
	tm := tunnel.NewManager()

	// Start API server
	srv := api.NewServer(cfg, mongoStore, tm)
	go func() {
		if err := srv.Run(":" + cfg.Port); err != nil {
			log.Fatalf("Server failed: %v", err)
		}
	}()

	log.Printf("Proxy server listening on :%s", cfg.Port)

	// Graceful shutdown
	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit
	log.Println("Shutting down...")
	cancel()
}
