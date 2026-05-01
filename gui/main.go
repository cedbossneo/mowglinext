// MowgliNext GUI API
//
// @title MowgliNext GUI API
// @version 1.0
// @description API for the MowgliNext autonomous robot mower GUI
// @host localhost:4200
// @BasePath /api
package main

import (
	"context"
	"os"
	"os/signal"
	"syscall"

	"github.com/cedbossneo/mowglinext/pkg/api"
	"github.com/cedbossneo/mowglinext/pkg/mobile"
	"github.com/cedbossneo/mowglinext/pkg/providers"
	"github.com/joho/godotenv"
	"github.com/sirupsen/logrus"
)

func main() {
	_ = godotenv.Load()

	dbProvider := providers.NewDBProvider()
	dockerProvider := providers.NewDockerProvider()
	rosProvider := providers.NewRosProvider(dbProvider)
	firmwareProvider := providers.NewFirmwareProvider(dbProvider)
	homekitEnabled, err := dbProvider.Get("system.homekit.enabled")
	if err != nil {
		panic(err)
	}
	if string(homekitEnabled) == "true" {
		providers.NewHomeKitProvider(rosProvider, dbProvider)
	}
	mqttEnabled, err := dbProvider.Get("system.mqtt.enabled")
	if err != nil {
		panic(err)
	}
	if string(mqttEnabled) == "true" {
		providers.NewMqttProvider(rosProvider, dbProvider)
	}
	providers.NewSchedulerProvider(rosProvider, dbProvider)

	// Mobile companion tunnel — opt-in. When MOWGLI_MOBILE_TUNNEL=1 is set,
	// noise_shim + pairing + cloud_sync are wired into the same Gin engine
	// the existing GUI uses. Default deployments are unaffected.
	if os.Getenv("MOWGLI_MOBILE_TUNNEL") == "1" {
		ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
		defer cancel()

		res, err := mobile.Setup(ctx, mobile.Config{
			StateDir:          os.Getenv("MOWGLI_STATE_DIR"),
			LocalAPIURL:       os.Getenv("MOWGLI_LOCAL_API_URL"),
			LANAddr:           os.Getenv("MOWGLI_LAN_ADDR"),
			TunnelDomain:      envOr("MOWGLI_TUNNEL_DOMAIN", "tunnel.mowgli.garden"),
			FirebaseProjectID: os.Getenv("MOWGLI_FIREBASE_PROJECT_ID"),
			FirebaseAdminJSON: os.Getenv("MOWGLI_FIREBASE_ADMIN_JSON"),
		})
		if err != nil {
			logrus.WithError(err).Fatal("mobile tunnel setup failed")
		}
		defer res.Cleanup()
		api.NewAPIWith(dbProvider, dockerProvider, rosProvider, firmwareProvider, res.Extension)
		return
	}

	api.NewAPI(dbProvider, dockerProvider, rosProvider, firmwareProvider)
}

func envOr(key, fallback string) string {
	if v := os.Getenv(key); v != "" {
		return v
	}
	return fallback
}
