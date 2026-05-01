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
	"github.com/cedbossneo/mowglinext/pkg/types"
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

	// Mobile companion tunnel — opt-in. The toggle lives in
	// system.mobile.enabled (set via the GUI's Pair-Mobile-App wizard step
	// or POST /api/config/system.mobile.enabled). When enabled, noise_shim
	// + pairing + cloud_sync are wired into the same Gin engine the
	// existing GUI uses. Env vars are honoured as deployment-time
	// overrides via DBProvider.EnvFallbacks (handy for CI / headless).
	mobileEnabledRaw, _ := dbProvider.Get("system.mobile.enabled")
	if string(mobileEnabledRaw) == "true" {
		ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
		defer cancel()

		res, err := mobile.Setup(ctx, mobile.Config{
			StateDir:          dbStr(dbProvider, "system.mobile.stateDir"),
			LocalAPIURL:       dbStr(dbProvider, "system.mobile.localApiUrl"),
			LANAddr:           dbStr(dbProvider, "system.mobile.lanAddr"),
			TunnelDomain:      dbStr(dbProvider, "system.mobile.tunnelDomain"),
			FirebaseProjectID: dbStr(dbProvider, "system.mobile.firebaseProjectId"),
			FirebaseAdminJSON: dbStr(dbProvider, "system.mobile.firebaseAdminJson"),
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

// dbStr is a small helper that swallows lookup errors and returns the
// (string-coerced) value. Used only for configuration reads where missing
// values are equivalent to the empty string and downstream code applies
// its own defaults.
func dbStr(db types.IDBProvider, key string) string {
	v, _ := db.Get(key)
	return string(v)
}
