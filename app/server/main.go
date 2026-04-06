package main

import (
	"github.com/cedbossneo/mowglinext/app/server/pkg/api"
	"github.com/cedbossneo/mowglinext/app/server/pkg/providers"
	"github.com/joho/godotenv"
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
	tunnelEnabled, err := dbProvider.Get("system.tunnel.enabled")
	if err != nil {
		panic(err)
	}
	if string(tunnelEnabled) == "true" {
		providers.NewTunnelProvider(dbProvider)
	}
	api.NewAPI(dbProvider, dockerProvider, rosProvider, firmwareProvider)
}
