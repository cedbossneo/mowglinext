package api

import (
	"github.com/cedbossneo/mowglinext/docs"
	"github.com/cedbossneo/mowglinext/pkg/providers"
	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-contrib/cors"
	"github.com/gin-contrib/static"
	"github.com/gin-gonic/gin"
	swaggerfiles "github.com/swaggo/files"
	ginSwagger "github.com/swaggo/gin-swagger"
	"log"
)

// gin-swagger middleware
// swagger embed files

// Extension is an optional hook invoked after all built-in routes are
// registered but before the engine starts listening. It receives both the
// raw engine (so callers can mount handlers OUTSIDE /api, e.g. /tunnel)
// and the /api group (so callers can mount additional API endpoints).
type Extension func(r *gin.Engine, apiGroup *gin.RouterGroup)

// NewAPI keeps the existing signature so main.go and any external callers
// continue to work unchanged. It is a thin wrapper around NewAPIWith.
func NewAPI(dbProvider types.IDBProvider, dockerProvider types.IDockerProvider, rosProvider types.IRosProvider, firmwareProvider *providers.FirmwareProvider) {
	NewAPIWith(dbProvider, dockerProvider, rosProvider, firmwareProvider)
}

// NewAPIWith builds and runs the API server, applying any provided
// extensions before the engine starts. main.go uses this to inject the
// mobile tunnel (Noise shim + pairing routes) when MOWGLI_MOBILE_TUNNEL=1
// without altering the behaviour of the default deployment.
func NewAPIWith(dbProvider types.IDBProvider, dockerProvider types.IDockerProvider, rosProvider types.IRosProvider, firmwareProvider *providers.FirmwareProvider, extensions ...Extension) {
	httpAddr, err := dbProvider.Get("system.api.addr")
	if err != nil {
		log.Fatal(err)
	}

	gin.SetMode(gin.ReleaseMode)
	docs.SwaggerInfo.BasePath = "/api"
	r := gin.Default()
	config := cors.DefaultConfig()
	config.AllowAllOrigins = true
	config.AllowWebSockets = true
	r.Use(cors.New(config))
	webDirectory, err := dbProvider.Get("system.api.webDirectory")
	if err != nil {
		log.Fatal(err)
	}
	r.Use(static.Serve("/", static.LocalFile(string(webDirectory), false)))
	apiGroup := r.Group("/api")
	ConfigRoute(apiGroup, dbProvider)
	SettingsRoutes(apiGroup, dbProvider)
	ContainersRoutes(apiGroup, dockerProvider)
	MowgliNextRoutes(apiGroup, rosProvider)
	SetupRoutes(apiGroup, firmwareProvider)
	SystemRoutes(apiGroup)
	DiagnosticsRoutes(apiGroup, dockerProvider, rosProvider, dbProvider)
	CalibrationRoutes(apiGroup, rosProvider)
	ScheduleRoutes(apiGroup, dbProvider)
	tileServer, err := dbProvider.Get("system.map.enabled")
	if err != nil {
		log.Fatal(err)
	}
	if string(tileServer) == "true" {
		TilesProxy(r, dbProvider)
	}
	r.GET("/swagger/*any", ginSwagger.WrapHandler(swaggerfiles.Handler))

	for _, ext := range extensions {
		if ext != nil {
			ext(r, apiGroup)
		}
	}

	r.Run(string(httpAddr))
}
