package api

import (
	"github.com/cedbossneo/mowglinext/app/proxy/auth"
	"github.com/cedbossneo/mowglinext/app/proxy/config"
	"github.com/cedbossneo/mowglinext/app/proxy/store"
	"github.com/cedbossneo/mowglinext/app/proxy/tunnel"
	"github.com/gin-gonic/gin"
)

type Server struct {
	engine *gin.Engine
	cfg    *config.Config
	store  store.Store
	tm     *tunnel.Manager
}

func NewServer(cfg *config.Config, s store.Store, tm *tunnel.Manager) *Server {
	srv := &Server{
		engine: gin.Default(),
		cfg:    cfg,
		store:  s,
		tm:     tm,
	}
	srv.setupRoutes()
	return srv
}

func (s *Server) setupRoutes() {
	// Public routes
	s.engine.POST("/api/auth/register", s.handleRegister)
	s.engine.POST("/api/auth/login", s.handleLogin)
	s.engine.POST("/api/auth/refresh", s.handleRefresh)

	// Tunnel endpoint (robot auth via token)
	s.engine.GET("/tunnel/connect", s.handleTunnelConnect)

	// Authenticated routes
	protected := s.engine.Group("/api")
	protected.Use(auth.JWTMiddleware(s.cfg.JWTSecret))
	{
		// User routes
		protected.GET("/user/me", s.handleGetMe)
		protected.POST("/auth/logout", s.handleLogout)

		// Robot management
		protected.POST("/robots", s.handleCreateRobot)
		protected.GET("/robots", s.handleListRobots)

		// Push notification registration
		protected.POST("/push/register", s.handleRegisterPush)

		// Robot-specific routes (with access check)
		robot := protected.Group("/robots/:id")
		robot.Use(auth.RobotAccess())
		{
			// HTTP proxy to robot
			robot.Any("/openmower/call/:command", s.handleProxyCall)
			robot.GET("/schedules", s.handleProxyHTTP)
			robot.POST("/schedules", s.handleProxyHTTP)
			robot.PUT("/schedules/:scheduleId", s.handleProxyHTTP)
			robot.DELETE("/schedules/:scheduleId", s.handleProxyHTTP)
			robot.GET("/settings/status", s.handleProxyHTTP)
			robot.GET("/system/info", s.handleProxyHTTP)

			// WS relay to robot
			robot.GET("/openmower/subscribe/:topic", s.handleWSSubscribe)
			robot.GET("/openmower/publish/:topic", s.handleWSPublish)
		}
	}
}

func (s *Server) Run(addr string) error {
	return s.engine.Run(addr)
}
