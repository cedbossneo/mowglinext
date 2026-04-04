package api

import (
	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
)

func ConfigRoute(r *gin.RouterGroup, db types.IDBProvider) {
	ConfigEnvRoute(r, db)
	ConfigGetKeysRoute(r, db)
	ConfigSetKeysRoute(r, db)
}

// ConfigGetKeysRoute get config from backend
//
// @Summary get config from backend
// @Description get config from backend
// @Tags config
// @Produce  json
// @Param settings body map[string]string true "settings"
// @Success 200 {object} map[string]string
// @Failure 500 {object} ErrorResponse
// @Router /config/keys/get [post]
func ConfigGetKeysRoute(r *gin.RouterGroup, db types.IDBProvider) gin.IRoutes {
	return r.POST("/config/keys/get", func(context *gin.Context) {
		var body gin.H
		err := context.BindJSON(&body)
		if err != nil {
			context.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		for key, _ := range body {
			get, err := db.Get(key)
			if err != nil {
				continue
			}
			body[key] = string(get)
		}

		context.JSON(200, body)
	})
}

// ConfigSetKeysRoute set config to backend
//
// @Summary set config to backend
// @Description set config to backend
// @Tags config
// @Produce  json
// @Param settings body map[string]string true "settings"
// @Success 200 {object} map[string]string
// @Failure 500 {object} ErrorResponse
// @Router /config/keys/set [post]
func ConfigSetKeysRoute(r *gin.RouterGroup, db types.IDBProvider) gin.IRoutes {
	return r.POST("/config/keys/set", func(context *gin.Context) {
		var body gin.H
		err := context.BindJSON(&body)
		if err != nil {
			context.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		for key, value := range body {
			err := db.Set(key, []byte(value.(string)))
			if err != nil {
				continue
			}
		}
		context.JSON(200, body)
	})
}

// ConfigEnvRoute get config from backend
//
// @Summary get config env from backend
// @Description get config env from backend
// @Tags config
// @Produce  json
// @Produce  json
// @Success 200 {object} GetConfigResponse
// @Failure 500 {object} ErrorResponse
// @Router /config/envs [get]
func ConfigEnvRoute(r *gin.RouterGroup, db types.IDBProvider) gin.IRoutes {
	return r.GET("/config/envs", func(context *gin.Context) {
		tileUri, err := db.Get("system.map.tileUri")
		if err != nil {
			context.JSON(500, ErrorResponse{
				Error: err.Error(),
			})
			return
		}
		context.JSON(200, GetConfigResponse{
			TileUri: string(tileUri),
		})
	})
}
