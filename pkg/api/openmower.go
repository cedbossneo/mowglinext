package api

import (
	"encoding/base64"
	"encoding/json"
	"errors"
	"log"
	"net/http"
	"time"

	"github.com/cedbossneo/openmower-gui/pkg/msgs/geometry"
	"github.com/cedbossneo/openmower-gui/pkg/msgs/mowgli"
	"github.com/cedbossneo/openmower-gui/pkg/types"
	"github.com/docker/distribution/uuid"
	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
)

var upgrader = websocket.Upgrader{
	ReadBufferSize:  1024,
	WriteBufferSize: 1024,
	CheckOrigin:     func(r *http.Request) bool { return true },
}

func OpenMowerRoutes(r *gin.RouterGroup, provider types.IRosProvider) {
	group := r.Group("/openmower")
	ServiceRoute(group, provider)
	AddMapAreaRoute(group, provider)
	SetDockingPointRoute(group, provider)
	ClearMapRoute(group, provider)
	ReplaceMapRoute(group, provider)
	SubscriberRoute(group, provider)
	PublisherRoute(group, provider)
}

// AddMapAreaRoute add a map area
//
// @Summary add a map area
// @Description add a map area
// @Tags openmower
// @Accept  json
// @Produce  json
// @Param CallReq body mowgli.AddMowingAreaReq true "request body"
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /openmower/map/area/add [post]
func AddMapAreaRoute(group *gin.RouterGroup, provider types.IRosProvider) {
	group.POST("/map/area/add", func(c *gin.Context) {
		var CallReq mowgli.AddMowingAreaReq
		err := unmarshalROSMessage[*mowgli.AddMowingAreaReq](c.Request.Body, &CallReq)
		if err != nil {
			return
		}
		err = provider.CallService(c.Request.Context(), "/map_server_node/add_area", &CallReq, &mowgli.AddMowingAreaRes{})
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
		} else {
			c.JSON(200, OkResponse{})
		}
	})
}

// ClearMapRoute delete a map area
//
// @Summary clear the map
// @Description clear the map
// @Tags openmower
// @Accept  json
// @Produce  json
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /openmower/map [delete]
func ClearMapRoute(group *gin.RouterGroup, provider types.IRosProvider) {
	group.DELETE("/map", func(c *gin.Context) {
		err := provider.CallService(c.Request.Context(), "/map_server_node/clear_map", &mowgli.ClearMapReq{}, &mowgli.ClearMapRes{})
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
		} else {
			c.JSON(200, OkResponse{})
		}
	})
}

// ReplaceMapRoute clear the map and insert areas
//
// @Summary clear the map and insert areas
// @Description clear the map and insert areas
// @Tags openmower
// @Accept  json
// @Produce  json
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /openmower/map [put]
func ReplaceMapRoute(group *gin.RouterGroup, provider types.IRosProvider) {
	group.PUT("/map", func(c *gin.Context) {
		err := provider.CallService(c.Request.Context(), "/map_server_node/clear_map", &mowgli.ClearMapReq{}, &mowgli.ClearMapRes{})
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		} else {
			var CallReq mowgli.ReplaceMapReq
			err := unmarshalROSMessage[*mowgli.ReplaceMapReq](c.Request.Body, &CallReq)
			if err != nil {
				c.JSON(500, ErrorResponse{Error: err.Error()})
				return
			}
			for _, element := range CallReq.Areas {
				areaReq := mowgli.AddMowingAreaReq{
					Area:             element.Area,
					IsNavigationArea: element.IsNavigationArea,
				}
				err = provider.CallService(c.Request.Context(), "/map_server_node/add_area", &areaReq, &mowgli.AddMowingAreaRes{})
				if err != nil {
					c.JSON(500, ErrorResponse{Error: err.Error()})
					return
				}
			}

			c.JSON(200, OkResponse{})
		}
	})
}

// SetDockingPointRoute set the docking point
//
// @Summary set the docking point
// @Description set the docking point
// @Tags openmower
// @Accept  json
// @Produce  json
// @Param CallReq body mowgli.SetDockingPointReq true "request body"
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /openmower/map/docking [post]
func SetDockingPointRoute(group *gin.RouterGroup, provider types.IRosProvider) {
	group.POST("/map/docking", func(c *gin.Context) {
		var CallReq mowgli.SetDockingPointReq
		err := unmarshalROSMessage[*mowgli.SetDockingPointReq](c.Request.Body, &CallReq)
		if err != nil {
			return
		}
		err = provider.CallService(c.Request.Context(), "/map_server_node/set_docking_point", &CallReq, &mowgli.SetDockingPointRes{})
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
		} else {
			c.JSON(200, OkResponse{})
		}
	})
}

// SubscriberRoute subscribe to a topic
//
// @Summary subscribe to a topic
// @Description subscribe to a topic
// @Tags openmower
// @Param topic path string true "logical topic key: diagnostics, status, highLevelStatus, gps, pose, imu, ticks, map, path, plan, mowingPath, power, emergency, dockingSensor, lidar"
// @Router /openmower/subscribe/{topic} [get]
func SubscriberRoute(group *gin.RouterGroup, provider types.IRosProvider) {
	group.GET("/subscribe/:topic", func(c *gin.Context) {
		var err error
		topic := c.Param("topic")
		conn, err := upgrader.Upgrade(c.Writer, c.Request, nil)
		if err != nil {
			return
		}
		defer conn.Close()

		var def func()
		switch topic {
		case "diagnostics":
			def, err = subscribe(provider, c, conn, "diagnostics", -1)
		case "status":
			def, err = subscribe(provider, c, conn, "status", -1)
		case "highLevelStatus":
			def, err = subscribe(provider, c, conn, "highLevelStatus", -1)
		case "gps":
			def, err = subscribe(provider, c, conn, "gps", 100)
		case "pose":
			def, err = subscribe(provider, c, conn, "pose", 100)
		case "imu":
			def, err = subscribe(provider, c, conn, "imu", 100)
		case "ticks":
			def, err = subscribe(provider, c, conn, "ticks", 100)
		case "map":
			def, err = subscribe(provider, c, conn, "map", -1)
		case "path":
			def, err = subscribe(provider, c, conn, "path", -1)
		case "plan":
			def, err = subscribe(provider, c, conn, "plan", -1)
		case "mowingPath":
			def, err = subscribe(provider, c, conn, "mowingPath", -1)
		case "power":
			def, err = subscribe(provider, c, conn, "power", -1)
		case "emergency":
			def, err = subscribe(provider, c, conn, "emergency", -1)
		case "dockingSensor":
			def, err = subscribe(provider, c, conn, "dockingSensor", -1)
		case "lidar":
			def, err = subscribe(provider, c, conn, "lidar", 100)
		}
		if err != nil {
			log.Println(err.Error())
			return
		}
		defer def()

		_, _, err = conn.ReadMessage()
		if err != nil {
			c.Error(err)
			return
		}
	})
}

// PublisherRoute publish to a topic
//
// @Summary publish to a topic
// @Description publish to a topic
// @Tags openmower
// @Param topic path string true "topic to publish to, could be: joy"
// @Router /openmower/publish/{topic} [get]
func PublisherRoute(group *gin.RouterGroup, provider types.IRosProvider) {
	group.GET("/publish/:topic", func(c *gin.Context) {
		var err error
		conn, err := upgrader.Upgrade(c.Writer, c.Request, nil)
		if err != nil {
			return
		}
		defer conn.Close()
		for {
			_, msg, err := conn.ReadMessage()
			if err != nil {
				c.Error(err)
				break
			}
			var msgObj geometry.Twist
			err = json.Unmarshal(msg, &msgObj)
			if err != nil {
				c.Error(err)
				break
			}
			err = provider.Publish("/cmd_vel", "geometry_msgs/msg/Twist", &msgObj)
			if err != nil {
				c.Error(err)
				break
			}
		}
	})
}

func subscribe(provider types.IRosProvider, c *gin.Context, conn *websocket.Conn, topic string, interval int) (func(), error) {
	id := uuid.Generate()
	uidString := id.String()
	err := provider.Subscribe(topic, uidString, func(msg []byte) {
		if interval > 0 {
			time.Sleep(time.Duration(interval) * time.Millisecond)
		}
		writer, err := conn.NextWriter(websocket.TextMessage)
		if err != nil {
			c.Error(err)
			return
		}
		_, err = writer.Write([]byte(base64.StdEncoding.EncodeToString(msg)))
		if err != nil {
			c.Error(err)
			return
		}
		err = writer.Close()
		if err != nil {
			c.Error(err)
			return
		}
	},
	)
	if err != nil {
		return nil, err
	}
	return func() {
		provider.UnSubscribe(topic, uidString)
	}, nil
}

// ServiceRoute call a service
//
// @Summary call a service
// @Description call a service
// @Tags openmower
// @Accept  json
// @Produce  json
// @Param command path string true "command to call, could be: high_level_control, emergency, mow_enabled, start_in_area"
// @Param CallReq body map[string]interface{} true "request body"
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /openmower/call/{command} [post]
func ServiceRoute(group *gin.RouterGroup, provider types.IRosProvider) {
	group.POST("/call/:command", func(c *gin.Context) {
		command := c.Param("command")
		var err error
		switch command {
		case "high_level_control":
			var CallReq mowgli.HighLevelControlReq
			err = c.BindJSON(&CallReq)
			if err != nil {
				return
			}
			err = provider.CallService(c.Request.Context(), "/behavior_tree_node/high_level_control", &CallReq, &mowgli.HighLevelControlRes{})
		case "emergency":
			var CallReq mowgli.EmergencyStopReq
			err = c.BindJSON(&CallReq)
			if err != nil {
				return
			}
			err = provider.CallService(c.Request.Context(), "/hardware_bridge/emergency_stop", &CallReq, &mowgli.EmergencyStopRes{})
		case "mow_enabled":
			var CallReq mowgli.MowerControlReq
			err = c.BindJSON(&CallReq)
			if err != nil {
				return
			}
			err = provider.CallService(c.Request.Context(), "/hardware_bridge/mower_control", &CallReq, &mowgli.MowerControlRes{})
		case "start_in_area":
			var CallReq mowgli.StartInAreaReq
			err = c.BindJSON(&CallReq)
			if err != nil {
				return
			}
			err = provider.CallService(c.Request.Context(), "/behavior_tree_node/start_in_area", &CallReq, &mowgli.StartInAreaRes{})
		default:
			err = errors.New("unknown command")
		}
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
		} else {
			c.JSON(200, OkResponse{})
		}
	})
}
