package api

import (
	"encoding/base64"
	"log"
	"net/http"
	"sync"
	"time"

	"github.com/cedbossneo/mowglinext/app/proxy/auth"
	tunnelpkg "github.com/cedbossneo/mowglinext/app/proxy/tunnel"
	"github.com/gin-gonic/gin"
	"github.com/gorilla/websocket"
	"go.mongodb.org/mongo-driver/v2/bson"
)

var wsUpgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool { return true },
}

var allowedSubscribeTopics = map[string]bool{
	"status":          true,
	"highLevelStatus": true,
	"power":           true,
	"gps":             true,
	"pose":            true,
	"emergency":       true,
	"imu":             true,
	"wheelTicks":      true,
	"map":             true,
	"path":            true,
	"obstacles":       true,
}

var allowedPublishTopics = map[string]time.Duration{
	"cmd_vel": 100 * time.Millisecond,
}

func (s *Server) handleWSSubscribe(c *gin.Context) {
	topic := c.Param("topic")
	robotID := c.Param("id")

	if !allowedSubscribeTopics[topic] {
		c.JSON(http.StatusForbidden, gin.H{"error": "topic not allowed"})
		return
	}

	userConn, err := wsUpgrader.Upgrade(c.Writer, c.Request, nil)
	if err != nil {
		log.Printf("WS upgrade error: %v", err)
		return
	}
	defer userConn.Close()

	tunnelConn, err := s.tm.Get(robotID)
	if err != nil {
		userConn.WriteJSON(gin.H{"error": "robot not connected"})
		return
	}

	streamID, ch, err := tunnelConn.Subscribe(topic)
	if err != nil {
		userConn.WriteJSON(gin.H{"error": "subscribe failed"})
		return
	}
	defer tunnelConn.Unsubscribe(streamID)

	done := make(chan struct{})
	go func() {
		defer close(done)
		for {
			_, _, err := userConn.ReadMessage()
			if err != nil {
				return
			}
		}
	}()

	for {
		select {
		case frame, ok := <-ch:
			if !ok {
				return
			}
			if err := userConn.WriteMessage(websocket.TextMessage, []byte(frame.Body)); err != nil {
				return
			}
		case <-done:
			return
		}
	}
}

func (s *Server) handleWSPublish(c *gin.Context) {
	topic := c.Param("topic")
	robotID := c.Param("id")

	rateLimit, allowed := allowedPublishTopics[topic]
	if !allowed {
		c.JSON(http.StatusForbidden, gin.H{"error": "topic not allowed for publish"})
		return
	}

	userConn, err := wsUpgrader.Upgrade(c.Writer, c.Request, nil)
	if err != nil {
		log.Printf("WS upgrade error: %v", err)
		return
	}
	defer userConn.Close()

	tunnelConn, err := s.tm.Get(robotID)
	if err != nil {
		userConn.WriteJSON(gin.H{"error": "robot not connected"})
		return
	}

	var lastSend time.Time
	var mu sync.Mutex

	for {
		_, msg, err := userConn.ReadMessage()
		if err != nil {
			return
		}

		mu.Lock()
		if time.Since(lastSend) < rateLimit {
			mu.Unlock()
			continue
		}
		lastSend = time.Now()
		mu.Unlock()

		encoded := base64.StdEncoding.EncodeToString(msg)
		if err := tunnelConn.Publish(topic, encoded); err != nil {
			return
		}
	}
}

func (s *Server) handleTunnelConnect(c *gin.Context) {
	token := c.Query("token")
	robotID := c.Query("robot_id")
	if token == "" || robotID == "" {
		c.JSON(http.StatusUnauthorized, gin.H{"error": "missing token or robot_id"})
		return
	}

	// Verify robot token
	ctx := c.Request.Context()
	robotObjID, err := bson.ObjectIDFromHex(robotID)
	if err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": "invalid robot_id"})
		return
	}

	robot, err := s.store.GetRobotByID(ctx, robotObjID)
	if err != nil {
		c.JSON(http.StatusNotFound, gin.H{"error": "robot not found"})
		return
	}

	// Verify token against stored hash
	if !auth.CheckPassword(token, robot.Token) {
		c.JSON(http.StatusUnauthorized, gin.H{"error": "invalid token"})
		return
	}

	ws, err := wsUpgrader.Upgrade(c.Writer, c.Request, nil)
	if err != nil {
		log.Printf("Tunnel WS upgrade error: %v", err)
		return
	}

	log.Printf("Robot %s connected via tunnel", robotID)

	conn := tunnelpkg.NewConnection(robotID, ws)
	s.tm.Register(robotID, conn)

	// Update last seen
	s.store.UpdateRobotLastSeen(ctx, robot.ID)

	// Block until connection closes
	<-conn.Done()
	s.tm.Remove(robotID)
	log.Printf("Robot %s disconnected", robotID)
}
