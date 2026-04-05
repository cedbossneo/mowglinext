package api

import (
	"crypto/rand"
	"encoding/hex"
	"net/http"

	"github.com/cedbossneo/mowglinext/app/proxy/auth"
	"github.com/cedbossneo/mowglinext/app/proxy/store"
	"github.com/gin-gonic/gin"
	"go.mongodb.org/mongo-driver/v2/bson"
)

type createRobotRequest struct {
	Name string `json:"name" binding:"required"`
}

func (s *Server) handleCreateRobot(c *gin.Context) {
	var req createRobotRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	userID, _ := c.Get("user_id")
	ownerID, err := bson.ObjectIDFromHex(userID.(string))
	if err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": "invalid user"})
		return
	}

	// Generate robot token
	tokenBytes := make([]byte, 32)
	if _, err := rand.Read(tokenBytes); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "failed to generate token"})
		return
	}
	rawToken := hex.EncodeToString(tokenBytes)
	tokenHash, err := auth.HashPassword(rawToken)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "failed to hash token"})
		return
	}

	robot := &store.Robot{
		Name:    req.Name,
		Token:   tokenHash,
		OwnerID: ownerID,
	}
	if err := s.store.CreateRobot(c.Request.Context(), robot); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "failed to create robot"})
		return
	}

	// Return the raw token only once — user must save it
	c.JSON(http.StatusCreated, gin.H{
		"id":    robot.ID.Hex(),
		"name":  robot.Name,
		"token": rawToken,
	})
}

func (s *Server) handleListRobots(c *gin.Context) {
	userID, _ := c.Get("user_id")
	ownerID, err := bson.ObjectIDFromHex(userID.(string))
	if err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": "invalid user"})
		return
	}

	robots, err := s.store.GetRobotsByOwner(c.Request.Context(), ownerID)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "failed to list robots"})
		return
	}

	// Enrich with online status
	type robotResponse struct {
		store.Robot
		Online bool `json:"online"`
	}
	result := make([]robotResponse, len(robots))
	for i, r := range robots {
		result[i] = robotResponse{
			Robot:  r,
			Online: s.tm.IsOnline(r.ID.Hex()),
		}
	}

	c.JSON(http.StatusOK, result)
}
