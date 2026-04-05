package api

import (
	"net/http"

	"github.com/cedbossneo/mowglinext/app/proxy/store"
	"github.com/gin-gonic/gin"
	"go.mongodb.org/mongo-driver/v2/bson"
)

type registerPushRequest struct {
	RobotID  string `json:"robot_id" binding:"required"`
	Platform string `json:"platform" binding:"required"`
	Token    string `json:"token" binding:"required"`
}

func (s *Server) handleRegisterPush(c *gin.Context) {
	var req registerPushRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	if req.Platform != "ios" && req.Platform != "android" {
		c.JSON(http.StatusBadRequest, gin.H{"error": "platform must be ios or android"})
		return
	}

	userID, _ := c.Get("user_id")
	userObjID, err := bson.ObjectIDFromHex(userID.(string))
	if err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": "invalid user"})
		return
	}

	robotObjID, err := bson.ObjectIDFromHex(req.RobotID)
	if err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": "invalid robot_id"})
		return
	}

	pushToken := &store.PushToken{
		UserID:   userObjID,
		RobotID:  robotObjID,
		Platform: req.Platform,
		Token:    req.Token,
	}

	if err := s.store.UpsertPushToken(c.Request.Context(), pushToken); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "failed to register push token"})
		return
	}

	c.JSON(http.StatusOK, gin.H{"ok": "push token registered"})
}
