package api

import (
	"net/http"
	"time"

	"github.com/cedbossneo/mowglinext/app/proxy/auth"
	"github.com/cedbossneo/mowglinext/app/proxy/store"
	"github.com/gin-gonic/gin"
	"go.mongodb.org/mongo-driver/v2/bson"
)

type loginRequest struct {
	Email    string `json:"email" binding:"required"`
	Password string `json:"password" binding:"required"`
}

type registerRequest struct {
	Email    string `json:"email" binding:"required"`
	Password string `json:"password" binding:"required,min=8"`
	Name     string `json:"name" binding:"required"`
}

type refreshRequest struct {
	RefreshToken string `json:"refresh_token" binding:"required"`
}

func (s *Server) handleRegister(c *gin.Context) {
	var req registerRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	hash, err := auth.HashPassword(req.Password)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "failed to hash password"})
		return
	}

	user := &store.User{
		Email:    req.Email,
		Password: hash,
		Name:     req.Name,
	}
	if err := s.store.CreateUser(c.Request.Context(), user); err != nil {
		c.JSON(http.StatusConflict, gin.H{"error": "email already registered"})
		return
	}

	c.JSON(http.StatusCreated, gin.H{"id": user.ID.Hex(), "email": user.Email, "name": user.Name})
}

func (s *Server) handleLogin(c *gin.Context) {
	var req loginRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	user, err := s.store.GetUserByEmail(c.Request.Context(), req.Email)
	if err != nil || !auth.CheckPassword(req.Password, user.Password) {
		c.JSON(http.StatusUnauthorized, gin.H{"error": "invalid credentials"})
		return
	}

	// Get user's robots for JWT claims
	robots, _ := s.store.GetRobotsByOwner(c.Request.Context(), user.ID)
	robotIDs := make([]string, len(robots))
	for i, r := range robots {
		robotIDs[i] = r.ID.Hex()
	}

	tokenPair, refreshStr, err := auth.GenerateTokenPair(s.cfg.JWTSecret, user.ID.Hex(), robotIDs)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "failed to generate tokens"})
		return
	}

	// Store refresh session
	session := &store.Session{
		UserID:       user.ID,
		RefreshToken: refreshStr,
		ExpiresAt:    time.Now().Add(30 * 24 * time.Hour),
	}
	if err := s.store.CreateSession(c.Request.Context(), session); err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "failed to create session"})
		return
	}

	c.JSON(http.StatusOK, gin.H{
		"access_token":  tokenPair.AccessToken,
		"refresh_token": tokenPair.RefreshToken,
		"expires_in":    tokenPair.ExpiresIn,
		"robots":        robots,
	})
}

func (s *Server) handleRefresh(c *gin.Context) {
	var req refreshRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": err.Error()})
		return
	}

	session, err := s.store.GetSessionByRefreshToken(c.Request.Context(), req.RefreshToken)
	if err != nil {
		c.JSON(http.StatusUnauthorized, gin.H{"error": "invalid refresh token"})
		return
	}

	if time.Now().After(session.ExpiresAt) {
		s.store.DeleteSession(c.Request.Context(), session.ID)
		c.JSON(http.StatusUnauthorized, gin.H{"error": "refresh token expired"})
		return
	}

	// Delete old session
	s.store.DeleteSession(c.Request.Context(), session.ID)

	// Get user and robots
	user, err := s.store.GetUserByID(c.Request.Context(), session.UserID)
	if err != nil {
		c.JSON(http.StatusUnauthorized, gin.H{"error": "user not found"})
		return
	}

	robots, _ := s.store.GetRobotsByOwner(c.Request.Context(), user.ID)
	robotIDs := make([]string, len(robots))
	for i, r := range robots {
		robotIDs[i] = r.ID.Hex()
	}

	tokenPair, refreshStr, err := auth.GenerateTokenPair(s.cfg.JWTSecret, user.ID.Hex(), robotIDs)
	if err != nil {
		c.JSON(http.StatusInternalServerError, gin.H{"error": "failed to generate tokens"})
		return
	}

	// Store new refresh session
	newSession := &store.Session{
		UserID:       user.ID,
		RefreshToken: refreshStr,
		ExpiresAt:    time.Now().Add(30 * 24 * time.Hour),
	}
	s.store.CreateSession(c.Request.Context(), newSession)

	c.JSON(http.StatusOK, gin.H{
		"access_token":  tokenPair.AccessToken,
		"refresh_token": tokenPair.RefreshToken,
		"expires_in":    tokenPair.ExpiresIn,
	})
}

func (s *Server) handleLogout(c *gin.Context) {
	userID, _ := c.Get("user_id")
	objID, err := bson.ObjectIDFromHex(userID.(string))
	if err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": "invalid user"})
		return
	}
	s.store.DeleteSessionsByUser(c.Request.Context(), objID)
	c.JSON(http.StatusOK, gin.H{"ok": "logged out"})
}

func (s *Server) handleGetMe(c *gin.Context) {
	userID, _ := c.Get("user_id")
	objID, err := bson.ObjectIDFromHex(userID.(string))
	if err != nil {
		c.JSON(http.StatusBadRequest, gin.H{"error": "invalid user"})
		return
	}
	user, err := s.store.GetUserByID(c.Request.Context(), objID)
	if err != nil {
		c.JSON(http.StatusNotFound, gin.H{"error": "user not found"})
		return
	}
	c.JSON(http.StatusOK, gin.H{"id": user.ID.Hex(), "email": user.Email, "name": user.Name})
}
