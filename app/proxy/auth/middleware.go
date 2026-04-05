package auth

import (
	"net/http"
	"strings"

	"github.com/gin-gonic/gin"
)

func JWTMiddleware(secret string) gin.HandlerFunc {
	return func(c *gin.Context) {
		var tokenStr string

		// Check Authorization header
		auth := c.GetHeader("Authorization")
		if strings.HasPrefix(auth, "Bearer ") {
			tokenStr = strings.TrimPrefix(auth, "Bearer ")
		}

		// Check query param (for WebSocket connections)
		if tokenStr == "" {
			tokenStr = c.Query("token")
		}

		if tokenStr == "" {
			c.AbortWithStatusJSON(http.StatusUnauthorized, gin.H{"error": "missing token"})
			return
		}

		claims, err := ValidateToken(secret, tokenStr)
		if err != nil {
			c.AbortWithStatusJSON(http.StatusUnauthorized, gin.H{"error": "invalid token"})
			return
		}

		c.Set("user_id", claims.Subject)
		c.Set("robots", claims.Robots)
		c.Next()
	}
}

// RobotAccess checks that the user has access to the requested robot
func RobotAccess() gin.HandlerFunc {
	return func(c *gin.Context) {
		robotID := c.Param("id")
		robots, exists := c.Get("robots")
		if !exists {
			c.AbortWithStatusJSON(http.StatusForbidden, gin.H{"error": "no robot access"})
			return
		}
		robotList, ok := robots.([]string)
		if !ok {
			c.AbortWithStatusJSON(http.StatusForbidden, gin.H{"error": "invalid robot claims"})
			return
		}
		for _, r := range robotList {
			if r == robotID {
				c.Next()
				return
			}
		}
		c.AbortWithStatusJSON(http.StatusForbidden, gin.H{"error": "access denied to robot"})
	}
}
