package store

import (
	"context"
	"time"

	"go.mongodb.org/mongo-driver/v2/bson"
)

// User represents a registered user
type User struct {
	ID        bson.ObjectID `bson:"_id,omitempty" json:"id"`
	Email     string        `bson:"email" json:"email"`
	Password  string        `bson:"password" json:"-"`
	Name      string        `bson:"name" json:"name"`
	CreatedAt time.Time     `bson:"created_at" json:"created_at"`
}

// Robot represents a registered robot
type Robot struct {
	ID        bson.ObjectID `bson:"_id,omitempty" json:"id"`
	Name      string        `bson:"name" json:"name"`
	Token     string        `bson:"token" json:"-"`
	OwnerID   bson.ObjectID `bson:"owner_id" json:"owner_id"`
	LastSeen  time.Time     `bson:"last_seen" json:"last_seen"`
	CreatedAt time.Time     `bson:"created_at" json:"created_at"`
}

// Session represents a refresh token session
type Session struct {
	ID           bson.ObjectID `bson:"_id,omitempty"`
	UserID       bson.ObjectID `bson:"user_id"`
	RefreshToken string        `bson:"refresh_token"`
	ExpiresAt    time.Time     `bson:"expires_at"`
	CreatedAt    time.Time     `bson:"created_at"`
}

// PushToken represents a push notification registration
type PushToken struct {
	ID        bson.ObjectID `bson:"_id,omitempty"`
	UserID    bson.ObjectID `bson:"user_id"`
	RobotID   bson.ObjectID `bson:"robot_id"`
	Platform  string        `bson:"platform"` // "ios" or "android"
	Token     string        `bson:"token"`
	CreatedAt time.Time     `bson:"created_at"`
}

// Store defines the interface for all data operations
type Store interface {
	// Users
	CreateUser(ctx context.Context, user *User) error
	GetUserByEmail(ctx context.Context, email string) (*User, error)
	GetUserByID(ctx context.Context, id bson.ObjectID) (*User, error)

	// Robots
	CreateRobot(ctx context.Context, robot *Robot) error
	GetRobotByID(ctx context.Context, id bson.ObjectID) (*Robot, error)
	GetRobotsByOwner(ctx context.Context, ownerID bson.ObjectID) ([]Robot, error)
	GetRobotByToken(ctx context.Context, tokenHash string) (*Robot, error)
	UpdateRobotLastSeen(ctx context.Context, id bson.ObjectID) error

	// Sessions
	CreateSession(ctx context.Context, session *Session) error
	GetSessionByRefreshToken(ctx context.Context, token string) (*Session, error)
	DeleteSession(ctx context.Context, id bson.ObjectID) error
	DeleteSessionsByUser(ctx context.Context, userID bson.ObjectID) error

	// Push tokens
	UpsertPushToken(ctx context.Context, token *PushToken) error
	GetPushTokensByRobot(ctx context.Context, robotID bson.ObjectID) ([]PushToken, error)

	// Lifecycle
	Close(ctx context.Context) error
	EnsureIndexes(ctx context.Context) error
}
