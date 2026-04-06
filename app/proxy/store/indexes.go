package store

import (
	"context"

	"go.mongodb.org/mongo-driver/v2/bson"
	"go.mongodb.org/mongo-driver/v2/mongo"
	"go.mongodb.org/mongo-driver/v2/mongo/options"
)

func (s *MongoStore) EnsureIndexes(ctx context.Context) error {
	// users.email — unique
	_, err := s.users.Indexes().CreateOne(ctx, mongo.IndexModel{
		Keys:    bson.D{{Key: "email", Value: 1}},
		Options: options.Index().SetUnique(true),
	})
	if err != nil {
		return err
	}

	// sessions.refresh_token — unique
	_, err = s.sessions.Indexes().CreateOne(ctx, mongo.IndexModel{
		Keys:    bson.D{{Key: "refresh_token", Value: 1}},
		Options: options.Index().SetUnique(true),
	})
	if err != nil {
		return err
	}

	// sessions.expires_at — TTL (auto-cleanup)
	expireAfter := int32(0) // Expire at the exact time specified in the field
	_, err = s.sessions.Indexes().CreateOne(ctx, mongo.IndexModel{
		Keys:    bson.D{{Key: "expires_at", Value: 1}},
		Options: options.Index().SetExpireAfterSeconds(expireAfter),
	})
	if err != nil {
		return err
	}

	// push_tokens.robot_id
	_, err = s.pushTokens.Indexes().CreateOne(ctx, mongo.IndexModel{
		Keys: bson.D{{Key: "robot_id", Value: 1}},
	})
	if err != nil {
		return err
	}

	// robots.owner_id
	_, err = s.robots.Indexes().CreateOne(ctx, mongo.IndexModel{
		Keys: bson.D{{Key: "owner_id", Value: 1}},
	})
	if err != nil {
		return err
	}

	return nil
}
