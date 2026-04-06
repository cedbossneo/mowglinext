package store

import (
	"context"
	"time"

	"go.mongodb.org/mongo-driver/v2/bson"
	"go.mongodb.org/mongo-driver/v2/mongo"
	"go.mongodb.org/mongo-driver/v2/mongo/options"
)

type MongoStore struct {
	client     *mongo.Client
	db         *mongo.Database
	users      *mongo.Collection
	robots     *mongo.Collection
	sessions   *mongo.Collection
	pushTokens *mongo.Collection
}

func NewMongoStore(ctx context.Context, uri string) (*MongoStore, error) {
	client, err := mongo.Connect(options.Client().ApplyURI(uri))
	if err != nil {
		return nil, err
	}
	if err := client.Ping(ctx, nil); err != nil {
		return nil, err
	}
	db := client.Database("mowglinext")
	return &MongoStore{
		client:     client,
		db:         db,
		users:      db.Collection("users"),
		robots:     db.Collection("robots"),
		sessions:   db.Collection("sessions"),
		pushTokens: db.Collection("push_tokens"),
	}, nil
}

func (s *MongoStore) Close(ctx context.Context) error {
	return s.client.Disconnect(ctx)
}

// --- Users ---

func (s *MongoStore) CreateUser(ctx context.Context, user *User) error {
	user.CreatedAt = time.Now()
	result, err := s.users.InsertOne(ctx, user)
	if err != nil {
		return err
	}
	user.ID = result.InsertedID.(bson.ObjectID)
	return nil
}

func (s *MongoStore) GetUserByEmail(ctx context.Context, email string) (*User, error) {
	var user User
	err := s.users.FindOne(ctx, bson.M{"email": email}).Decode(&user)
	if err != nil {
		return nil, err
	}
	return &user, nil
}

func (s *MongoStore) GetUserByID(ctx context.Context, id bson.ObjectID) (*User, error) {
	var user User
	err := s.users.FindOne(ctx, bson.M{"_id": id}).Decode(&user)
	if err != nil {
		return nil, err
	}
	return &user, nil
}

// --- Robots ---

func (s *MongoStore) CreateRobot(ctx context.Context, robot *Robot) error {
	robot.CreatedAt = time.Now()
	result, err := s.robots.InsertOne(ctx, robot)
	if err != nil {
		return err
	}
	robot.ID = result.InsertedID.(bson.ObjectID)
	return nil
}

func (s *MongoStore) GetRobotByID(ctx context.Context, id bson.ObjectID) (*Robot, error) {
	var robot Robot
	err := s.robots.FindOne(ctx, bson.M{"_id": id}).Decode(&robot)
	if err != nil {
		return nil, err
	}
	return &robot, nil
}

func (s *MongoStore) GetRobotsByOwner(ctx context.Context, ownerID bson.ObjectID) ([]Robot, error) {
	cursor, err := s.robots.Find(ctx, bson.M{"owner_id": ownerID})
	if err != nil {
		return nil, err
	}
	var robots []Robot
	if err := cursor.All(ctx, &robots); err != nil {
		return nil, err
	}
	return robots, nil
}

func (s *MongoStore) GetRobotByToken(ctx context.Context, tokenHash string) (*Robot, error) {
	var robot Robot
	err := s.robots.FindOne(ctx, bson.M{"token": tokenHash}).Decode(&robot)
	if err != nil {
		return nil, err
	}
	return &robot, nil
}

func (s *MongoStore) UpdateRobotLastSeen(ctx context.Context, id bson.ObjectID) error {
	_, err := s.robots.UpdateByID(ctx, id, bson.M{"$set": bson.M{"last_seen": time.Now()}})
	return err
}

// --- Sessions ---

func (s *MongoStore) CreateSession(ctx context.Context, session *Session) error {
	session.CreatedAt = time.Now()
	result, err := s.sessions.InsertOne(ctx, session)
	if err != nil {
		return err
	}
	session.ID = result.InsertedID.(bson.ObjectID)
	return nil
}

func (s *MongoStore) GetSessionByRefreshToken(ctx context.Context, token string) (*Session, error) {
	var session Session
	err := s.sessions.FindOne(ctx, bson.M{"refresh_token": token}).Decode(&session)
	if err != nil {
		return nil, err
	}
	return &session, nil
}

func (s *MongoStore) DeleteSession(ctx context.Context, id bson.ObjectID) error {
	_, err := s.sessions.DeleteOne(ctx, bson.M{"_id": id})
	return err
}

func (s *MongoStore) DeleteSessionsByUser(ctx context.Context, userID bson.ObjectID) error {
	_, err := s.sessions.DeleteMany(ctx, bson.M{"user_id": userID})
	return err
}

// --- Push Tokens ---

func (s *MongoStore) UpsertPushToken(ctx context.Context, token *PushToken) error {
	token.CreatedAt = time.Now()
	filter := bson.M{
		"user_id":  token.UserID,
		"robot_id": token.RobotID,
		"platform": token.Platform,
	}
	update := bson.M{"$set": token}
	opts := options.UpdateOne().SetUpsert(true)
	_, err := s.pushTokens.UpdateOne(ctx, filter, update, opts)
	return err
}

func (s *MongoStore) GetPushTokensByRobot(ctx context.Context, robotID bson.ObjectID) ([]PushToken, error) {
	cursor, err := s.pushTokens.Find(ctx, bson.M{"robot_id": robotID})
	if err != nil {
		return nil, err
	}
	var tokens []PushToken
	if err := cursor.All(ctx, &tokens); err != nil {
		return nil, err
	}
	return tokens, nil
}
