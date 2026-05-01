package cloud_sync

import (
	"context"
	"fmt"

	"cloud.google.com/go/firestore"
	firebase "firebase.google.com/go/v4"
	"firebase.google.com/go/v4/messaging"
	"github.com/sirupsen/logrus"
	"google.golang.org/api/option"
)

// Payload is the notification content sent to mobile devices.
type Payload struct {
	// Title is the notification title shown in the OS notification drawer.
	Title string
	// Body is the notification body text.
	Body string
	// Data carries arbitrary key-value pairs delivered alongside the notification.
	Data map[string]string
	// Channel maps to an Android notification channel ID.
	// Known values: "alerts", "info", "low-battery", "errors", "emergency".
	Channel string
}

// NotifierOptions carries all constructor parameters for Notifier.
type NotifierOptions struct {
	// ProjectID is the Firebase/GCP project identifier.
	ProjectID string
	// ServiceAccountJSON is the path to a Firebase Admin SDK service-account
	// credentials JSON file. When empty the SDK falls back to Application
	// Default Credentials.
	ServiceAccountJSON string
	// Logger is optional; defaults to logrus.StandardLogger().
	Logger logrus.FieldLogger
}

// Notifier sends FCM v1 push messages to all registered device tokens for a
// robot. It is safe for concurrent use.
type Notifier struct {
	opts      NotifierOptions
	log       logrus.FieldLogger
	fsClient  *firestore.Client
	msgClient *messaging.Client
}

// NewNotifier creates a Firebase app, a Firestore client (to read device
// tokens), and an FCM messaging client.
func NewNotifier(ctx context.Context, opts NotifierOptions) (*Notifier, error) {
	if opts.ProjectID == "" {
		return nil, fmt.Errorf("cloud_sync: NewNotifier: ProjectID is required")
	}

	log := opts.Logger
	if log == nil {
		log = logrus.StandardLogger()
	}

	var fbOpts []option.ClientOption
	if opts.ServiceAccountJSON != "" {
		fbOpts = append(fbOpts, option.WithCredentialsFile(opts.ServiceAccountJSON))
	}

	app, err := firebase.NewApp(ctx, &firebase.Config{ProjectID: opts.ProjectID}, fbOpts...)
	if err != nil {
		return nil, fmt.Errorf("cloud_sync: NewNotifier: firebase.NewApp: %w", err)
	}

	fsClient, err := app.Firestore(ctx)
	if err != nil {
		return nil, fmt.Errorf("cloud_sync: NewNotifier: app.Firestore: %w", err)
	}

	msgClient, err := app.Messaging(ctx)
	if err != nil {
		_ = fsClient.Close()
		return nil, fmt.Errorf("cloud_sync: NewNotifier: app.Messaging: %w", err)
	}

	return &Notifier{
		opts:      opts,
		log:       log,
		fsClient:  fsClient,
		msgClient: msgClient,
	}, nil
}

// SendToRobot fans out a notification to all active device tokens registered
// under robots/{robotID}/devices/*. Tokens that respond with UNREGISTERED or
// INVALID_ARGUMENT are soft-deleted by setting deletedAt = serverTimestamp().
func (n *Notifier) SendToRobot(ctx context.Context, robotID string, p Payload) error {
	tokens, docIDs, err := n.loadActiveTokens(ctx, robotID)
	if err != nil {
		return fmt.Errorf("cloud_sync: Notifier.SendToRobot: load tokens: %w", err)
	}
	if len(tokens) == 0 {
		n.log.WithField("robotID", robotID).Debug("cloud_sync: no active device tokens")
		return nil
	}

	// Build data map — always non-nil so the mobile app can pattern-match on
	// channel without checking for nil.
	data := make(map[string]string, len(p.Data)+1)
	for k, v := range p.Data {
		data[k] = v
	}
	if p.Channel != "" {
		data["channel"] = p.Channel
	}

	msg := &messaging.MulticastMessage{
		Tokens: tokens,
		Notification: &messaging.Notification{
			Title: p.Title,
			Body:  p.Body,
		},
		Data: data,
		Android: &messaging.AndroidConfig{
			Notification: &messaging.AndroidNotification{
				ChannelID: p.Channel,
			},
		},
	}

	resp, err := n.msgClient.SendEachForMulticast(ctx, msg)
	if err != nil {
		return fmt.Errorf("cloud_sync: Notifier.SendToRobot: SendEachForMulticast: %w", err)
	}

	n.log.WithFields(logrus.Fields{
		"robotID": robotID,
		"sent":    resp.SuccessCount,
		"failed":  resp.FailureCount,
	}).Debug("cloud_sync: FCM multicast done")

	// Soft-delete stale tokens.
	for i, res := range resp.Responses {
		if res.Success || res.Error == nil {
			continue
		}
		if messaging.IsRegistrationTokenNotRegistered(res.Error) || messaging.IsInvalidArgument(res.Error) {
			if sdErr := n.softDeleteDevice(ctx, robotID, docIDs[i]); sdErr != nil {
				n.log.WithError(sdErr).WithFields(logrus.Fields{
					"robotID": robotID,
					"docID":   docIDs[i],
				}).Warn("cloud_sync: failed to soft-delete stale device token")
			} else {
				n.log.WithFields(logrus.Fields{
					"robotID": robotID,
					"docID":   docIDs[i],
				}).Info("cloud_sync: soft-deleted unregistered device token")
			}
		}
	}

	return nil
}

// Close releases the Firestore client held by the Notifier.
func (n *Notifier) Close() error {
	if err := n.fsClient.Close(); err != nil {
		return fmt.Errorf("cloud_sync: Notifier.Close: %w", err)
	}
	return nil
}

// loadActiveTokens returns the FCM registration tokens and their Firestore
// document IDs for all devices under robots/{robotID}/devices where
// deletedAt is absent or null.
func (n *Notifier) loadActiveTokens(ctx context.Context, robotID string) (tokens []string, docIDs []string, err error) {
	col := n.fsClient.Collection("robots").Doc(robotID).Collection("devices")

	// Attempt a server-side filter first; fall back to client-side filtering
	// because Firestore requires an index for inequality queries on missing fields.
	docs, queryErr := col.Where("deletedAt", "==", nil).Documents(ctx).GetAll()
	if queryErr != nil {
		docs, err = col.Documents(ctx).GetAll()
		if err != nil {
			return nil, nil, fmt.Errorf("load devices: %w", err)
		}
	}

	for _, d := range docs {
		// Skip soft-deleted documents even when the fallback path ran.
		if v, exists := d.Data()["deletedAt"]; exists && v != nil {
			continue
		}
		token, _ := d.Data()["token"].(string)
		if token == "" {
			continue
		}
		tokens = append(tokens, token)
		docIDs = append(docIDs, d.Ref.ID)
	}
	return tokens, docIDs, nil
}

// softDeleteDevice sets deletedAt = serverTimestamp() on the given device doc.
func (n *Notifier) softDeleteDevice(ctx context.Context, robotID, docID string) error {
	ref := n.fsClient.Collection("robots").Doc(robotID).Collection("devices").Doc(docID)
	_, err := ref.Update(ctx, []firestore.Update{
		{Path: "deletedAt", Value: firestore.ServerTimestamp},
	})
	if err != nil {
		return fmt.Errorf("softDeleteDevice %s: %w", docID, err)
	}
	return nil
}
