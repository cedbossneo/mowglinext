// Package cloud_sync keeps the robot's allow-list in sync with Firestore and
// pushes FCM v1 notifications to registered mobile devices.
package cloud_sync

import (
	"context"
	"fmt"
	"sync"
	"time"

	"cloud.google.com/go/firestore"
	firebase "firebase.google.com/go/v4"
	"github.com/sirupsen/logrus"
	"google.golang.org/api/option"

	"github.com/cedbossneo/mowglinext/pkg/noise_shim"
)

// SyncOptions carries all constructor parameters for Sync.
type SyncOptions struct {
	// ProjectID is the Firebase/GCP project identifier.
	ProjectID string
	// RobotID is the Firestore document key under robots/{robotId}.
	RobotID string
	// ServiceAccountJSON is the path to a Firebase Admin SDK service-account
	// credentials JSON file. When empty the SDK falls back to Application
	// Default Credentials.
	ServiceAccountJSON string
	// AllowList is the in-memory allow-list that Sync keeps up-to-date.
	AllowList *noise_shim.MemoryAllowList
	// Logger is optional; defaults to logrus.StandardLogger().
	Logger logrus.FieldLogger
}

// Sync watches a Firestore document and mirrors the allowedUids field into a
// MemoryAllowList. It is safe for concurrent use.
type Sync struct {
	opts   SyncOptions
	log    logrus.FieldLogger
	client *firestore.Client

	mu       sync.Mutex
	lastSeen time.Time
}

// NewSync creates a Firestore client authenticated with opts.ServiceAccountJSON
// (or ADC when empty) and returns a ready-to-run Sync.
func NewSync(ctx context.Context, opts SyncOptions) (*Sync, error) {
	if opts.ProjectID == "" {
		return nil, fmt.Errorf("cloud_sync: NewSync: ProjectID is required")
	}
	if opts.RobotID == "" {
		return nil, fmt.Errorf("cloud_sync: NewSync: RobotID is required")
	}
	if opts.AllowList == nil {
		return nil, fmt.Errorf("cloud_sync: NewSync: AllowList is required")
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
		return nil, fmt.Errorf("cloud_sync: NewSync: firebase.NewApp: %w", err)
	}

	client, err := app.Firestore(ctx)
	if err != nil {
		return nil, fmt.Errorf("cloud_sync: NewSync: app.Firestore: %w", err)
	}

	return &Sync{
		opts:   opts,
		log:    log,
		client: client,
	}, nil
}

// Run watches robots/{robotId} with Firestore Snapshots() and keeps the
// allow-list in sync. It blocks until ctx is cancelled, returning ctx.Err()
// or a Firestore error. The caller should restart Run on non-context errors.
func (s *Sync) Run(ctx context.Context) error {
	docRef := s.client.Collection("robots").Doc(s.opts.RobotID)
	iter := docRef.Snapshots(ctx)
	defer iter.Stop()

	for {
		snap, err := iter.Next()
		if err != nil {
			// ctx.Err() means normal shutdown.
			if ctx.Err() != nil {
				return ctx.Err()
			}
			return fmt.Errorf("cloud_sync: Sync.Run: iter.Next: %w", err)
		}

		if !snap.Exists() {
			s.log.WithField("robotID", s.opts.RobotID).Warn("cloud_sync: robot document does not exist yet")
			continue
		}

		var doc robotDoc
		if err := snap.DataTo(&doc); err != nil {
			s.log.WithError(err).Error("cloud_sync: failed to unmarshal robot document")
			continue
		}

		s.opts.AllowList.Replace(doc.AllowedUIDs)
		s.log.WithFields(logrus.Fields{
			"robotID": s.opts.RobotID,
			"count":   len(doc.AllowedUIDs),
		}).Debug("cloud_sync: allow-list refreshed")

		s.mu.Lock()
		s.lastSeen = time.Now()
		s.mu.Unlock()
	}
}

// Heartbeat writes lastSeen = serverTimestamp() and status = "online" into
// robots/{robotId}. Safe to call concurrently.
func (s *Sync) Heartbeat(ctx context.Context) error {
	docRef := s.client.Collection("robots").Doc(s.opts.RobotID)
	_, err := docRef.Set(ctx, map[string]interface{}{
		"lastSeen": firestore.ServerTimestamp,
		"status":   "online",
	}, firestore.MergeAll)
	if err != nil {
		return fmt.Errorf("cloud_sync: Sync.Heartbeat: %w", err)
	}

	s.mu.Lock()
	s.lastSeen = time.Now()
	s.mu.Unlock()
	return nil
}

// Close releases the Firestore client. Any concurrent Run call will receive an
// error once the underlying stream is torn down.
func (s *Sync) Close() error {
	if err := s.client.Close(); err != nil {
		return fmt.Errorf("cloud_sync: Sync.Close: %w", err)
	}
	return nil
}

// robotDoc is the Firestore shape of a robots/{id} document. Only the fields
// Sync cares about are declared; extra fields are silently ignored.
type robotDoc struct {
	AllowedUIDs []string `firestore:"allowedUids"`
}
