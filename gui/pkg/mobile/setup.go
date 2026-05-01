// Package mobile wires the noise_shim, pairing, and cloud_sync packages
// into the existing API server. It is the single entry-point main.go uses
// when MOWGLI_MOBILE_TUNNEL=1; when the env var is unset, this package is
// not exercised at all.
//
// All long-running goroutines are bound to a context the caller controls.
// On context cancellation each component exits cleanly and Setup's
// returned cleanup function returns once everything has stopped.
package mobile

import (
	"context"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/api"
	"github.com/cedbossneo/mowglinext/pkg/cloud_sync"
	"github.com/cedbossneo/mowglinext/pkg/noise_shim"
	"github.com/cedbossneo/mowglinext/pkg/pairing"
	"github.com/gin-gonic/gin"
	"github.com/sirupsen/logrus"
)

// Config bundles every value Setup needs. Empty strings disable the
// corresponding feature: e.g. an empty FirebaseProjectID skips the JWT
// verifier (handshakes are rejected), an empty FirebaseAdminJSON skips
// the Firestore allow-list watcher (allow-list stays whatever was passed
// in InitialAllowed).
type Config struct {
	// StateDir holds noise.key, google-jwk.json, and the cloudflared
	// credentials. Defaults to /var/lib/mowgli when empty.
	StateDir string
	// LocalAPIURL is where noise_shim's HTTP/WS proxy forwards decrypted
	// frames. Defaults to http://127.0.0.1:4006 when empty.
	LocalAPIURL string
	// LANAddr is the host:port embedded in QR payloads as the "lan" field.
	// Empty means the request's Host header is used at QR-render time.
	LANAddr string
	// TunnelDomain is the wildcard zone we operate on Cloudflare. Used to
	// derive the per-robot tunnel hostname r-<rid>.<TunnelDomain>. Empty
	// means the QR's "cf" field is left blank (mobile must wait for the
	// Cloud Function response before connecting).
	TunnelDomain string

	// FirebaseProjectID enables Firebase ID-token verification on the
	// noise_shim handshake. Empty means tokens are rejected.
	FirebaseProjectID string
	// FirebaseAdminJSON enables Firestore allow-list sync, FCM publish,
	// and ROS event detection. Empty means cloud_sync is skipped.
	FirebaseAdminJSON string
	// HeartbeatInterval is how often cloud_sync writes
	// robots/{robotID}.lastSeen. Defaults to 30s.
	HeartbeatInterval time.Duration

	// InitialAllowed is the uid set used by noise_shim before the first
	// Firestore snapshot arrives. Typically empty — first connection
	// requires Firestore to have populated allowedUids first.
	InitialAllowed []string

	// ROS is an optional event subscriber. When nil, no FCM events fire.
	// The caller (main.go) wires the existing rosProvider via an adapter.
	ROS cloud_sync.ROSSubscriber

	// Logger is used by every sub-component. Defaults to logrus standard.
	Logger *logrus.Entry
}

// Result is what Setup returns. Extension is mounted on the existing API
// server via api.NewAPIWith. Cleanup blocks until every goroutine has
// stopped and is safe to call multiple times.
type Result struct {
	Extension api.Extension
	Service   *pairing.Service
	Allow     *noise_shim.MemoryAllowList
	Cleanup   func()
}

// Setup builds the noise_shim + pairing + cloud_sync stack and returns an
// api.Extension that mounts /tunnel and /api/pair/* on the existing engine.
// Long-running goroutines (Firestore watcher, FCM publisher, event
// detector, periodic heartbeat) start under ctx and stop when ctx is
// cancelled or Result.Cleanup is invoked.
func Setup(ctx context.Context, cfg Config) (*Result, error) {
	cfg = applyDefaults(cfg)
	log := cfg.Logger

	priv, pub, err := pairing.LoadOrCreate(cfg.StateDir)
	if err != nil {
		return nil, fmt.Errorf("load/create keypair: %w", err)
	}
	robotID := pairing.RobotIDFromPub(pub)
	log = log.WithField("robot_id", robotID)
	log.Info("mobile tunnel: identity loaded")

	pairSvc := pairing.NewService(robotID, pub)
	pairSvc.StartFirstBoot()
	tunnelHost := ""
	if cfg.TunnelDomain != "" {
		tunnelHost = "r-" + robotID + "." + strings.TrimLeft(cfg.TunnelDomain, ".")
	}
	pairSvc.SetEndpoints(cfg.LANAddr, tunnelHost)

	allow := noise_shim.NewMemoryAllowList(cfg.InitialAllowed)

	var verifier noise_shim.TokenVerifier
	if cfg.FirebaseProjectID != "" {
		verifier = noise_shim.NewFirebaseVerifier(
			cfg.FirebaseProjectID,
			filepath.Join(cfg.StateDir, "google-jwk.json"),
		)
	} else {
		log.Warn("MOWGLI_FIREBASE_PROJECT_ID unset — every Noise handshake will fail")
		verifier = rejectAllVerifier{}
	}

	shim := noise_shim.New(noise_shim.Config{
		StaticPriv: priv,
		RobotID:    robotID,
		RobotName:  os.Getenv("MOWGLI_ROBOT_NAME"),
		Version:    os.Getenv("MOWGLI_VERSION"),
		Allow:      allow,
		Verifier:   verifier,
		Proxy:      noise_shim.NewLocalHTTPProxy(cfg.LocalAPIURL),
		Logger:     log.WithField("c", "noise_shim"),
	})

	syncCtx, syncCancel := context.WithCancel(ctx)
	var syncWaiters []func()
	if cfg.FirebaseAdminJSON != "" {
		fs, err := cloud_sync.NewSync(ctx, cloud_sync.SyncOptions{
			ProjectID:          cfg.FirebaseProjectID,
			RobotID:            robotID,
			ServiceAccountJSON: cfg.FirebaseAdminJSON,
			AllowList:          allow,
			Logger:             log.WithField("c", "firestore_sync"),
		})
		if err != nil {
			syncCancel()
			return nil, fmt.Errorf("cloud_sync.NewSync: %w", err)
		}
		runDone := make(chan struct{})
		go func() {
			defer close(runDone)
			if err := fs.Run(syncCtx); err != nil && !errors.Is(err, context.Canceled) {
				log.WithError(err).Warn("firestore watcher exited")
			}
		}()
		hbDone := make(chan struct{})
		go func() {
			defer close(hbDone)
			t := time.NewTicker(cfg.HeartbeatInterval)
			defer t.Stop()
			for {
				select {
				case <-syncCtx.Done():
					return
				case <-t.C:
					if err := fs.Heartbeat(syncCtx); err != nil &&
						!errors.Is(err, context.Canceled) {
						log.WithError(err).Debug("heartbeat failed")
					}
				}
			}
		}()
		syncWaiters = append(syncWaiters, func() { <-runDone }, func() { <-hbDone })
		syncWaiters = append(syncWaiters, func() { _ = fs.Close() })

		notif, err := cloud_sync.NewNotifier(ctx, cloud_sync.NotifierOptions{
			ProjectID:          cfg.FirebaseProjectID,
			ServiceAccountJSON: cfg.FirebaseAdminJSON,
			Logger:             log.WithField("c", "fcm"),
		})
		if err != nil {
			syncCancel()
			return nil, fmt.Errorf("cloud_sync.NewNotifier: %w", err)
		}
		syncWaiters = append(syncWaiters, func() { _ = notif.Close() })

		if cfg.ROS != nil {
			det := cloud_sync.NewEventDetector(cloud_sync.DetectorOptions{
				ROS: cfg.ROS,
				Notify: func(p cloud_sync.Payload) error {
					return notif.SendToRobot(syncCtx, robotID, p)
				},
			})
			detDone := make(chan struct{})
			go func() {
				defer close(detDone)
				if err := det.Run(syncCtx); err != nil && !errors.Is(err, context.Canceled) {
					log.WithError(err).Warn("event detector exited")
				}
			}()
			syncWaiters = append(syncWaiters, func() { <-detDone })
		} else {
			log.Info("no ROS subscriber provided — FCM event detector disabled")
		}
	} else {
		log.Warn("MOWGLI_FIREBASE_ADMIN_JSON unset — cloud_sync disabled (allow-list will stay empty)")
	}

	ext := func(r *gin.Engine, _ *gin.RouterGroup) {
		// /tunnel is the Cloudflare Tunnel ingress target. cloudflared is
		// configured to forward only this path through to us.
		r.GET("/tunnel", gin.WrapH(shim.Handler()))
		// Pairing routes are served on the LAN port only — the LAN-only
		// middleware inside RegisterRoutes rejects non-private IPs.
		pairing.RegisterRoutes(r.Group("/"), pairSvc)
	}

	cleanup := func() {
		syncCancel()
		for _, w := range syncWaiters {
			w()
		}
	}

	return &Result{
		Extension: ext,
		Service:   pairSvc,
		Allow:     allow,
		Cleanup:   cleanup,
	}, nil
}

func applyDefaults(cfg Config) Config {
	if cfg.StateDir == "" {
		cfg.StateDir = "/var/lib/mowgli"
	}
	if cfg.LocalAPIURL == "" {
		cfg.LocalAPIURL = "http://127.0.0.1:4006"
	}
	if cfg.HeartbeatInterval == 0 {
		cfg.HeartbeatInterval = 30 * time.Second
	}
	if cfg.Logger == nil {
		cfg.Logger = logrus.NewEntry(logrus.StandardLogger()).WithField("pkg", "mobile")
	}
	return cfg
}

// rejectAllVerifier short-circuits when no Firebase project is configured.
type rejectAllVerifier struct{}

func (rejectAllVerifier) Verify(_ context.Context, _ string) (string, error) {
	return "", errors.New("MOWGLI_FIREBASE_PROJECT_ID is not set")
}
