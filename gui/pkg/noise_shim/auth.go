package noise_shim

import (
	"context"
	"crypto/rsa"
	"crypto/x509"
	"encoding/json"
	"encoding/pem"
	"errors"
	"fmt"
	"io"
	"net/http"
	"os"
	"path/filepath"
	"sync"
	"time"

	"github.com/golang-jwt/jwt/v5"
)

// MemoryAllowList is the in-memory implementation of AllowList. cloud_sync
// writes; the Server reads.
type MemoryAllowList struct {
	mu  sync.RWMutex
	set map[string]struct{}
}

func NewMemoryAllowList(initial []string) *MemoryAllowList {
	a := &MemoryAllowList{set: make(map[string]struct{}, len(initial))}
	for _, u := range initial {
		a.set[u] = struct{}{}
	}
	return a
}

func (a *MemoryAllowList) Contains(uid string) bool {
	a.mu.RLock()
	defer a.mu.RUnlock()
	_, ok := a.set[uid]
	return ok
}

// Replace atomically swaps the set with the given slice. Called by
// cloud_sync after each Firestore snapshot.
func (a *MemoryAllowList) Replace(uids []string) {
	next := make(map[string]struct{}, len(uids))
	for _, u := range uids {
		next[u] = struct{}{}
	}
	a.mu.Lock()
	a.set = next
	a.mu.Unlock()
}

// Snapshot returns a copy of the current set, sorted is not guaranteed.
func (a *MemoryAllowList) Snapshot() []string {
	a.mu.RLock()
	defer a.mu.RUnlock()
	out := make([]string, 0, len(a.set))
	for u := range a.set {
		out = append(out, u)
	}
	return out
}

// FirebaseVerifier validates Firebase ID tokens (RS256 JWT) using the public
// keys published by Google. Verification is offline against a cached key set.
type FirebaseVerifier struct {
	ProjectID string
	CachePath string

	mu        sync.RWMutex
	keys      map[string]*rsa.PublicKey
	fetchedAt time.Time
}

const jwkURL = "https://www.googleapis.com/robot/v1/metadata/x509/securetoken@system.gserviceaccount.com"
const jwkRefresh = 6 * time.Hour
const tokenLeeway = 60 * time.Second

func NewFirebaseVerifier(projectID, cachePath string) *FirebaseVerifier {
	return &FirebaseVerifier{
		ProjectID: projectID,
		CachePath: cachePath,
		keys:      make(map[string]*rsa.PublicKey),
	}
}

func (f *FirebaseVerifier) Verify(ctx context.Context, idToken string) (string, error) {
	if err := f.ensureKeysFresh(ctx); err != nil {
		return "", fmt.Errorf("refresh jwk: %w", err)
	}
	parsed, err := jwt.Parse(idToken, func(t *jwt.Token) (interface{}, error) {
		if _, ok := t.Method.(*jwt.SigningMethodRSA); !ok {
			return nil, fmt.Errorf("unexpected alg: %v", t.Header["alg"])
		}
		kid, _ := t.Header["kid"].(string)
		f.mu.RLock()
		k, ok := f.keys[kid]
		f.mu.RUnlock()
		if !ok {
			return nil, fmt.Errorf("unknown kid: %q", kid)
		}
		return k, nil
	}, jwt.WithLeeway(tokenLeeway))
	if err != nil {
		return "", err
	}
	if !parsed.Valid {
		return "", errors.New("token invalid")
	}
	claims, ok := parsed.Claims.(jwt.MapClaims)
	if !ok {
		return "", errors.New("unexpected claim type")
	}
	if iss, _ := claims["iss"].(string); iss != "https://securetoken.google.com/"+f.ProjectID {
		return "", fmt.Errorf("bad iss: %q", iss)
	}
	aud, _ := claims["aud"].(string)
	if aud != f.ProjectID {
		return "", fmt.Errorf("bad aud: %q", aud)
	}
	uid, _ := claims["sub"].(string)
	if uid == "" {
		return "", errors.New("missing sub")
	}
	return uid, nil
}

func (f *FirebaseVerifier) ensureKeysFresh(ctx context.Context) error {
	f.mu.RLock()
	stale := time.Since(f.fetchedAt) > jwkRefresh || len(f.keys) == 0
	f.mu.RUnlock()
	if !stale {
		return nil
	}
	if len(f.keys) == 0 && f.CachePath != "" {
		_ = f.loadDiskCache()
	}
	return f.fetchJWKs(ctx)
}

func (f *FirebaseVerifier) fetchJWKs(ctx context.Context) error {
	req, err := http.NewRequestWithContext(ctx, "GET", jwkURL, nil)
	if err != nil {
		return err
	}
	resp, err := http.DefaultClient.Do(req)
	if err != nil {
		return err
	}
	defer resp.Body.Close()
	if resp.StatusCode != 200 {
		return fmt.Errorf("jwk: status %d", resp.StatusCode)
	}
	body, err := io.ReadAll(resp.Body)
	if err != nil {
		return err
	}
	var raw map[string]string
	if err := json.Unmarshal(body, &raw); err != nil {
		return err
	}
	parsed, err := parseCertMap(raw)
	if err != nil {
		return err
	}
	f.mu.Lock()
	f.keys = parsed
	f.fetchedAt = time.Now()
	f.mu.Unlock()
	if f.CachePath != "" {
		_ = os.MkdirAll(filepath.Dir(f.CachePath), 0o755)
		_ = os.WriteFile(f.CachePath, body, 0o644)
	}
	return nil
}

func (f *FirebaseVerifier) loadDiskCache() error {
	if f.CachePath == "" {
		return errors.New("no cache path")
	}
	body, err := os.ReadFile(f.CachePath)
	if err != nil {
		return err
	}
	var raw map[string]string
	if err := json.Unmarshal(body, &raw); err != nil {
		return err
	}
	parsed, err := parseCertMap(raw)
	if err != nil {
		return err
	}
	f.mu.Lock()
	f.keys = parsed
	f.mu.Unlock()
	return nil
}

func parseCertMap(raw map[string]string) (map[string]*rsa.PublicKey, error) {
	out := make(map[string]*rsa.PublicKey, len(raw))
	for kid, certPEM := range raw {
		block, _ := pem.Decode([]byte(certPEM))
		if block == nil {
			return nil, fmt.Errorf("kid %q: pem decode", kid)
		}
		cert, err := x509.ParseCertificate(block.Bytes)
		if err != nil {
			return nil, fmt.Errorf("kid %q: parse cert: %w", kid, err)
		}
		pub, ok := cert.PublicKey.(*rsa.PublicKey)
		if !ok {
			return nil, fmt.Errorf("kid %q: not RSA", kid)
		}
		out[kid] = pub
	}
	return out, nil
}

// StaticVerifier is a TokenVerifier that always returns a fixed uid. Tests only.
type StaticVerifier struct{ UID string }

func (s StaticVerifier) Verify(_ context.Context, _ string) (string, error) {
	if s.UID == "" {
		return "", errors.New("static verifier rejects all")
	}
	return s.UID, nil
}
