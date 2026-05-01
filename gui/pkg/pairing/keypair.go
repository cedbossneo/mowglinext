// Package pairing implements the first-boot pairing flow between the MowgliNext
// robot and the mobile app. It manages the Curve25519 keypair, QR code
// generation, the in-memory pairing state machine, and the Gin API routes.
package pairing

import (
	"crypto/rand"
	"crypto/sha256"
	"encoding/hex"
	"fmt"
	"os"
	"path/filepath"

	"golang.org/x/crypto/curve25519"
)

const keyFile = "noise.key"

// LoadOrCreate loads the Curve25519 private key from <stateDir>/noise.key if
// the file exists. If it does not exist, a fresh keypair is generated,
// the private key is written to disk with mode 0600, and both keys are
// returned. The file always contains exactly 32 raw bytes (no encoding).
func LoadOrCreate(stateDir string) (priv []byte, pub []byte, err error) {
	path := filepath.Join(stateDir, keyFile)

	existing, readErr := os.ReadFile(path)
	if readErr == nil {
		if len(existing) != 32 {
			return nil, nil, fmt.Errorf("pairing: keypair file %q has wrong length %d (want 32)", path, len(existing))
		}
		derived, deriveErr := derivePublic(existing)
		if deriveErr != nil {
			return nil, nil, fmt.Errorf("pairing: derive public from existing key: %w", deriveErr)
		}
		return existing, derived, nil
	}
	if !os.IsNotExist(readErr) {
		return nil, nil, fmt.Errorf("pairing: read key file: %w", readErr)
	}

	// Generate a fresh private key scalar.
	scalar := make([]byte, 32)
	if _, err := rand.Read(scalar); err != nil {
		return nil, nil, fmt.Errorf("pairing: generate private key: %w", err)
	}

	derived, deriveErr := derivePublic(scalar)
	if deriveErr != nil {
		return nil, nil, fmt.Errorf("pairing: derive public key: %w", deriveErr)
	}

	if err := os.MkdirAll(stateDir, 0o755); err != nil {
		return nil, nil, fmt.Errorf("pairing: create state dir: %w", err)
	}
	// Write with mode 0600 — private key must not be readable by other users.
	if err := os.WriteFile(path, scalar, 0o600); err != nil {
		return nil, nil, fmt.Errorf("pairing: write key file: %w", err)
	}

	return scalar, derived, nil
}

// derivePublic computes the Curve25519 public key from a 32-byte scalar using
// the standard X25519 base-point multiplication.
func derivePublic(priv []byte) ([]byte, error) {
	if len(priv) != 32 {
		return nil, fmt.Errorf("private key must be 32 bytes, got %d", len(priv))
	}
	pub, err := curve25519.X25519(priv, curve25519.Basepoint)
	if err != nil {
		return nil, err
	}
	return pub, nil
}

// RobotIDFromPub returns the robot's stable identifier: the first 6 hex
// characters of SHA-256(pub), uppercased. E.g. "A3F9B2".
func RobotIDFromPub(pub []byte) string {
	sum := sha256.Sum256(pub)
	return hex.EncodeToString(sum[:3])
}
