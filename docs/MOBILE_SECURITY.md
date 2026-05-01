# MowgliNext Mobile Companion: Security Analysis

## Executive Summary

The mobile companion architecture is designed for **end-to-end encryption** via Noise IK, ensuring that Firebase and Cloudflare cannot read command/telemetry traffic. Pairing is TOFU (trust-on-first-use) anchored by the QR code's public key. Multi-user access is mediated by Firestore security rules and allow-lists synced in real-time.

**Threat model:** We trust Firebase (Google) and Cloudflare as platforms, but we don't trust them with plaintext user data. Noise IK guarantees they cannot decrypt traffic.

---

## Assets and Adversaries

### Assets

1. **Blade authority** (highest criticality)
   - Robot's ability to spin blades
   - Firmware is the sole safety authority; ROS can request but firmware decides
   - This architecture does NOT grant direct blade control via mobile app (planned, but not implemented)
   - Risk: Low (out of scope for first release)

2. **Location data** (GPS, pose)
   - Longitude, latitude, heading from `map` frame
   - User's mowing area may reveal home address or work location
   - Risk: Medium (personally identifiable)

3. **Live video** (if added later)
   - Camera feed from the robot
   - Highly sensitive; enables surveillance
   - Risk: High (but not implemented yet)

4. **Network access on user's LAN**
   - During pairing, robot's HTTP API is exposed on `http://mowgli.local`
   - After pairing, tunnel is outbound-only (robot initiates)
   - Risk: Medium (mitigated by LAN-only enforcement)

5. **Control authority**
   - Ability to start/stop mowing, set navigation goals
   - Only authenticated, invited users can send commands
   - Risk: Medium (mitigated by allow-list + JWT validation)

---

### Adversaries

#### 1. Cloud Relay (Cloudflare)

**Capabilities:**
- See TLS metadata: source IP, destination IP, request size, timing
- See DNS queries: `r-ABC123.tunnel.mowgli.garden`
- Perform active MITM via tunnel configuration
- DoS by dropping tunnel connection

**Cannot:**
- Read traffic contents (Noise IK encrypts)
- Forge JWTs (Google's keys)
- Modify allow-list without Firestore access

**Mitigation:**
- Noise IK provides confidentiality despite Cloudflare's position
- Cloudflare can inspect metadata (IP, timing) — not fixable in MVP
- Future: Consider Tailscale (decentralized relay) if Cloudflare trust is insufficient

---

#### 2. Firebase Platform (Google)

**Capabilities:**
- Read/write Firestore documents (if service account is compromised)
- View allow-list, device tokens, user emails
- Perform active MITM on JWT validation (if they choose to)
- DoS by rate-limiting or disabling services

**Cannot:**
- Read Noise-encrypted traffic (Google has no keys)
- Forge JWTs without their signing key
- Access robot's private Noise key (only on-device)

**Mitigation:**
- We validate JWTs offline (cached JWK, 6h refresh)
- Even if Firebase is compromised, existing sessions continue (only new handshakes break)
- Firestore rules enforce that only the owner can modify allow-list

---

#### 3. Network Attacker on User's LAN (e.g., Guest on WiFi)

**Capabilities:**
- Intercept unencrypted HTTP pairing traffic
- Read setup token, uid, idToken from `/api/pair/start` POST
- MITM on LAN HTTP (no TLS before Noise)
- Spy on traffic to `mowgli.local`

**Cannot:**
- Decrypt Noise IK sessions (requires robot's private key)
- Forge Firebase ID tokens (requires Google's key)

**Mitigation:**
- QR code carries robot's public key → TOFU model
- 4-digit confirmation code is derived from random, not handshake (future: handshake-bound)
- If attacker replaces QR, confirmation code mismatch flags MITM
- LAN pairing is plaintext HTTP by design (single-use token, short TTL, 4-digit check)
- Future: Move LAN pairing to Noise-XX or mDNS-anchored HTTPS

**Risk:** If attacker replaces QR code shown on the wizard screen, they could pair as the real attacker. The 4-digit code provides weak defense (10,000 possibilities). **Defense strategy:** User must visually confirm the QR code is from the robot (not replaced by attacker's phone).

---

#### 4. Stolen Phone with Active Session

**Capabilities:**
- Phone has valid Firebase ID token and active Noise IK session
- Can send arbitrary commands (start/stop mowing, navigate)
- Can read real-time telemetry

**Cannot:**
- Access other users' passwords (Firebase Auth is per-device)
- See historical data (not stored on phone)

**Mitigation:**
- User can revoke all sessions from another device: `revokeUser` Cloud Function
- Firebase ID token auto-refreshes every hour; revocation is near-instantaneous
- Next handshake attempt by stolen phone fails (uid removed from allow-list)
- Existing sessions remain open for ~1s (listener latency)

**Recovery:** User immediately calls owner and says "revoke this phone". Owner runs:
```bash
firebase firestore-update robots/ABC123 allowedUids --remove <uid-of-thief>
# or via mobile app: Settings → Users → Revoke
```

---

#### 5. Compromised Robot (Firmware, ROS Stack, GUI Binary)

**Out of scope for this doc.** Once the robot's binary is compromised, the attacker can:
- Steal the Noise private key
- Modify allow-list in memory
- Send fake telemetry

**Defense:** Firmware code signing, container image scanning, rootfs integrity checks (not implemented yet).

---

#### 6. Lost or Compromised Cloudflare Tunnel Credentials

**What if `/var/lib/mowgli/cloudflared/credentials.json` is exfiltrated?**

Attacker can:
- Impersonate the robot to Cloudflare
- Create a second tunnel as the robot
- Potentially MitM the tunnel (depends on Cloudflare's auth scheme)

**Mitigation:**
- Credentials are per-robot; stealing one robot's credentials does not affect others
- Robot can be re-paired (generates new tunnel)
- File permissions: `mode 0600` (readable only by robot process)
- Future: FIM (File Integrity Monitoring) on this file

**Rotation:** Re-pair the robot (runs `pairRobot` again, generates fresh tunnel credentials).

---

## Cryptographic Primitives

### Noise IK (Noise_IK_25519_ChaChaPoly_BLAKE2s)

**Reference:** [Noise Protocol Framework, Trevor Perrin, Rev 34](https://noiseprotocol.org/noise.html)

**Pattern:** IK (initiator has responder's public key)

**DH Function:** Curve25519 (X25519, IETF standard RFC 7748)
- 32-byte private key, 32-byte public key
- Scalar multiplication in the field

**Cipher:** ChaCha20-Poly1305 (AEAD)
- 256-bit key, 96-bit nonce
- 16-byte authentication tag
- Authenticated encryption + decryption

**Hash:** BLAKE2s
- 256-bit output
- Fast, secure, standardized

**Handshake Pattern:**

```
IK:
  -> e, payload
  <- e, ee, payload
```

- **Initiator message 1:** Sends ephemeral public key `e` + payload encrypted with DH(e, rs)
- **Responder message 2:** Sends ephemeral public key `e` + payload encrypted with DH(e, is) and DH(ee, cs)
- After message 2, both sides derive symmetric keys and the handshake completes

**Forward Secrecy:**

- Ephemeral keys are used once per session, then deleted
- If the robot's **static private key** (is) is compromised **tomorrow**, an attacker cannot:
  - Decrypt yesterday's handshakes (ephemeral keys are gone)
  - Decrypt yesterday's traffic (session keys are gone)
- Backward secrecy is not guaranteed (compromised static key allows decryption of handshakes with that key)

**Zero Handshake RTTs:**

- Initiator's first message (msg1) carries encrypted payload (DH(e, rs))
- Responder decrypts, sends response (msg2)
- After msg2, data is transmitted within one round-trip
- Compared to TLS 1.3 (2 RTTs for full handshake), this is faster

**Replay Defense:**

- Handshake nonce (32 bytes) sent by initiator
- Robot tracks nonces in a map (LRU, 10-minute window)
- Duplicate nonces are rejected immediately
- Per-frame AEAD nonce (incremented on every message) prevents replays within a session

---

### Firebase ID Token (RS256 JWT)

**Issuer:** `https://securetoken.google.com/{projectId}`

**Signature:** RSA-256 (RSA with SHA256)
- Google publishes public keys at: `https://www.googleapis.com/robot/v1/metadata/x509/securetoken@system.gserviceaccount.com`

**Validation (offline, on robot):**

1. Fetch JWK (public key certificates) from googleapis.com (cached for 6 hours)
2. Decode JWT (no verification yet)
3. Look up key ID (kid) in header, find matching certificate
4. Verify signature using certificate's RSA public key
5. Check claims:
   - `iss` (issuer) matches Google's issuer URL
   - `aud` (audience) matches our projectId
   - `sub` (subject) is the Firebase UID
   - `exp` (expiration) is in the future
6. Extract uid from `sub` claim

**Token Leeway:** 60 seconds (accounts for clock skew between robot and Google)

**Refresh:** Firebase SDKs auto-refresh tokens every ~1 hour before expiry

---

### Firestore Security Rules

**Pattern:** Document-level access control

```
match /robots/{robotId} {
  allow read: if request.auth.uid in resource.data.allowedUids;
  allow write: if request.auth.uid == resource.data.ownerUid;
  
  match /devices/{docId} {
    allow write: if request.auth.uid != null;  // User registers own FCM token
    allow read: if false;  // Never read device tokens via direct client query
  }
}

match /users/{uid}/robots/{robotId} {
  allow read: if request.auth.uid == uid;
  allow write: if false;  // Cloud Functions only
}
```

These rules enforce:
- Only the owner can modify robot settings (status, robotName, etc.)
- Only invited users (in allowedUids) can read the robot document
- Device tokens are write-only from the app, never directly readable
- User's robot list is private to that user

---

## Security Guarantees and Limitations

### Guarantees

1. **Confidentiality of traffic post-handshake**
   - All commands and telemetry are encrypted with Noise IK
   - Cloudflare cannot read
   - Firebase cannot read
   - Only the robot and authenticated user can decrypt

2. **Integrity of traffic post-handshake**
   - AEAD provides authentication + encryption
   - Attacker cannot modify traffic without breaking decryption

3. **Authentication at handshake**
   - Robot validates Firebase ID token (offline)
   - User's uid must be in allow-list
   - Attacker without valid token cannot establish session

4. **Freshness of allow-list**
   - Firestore listener updates in real-time (~100ms)
   - Revocation takes effect within ~1 second

5. **Forward secrecy**
   - Ephemeral keys deleted after session
   - Compromise of static key tomorrow ≠ compromise of yesterday's traffic

---

### Limitations (Known Risks)

#### LAN Pairing is Plaintext HTTP

**Risk:** Attacker on the LAN can intercept setup_token and uid from `/api/pair/start` request.

**Current Mitigation:**
- 4-digit confirmation code (10,000 possibilities)
- Setup token is single-use (consumed immediately)
- User must visually confirm the QR code is correct

**Why Plaintext?**
- Before pairing completes, there is no shared secret for HTTPS
- Self-signed certs would require TOFU (trust-on-first-use), which is complex for a web wizard
- Future: Use Noise-XX (mutual authentication) during pairing

**Upgrade Path:**
1. Move pairing to Noise-XX (both parties have each other's keys)
2. Or: mDNS-anchored HTTPS (publish cert via mDNS, browser pins to mDNS cert)
3. Or: Require user to enter a PIN shown on the robot, sent by QR (increases friction)

---

#### Robot's Firestore Service Account Key on Disk

**File:** `/var/lib/mowgli/firestore-credentials.json`

**Risk:** If exfiltrated, attacker can:
- Write arbitrary documents to `robots/{robotId}` (modify allow-list)
- Read any Firestore document (no rules apply to service accounts)

**Current Mitigation:**
- File permissions: mode 0600 (only robot process can read)
- Container isolation (robot process runs as non-root)
- Firestore rules do NOT validate service-account writes (rules only validate client writes)

**Recommendation:**
- **Enable File Integrity Monitoring (FIM)** on `/var/lib/mowgli/`
- Periodically audit Firestore write logs (Cloud Logging)
- Rotate service account if compromise is suspected
- Future: Use Workload Identity Federation (GKE/Cloud Run) to avoid long-lived keys

---

#### No Certificate Pinning for JWK Fetch

**Current flow:**
1. Robot makes HTTPS request to `https://www.googleapis.com/...` (default system trust store)
2. If attacker compromises a CA, they can MITM JWK fetch

**Mitigation:**
- Google's certificate is issued by a major CA (Digicert, etc.)
- Browser/OS attacks are rare
- Traffic is over HTTPS (encrypted at TLS layer)

**Upgrade:** Pin the certificate SHA256 in the robot's code:
```go
pinnedCerts := []string{
  "sha256/AAAAAAAAAAAAA==",  // Google's cert
}
transport := &http.Transport{
  // ... pin logic
}
```

---

#### Firestore Allow-List is Centralized

**Risk:** If Firestore is compromised, attacker can modify allow-list server-side, granting access to attackers.

**Mitigation:**
- Firestore rules prevent non-owner writes
- Cloud Audit Logs record who modified the document
- User can audit allow-list from mobile app (see "Users" section)

---

#### No Rate Limiting on Noise Handshake

**Risk:** Attacker can hammer the `/tunnel` endpoint with handshake attempts.

**Current:** No rate limiting implemented.

**Mitigation:**
- Firewall rules (Cloudflare WAF or robot's iptables)
- Handshake timeout (10 seconds) limits resources per attempt
- Future: Implement rate limiting by source IP in noise_shim

---

#### Mobile App Sessions Never Auto-Logout

**Risk:** If phone is left unlocked, anyone with physical access can use the app.

**Mitigation:**
- Implement app-level session timeout (not done yet)
- Firebase ID token auto-refreshes every hour (not revoked until owner calls revokeUser)
- User can revoke the phone from another device

---

## Threat Scenarios

### Scenario A: Attacker Intercepts QR Code During Pairing

**Setup:** Attacker is on the same LAN as the robot and the user's phone.

**Attack:**
1. Attacker replaces the QR code shown in the web wizard (via WiFi MITM or replaced QR on printed label)
2. Attacker's QR contains their own public key (not robot's)
3. User scans the fake QR and taps "Confirm"
4. Attacker receives the 4-digit code from `/api/pair/start`

**User Impact:**
- User sees a 4-digit code on their phone (attacker's code)
- User sees a different 4-digit code on the web wizard (robot's code)
- Codes don't match → User cancels pairing
- No damage

**Why it fails:** The 4-digit code is generated from the setup token, not the handshake. Both wizard and app query the robot's `/api/pair/start`, so they both get the robot's code. If the attacker replaces the QR, they get a DIFFERENT code.

**Improved version (handshake-bound code):** Once we move pairing to Noise-XX, the code will be derived from the handshake hash. At that point, if the attacker replaces the QR, they can establish a handshake with the user's phone, but the code will be different from the robot's code. Mitigation remains the same.

---

### Scenario B: Attacker Gains Access to Firestore Service Account Key

**Setup:** Attacker exfiltrates `/var/lib/mowgli/firestore-credentials.json` from the robot.

**Attack:**
1. Attacker uses the service account to write `robots/ABC123.allowedUids = ["attacker_uid"]`
2. Attacker's own Firebase account (uid=attacker_uid) is now in the allow-list
3. Attacker connects to the robot over Noise IK with their valid token

**Robot Impact:**
- Attacker can read telemetry, send commands
- No impact on firmware safety (commands are still fire-and-forget)

**Detection:**
- Owner notices an unknown user in Settings → Users
- Cloud Audit Log shows: `updateDocument(robots/ABC123, allowedUids) by service-account@...`

**Recovery:**
1. Owner calls `revokeUser({ robotId: "ABC123", uid: "attacker_uid" })`
2. Owner rotates the service account (regenerate key via GCP console)
3. Deploy new key to the robot

---

### Scenario C: Attacker Compromises the Robot (Root Access)

**Setup:** Attacker gains root on the robot via a zero-day in ROS or other service.

**Attack:**
1. Attacker reads the robot's Noise private key from `/var/lib/mowgli/noise.key`
2. Attacker can now:
   - Decrypt ALL future Noise IK sessions (ephemeral keys are known)
   - Modify allow-list in memory
   - Forge telemetry

**Impact:** Total compromise.

**Out of Scope:** This document assumes the robot binary is trusted. Defense requires code signing, container image scanning, and runtime integrity checks.

---

### Scenario D: Firebase is Compromised (Google's Systems)

**Setup:** Attacker gains access to Google's Firebase infrastructure.

**Attack:**
1. Attacker modifies Google's JWK public keys
2. Attacker forges a Firebase ID token with arbitrary uid
3. Attacker connects to the robot with the forged token

**Robot Impact:**
- Attacker can log in as any uid (including the owner)

**Mitigation:**
- JWK cache is fetched and validated over HTTPS
- If attacker is intercepting HTTPS, they can MITM JWK fetch (see "No Certificate Pinning" above)
- Robot has on-disk cache of last-known JWK (valid for 6 hours) — if network is completely cut during attack window, old cache is used

**Detection:**
- Cloud Audit Logs show suspicious login activity
- Owner can revoke all sessions

**Why it's unlikely:** Google's infrastructure is heavily defended. If Google is fully compromised, the entire internet ecosystem is at risk. We accept this as a baseline assumption.

---

### Scenario E: Attacker Intercepts Cloudflare Tunnel Configuration

**Setup:** Attacker is positioned between the robot and Cloudflare's edge network.

**Attack:**
1. Attacker intercepts the robot's tunnel connection request
2. Attacker configures a fake tunnel pointing to their own server
3. User tries to connect to the robot, but connects to attacker's server instead

**User Impact:**
- User sees Noise IK handshake fail ("invalid signature" or "token invalid")
- User cannot connect

**Why it fails:** Even if the attacker can MITM the tunnel setup, they don't have the robot's Noise private key. When the user attempts a Noise IK handshake, it fails because the attacker's server cannot decrypt the initiator's message (doesn't have the ephemeral key).

**Note:** The attacker can learn that a tunnel exists (`r-ABC123.tunnel.mowgli.garden`) via DNS queries, but cannot read the traffic.

---

## Recommendations

### MVP (Now)

- [ ] Document pairing protocol (this doc)
- [ ] Implement allow-list sync from Firestore
- [ ] Implement FCM push (event detector + notifier)
- [ ] Validate JWTs offline (cached JWK)
- [ ] LAN-only pairing endpoints (no remote pairing)

### Short-term (1–2 months)

- [ ] Add rate limiting to Noise handshake (e.g., 10 attempts/second per IP)
- [ ] Implement session timeout in mobile app (logout after 30 min of inactivity)
- [ ] Add revoke endpoint to mobile app (don't make users call Cloud Function)
- [ ] File Integrity Monitoring (FIM) on `/var/lib/mowgli/` (via `auditd`)
- [ ] Periodic audit of Firestore logs (query changes to allowedUids)

### Medium-term (3–6 months)

- [ ] Move LAN pairing to Noise-XX (mutual authentication, encrypted QR exchange)
- [ ] Add certificate pinning to JWK fetch (pin Google's cert)
- [ ] Implement Workload Identity Federation (avoid long-lived service account keys)
- [ ] Add app-level session timeout (OAuth 2 refresh token rotation)
- [ ] Log all Noise handshakes (connect/disconnect events to Cloud Logging)

### Long-term (6+ months)

- [ ] Implement command queuing (store commands in Firestore with TTL)
- [ ] Support offline-first mode (local allow-list backup, Firestore sync when online)
- [ ] Implement Tailscale fallback (decentralized relay if Cloudflare is unavailable)
- [ ] Add blade control via mobile app (with firmware safety checks)
- [ ] Live video streaming (over Noise IK, with bandwidth limits)
- [ ] Hardware security module (HSM) for Noise private key storage

---

## Conclusion

The mobile companion architecture provides **end-to-end encryption** for all traffic after pairing, ensuring that Firebase and Cloudflare cannot read user data. Pairing uses TOFU anchored by the QR code's public key and verified by a 4-digit confirmation code.

**The primary risk** is LAN pairing over plaintext HTTP. Mitigation is the 4-digit code; upgrade path is Noise-XX during pairing.

**Secondary risks** include service account key exfiltration (mitigated by file permissions and audit logging) and Firebase/Google compromise (mitigated by offline JWT validation and cache fallback).

**For production deployment**, we recommend:
1. Enable Cloud Audit Logging for Firestore
2. Implement File Integrity Monitoring on `/var/lib/mowgli/`
3. Set up alerts for unusual Firestore mutations
4. Rotate service account keys quarterly
5. Regular security audits of the pairing flow

This architecture is suitable for home and small-scale deployments. Large enterprises should add additional layers (WAF, rate limiting, VPN gateways, EDR).
