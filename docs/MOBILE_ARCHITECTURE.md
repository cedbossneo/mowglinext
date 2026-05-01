# MowgliNext Mobile Companion Architecture

## System Topology

```
┌─────────────────────────────────────────────────────────────────┐
│                        Internet / Cloud                         │
│                                                                 │
│  ┌──────────────────┐         ┌─────────────────┐              │
│  │ Firebase         │         │ Cloudflare      │              │
│  │  • Auth (JWT)    │←───────→│  • Tunnel       │              │
│  │  • Firestore     │         │  • DNS (r-*.    │              │
│  │  • FCM           │         │    mowgli.      │              │
│  │  • Functions     │         │    garden)      │              │
│  └──────────────────┘         └─────────────────┘              │
│           ↑                            ↑                        │
│           │ JWTs, allow-list          │ TLS metadata          │
│           │ device tokens, events     │ (Noise IK inside)     │
└───────────┼────────────────────────────┼──────────────────────┘
            │                            │
       [1] │                            │ [2] outbound tunnel
            │                            │
    ┌───────▼────────────────────────────▼─────────────┐
    │         MowgliNext Robot (x86/ARM)                │
    │                                                   │
    │  ┌─────────────────┐  ┌──────────────────────┐   │
    │  │  gui (Go)       │  │ cloudflared daemon   │   │
    │  │                 │  │  └─ tunnel upstream  │   │
    │  │ ┌─────────────┐ │  │    to CF              │   │
    │  │ │noise_shim   │ │  │  └─ ingress /tunnel   │   │
    │  │ │  └─ auth    │ │  │    → localhost:4006   │   │
    │  │ │  └─ proxy   │ │  │                      │   │
    │  │ │  └─ allow-  │←─┼──┘                      │   │
    │  │ │    list     │ │                          │   │
    │  │ │  └─ Noise   │ │                          │   │
    │  │ │    IK       │ │                          │   │
    │  │ └─────────────┘ │                          │   │
    │  │                 │                          │   │
    │  │ ┌─────────────┐ │                          │   │
    │  │ │pairing      │ │                          │   │
    │  │ │  └─ HTTP    │ │  [LAN only]              │   │
    │  │ │  └─ QR code │ │                          │   │
    │  │ │  └─ 4-digit │ │                          │   │
    │  │ │    confirm  │ │                          │   │
    │  │ └─────────────┘ │                          │   │
    │  │                 │                          │   │
    │  │ ┌─────────────┐ │                          │   │
    │  │ │cloud_sync   │ │                          │   │
    │  │ │  └─ sync    │ │  [4] Firestore listener │   │
    │  │ │    allow-   │ │  [5] FCM push detector  │   │
    │  │ │    list     │ │                          │   │
    │  │ │  └─ detect  │ │                          │   │
    │  │ │    events   │ │                          │   │
    │  │ │  └─ send    │ │                          │   │
    │  │ │    FCM      │ │                          │   │
    │  │ └─────────────┘ │                          │   │
    │  │                 │                          │   │
    │  │ ┌─────────────┐ │                          │   │
    │  │ │ROS 2 stack  │ │  [3] topics available   │   │
    │  │ │  /cmd_vel   │ │   via reverse proxy      │   │
    │  │ │  /status    │ │                          │   │
    │  │ │  /imu       │ │                          │   │
    │  │ │  /battery   │ │                          │   │
    │  │ │  ...        │ │                          │   │
    │  │ └─────────────┘ │                          │   │
    │  └─────────────────┘                          │   │
    └──────────────────────────────────────────────────┘
            ↑
            │ [6] WebSocket over /tunnel
            │     (HTTP-like requests, WS subscriptions)
            │
    ┌───────┴──────────────────┐
    │   Mobile App (iOS/Android)│
    │                           │
    │  • Firebase Auth token    │
    │  • Noise IK client        │
    │  • HTTP + WS client       │
    │  • FCM device token reg   │
    │  • Push notifications     │
    └───────────────────────────┘
```

## Component Matrix

| Component | Location | Language | Responsibility |
|-----------|----------|----------|-----------------|
| **noise_shim** | `gui/pkg/noise_shim/` | Go | Noise IK responder, token validation, reverse proxy to localhost:4006 |
| **pairing** | `gui/pkg/pairing/` | Go | HTTP API (LAN-only), setup token, 4-digit confirm code, state machine |
| **cloud_sync** | `gui/pkg/cloud_sync/` | Go | Firestore listener (allow-list), FCM push (event detector), heartbeat |
| **cloudflared overlay** | `docker/docker-compose.cloudflared.yaml` | YAML | Outbound Cloudflare tunnel (TLS L1), ingress /tunnel → localhost:4006 |
| **Mobile App** | `app/mobile/src/` | TypeScript/React Native | Firebase Auth sign-up, Noise IK initiator, HTTP+WS client, FCM device token, UI |
| **Firebase Backend** | `cloud/firebase/` | TypeScript (v2 Functions) | pairRobot, inviteUser, revokeUser, registerFcmToken, deleteRobot |
| **Firestore Schema** | `cloud/firebase/firestore.rules` | Firestore Security Rules | Enforce access control, validate allow-list mutations |
| **Google JWK Verifier** | `gui/pkg/noise_shim/auth.go` | Go | Cache Google's RS256 public keys (6h refresh), offline JWT validation |

## Data Flow Diagrams

### Pairing Flow (8 Steps)

```
Step 1: Robot boots → Generate random setup token (30-min TTL)
        ↓
Step 2: User opens http://mowgli.local/onboarding
        ↓
Step 3: Onboarding wizard shows QR code (contains: robot_id, pub_key, setup_token, [lan_ip])
        QR payload format: "mowgli://pair?v=1&rid=ABC123&pub=<b64>&tok=<hex>&lan=192.168.1.1&cf="
        ↓
Step 4: Mobile app scans QR → extracts setup_token, robot_id, public_key
        ↓
Step 5: Mobile app calls POST /api/pair/start (LAN HTTP, robot at mowgli.local)
        Body: { uid, idToken, displayName, setupToken }
        ↓
        Robot checks:
          • setupToken matches current token AND not expired
          • uid is valid (at this point, only syntactic check)
        Robot generates random 4-digit confirm code → stored in Pending state
        ↓
        Response: { confirmCode: "5729", expiresAt: timestamp }
        ↓
Step 6: Both wizard and app display their 4-digit codes
        User compares and confirms on BOTH screens
        ↓
Step 7: Web wizard POST /api/pair/confirm → confirms on robot
        Robot transitions state: Pending → Confirmed
        Robot calls OnPaired callback → cloud_sync.Sync.RegisterRobot(uid)
        ↓
        Cloud function (pairRobot):
          • Called by mobile app with Firebase ID token
          • Firebase Auth token proves uid
          • Creates robots/{robotId} in Firestore
          • Creates Cloudflare tunnel mowgli-ABC123
          • Creates DNS CNAME r-ABC123.tunnel.mowgli.garden → {tunnel_id}.cfargotunnel.com
          • Returns: { tunnelHostname, credentialsJson }
        ↓
Step 8: Mobile app sends credentialsJson to robot (via Noise IK or LAN HTTP)
        Robot writes to ${MOWGLI_STATE_DIR}/cloudflared/{credentials.json, config.yml}
        cloudflared daemon restarts, connects outbound to Cloudflare
        Robot is now reachable at https://r-ABC123.tunnel.mowgli.garden
        ↓
        state → Paired
```

**Why 4-digit confirm?** Prevents accidental pairing via MITM on LAN HTTP. If attacker intercepts /start, they learn a setup token that lets them call /start again, but they don't know the correct confirm code without also intercepting the robot's response. Confirmation mismatch flags MITM. Future: move LAN pairing to Noise-XX or mDNS-anchored HTTPS to eliminate plaintext entirely.

---

### Steady-State HTTP Request (Through Tunnel)

```
Mobile app:
  POST https://r-ABC123.tunnel.mowgli.garden/tunnel
  [TLS 1.3 to Cloudflare]
    Header: Content-Type: application/octet-stream
    Body: WebSocket upgrade request (Noise + inner-protocol frames)
    ↓
Cloudflare (no decryption):
  Receives TLS stream → forwards to tunnel {tunnel-uuid}
    ↓
cloudflared (robot host):
  Receives WS stream from tunnel → ingress routes /tunnel → localhost:4006
    ↓
noise_shim.Handler (localhost:4006):
  WS message received (binary frame: u16 length + Noise ciphertext)
    ↓
  Handshake already complete (session established)
    ↓
  Decrypt frame via AEAD → inner-protocol Frame (CBOR decoded)
    Frame.Type = "req"
    Frame.Method = "GET", Frame.Path = "/odometry/filtered"
    Frame.Headers = {"Accept": "application/json"}
    ↓
  Validate uid is in allow-list (already cached from Firestore listener)
    ↓
  Reverse proxy call:
    proxy.HTTP(ctx, "GET", "/odometry/filtered", {}, nil)
      ↓
    LocalHTTPProxy makes outbound request:
      GET http://localhost:4006/odometry/filtered
      ↓
      ROS HTTP bridge responds:
        { "x": 10.2, "y": 5.1, "z": 0, "theta": 0.5, ... }
    ↓
  Construct inner-protocol response:
    Frame.Type = "res"
    Frame.Status = 200
    Frame.Headers = {"Content-Type": "application/json"}
    Frame.Body = JSON bytes
    ↓
  Encrypt via AEAD → Noise ciphertext
    ↓
  Send back over WS:
    [u16 length + ciphertext]
    ↓
Cloudflare (no read):
  Forwards stream back through tunnel
    ↓
Mobile app:
  Receives WS message → decrypts frame → parses JSON
  200 OK ✓
```

---

### Steady-State WebSocket Subscription (Through Tunnel)

```
Mobile app wants to stream /battery topic:
  ↓
  Send inner-protocol Frame (CBOR):
    { "t": "wso", "id": 42, "path": "/battery", "headers": {} }
    ↓
  Encrypt via AEAD → send over WS
    ↓
noise_shim (robot):
  Decrypt → parse Frame
  Frame.Type = "wso" (WebSocket open request)
    ↓
  Call: proxy.OpenWS(ctx, "/battery", {})
    ↓
  LocalHTTPProxy.OpenWS dials upstream (localhost:4006/battery)
    ↓
  Upstream ROS HTTP bridge upgrades to WebSocket
    ↓
  Upstream sends first message: { "v_battery": 24.3 }
    ↓
  noise_shim receives upstream frame
    ↓
  Construct inner-protocol Frame:
    { "t": "wsd", "id": 42, "data": [JSON bytes], "binary": false }
    ↓
  Encrypt + send over outer WS to mobile app
    ↓
Mobile app:
  Receives frame → appends to stream
  Displays battery % in real time
    ↓
[Repeat for every upstream message]
    ↓
Mobile app closes subscription:
  { "t": "wsc", "id": 42, "code": 1000, "reason": "" }
    ↓
noise_shim closes upstream connection
    ↓
Done
```

---

### FCM Event Delivery (Robot → Mobile)

```
ROS topic: /hardware_bridge/power
  { "v_battery": 20.5, ... }
    ↓
EventDetector.onPower:
  v_battery < 22.0 → maybeNotify("low-battery", payload)
    ↓
maybeNotify:
  Check per-channel cooldown (5 min) → allow
    ↓
  Call: Notifier.SendToRobot(ctx, robotID, payload)
    ↓
Notifier.SendToRobot:
  Query: robots/{robotID}/devices where deletedAt is null
    ↓
  Load active FCM registration tokens: ["token_device_1", "token_device_2", ...]
    ↓
  Firebase Messaging API: multicastMessage.SendEachForMulticast(tokens, notification)
    ↓
  FCM routes to each token via APNs (iOS) / GCM (Android)
    ↓
Mobile device:
  Push notification delivered (even if app is closed)
  User sees: "Battery low" | "Voltage is low — robot may stop soon"
    ↓
  App wakes up, subscribes to WebSocket for live telemetry
    ↓
Soft-delete stale tokens:
  Any token that returns UNREGISTERED or INVALID_ARGUMENT
    ↓
  Notifier.softDeleteDevice(ctx, robotID, docID):
    robots/{robotID}/devices/{docID}.deletedAt = serverTimestamp
    (not hard-deleted, for audit trail)
```

---

## Failure Modes and Recovery

### Robot Offline (No Internet)

**What happens:**
- Cloudflare tunnel cannot establish outbound connection → TLS to Cloudflare times out
- Mobile app sees connection error after ~30s timeout
- Firebase shows `robots/{robotId}.status = "online"` is stale (lastSeen > 5 min)

**Recovery:**
- App displays cached `lastSeen` timestamp; shows "Robot last seen 12 minutes ago"
- Commands are NOT queued (future enhancement)
- Once robot reconnects to internet, cloudflared re-establishes tunnel
- App can reconnect immediately; Firestore listener picks up status change
- No data is lost (Firestore is source of truth for allow-list)

---

### Cloudflare Outage

**What happens:**
- Cloudflare tunnel endpoint is unreachable from mobile network
- Tunnel ingress cannot reach `r-ABC123.tunnel.mowgli.garden`
- TLS handshake fails at Cloudflare, app gets connection timeout

**Recovery:**
- Identical to offline: app shows lastSeen, user can try again
- Tailscale fallback is a future enhancement (not implemented)
- No local-network fallback yet (QR code has lan= param reserved for this)

---

### Firebase Outage

**What happens:**
- Pairing flow breaks: pairRobot Cloud Function returns error
- **Existing connections continue** because noise_shim validates tokens offline:
  - JWK cache (6h TTL) is fresh → token validation succeeds
  - Allow-list was synced recently → Contains(uid) check passes
  - Firestore listener will retry but is not on the critical path

**Recovery:**
- No action needed for ongoing sessions
- Pairing is blocked until Firebase recovers
- Once Firebase is back, next pairing attempt succeeds

---

### JWK Cache Stale

**What happens:**
- Google publishes a new RS256 public key
- Robot's 6h JWK cache is still old
- New tokens signed with new key_id fail validation

**Recovery:**
- noise_shim.FirebaseVerifier.ensureKeysFresh triggers an HTTP fetch to googleapis.com
- On-disk cache at `/var/lib/mowgli/google-jwk.json` is checked if network fetch fails
- Fallback strategy: cache is written during container startup and persisted
- JWK fetch has a 6h refresh window + immediate fetch on startup

---

### Allow-List Revocation

**What happens:**
- Owner calls `revokeUser({ robotId, uid })` Cloud Function
- Cloud Function removes uid from `robots/{robotId}.allowedUids` array in Firestore
- Firestore listener in cloud_sync.Sync.Run detects snapshot change
- MemoryAllowList.Replace([...remaining uids...]) is called
- Within ~1s, the revoked uid is no longer in Allow.Contains()
- Next Noise handshake from that uid fails: "uid not in allow-list"

**Recovery:**
- Revoked user cannot reconnect
- Re-inviting requires owner to call inviteUser Cloud Function again
- uid is re-added to allowedUids

---

### Compromised Firebase Service Account Key

**Risk:** If `/var/lib/mowgli/firestore-credentials.json` is exfiltrated, attacker can:
- Write arbitrary documents to `robots/{robotId}`
- Modify allow-list
- Register fake FCM tokens

**Mitigation:**
- **Firestore Security Rules** enforce that only authenticated requests (verified via `request.auth.uid`) can modify allow-list
- If attacker has the service account, they can forge any write as app-level auth (no uid)
- Defense in depth: File Integrity Monitoring (FIM) on `/var/lib/mowgli/`
- Rotate credentials if compromise is suspected (re-pair to get new Cloudflare credentials)

---

## Design Justification

### Why Noise IK, not mTLS?

Noise IK (Noise_IK_25519_ChaChaPoly_BLAKE2s, [Trevor Perrin spec rev 34](https://noiseprotocol.org/noise.html)):
- **Forward secrecy:** Ephemeral keys are deleted after each session. Compromise of robot's static private key tomorrow does NOT decrypt yesterday's traffic.
- **Zero handshake RTTs:** Initiator's first message carries the encrypted payload (IK pattern). No separate key exchange round-trip.
- **No certificate chain:** mTLS requires either:
  - Self-signed certs + TOFU (tofu_verify on every connection, tedious)
  - CA infrastructure (complex, requires PKI)
  - Noise instead: responder's static public key is in the QR code → trust via TOFU on pairing, then cached
- **Replay defense:** Handshake nonce LRU (10 min window) + per-frame AEAD nonce prevents replays
- **All crypto is well-vetted:** Curve25519 (IETF standard), ChaCha20-Poly1305 (standardized AEAD), BLAKE2s (secure hash)

Why not Firebase Realtime Database + custom TLS client certs? Because Firebase ID tokens (signed JWTs) are simpler and rotate automatically; client certs are a static credential that would need manual renewal.

---

### Why Cloudflare Tunnels, not ngrok/Tailscale/custom proxy?

| Solution | Uptime | Cost | Ease | E2E Notes |
|----------|--------|------|------|-----------|
| Cloudflare Tunnels | 99.99% | Free tier OK | Low (one env var) | Noise IK inside |
| ngrok | 99.99% | $5–20/mo | Low | Noise IK inside |
| Tailscale | 99.99% | $5–10/mo | Medium (app required on phone) | Phone app auto-routes |
| Custom Go proxy + Mongo | N/A (self-hosted) | Hosting cost | High (lots of code) | Weaker E2E |

Cloudflare Tunnels are chosen because:
1. **Free tier is sufficient** (100 tunnels, unlimited bandwidth)
2. **Cloudflare is a known, stable platform** (used by millions)
3. **Minimal robot-side footprint:** cloudflared binary (no custom proxy code)
4. **Noise IK ensures Cloudflare cannot read traffic**, so we don't trust them with data
5. **Tailscale would require phone app**, limiting adoption

---

### Why not Firebase Cloud Messaging alone (no Noise)?

Firebase allows hosting a custom HTTPS callable function that can relay commands to robots. Advantages:
- Simpler (no Noise shim code)

Disadvantages:
- Cloud Function can read all traffic in plaintext (Firebase engineers, attackers with compromised Google account)
- Requires HTTP → Firebase → HTTP round-trip (higher latency)
- Firebase Functions have execution limits (6 min timeout)
- **Does not provide encryption that Firebase cannot decrypt**

Noise IK ensures **end-to-end encryption**: Cloudflare and Firebase see only ciphertext and metadata (IP, DNS, request size).

---

### Why not command queuing (yet)?

Current design assumes the user connects in real-time and sends commands immediately. Commands are NOT queued when the robot is offline.

**Rationale:**
- Blade safety: queueing blade commands creates risk if robot wakes unexpectedly
- Complexity: queue durability, ordering, timeout semantics, retry logic
- MVP: real-time is sufficient (user opens app, robot is usually online)

**Future enhancement:**
- Store commands in Firestore with TTL (1 hour)
- When robot reconnects, fetch pending commands and execute
- Blade commands remain fire-and-forget (firmware safety)

---

## Setup Summary

See `docs/MOBILE_INTEGRATION.md` for the full operator runbook.

In 30 seconds:
1. Create Firebase project, enable Auth/Firestore/FCM
2. Create Cloudflare API token
3. Set Firebase secrets
4. `firebase deploy`
5. Create `.env.local` in `app/mobile/`
6. Pair a robot from the onboarding wizard
7. Install mobile app, sign up, scan QR, confirm 4-digit code
8. Done

---

## Security Summary

See `docs/MOBILE_SECURITY.md` for threat model and risk assessment.

**Assumptions:**
- Firebase signing keys are secure (Google's responsibility)
- Cloudflare does not actively attack (trusted platform, see Noise IK for E2E)
- LAN is under user's control (pairing is HTTP, not HTTPS)

**Guarantees:**
- Forward secrecy for all traffic after pairing
- Confidentiality from Cloudflare and Firebase via Noise IK
- Allow-list is refreshed ~1s after revocation
- Firebase tokens are validated offline (JWK cache)
- FCM tokens are soft-deleted on unregister
