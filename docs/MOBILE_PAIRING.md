# MowgliNext Mobile Pairing Protocol

## User Walkthrough

### Happy Path: 5 Minutes to Connected

1. **Power on the robot** — WiFi boots, runs `main.go` with `MOWGLI_MOBILE_TUNNEL=1`
   - `pairing.Service.StartFirstBoot()` generates a fresh setup token (32 random bytes, hex-encoded)
   - Token is valid for 30 minutes
   - GUI listens on `http://mowgli.local:80` (mDNS auto-discovery)

2. **Open onboarding on the web**
   - Desktop or tablet: navigate to `http://mowgli.local/onboarding`
   - Web wizard loads; you're on step 1 (WiFi setup) or later
   - Proceed to step 6 "Pair Mobile App"

3. **Web wizard displays QR code**
   - QR encodes the pairing payload:
     ```
     mowgli://pair?v=1&rid=ABC123&pub=<base64-pubkey>&tok=<hex-token>&lan=192.168.1.100&cf=
     ```
   - Fields explained below under "QR Payload Format"
   - Code is generated in real-time from `GET /api/pair/qr.png` (robot's LAN HTTP listener)

4. **Install mobile app**
   - iOS: TestFlight link (or App Store once published)
   - Android: Google Play internal testing or Play Store
   - Create account: sign up with email, Google, or Apple

5. **Scan QR code from mobile app**
   - App opens camera, scans code
   - Parses: robotId, public key, setup token
   - Extracts: LAN IP (optional — for fallback to direct connection)
   - Displays robot name if available: "Garden Mower (ABC123)"

6. **App calls `/api/pair/start` (LAN HTTP to robot)**
   ```
   POST http://mowgli.local/api/pair/start
   Content-Type: application/json
   
   {
     "uid": "firebase-uid-abc123xyz",
     "idToken": "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9...",
     "displayName": "Alice",
     "setupToken": "a1b2c3d4e5f6..."
   }
   ```
   - Robot validates setupToken (must match current token, not expired)
   - Robot verifies uid is syntactically valid
   - Robot generates a random 4-digit code: `"5729"`
   - Response:
     ```json
     {
       "confirmCode": "5729",
       "expiresAt": "2026-05-01T10:05:00Z"
     }
     ```

7. **Both screens show 4-digit code**
   - Web wizard: displays code from the robot
   - Mobile app: displays code received from `/api/pair/start`
   - User compares: both must show `5729`
   - If mismatch → MITM detected, user must restart

8. **User confirms on BOTH screens**
   - Web wizard: user clicks "Confirm" button → calls `POST /api/pair/confirm`
   - Mobile app: user taps "Confirm" → also calls `POST /api/pair/confirm`
   - Whichever confirms first wins (state machine is single-phase)
   - Robot transitions: Pending → Confirmed

9. **Cloud Function runs (pairRobot)**
   - Mobile app POSTs to Cloud Function with Firebase ID token
   - Function verifies token is valid for the calling uid
   - Function checks robot is not already claimed (no prior Paired document)
   - Function creates Cloudflare tunnel: `mowgli-ABC123`
   - Function creates DNS CNAME: `r-ABC123.tunnel.mowgli.garden` → `{tunnel-uuid}.cfargotunnel.com`
   - Function writes Firestore:
     ```
     robots/ABC123:
       ownerUid: firebase-uid-abc123xyz
       allowedUids: [firebase-uid-abc123xyz]
       status: "offline"
       createdAt: serverTimestamp
       robotName: "Garden Mower"
       robotId: "ABC123"
     
     users/{uid}/robots/ABC123:
       addedAt: serverTimestamp
       role: "owner"
     ```
   - Function returns:
     ```json
     {
       "tunnelHostname": "r-ABC123.tunnel.mowgli.garden",
       "credentialsJson": "{\"AccountTag\":\"...\", \"TunnelSecret\":\"...\", ...}"
     }
     ```

10. **Robot receives credentials and wires up cloudflared**
    - Mobile app sends credentialsJson to robot (either over Noise after handshake, or via LAN HTTP before tunnel is live)
    - Robot writes to `${MOWGLI_STATE_DIR}/cloudflared/`:
      - `credentials.json` (from Cloudflare API)
      - `config.yml` (locally generated, routes `/tunnel` to `localhost:4006`)
    - `cloudflared` daemon (via Docker overlay compose) reads config and connects outbound
    - Robot is now reachable at `https://r-ABC123.tunnel.mowgli.garden/tunnel`

11. **Mobile app connects over Noise IK**
    - App opens WebSocket to `wss://r-ABC123.tunnel.mowgli.garden/tunnel`
    - Performs Noise IK handshake (initiator → responder)
    - Handshake payload carries Firebase ID token + nonce
    - Robot validates token (offline, using cached Google JWK)
    - Checks uid is in allow-list (Firestore listener keeps it synced)
    - Handshake completes → session encrypted

12. **App commands the robot**
    - Send HTTP-like requests over Noise IK encrypted channel
    - Subscribe to WebSocket topics (battery, status, GPS)
    - Receive push notifications (battery low, mowing started, etc.)

---

## Wire Protocol Details

### LAN Pairing API Endpoints (Robot's HTTP listener, LAN-only)

All endpoints are protected by the `lanOnly` middleware: requests from non-RFC1918 IPs receive 403 Forbidden.

#### `POST /api/pair/start`

**Request:**
```json
{
  "uid": "string",           // Firebase UID
  "idToken": "string",       // Firebase ID token (JWT)
  "displayName": "string",   // User's display name
  "setupToken": "string"     // Hex-encoded setup token from QR code
}
```

**Validation:**
- setupToken must match pairing.Service.setupToken AND not be expired (> 30 min old)
- uid must be syntactically valid (non-empty)
- On success, setupToken is immediately consumed (single-use) and cleared from memory
- On failure, pairing.Status.State remains Unstarted

**Response (200 OK):**
```json
{
  "confirmCode": "5729",
  "expiresAt": "2026-05-01T10:05:00Z"
}
```

**Error response (400 Bad Request):**
```json
{
  "error": "pairing: setup token expired"
}
```

---

#### `POST /api/pair/confirm`

**Request:**
```json
{
  "setupToken": "string"  // Hex-encoded setup token (for API symmetry, not used)
}
```

**Validation:**
- pairing.Status.State must be Pending (check the state machine)
- On success, Confirm fires the OnPaired callback (wired to cloud_sync.Sync.RegisterRobot in main.go)
- If OnPaired returns error, state transitions to Failed (and is sticky until Reset is called)
- On success, state transitions to Paired

**Response (200 OK):**
```json
{
  "ok": true
}
```

**Error response (400 Bad Request):**
```json
{
  "error": "pairing: confirm called in state 0, want Pending"
}
```

---

#### `GET /api/pair/status`

**Response (200 OK):**
```json
{
  "state": 1,                                    // 0=Unstarted, 1=Pending, 2=Confirmed, 3=Failed, 4=Paired
  "robotID": "ABC123",
  "ownerUID": "firebase-uid-abc123xyz",
  "ownerName": "Alice",
  "confirmCode": "5729",                        // Only present when state == 1 (Pending)
  "error": ""                                   // Only non-empty when state == 3 (Failed)
}
```

**Note:** `confirmCode` is **only populated when State == Pending**. Once confirmed, it is cleared from the snapshot to prevent accidental leakage.

---

#### `GET /api/pair/qr.png`

**Response (200 OK):**
- Content-Type: `image/png`
- 256×256 PNG image (QR code)
- Encodes the payload from `GET /api/pair/qr`

**Error response (503 Service Unavailable):**
```json
{
  "error": "no active setup token"
}
```

---

#### `GET /api/pair/qr`

**Response (200 OK):**
```json
{
  "payload": "mowgli://pair?v=1&rid=ABC123&pub=...",
  "robotID": "ABC123",
  "setupToken": "a1b2c3d4..."
}
```

**Use case:** Clients that render their own QR code (e.g., e-ink display on robot)

---

### QR Payload Format

```
mowgli://pair?v=1&rid=ABC123&pub=<base64-pubkey>&tok=<hex-token>&lan=192.168.1.100&cf=
```

| Parameter | Example | Notes |
|-----------|---------|-------|
| `v` | `1` | Protocol version; future-proofs format changes |
| `rid` | `ABC123` | Robot ID (stable identifier, derived from Curve25519 pub key) |
| `pub` | `<base64>` | Curve25519 public key (base64-encoded, 32 bytes) |
| `tok` | `a1b2c3d4...` | Setup token (hex-encoded, 32 bytes = 64 hex chars) |
| `lan` | `192.168.1.100` | [Optional] LAN IP for direct connection fallback |
| `cf` | `` | [Reserved] Cloudflare tunnel domain (empty until after pairing completes) |

**Why base64 for pub key, but hex for token?** 
- Pub key is fixed at generation time, base64 is standard for binary crypto material
- Token is generated fresh each boot and displayed in URLs, hex avoids padding ambiguity

**Why include pub key in QR?**
- Mobile app trusts the public key from the QR code (TOFU)
- App stores it and uses it for all future Noise IK handshakes
- If attacker replaces QR with their own, comparison of confirmed codes catches the swap

---

### 4-Digit Confirm Code Derivation

**Current (random):**
```
randomFourDigit():
  Read 2 random bytes from crypto/rand
  n = BigEndian.Uint16(bytes) % 10000
  return fmt.Sprintf("%04d", n)  // leading zeros preserved
```

Result: "0000" through "9999", all equally likely.

**Future enhancement:**
Once the Noise IK handshake completes during pairing (currently it doesn't — pairing is HTTP), we can derive the code from the handshake state:
```
confirmCode = BLAKE2s(handshakeHash)[0:2]    // first 4 decimal digits of the hash
```

This binds the code to the cryptographic handshake, making MITM detection even stronger. Requires architectural change to run Noise during pairing (currently Noise only after pairing completes).

---

### LAN-Only Enforcement

The `lanOnly` middleware inspects the HTTP request's effective source IP:

1. Check `X-Forwarded-For` header (set by cloudflared on LAN)
2. Fall back to `X-Real-IP` header (if set by proxy)
3. Fall back to `RemoteAddr` (socket address)

**Allowed ranges:**
- `10.0.0.0/8` (RFC 1918 private)
- `172.16.0.0/12` (RFC 1918 private)
- `192.168.0.0/16` (RFC 1918 private)
- `127.0.0.0/8` (loopback)
- `::1/128` (IPv6 loopback)
- `fc00::/7` (IPv6 unique local)
- `fe80::/10` (IPv6 link-local)

**What happens for VPN'd clients?**
- If user connects to a personal VPN that routes their traffic through `192.168.x.x`, they are allowed
- If user is on a corporate VPN with a non-RFC1918 address, pairing endpoints reject them (403 Forbidden)
- Corporate VPN users must pair over LAN or via direct IP if on same subnet

---

## Setup Token Lifecycle

```
Robot boots:
  ↓
pairing.StartFirstBoot():
  token := rand(32 bytes)
  setupToken := hex(token)                  // "a1b2c3d4e5f6..."
  tokenExpiry := now + 30 minutes
  state := Unstarted
  
  ↓
  [QR code generated, user scans]
  ↓
  
User calls /api/pair/start with setupToken:
  ↓
Service.Start(req):
  Validate req.setupToken == s.setupToken
  Validate now < s.tokenExpiry
  s.setupToken = ""                        // CONSUME immediately
  state := Pending
  Return confirmCode
  
  ↓
  [User confirms on both screens]
  ↓
  
User calls /api/pair/confirm:
  ↓
Service.Confirm():
  Validate state == Pending
  state := Confirmed
  Call OnPaired callback (fires cloud_sync.RegisterRobot)
  state := Paired
  
  ↓
  [Robot now reachable via Noise IK]
  ↓
  
If something fails (wrong token, network error during confirm):
  ↓
pairing.Reset():
  Calls StartFirstBoot() again
  state := Unstarted
  NEW token generated (old token is forgotten)
```

**Important:** SetupToken is **single-use**. Calling `/api/pair/start` with the same token a second time is rejected ("invalid setup token"). A new token must be generated via Reset().

**TTL enforcement:** Tokens expire after 30 minutes. If the user scans the QR code but waits 35 minutes before calling `/api/pair/start`, the token is rejected ("setup token expired"). The QR code becomes invalid.

---

## Recovery Flows

### Pairing Timeout

**Scenario:** User scans QR, but then doesn't confirm for 1 hour.

**What happens:**
- QR code's setupToken is still in memory
- `/api/pair/start` succeeds if called within 30 min of robot boot
- After 30 min, `/api/pair/start` is rejected ("setup token expired")

**Recovery:**
- User must restart the pairing flow
- Operator calls `/api/pair/reset` (HTTP endpoint, LAN-only, may not exist — discuss with team)
- Or: robot reboots, fresh token generated

---

### Mismatched 4-Digit Code

**Scenario:** Web wizard shows "5729", but mobile app shows "2857".

**What happens:**
- Code mismatch indicates MITM or bug
- User must NOT confirm (will lock pairing in an invalid state)
- Both screens show error: "Code mismatch — restart pairing"

**Recovery:**
- User cancels on both screens
- Reset the robot: `pairing.Reset()` (triggered by reboot or admin endpoint)
- Rescan QR code, new token, new code

---

### Cloud Function Failure During Pairing

**Scenario:** Mobile app calls `/api/pair/confirm`, robot state → Confirmed, OnPaired fires, but Firestore write fails.

**What happens:**
- OnPaired callback in cloud_sync.RegisterRobot returns error
- Service.Confirm() sees error, transitions state: Confirmed → Failed
- pairing.Status.error is set to the error message

**Recovery:**
- Operator inspects logs: `firebase functions:logs` or `gcloud functions logs read pairRobot`
- Most common: Cloudflare API token is invalid (check secret)
- Fix the issue, then reset pairing: `pairing.Reset()`
- Retry the flow

---

### Partial Failure: Cloud Function Succeeded but Robot Didn't See Confirm

**Scenario:** Network drops after cloud_sync.RegisterRobot completes. Robot never receives the Firestore write confirmation.

**What happens:**
- Firestore now has `robots/{robotId}` with ownerUid
- Robot's pairing.Status.state is still Pending or Failed
- Mobile app shows "Confirm successful, robot is paired"
- Next time robot boots, cloud_sync.Sync.Run() fetches `robots/{robotId}`, finds allowedUids
- Robot's in-memory allow-list is synced

**Recovery:**
- No recovery needed — allow-list syncs on next Firestore snapshot
- If robot was in Failed state, operator must manually reset pairing or restart

---

## Multi-User Invite/Revoke Flow

### Invite Another User

**Owner (already paired) invites a friend:**

1. Friend signs up for MowgliNext account (Firebase Auth)
2. Owner calls `inviteUser({ robotId: "ABC123", inviteeEmail: "friend@example.com" })` Cloud Function
3. Function validates:
   - Caller is authenticated (has valid Firebase ID token)
   - Caller is the owner of `robots/ABC123` (checks ownerUid in Firestore)
   - Invitee exists in Firebase Auth (queries by email)
4. Function writes to Firestore:
   ```
   robots/ABC123/allowedUids = [owner_uid, invitee_uid]
   users/{invitee_uid}/robots/ABC123 = { addedAt, addedBy: owner_uid, role: "invited" }
   ```
5. Cloud_sync listener picks up the snapshot (within ~1s) and updates allow-list
6. Friend can now connect over Noise IK (uid is in allow-list)

---

### Revoke Access

**Owner revokes a user:**

1. Owner calls `revokeUser({ robotId: "ABC123", uid: "invitee-uid" })` Cloud Function
2. Function validates:
   - Caller is the owner
   - Target uid is not the owner (cannot revoke self)
3. Function removes uid from `robots/{robotId}/allowedUids` array
4. Function hard-deletes `users/{invitee_uid}/robots/ABC123`
5. Cloud_sync listener picks up snapshot and removes uid from allow-list
6. Within ~1s, the revoked user cannot handshake (uid not in allow-list)
7. Existing connections from that uid remain open (no active disconnection)

---

## Firestore Rules Semantics

The `firestore.rules` file enforces the following:

- **Only authenticated users (with valid Firebase Auth token) can call Functions**
- **Only the owner can invite/revoke users**
- **Only the owner or invited user can read their own robot data**
- **Allow-list mutations are validated: only Firebase Functions can modify allowedUids**
- **Device tokens can be registered by any authenticated user** (user registers their own phone's FCM token)

See `cloud/firebase/firestore.rules` for the exact Firestore Security Rules.

---

## Operator Command Reference

### Bring Up the Cloudflare Tunnel

Once pairing has written credentials:

```bash
# Activate the cloudflared overlay (requires pairing credentials to already exist)
docker compose \
  -f install/compose/docker-compose.base.yml \
  -f install/compose/docker-compose.gui.yml \
  -f install/compose/docker-compose.cloudflared.yml up -d

# Verify tunnel is connected
docker logs mowgli-cloudflared | tail -20
# Should show: "Your tunnel is running! ..."
```

### Check Pairing State

```bash
# Query the robot's pairing status (over LAN HTTP)
curl -s http://mowgli.local/api/pair/status | jq .
# Output:
# {
#   "state": 4,              # 4 = Paired
#   "robotID": "ABC123",
#   "ownerUID": "uid-...",
#   "ownerName": "Alice",
#   "confirmCode": "",       # Empty when paired
#   "error": ""
# }
```

### Get QR Code (for re-pairing another user)

```bash
# If you need to reset and re-pair
curl -s http://mowgli.local/api/pair/qr | jq .
# Output:
# {
#   "payload": "mowgli://pair?v=1&rid=ABC123&pub=...",
#   "robotID": "ABC123",
#   "setupToken": "..."
# }
```

Or download PNG:
```bash
curl -s http://mowgli.local/api/pair/qr.png > qr.png
# Display on a monitor, scan with phone
```

---

## Troubleshooting

| Issue | Cause | Fix |
|-------|-------|-----|
| "Setup token invalid" on /api/pair/start | Token doesn't match current token, or bot is in non-Unstarted state | Restart robot, rescan QR |
| "Setup token expired" on /api/pair/start | >30 min have passed since boot | Restart robot |
| Code mismatch (5729 vs 2857) | MITM on LAN or mobile app bug | Do not confirm; reset pairing |
| `/api/pair/confirm` returns 400 "state != Pending" | /start was called twice, or state is Failed | Check logs, reset with `pairing.Reset()` |
| Tunnel creation fails (Cloudflare API error) | Invalid Cloudflare credentials or account limits | Check Firebase secrets, regenerate token |
| Robot stays "offline" in app | Tunnel is not connected, or heartbeat is not running | Check `docker logs mowgli-cloudflared` and `docker logs mowgli-ros2` |
| "uid not in allow-list" on Noise handshake | User was revoked or invite failed | Owner must re-invite user, check Firestore docs |
