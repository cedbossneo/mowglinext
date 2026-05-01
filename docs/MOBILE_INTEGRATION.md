# MowgliNext Mobile Companion: Operator Integration Runbook

**Audience:** DevOps engineers and product teams bringing up the mobile companion for the first time.

This runbook walks you through deploying the mobile backend and wiring the robot to support mobile clients. All steps must be completed before the first user can pair a robot.

---

## Prerequisites

- **GCP Account** with an existing or new project (recommend `mowglinext-prod` or similar)
- **Cloudflare Account** with access to the `mowgli.garden` zone (or your own domain)
- **Local machine:** Node.js 22+, npm, Firebase CLI, `gcloud` CLI
- **Robot:** Running the MowgliNext GUI binary with support for `MOWGLI_MOBILE_TUNNEL=1`

---

## 1. Create Firebase Project and Enable Auth

1. Go to [Firebase Console](https://console.firebase.google.com)
2. Click **Add project**
3. Name it (e.g., `mowglinext-prod`)
4. Disable Google Analytics (optional)
5. Click **Create**
6. Wait for provisioning (2–3 minutes)

### Configure Authentication Providers

1. In the Firebase console, go to **Build → Authentication → Sign-in method**
2. Enable these providers:
   - **Email/Password** — for developer and power-user accounts
   - **Google** — recommended for mobile UX
   - **Apple** — required for iOS App Store
3. Go to **Settings → Authorized domains**
   - The default domain `*.firebaseapp.com` is already added
   - Add any custom domains you use (e.g., `mowgli.garden`)

### Capture Project ID

1. Go to **Project settings** (gear icon → Project settings)
2. Note the **Project ID** (e.g., `mowglinext-prod`)
3. Note the **Project Number** (used for FCM)
4. Keep this tab open; you'll need these values below

---

## 2. Create Firestore Database (Native Mode)

1. In the Firebase console, go to **Build → Firestore Database → Create database**
2. Choose **Native mode** (not Datastore mode)
3. Select a region close to your users (recommend `us-central1` to co-locate with Cloud Functions)
4. Click **Create**
5. Wait for provisioning (1–2 minutes)

The database starts with an empty security rules file. We'll deploy the correct rules in step 8.

---

## 3. Enable Firebase Cloud Messaging (FCM)

1. Go to **Build → Cloud Messaging**
2. Confirm **Cloud Messaging API** is enabled (should be automatic)
3. Go to **Cloud Messaging** → **Sender ID**
   - Note the **Sender ID** (usually the Project Number)
   - This is used by mobile devices to register for push notifications
4. Go to **Project settings** → **Cloud Messaging**
   - Note the **Server API key** (used by backend to send messages)

---

## 4. Create a Service Account for Cloud Functions

Cloud Functions v2 runs as the default App Engine service account. For local testing and CI/CD, you'll need a service account JSON key.

1. Go to **Project settings → Service accounts**
2. Click **Generate new private key**
3. Download the JSON file (e.g., `mowglinext-prod-key.json`)
4. **Keep this file secure** — it grants full Firestore read/write access
5. On your local machine, set:
   ```bash
   export GOOGLE_APPLICATION_CREDENTIALS=/path/to/mowglinext-prod-key.json
   ```

---

## 5. Create Cloudflare API Token

Cloud Functions will create and delete Cloudflare Tunnels during the pairing flow.

1. Go to https://dash.cloudflare.com/profile/api-tokens
2. Click **Create Token** → **Create Custom Token**
3. Configure permissions:
   - **Account** → **Cloudflare Tunnel** → **Edit**
   - **Zone** → **DNS** → **Edit** (scoped to `mowgli.garden`)
4. Under **Account Resources**, select your Cloudflare account
5. Click **Continue to summary** → **Create Token**
6. **Copy the token value immediately** (shown only once)

Also collect:
- **Account ID** from Cloudflare Dashboard home (right sidebar)
- **Zone ID** from `mowgli.garden` overview page (right sidebar)

You now have three Cloudflare values:
- `CLOUDFLARE_ACCOUNT_ID` (UUID, ~36 chars)
- `CLOUDFLARE_API_TOKEN` (string, ~40 chars)
- `CLOUDFLARE_ZONE_ID` (UUID, ~32 chars)

---

## 6. Set Firebase Secrets

Cloud Functions read secrets from Firebase Secret Manager. These credentials are stored securely and never appear in source control.

```bash
cd cloud/firebase

# Authenticate with Firebase (if not already)
firebase login

# Set each secret (you'll be prompted to paste the value)
firebase functions:secrets:set CLOUDFLARE_ACCOUNT_ID
# Paste: <your-account-id>

firebase functions:secrets:set CLOUDFLARE_API_TOKEN
# Paste: <your-api-token>

firebase functions:secrets:set CLOUDFLARE_ZONE_ID
# Paste: <your-zone-id>

firebase functions:secrets:set TUNNEL_DOMAIN
# Paste: tunnel.mowgli.garden  (or your own subdomain base)

# Verify secrets were stored
firebase functions:secrets:list
# Should show: CLOUDFLARE_ACCOUNT_ID, CLOUDFLARE_API_TOKEN, CLOUDFLARE_ZONE_ID, TUNNEL_DOMAIN
```

---

## 7. Deploy Cloud Functions and Firestore Rules

```bash
cd cloud/firebase

# Install dependencies (first time only)
npm install
npm run build

# Verify TypeScript compiles
npm run typecheck

# Run tests (mocked, no credentials needed)
npm test

# Deploy to Firebase
firebase login  # if not already authenticated
firebase deploy
# Output should show:
#   ✓ functions[pairRobot]
#   ✓ functions[inviteUser]
#   ✓ functions[revokeUser]
#   ✓ functions[registerFcmToken]
#   ✓ functions[deleteRobot]
#   ✓ firestore:rules
#   ✓ firestore:indexes

# First deployment takes 2–5 minutes (Cloud Functions are built and pushed to Cloud Run)
```

**Note:** Cloud Functions v2 requires the **Blaze (pay-as-you-go)** billing plan. If you're on the Spark (free) tier, you must upgrade at https://console.firebase.google.com/project/YOUR_PROJECT_ID/usage/details

---

## 8. Configure Mobile App Environment

The mobile app reads Firebase configuration from a `.env.local` file.

1. In your Firebase project **Project settings**, find these values:
   - `apiKey` (Web API key, from "Your apps" section)
   - `projectId` (from step 1)
   - `messagingSenderId` (Project Number)
   - `appId` (from "Your apps" section, iOS/Android app)
   - `vapidKey` (Web Push certificate, if you set one up)

2. Create `app/mobile/.env.local`:
   ```
   VITE_FIREBASE_API_KEY=AIzaSyAA...
   VITE_FIREBASE_PROJECT_ID=mowglinext-prod
   VITE_FIREBASE_MESSAGING_SENDER_ID=123456789012
   VITE_FIREBASE_APP_ID=1:123456789012:web:abc123def456
   VITE_FIREBASE_VAPID_KEY=BF...  # or leave empty for now
   ```

3. Build the app:
   ```bash
   cd app/mobile
   npm install
   npm run build
   
   # For iOS (requires macOS)
   npx cap add ios
   npx cap copy
   open ios/App/App.xcworkspace  # Opens Xcode
   
   # For Android
   npx cap add android
   npx cap copy
   open android  # Opens Android Studio
   ```

---

## 9. Wire main.go on the Robot

The robot's `main.go` must initialize the pairing, cloud_sync, and noise_shim components. **Do not modify the existing GUI features.**

Add this code (approximately 30 lines) to `gui/main.go` in the initialization section (after ROS setup, before `http.ListenAndServe`):

```go
// --- Mobile Companion (opt-in via MOWGLI_MOBILE_TUNNEL env var) ---
if os.Getenv("MOWGLI_MOBILE_TUNNEL") == "1" {
    // Pairing state machine
    robotID := "ABC123"  // Replace with your robot ID (derived from public key)
    privKey := loadOrGenerateNoiseKey()  // Helper: load from /var/lib/mowgli/noise.key or gen
    pubKey := derivePublicKey(privKey)
    pairingSvc := pairing.NewService(robotID, pubKey)
    pairingSvc.StartFirstBoot()

    // Allow-list (synced from Firestore)
    allowList := noise_shim.NewMemoryAllowList(nil)

    // Firebase JWT verifier (cached JWK, 6h refresh)
    projectID := os.Getenv("FIREBASE_PROJECT_ID")  // e.g., "mowglinext-prod"
    jwkCachePath := filepath.Join(os.Getenv("MOWGLI_STATE_DIR"), "google-jwk.json")
    verifier := noise_shim.NewFirebaseVerifier(projectID, jwkCachePath)

    // Cloud sync (Firestore listener + FCM notifier)
    fsProjectID := os.Getenv("FIREBASE_PROJECT_ID")
    fsCredsPath := filepath.Join(os.Getenv("MOWGLI_STATE_DIR"), "firestore-credentials.json")
    syncOpts := cloud_sync.SyncOptions{
        ProjectID:             fsProjectID,
        RobotID:               robotID,
        ServiceAccountJSON:    fsCredsPath,
        AllowList:             allowList,
        Logger:                logrus.WithField("c", "cloud_sync"),
    }
    cloudSync, err := cloud_sync.NewSync(ctx, syncOpts)
    if err != nil {
        logrus.WithError(err).Fatal("cloud_sync.NewSync failed")
    }
    defer cloudSync.Close()

    // Noise shim (reverse proxy to localhost:4006, runs Noise IK handshake)
    noiseServer := noise_shim.New(noise_shim.Config{
        StaticPriv: privKey,
        RobotID:    robotID,
        RobotName:  "Garden Mower",  // Or load from config
        Version:    "0.1.0",
        Allow:      allowList,
        Verifier:   verifier,
        Proxy:      noise_shim.NewLocalHTTPProxy("http://127.0.0.1:4006"),
        Logger:     logrus.WithField("c", "noise_shim"),
    })

    // Register pairing routes (LAN-only HTTP)
    pairing.RegisterRoutes(router.Group("/"), pairingSvc)

    // Register Noise shim (WebSocket on /tunnel, port 8443 or behind cloudflared)
    tunnelListener, err := net.Listen("tcp", "127.0.0.1:8443")
    if err != nil {
        logrus.WithError(err).Fatal("noise_shim listen failed")
    }
    go func() {
        httpServer := &http.Server{Handler: noiseServer.Handler()}
        if err := httpServer.Serve(tunnelListener); err != nil {
            logrus.WithError(err).Error("noise_shim server error")
        }
    }()

    // Start Firestore listener (keeps allow-list in sync)
    go func() {
        if err := cloudSync.Run(ctx); err != nil {
            logrus.WithError(err).Error("cloud_sync.Run error")
        }
    }()

    // Periodically send heartbeat to Firestore
    go func() {
        ticker := time.NewTicker(5 * time.Minute)
        defer ticker.Stop()
        for {
            select {
            case <-ctx.Done():
                return
            case <-ticker.C:
                if err := cloudSync.Heartbeat(ctx); err != nil {
                    logrus.WithError(err).Warn("heartbeat failed")
                }
            }
        }
    }()

    // Event detector (subscribes to ROS topics, sends FCM notifications)
    notifierOpts := cloud_sync.NotifierOptions{
        ProjectID:          fsProjectID,
        ServiceAccountJSON: fsCredsPath,
        Logger:             logrus.WithField("c", "notifier"),
    }
    notifier, err := cloud_sync.NewNotifier(ctx, notifierOpts)
    if err != nil {
        logrus.WithError(err).Error("NewNotifier failed")
    }
    defer notifier.Close()

    detector := cloud_sync.NewEventDetector(cloud_sync.DetectorOptions{
        ROS: rosProvider,  // Your existing ROS provider
        Notify: func(p cloud_sync.Payload) error {
            return notifier.SendToRobot(ctx, robotID, p)
        },
    })
    go func() {
        if err := detector.Run(ctx); err != nil {
            logrus.WithError(err).Error("event_detector.Run error")
        }
    }()

    logrus.WithField("robotID", robotID).Info("mobile companion initialized")
}
```

**Key notes:**
- Replace `"ABC123"` with your robot ID (a stable identifier, ideally derived from the Noise public key)
- The robot must set `FIREBASE_PROJECT_ID`, `MOWGLI_STATE_DIR` env vars
- For local testing, you can hardcode values or use a config file
- `cloudflared` will be started separately via Docker Compose overlay

---

## 10. Build Robot Image and Deploy

```bash
# From the monorepo root
make build-gui

# Or with Docker (if you're using containers)
docker build -f gui/Dockerfile -t mowgli-gui:latest .

# Run with mobile tunnel enabled
export FIREBASE_PROJECT_ID=mowglinext-prod
export MOWGLI_STATE_DIR=/var/lib/mowgli
export MOWGLI_MOBILE_TUNNEL=1
./openmower-gui
```

---

## 11. Start the Cloudflare Tunnel (After Pairing)

The tunnel credentials are written by the pairing flow. Once pairing completes, the robot has:
- `${MOWGLI_STATE_DIR}/cloudflared/credentials.json`
- `${MOWGLI_STATE_DIR}/cloudflared/config.yml`

Then activate the tunnel:

```bash
# Activate the cloudflared overlay
docker compose -f docker-compose.yaml -f docker-compose.cloudflared.yaml up -d

# Verify the tunnel is connected
docker logs mowgli-cloudflared | tail -20
# Should show: "Your tunnel is running! Ready to accept traffic."
```

---

## 12. Verify the Deployment

### Test Firebase Functions

```bash
curl -X POST \
  https://us-central1-mowglinext-prod.cloudfunctions.net/pairRobot \
  -H "Content-Type: application/json" \
  -d '{"data":{}}'
# Expected: {"error":{"status":"UNAUTHENTICATED",...}} (correct — no auth token)
```

### Test Robot HTTP API

```bash
# From a machine on the same LAN as the robot
curl -s http://mowgli.local/api/pair/status | jq .
# Output:
# {
#   "state": 0,
#   "robotID": "ABC123",
#   "ownerUID": "",
#   "ownerName": "",
#   "confirmCode": "",
#   "error": ""
# }
```

### Test Noise Shim (After Pairing)

```bash
# Once the tunnel is connected
curl -s https://r-ABC123.tunnel.mowgli.garden/tunnel
# Expected: 101 Switching Protocols (WebSocket upgrade) or 403 Forbidden (no Noise handshake)
```

---

## 13. First Pairing Flow

1. **Power on the robot** with `MOWGLI_MOBILE_TUNNEL=1`
2. **Open onboarding** at `http://mowgli.local/onboarding`
3. **Navigate to step 6** ("Pair Mobile App")
4. **Scan the QR code** with the mobile app (TestFlight/Play Store)
5. **Install the app** if not already done
6. **Confirm the 4-digit code** on both screens
7. **Wait** for the Cloud Function to create the Cloudflare tunnel (~5–10 seconds)
8. **Activate cloudflared** (step 11 above) or it will auto-start via Docker Compose
9. **Open the app** and verify the robot is online

---

## Troubleshooting

| Issue | Diagnosis | Fix |
|-------|-----------|-----|
| "Cloud Messaging API not enabled" | FCM is not turned on | Enable it in Firebase console → Cloud Messaging |
| "Blaze plan required" | Using free Spark tier | Upgrade billing at Firebase console → Usage & billing → Upgrade |
| "Cloudflare API token is invalid" | Token was malformed or expired | Regenerate token at https://dash.cloudflare.com/profile/api-tokens |
| Tunnel creation fails in pairRobot | Cloudflare credentials are wrong | Check `firebase functions:secrets:list` and verify values |
| Robot stays "offline" in app | Tunnel is not connected | Check `docker logs mowgli-cloudflared` and verify credentials exist |
| "uid not in allow-list" on app connect | Allow-list sync failed | Check `docker logs mowgli-ros2` for `cloud_sync` errors |
| Pairing hangs on confirm screen | Cloud Function is timing out | Check `firebase functions:logs` and verify internet connectivity |
| Mobile app build fails | Env vars are missing or wrong | Verify `app/mobile/.env.local` has all VITE_FIREBASE_* values |

---

## Operational Checklist

Before going live:

- [ ] Firebase project created and Auth enabled (Email, Google, Apple)
- [ ] Firestore database in Native mode
- [ ] FCM enabled and Sender ID captured
- [ ] Service account key generated and secured
- [ ] Cloudflare API token created (Tunnel Edit + DNS Edit)
- [ ] All secrets set in Firebase (`CLOUDFLARE_*`, `TUNNEL_DOMAIN`)
- [ ] Cloud Functions deployed (`firebase deploy`)
- [ ] Mobile app `.env.local` configured with Firebase values
- [ ] Robot `main.go` wired with pairing, cloud_sync, noise_shim
- [ ] Robot built with `MOWGLI_MOBILE_TUNNEL=1` support
- [ ] First pairing tested end-to-end
- [ ] Cloudflare tunnel connects and stays active
- [ ] Mobile app can connect and receive telemetry
- [ ] Cloud Audit Logging enabled for Firestore
- [ ] Backups enabled for Firestore database
- [ ] Monitoring set up (Firestore quota alerts, Function error rates, Tunnel uptime)

---

## Next Steps

- For pairing protocol details, see `docs/MOBILE_PAIRING.md`
- For security considerations, see `docs/MOBILE_SECURITY.md`
- For architecture deep-dive, see `docs/MOBILE_ARCHITECTURE.md`
- For Firebase emulator testing, see `cloud/firebase/SETUP.md` § "Local Emulator Setup"
