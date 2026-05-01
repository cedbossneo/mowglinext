# MowgliNext Firebase Backend — Setup Guide

This guide walks you through deploying the Firebase backend that powers the
MowgliNext mobile companion app. It covers creating the Firebase project,
configuring Cloudflare tunnels, setting secrets, and running the emulator
locally.

---

## Prerequisites

- Node.js 22 and npm installed
- Firebase CLI: `npm install -g firebase-tools`
- A Cloudflare account with access to the `mowgli.garden` zone (or your own domain)
- A Google account for Firebase

---

## 1. Create the Firebase Project

1. Go to https://console.firebase.google.com and click **Add project**.
2. Name it (e.g. `mowglinext-prod`). Disable Google Analytics if not needed.
3. Note the **Project ID** shown in the project settings (e.g. `mowglinext-prod`).
4. Edit `.firebaserc` in this directory and replace `REPLACE_WITH_PROJECT_ID`
   with your actual project ID.

---

## 2. Enable Firebase Authentication

1. In the Firebase Console, go to **Build → Authentication → Sign-in method**.
2. Enable the following providers:
   - **Email/Password** — for developer and power-user accounts
   - **Google** — recommended for mobile convenience
   - **Apple** — required for iOS App Store submission
3. Under **Settings → Authorized domains**, add any custom domains you use.

---

## 3. Create Firestore in Native Mode

1. Go to **Build → Firestore Database → Create database**.
2. Choose **Native mode** (not Datastore mode).
3. Select a region close to your users (e.g. `us-central1` to match Functions).
4. Security rules and indexes are deployed automatically in step 8.

---

## 4. Service Account for Cloud Functions

Cloud Functions v2 run as the default App Engine service account, which already
has Firestore read/write and FCM send access. No manual service account
configuration is needed for standard deployments.

If you need a custom service account (e.g. for CI/CD pipelines):

1. Go to **Project Settings → Service accounts**.
2. Click **Generate new private key** and download the JSON file.
3. Set `GOOGLE_APPLICATION_CREDENTIALS=/path/to/key.json` in your shell before
   running `firebase deploy`.

---

## 5. Create a Cloudflare API Token

The Functions create and delete Cloudflare Tunnels and DNS records during the
robot pairing and deletion flows.

1. Go to https://dash.cloudflare.com/profile/api-tokens.
2. Click **Create Token** → **Create Custom Token**.
3. Configure the following permissions:
   - **Account** → **Cloudflare Tunnel** → **Edit**
   - **Zone** → **DNS** → **Edit** — scoped to zone `mowgli.garden`
4. Under **Account Resources**, select your Cloudflare account.
5. Click **Continue to summary** → **Create Token**.
6. Copy the token value immediately — it is shown only once.

Also collect these two values from the Cloudflare Dashboard:
- **Account ID**: right sidebar on the Cloudflare Dashboard home page
- **Zone ID**: right sidebar on the `mowgli.garden` zone overview page

---

## 6. Set Firebase Secrets

Cloud Functions v2 reads secrets from Firebase Secret Manager. These values
are never stored in source control. Run the following from the `cloud/firebase/`
directory (you will be prompted to paste each value):

```bash
firebase functions:secrets:set CLOUDFLARE_ACCOUNT_ID
firebase functions:secrets:set CLOUDFLARE_API_TOKEN
firebase functions:secrets:set CLOUDFLARE_ZONE_ID
firebase functions:secrets:set TUNNEL_DOMAIN
# Enter: tunnel.mowgli.garden  (or your own subdomain base domain)
```

To verify a secret was stored correctly:
```bash
firebase functions:secrets:access CLOUDFLARE_ACCOUNT_ID
```

To list all secrets:
```bash
firebase functions:secrets:list
```

---

## 7. Install Dependencies and Build

```bash
cd cloud/firebase/functions
npm install
npm run build
```

Verify TypeScript compiles without errors:
```bash
npm run typecheck
```

Run the test suite (all external calls are mocked — no credentials needed):
```bash
npm test
```

---

## 8. Deploy to Firebase

From the `cloud/firebase/` directory:

```bash
# Log in if not already authenticated
firebase login

# Deploy everything: Functions, Firestore rules, and indexes
firebase deploy

# Or deploy individual targets:
firebase deploy --only functions
firebase deploy --only firestore:rules
firebase deploy --only firestore:indexes
```

The first deployment takes 2–5 minutes as Cloud Functions are built and
pushed to Cloud Run.

> **Note:** Cloud Functions v2 requires the **Blaze (pay-as-you-go)** billing
> plan. The free tier does not support v2 functions. Upgrade at
> https://console.firebase.google.com/project/YOUR_PROJECT_ID/usage/details

---

## 9. Verify the Deployment

After a successful deploy, the Firebase Console shows your functions under
**Build → Functions**:

| Function | Trigger | Region |
|---|---|---|
| `pairRobot` | HTTPS callable | us-central1 |
| `inviteUser` | HTTPS callable | us-central1 |
| `revokeUser` | HTTPS callable | us-central1 |
| `registerFcmToken` | HTTPS callable | us-central1 |
| `deleteRobot` | HTTPS callable | us-central1 |

Test that the functions are reachable (expect `unauthenticated` — correct
behaviour when called without a Firebase Auth token):

```bash
curl -X POST \
  https://us-central1-YOUR_PROJECT_ID.cloudfunctions.net/pairRobot \
  -H "Content-Type: application/json" \
  -d '{"data":{}}'
# Expected: {"error":{"status":"UNAUTHENTICATED", ...}}
```

---

## 10. Local Emulator Setup

Use the Firebase emulator suite for development without touching production
resources:

```bash
# Install emulators (first time only)
firebase setup:emulators:firestore
firebase setup:emulators:functions
firebase setup:emulators:auth

# Start emulators (Functions + Firestore + Auth)
firebase emulators:start --only functions,firestore,auth
```

The Emulator UI is available at http://localhost:4000.

To provide Cloudflare credentials to the emulated Functions, create a
`functions/.env.local` file (this file is gitignored):

```
CLOUDFLARE_ACCOUNT_ID=your-test-account-id
CLOUDFLARE_API_TOKEN=your-test-token
CLOUDFLARE_ZONE_ID=your-test-zone-id
TUNNEL_DOMAIN=tunnel.mowgli.garden
```

Then start the emulator normally — it picks up `.env.local` automatically.

---

## 11. Robot Pairing Flow (End-to-End)

When a user scans the robot's QR code in the mobile app, the app calls
`pairRobot` with:

```json
{
  "robotId": "ABC123",
  "robotName": "Garden Mower",
  "robotPubKey": "<base64-encoded Curve25519 public key from robot>",
  "setupToken": "<one-time token from the robot's local web UI>"
}
```

The Function:
1. Verifies the caller is authenticated (Firebase Auth)
2. Checks the robot is not already claimed
3. Creates a Cloudflare tunnel named `mowgli-ABC123`
4. Creates a CNAME DNS record `r-ABC123.tunnel.mowgli.garden → <tunnelId>.cfargotunnel.com`
5. Writes `robots/ABC123` and `users/{uid}/robots/ABC123` to Firestore
6. Returns `{ tunnelHostname, credentialsJson }` to the mobile app

The mobile app sends `credentialsJson` to the robot's local HTTP API. The
robot writes it to its cloudflared configuration and connects outbound to
Cloudflare. The robot is then reachable at
`https://r-ABC123.tunnel.mowgli.garden` through the Noise IK encrypted tunnel.

---

## 12. User Management

**Invite a user:**
```js
const invite = httpsCallable(functions, 'inviteUser');
await invite({ robotId: 'ABC123', inviteeEmail: 'friend@example.com' });
// The invitee must already have a MowgliNext account.
```

**Revoke access:**
```js
const revoke = httpsCallable(functions, 'revokeUser');
await revoke({ robotId: 'ABC123', uid: 'uid-of-user-to-remove' });
// Cannot revoke the owner. Use deleteRobot to fully decommission.
```

**Delete robot:**
```js
const del = httpsCallable(functions, 'deleteRobot');
await del({ robotId: 'ABC123' });
// Deletes Cloudflare tunnel + DNS + all Firestore docs for this robot.
```

---

## Troubleshooting

**`Cloudflare API token is invalid or missing required scopes`**
Verify the token has both **Cloudflare Tunnel: Edit** (Account scope) and
**DNS: Edit** (Zone scope for `mowgli.garden`). Regenerate the token if needed
and update the secret: `firebase functions:secrets:set CLOUDFLARE_API_TOKEN`.

**`Robot ABC123 is already paired`**
The `robots/ABC123` document already exists with a different `setupToken`.
Delete the robot first via `deleteRobot`, then re-pair.

**`No user found with email ...`**
The invitee must sign up for a MowgliNext account before they can be invited.
Inviting by email only works for existing Firebase Auth users.

**Functions fail to deploy: `Quota exceeded` or `Billing account required`**
Cloud Functions v2 requires the Blaze billing plan. The Spark (free) plan does
not support v2. See https://firebase.google.com/pricing.

**Emulator: Functions cannot reach Firestore**
Ensure `FIRESTORE_EMULATOR_HOST` is set (the Firebase CLI sets it automatically
when starting via `firebase emulators:start`). Do not start Functions separately
with `node` — always use the emulator.

**Cloudflare tunnel created but DNS record missing**
This can happen if the DNS step fails after tunnel creation. The tunnel will
exist in Cloudflare but the robot will not be reachable. Run `deleteRobot`
(which cleans up both) and re-pair. Stale orphan tunnels can be removed
manually at https://dash.cloudflare.com → Zero Trust → Networks → Tunnels.
