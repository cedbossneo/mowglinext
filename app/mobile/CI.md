# Mobile CI — Android + iOS builds without a local SDK

Two GitHub Actions workflows build the Capacitor mobile app entirely in
CI, so you don't need Android Studio or Xcode installed locally:

| Workflow | File | Runner | Always produces | Optional outputs |
|---|---|---|---|---|
| Android | `.github/workflows/mobile-android.yml` | `ubuntu-latest` | unsigned debug APK | signed release APK + AAB (when keystore secrets set) |
| iOS | `.github/workflows/mobile-ios.yml` | `macos-14` (free for public repos) | unsigned `.xcarchive` | signed `.ipa`, TestFlight upload (when Apple secrets set) |

Both run on every push to `main` / `dev` / `feat/**` and on PRs that
touch `app/mobile/**` or the workflow file itself, plus on
`workflow_dispatch` (Run workflow button on the Actions tab).

The native `android/` and `ios/` folders are **generated fresh on each
run** via `npx cap add` + `npx cap sync` — nothing platform-specific is
committed to the repo.

## Modes

### Mode 0 — smoke build (no secrets required, default)

Both workflows succeed out-of-the-box on a public repo. You get:

- Android: unsigned debug APK as a workflow artifact (`mowgli-android-debug`)
- iOS: unsigned `.xcarchive` as a workflow artifact (`mowgli-ios-unsigned`)

The unsigned debug APK can be sideloaded onto an Android device via ADB:

```bash
adb install mowgli-debug.apk
```

The unsigned iOS archive **cannot** be installed on a device without
re-signing — it's purely a build-validity smoke test. To run on a real
iPhone you need either Mode 1 or Mode 2 below.

### Mode 1 — signed builds (no Apple/Play distribution)

Add the appropriate signing secrets to your repo settings
(Settings → Secrets and variables → Actions). Then every build also
produces the signed artifact.

**Android signing secrets**:
- `APK_SIGNING_KEYSTORE_BASE64` — base64-encoded JKS keystore
  (`base64 -i release.keystore`)
- `APK_SIGNING_STORE_PASSWORD` — keystore password
- `APK_SIGNING_KEY_ALIAS` — key alias inside the keystore
- `APK_SIGNING_KEY_PASSWORD` — key password

**iOS signing secrets**:
- `APPLE_DISTRIBUTION_CERT_BASE64` — base64-encoded `.p12` distribution
  cert (`base64 -i AppleDistribution.p12`)
- `APPLE_DISTRIBUTION_CERT_PASSWORD` — `.p12` password
- `APPLE_API_KEY_BASE64` — base64-encoded App Store Connect API key
- `APPLE_API_KEY_ID` — the key ID (10-char string from App Store Connect)
- `APPLE_API_ISSUER_ID` — the issuer UUID
- `APPLE_TEAM_ID` — your Developer Team ID (10-char)

Bundle ID is hard-coded as `garden.mowgli.app` in both
`capacitor.config.ts` and the iOS workflow's provisioning download. If
you fork the project, change both.

### Mode 2 — TestFlight upload (iOS only)

Add **all** of Mode 1's iOS secrets, **plus**:

- `APPLE_ID` — your Apple ID email
- `APPLE_APP_SPECIFIC_PASSWORD` — generated at appleid.apple.com →
  Sign-In and Security → App-Specific Passwords. Required because
  Apple's CLI uploader does not accept your normal password.

Each successful build is uploaded to TestFlight as a new internal-test
build. Configure internal testers in App Store Connect after the first
upload arrives.

## Firebase config secrets (optional)

The Vite build needs `VITE_FIREBASE_*` values. If you don't set them as
GitHub secrets, the workflows substitute placeholders and the resulting
build cannot actually authenticate. Set the same values from your
`app/mobile/.env.local`:

- `VITE_FIREBASE_API_KEY`
- `VITE_FIREBASE_AUTH_DOMAIN`
- `VITE_FIREBASE_PROJECT_ID`
- `VITE_FIREBASE_STORAGE_BUCKET`
- `VITE_FIREBASE_MESSAGING_SENDER_ID`
- `VITE_FIREBASE_APP_ID`
- `VITE_FCM_VAPID_KEY`
- `VITE_TUNNEL_DOMAIN` (defaults to `tunnel.mowgli.garden`)

For private repos, prefer Mode 1+ with all secrets set; otherwise the
build is just a TypeScript/Capacitor smoke test.

## Cost

- Android (`ubuntu-latest`): ~3-5 min per run, free on all repo tiers.
- iOS (`macos-14`): ~10-15 min per run; **free for public repositories**;
  $0.08/min for private repos (around 10× the Linux rate).

## Triggering manually

Actions tab → pick the workflow → "Run workflow" → choose branch.
This is the fastest way to validate a change without pushing first.
