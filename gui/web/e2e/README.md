# E2E Tests

Playwright E2E tests for the MowgliNext GUI.

## Running

```bash
# Install Chromium once (skip if already done)
yarn e2e:install

# Run headless (CI / default)
yarn e2e

# Run with the Playwright UI explorer
yarn e2e:ui
```

The `webServer` block in `playwright.config.ts` starts `yarn dev` automatically
on port 5173 if it is not already running.  All backend (`/api/**`) calls are
intercepted by `page.route` — no Go server is needed.

## Coverage

### `pairing-wizard.spec.ts`

Exercises the full **Pair Mobile App** wizard step (step index 6) end-to-end:

- Clicks through steps 0-5 (Welcome, Robot Model, GPS, Sensors, IMU Yaw, Firmware) using mocked settings/calibration APIs.
- Asserts the QR code SVG and Robot ID render immediately on step 6.
- Simulates a mobile-app scan by advancing the `/api/pair/status` mock to `pending`.
- Asserts the 4-digit confirm code (`1234`) appears within the polling window.
- Clicks "Confirm on robot", advances mock to `paired`, asserts success screen.

## What is NOT covered

- Steps 1-5 content (only navigation is tested; form correctness is covered by Vitest unit tests).
- The "Skip - I'll pair later" flow.
- The `failed` pairing state.
- The `reset` / Cancel pairing flow.
- Mobile viewport layout (only desktop Chromium project is configured).
- Real Go backend integration (all API calls are mocked).

## Adding more tests

Add `*.spec.ts` files inside this `e2e/` directory. They are picked up
automatically by `playwright.config.ts`.
