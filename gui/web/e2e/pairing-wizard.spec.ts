/**
 * E2E: Pair Mobile App wizard step (step index 6)
 *
 * Strategy
 * --------
 * The app is a hash-router SPA. Navigating to /#/onboarding renders
 * OnboardingWizard starting at step 0.  root.tsx fetches
 * http://localhost:4006/api/settings/status on mount and redirects to
 * /onboarding when onboarding_completed is false.  We intercept that
 * request (and all other backend calls) so the test never needs a real
 * Go server running.
 *
 * Steps 0-5 are clicked through with the minimal interactions required:
 *   0  Welcome       – click "Get Started"
 *   1  Robot Model   – click "Next"  (schema/yaml mocked)
 *   2  GPS           – click "Next"  (no RTK fix → datum button disabled, fine)
 *   3  Sensors       – click "Save & Continue"
 *   4  IMU Yaw       – click "Save & Continue"
 *   5  Firmware      – click "Skip — Already Flashed"
 *   6  Pair App      – assertions
 *
 * All /api/** calls are intercepted via page.route so no real backend is
 * required.  The pairing mock advances through three states:
 *   Phase 1 (initial):   status → unstarted, QR rendered
 *   Phase 2 (after 3 s): status → pending, confirm code "1234" shown
 *   Phase 3 (after confirm POST): status → paired, success screen shown
 */

import { test, expect, type Page, type Route } from "@playwright/test";

// ---------------------------------------------------------------------------
// Mock data
// ---------------------------------------------------------------------------

const QR_PAYLOAD =
  "mowgli://pair?v=1&rid=ABC123&pub=FAKEPUBKEY&tok=test-token&lan=mowgli.local:80&cf=r-ABC123.tunnel.mowgli.garden";

const QR_DATA = {
  payload: QR_PAYLOAD,
  robotID: "ABC123",
  setupToken: "test-token",
};

const STATUS_UNSTARTED = { state: "unstarted", robotID: "ABC123" };
const STATUS_PENDING = {
  state: "pending",
  robotID: "ABC123",
  ownerName: "Test User",
  confirmCode: "1234",
};
const STATUS_PAIRED = {
  state: "paired",
  robotID: "ABC123",
  ownerName: "Test User",
};

// Minimal settings schema & values — enough for the wizard to load without errors.
const SETTINGS_SCHEMA = { type: "object", properties: {} };
const SETTINGS_VALUES: Record<string, unknown> = {};
const CALIBRATION_STATUS = {
  dock: { present: false },
  imu: { present: false },
  mag: { present: false },
};

// ---------------------------------------------------------------------------
// Route helpers
// ---------------------------------------------------------------------------

/**
 * Install all backend mocks before the page loads.
 * Returns `setPairingPhase` so the test can advance the pairing state.
 *
 * usePairingStatus and root.tsx both use `http://localhost:4006` as base
 * when import.meta.env.DEV is true, so we intercept that origin directly.
 * The vite dev-server proxy (/api → localhost:4006) is never reached because
 * Playwright intercepts at the browser level before any network packet leaves.
 */
function installMocks(page: Page): {
  setPairingPhase: (phase: "unstarted" | "pending" | "paired") => void;
} {
  let pairingPhase: "unstarted" | "pending" | "paired" = "unstarted";

  const setPairingPhase = (phase: "unstarted" | "pending" | "paired"): void => {
    pairingPhase = phase;
  };

  // Both origins that DEV-mode fetch calls can use.
  const origins = [
    "http://127.0.0.1:5173",
    "http://localhost:4006",
    "http://127.0.0.1:4006",
  ];

  for (const origin of origins) {
    // settings/status — prevents root.tsx from redirecting away from /onboarding
    void page.route(`${origin}/api/settings/status`, (route: Route) => {
      void route.fulfill({
        status: 200,
        contentType: "application/json",
        body: JSON.stringify({ onboarding_completed: true }),
      });
    });

    // settings/schema
    void page.route(`${origin}/api/settings/schema`, (route: Route) => {
      void route.fulfill({
        status: 200,
        contentType: "application/json",
        body: JSON.stringify(SETTINGS_SCHEMA),
      });
    });

    // settings/yaml (GET + POST both return the same safe stub)
    void page.route(`${origin}/api/settings/yaml`, (route: Route) => {
      void route.fulfill({
        status: 200,
        contentType: "application/json",
        body: JSON.stringify(SETTINGS_VALUES),
      });
    });

    // calibration/status
    void page.route(`${origin}/api/calibration/status`, (route: Route) => {
      void route.fulfill({
        status: 200,
        contentType: "application/json",
        body: JSON.stringify(CALIBRATION_STATUS),
      });
    });

    // mowglinext/* (set_datum, subscribe, etc.)
    void page.route(`${origin}/api/mowglinext/**`, (route: Route) => {
      void route.fulfill({
        status: 200,
        contentType: "application/json",
        body: JSON.stringify({ message: "" }),
      });
    });

    // pairing QR
    void page.route(`${origin}/api/pair/qr`, (route: Route) => {
      void route.fulfill({
        status: 200,
        contentType: "application/json",
        body: JSON.stringify(QR_DATA),
      });
    });

    // pairing status — dynamically returns the current phase
    void page.route(`${origin}/api/pair/status`, (route: Route) => {
      const body =
        pairingPhase === "unstarted"
          ? STATUS_UNSTARTED
          : pairingPhase === "pending"
            ? STATUS_PENDING
            : STATUS_PAIRED;
      void route.fulfill({
        status: 200,
        contentType: "application/json",
        body: JSON.stringify(body),
      });
    });

    // pairing confirm POST
    void page.route(`${origin}/api/pair/confirm`, (route: Route) => {
      pairingPhase = "paired";
      void route.fulfill({
        status: 200,
        contentType: "application/json",
        body: JSON.stringify({ ok: true }),
      });
    });

    // pairing reset POST
    void page.route(`${origin}/api/pair/reset`, (route: Route) => {
      void route.fulfill({
        status: 200,
        contentType: "application/json",
        body: JSON.stringify({ ok: true }),
      });
    });
  }

  return { setPairingPhase };
}

// ---------------------------------------------------------------------------
// Test
// ---------------------------------------------------------------------------

test(
  "Pair Mobile App wizard step: QR renders, pending state shows confirm code, confirm advances to success",
  async ({ page }) => {
    // Install all mocks BEFORE the page navigates so no real request escapes.
    const { setPairingPhase } = installMocks(page);

    await page.goto("/#/onboarding");

    // ── Step 0: Welcome ──────────────────────────────────────────────────
    await expect(
      page.getByRole("heading", { name: /Welcome to Mowgli/i })
    ).toBeVisible();
    await page.getByRole("button", { name: /Get Started/i }).click();

    // ── Step 1: Robot Model ──────────────────────────────────────────────
    await expect(
      page.getByRole("heading", { name: /Choose Your Robot/i })
    ).toBeVisible();
    // The nav-bar button's accessible name includes the Antd icon label prefix
    // ("arrow-right Next"), so we match on the visible text span, not the full
    // aria label.  getByRole with name:/Next/i (non-anchored) handles both.
    await page.getByRole("button", { name: /Next/i }).click();

    // ── Step 2: GPS ──────────────────────────────────────────────────────
    await expect(
      page.getByRole("heading", { name: /GPS & Positioning/i })
    ).toBeVisible();
    await page.getByRole("button", { name: /Next/i }).click();

    // ── Step 3: Sensors ──────────────────────────────────────────────────
    // Nav bar still shows "Next" for steps 1-3 (label becomes "Save & Continue"
    // only at step 4 — see OnboardingPage.tsx: currentStep < 4 ? "Next" : "Save & Continue").
    await expect(
      page.getByRole("heading", { name: /Sensor Placement/i })
    ).toBeVisible();
    await page.getByRole("button", { name: /Next/i }).click();

    // ── Step 4: IMU Yaw ──────────────────────────────────────────────────
    await expect(
      page.getByRole("heading", { name: /IMU Mounting Calibration/i })
    ).toBeVisible();
    // At step 4 the nav bar switches to "Save & Continue".
    await page.getByRole("button", { name: /Save & Continue/i }).click();

    // ── Step 5: Firmware ─────────────────────────────────────────────────
    await expect(
      page.getByRole("heading", { name: /Firmware/i })
    ).toBeVisible();
    await page.getByRole("button", { name: /Skip.*Already Flashed/i }).click();

    // ── Step 6: Pair Mobile App ──────────────────────────────────────────
    await expect(
      page.getByRole("heading", { name: /Pair the Mobile App/i })
    ).toBeVisible();

    // Assert: QR SVG is rendered inside the pairing card.
    const qrSvg = page.locator("svg").first();
    await expect(qrSvg).toBeVisible();

    // Assert: Robot ID is displayed.
    await expect(page.getByText("ABC123").first()).toBeVisible();

    // Assert: "Waiting for the app to scan…" helper text is visible.
    await expect(page.getByText(/Waiting for the app to scan/i)).toBeVisible();

    // ── Simulate mobile-app scan: switch to pending state ─────────────────
    // Advance the mock; the hook polls every 1500 ms, allow up to 6 s.
    setPairingPhase("pending");

    await expect(
      page.getByRole("heading", { name: /Confirm Pairing/i })
    ).toBeVisible({ timeout: 6_000 });

    // Assert: 4-digit confirmation code is shown.
    await expect(page.getByText("1234")).toBeVisible();

    // Assert: Robot ID still visible in the confirm screen.
    await expect(page.getByText("ABC123").first()).toBeVisible();

    // Assert: "Confirm on robot" button is present and enabled.
    const confirmButton = page.getByRole("button", {
      name: /Confirm on robot/i,
    });
    await expect(confirmButton).toBeVisible();
    await expect(confirmButton).toBeEnabled();

    // ── Click confirm — mock flips to paired ──────────────────────────────
    await confirmButton.click();

    // Polling picks up "paired" on the next interval.
    await expect(page.getByText(/Mobile app paired!/i)).toBeVisible({
      timeout: 6_000,
    });
    await expect(page.getByText(/Test User/i)).toBeVisible();
  }
);
