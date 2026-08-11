import { expect, test, type Page } from "@playwright/test";
import { installMockBackend } from "./mock/mockBackend.ts";
import { SCENARIOS } from "./mock/scenarios.ts";

const mowing = SCENARIOS.find(({ name }) => name === "mowing-area2-rtk-fixed")!;
const emergency = SCENARIOS.find(({ name }) => name === "emergency-latched")!;

async function setDisplayMode(page: Page, mode: "visual" | "balanced" | "efficient") {
  await page.addInitScript((selectedMode) => {
    window.localStorage.setItem("mowgli.display-mode", selectedMode);
  }, mode);
}

const activeVisualEffects = (page: Page) =>
  page.evaluate(() => {
    const blurred = [...document.querySelectorAll<HTMLElement>("*")].filter(
      (element) => {
        const style = getComputedStyle(element);
        const webkitBackdrop = style.getPropertyValue(
          "-webkit-backdrop-filter",
        );
        return (
          style.backdropFilter !== "none" ||
          (webkitBackdrop !== "" && webkitBackdrop !== "none")
        );
      },
    );
    const infinite = document.getAnimations().filter((animation) => {
      const iterations = animation.effect?.getTiming().iterations;
      return iterations === Infinity && animation.playState === "running";
    });

    return {
      blurCount: blurred.length,
      blurAreaPx: Math.round(
        blurred.reduce((sum, element) => {
          const rect = element.getBoundingClientRect();
          return sum + rect.width * rect.height;
        }, 0),
      ),
      infiniteAnimationNames: infinite.map(
        (animation) =>
          (animation as CSSAnimation).animationName || "script-animation",
      ),
    };
  });

test("Balanced and Efficient views avoid continuous decorative effects", async ({
  page,
}) => {
  await setDisplayMode(page, "balanced");
  await installMockBackend(page, mowing);

  for (const route of ["/mowglinext", "/settings", "/map"]) {
    await page.goto(`/#${route}`);
    await page.getByText("MOWGLI").first().waitFor({ state: "visible" });
    await page.waitForTimeout(1_500);

    await expect(page.locator("html")).toHaveAttribute("data-display-mode", "balanced");
    const result = await activeVisualEffects(page);
    expect(result, `${route} visual effects`).toEqual({
      blurCount: 0,
      blurAreaPx: 0,
      infiniteAnimationNames: [],
    });

    if (route === "/mowglinext") {
      await page.screenshot({
        path: "tests/e2e/.artifacts/mowglinext__balanced-mode.png",
        fullPage: true,
      });
    }
  }

  await page.setViewportSize({ width: 390, height: 844 });
  for (const route of ["/mowglinext", "/map"]) {
    await page.goto(`/#${route}`);
    await page.getByText("MOWGLI").first().waitFor({ state: "visible" });
    await page.waitForTimeout(1_500);

    expect(
      await activeVisualEffects(page),
      `${route} mobile visual effects`,
    ).toEqual({
      blurCount: 0,
      blurAreaPx: 0,
      infiniteAnimationNames: [],
    });
  }
});

test("Visual mode restores glass depth and restrained active-state motion", async ({
  page,
}) => {
  await setDisplayMode(page, "visual");
  await installMockBackend(page, mowing);
  await page.goto("/#/mowglinext");
  await page.getByText("MOWGLI").first().waitFor({ state: "visible" });
  await page.waitForTimeout(1_500);

  await expect(page.locator("html")).toHaveAttribute("data-display-mode", "visual");
  const result = await activeVisualEffects(page);
  expect(result.blurCount).toBeGreaterThan(0);
  expect(result.blurAreaPx).toBeGreaterThan(0);
  expect(result.infiniteAnimationNames).toEqual(
    expect.arrayContaining(["liveStripSheen", "mowerPulseGreen"]),
  );
  await page.screenshot({
    path: "tests/e2e/.artifacts/mowglinext__visual-mode.png",
    fullPage: true,
  });
});

test("Efficient mode keeps the low-compositing rendering path", async ({page}) => {
  await setDisplayMode(page, "efficient");
  await installMockBackend(page, mowing);
  await page.goto("/#/mowglinext");
  await page.getByText("MOWGLI").first().waitFor({ state: "visible" });
  await page.waitForTimeout(1_500);

  await expect(page.locator("html")).toHaveAttribute("data-display-mode", "efficient");
  expect(await activeVisualEffects(page)).toEqual({
    blurCount: 0,
    blurAreaPx: 0,
    infiniteAnimationNames: [],
  });
  await page.screenshot({
    path: "tests/e2e/.artifacts/mowglinext__efficient-mode.png",
    fullPage: true,
  });
});

test("emergency state retains its focused visual emphasis", async ({
  page,
}) => {
  await installMockBackend(page, emergency);
  await page.goto("/#/mowglinext");
  await page.getByText("MOWGLI").first().waitFor({ state: "visible" });
  await page.waitForTimeout(1_500);

  const { infiniteAnimationNames } = await activeVisualEffects(page);
  expect(infiniteAnimationNames).toEqual(
    expect.arrayContaining([
      "liveStripPulse",
      "mowerPulseRed",
      "script-animation",
    ]),
  );
});

test("emergency emphasis respects reduced-motion preference", async ({
  page,
}) => {
  await page.emulateMedia({ reducedMotion: "reduce" });
  await setDisplayMode(page, "visual");
  await installMockBackend(page, emergency);
  await page.goto("/#/mowglinext");
  await page.getByText("MOWGLI").first().waitFor({ state: "visible" });
  await page.waitForTimeout(1_500);

  const result = await activeVisualEffects(page);
  expect(result.infiniteAnimationNames).toEqual([]);
  await page.screenshot({
    path: "tests/e2e/.artifacts/emergency-reduced-motion.png",
    fullPage: true,
  });
});
