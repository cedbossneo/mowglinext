import { expect, test } from "@playwright/test";
import { installMockBackend } from "./mock/mockBackend.ts";

test("map layers mount without source or fragment errors", async ({ page }) => {
  const layerErrors: string[] = [];
  page.on("console", (message) => {
    const text = message.text();
    if (
      message.type() === "error" &&
      (text.includes('missing required property "source"') ||
        text.includes("React.Fragment"))
    ) {
      layerErrors.push(text);
    }
  });

  await installMockBackend(page, {
    name: "map-layer-regression",
    rest: {
      "/api/settings/yaml": { datum_lat: 48.1, datum_lon: 11.5 },
    },
    topics: {
      map: {
        working_area: [
          {
            area: {
              points: [
                { x: -5, y: -5 },
                { x: 5, y: -5 },
                { x: 5, y: 5 },
                { x: -5, y: 5 },
              ],
            },
          },
        ],
      },
      pose: {
        pose: { pose: { position: { x: 1.2, y: -0.6, z: 0 } } },
        motion_heading: 0.4,
      },
      obstacles: {
        obstacles: [
          {
            id: 7,
            status: 1,
            polygon: {
              points: [
                { x: 1, y: 1 },
                { x: 2, y: 1 },
                { x: 2, y: 2 },
              ],
            },
          },
        ],
      },
    },
  });

  await page.goto("/#/map");
  await expect(page.locator(".mapboxgl-canvas")).toBeVisible({
    timeout: 15_000,
  });
  await page.waitForTimeout(1_000);
  await page.screenshot({
    path: "tests/e2e/.artifacts/map-layer-regression.png",
    fullPage: true,
  });

  expect(layerErrors).toEqual([]);
});
