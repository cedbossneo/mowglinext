import { expect, test } from "@playwright/test";
import { installMockBackend } from "./mock/mockBackend.ts";

test("high-rate container logs are retained and reach the live tail", async ({
  page,
}) => {
  const lines = Array.from(
    { length: 1_000 },
    (_, index) => `INFO synthetic log line ${index}`,
  );
  await installMockBackend(page, {
    name: "high-rate-container-logs",
    rest: {
      "/api/containers": {
        containers: [
          {
            id: "mock-container",
            names: ["/mock-container"],
            state: "running",
            labels: { app: "mock" },
          },
        ],
      },
    },
    containerLogs: lines,
  });

  await page.goto("/#/logs");
  const renderedLines = page.getByTestId("log-line");
  await expect(renderedLines).toHaveCount(1_000, { timeout: 15_000 });
  await expect(
    page.getByText("INFO synthetic log line 999", { exact: true }),
  ).toBeVisible();
  await page.screenshot({
    path: "tests/e2e/.artifacts/log-stream-1000-lines.png",
    fullPage: true,
  });
});
