import {describe, expect, it} from "vitest";
import {MAP_RENDER_BUDGETS} from "./mapRenderBudget.ts";

describe("MAP_RENDER_BUDGETS", () => {
    it("orders visual smoothness ahead of balanced and efficient rendering", () => {
        expect(MAP_RENDER_BUDGETS.visual.poseIntervalMs)
            .toBeLessThan(MAP_RENDER_BUDGETS.balanced.poseIntervalMs);
        expect(MAP_RENDER_BUDGETS.balanced.poseIntervalMs)
            .toBeLessThan(MAP_RENDER_BUDGETS.efficient.poseIntervalMs);
        expect(MAP_RENDER_BUDGETS.visual.lidarIntervalMs)
            .toBeLessThan(MAP_RENDER_BUDGETS.balanced.lidarIntervalMs);
        expect(MAP_RENDER_BUDGETS.balanced.lidarIntervalMs)
            .toBeLessThan(MAP_RENDER_BUDGETS.efficient.lidarIntervalMs);
    });
});
