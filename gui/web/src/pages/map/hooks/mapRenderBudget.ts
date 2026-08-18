import type {DisplayMode} from "../../../theme/ThemeContext.tsx";

export type MapRenderBudget = {
    poseIntervalMs: number;
    lidarIntervalMs: number;
};

// These are presentation budgets only: subscriptions and robot-side data stay
// unchanged. Visual uses a smooth 20 Hz pose / 10 Hz lidar view; Balanced keeps
// the established low-overhead cadence; Efficient is for constrained clients.
export const MAP_RENDER_BUDGETS: Record<DisplayMode, MapRenderBudget> = {
    visual: {poseIntervalMs: 50, lidarIntervalMs: 100},
    balanced: {poseIntervalMs: 100, lidarIntervalMs: 200},
    efficient: {poseIntervalMs: 200, lidarIntervalMs: 500},
};
