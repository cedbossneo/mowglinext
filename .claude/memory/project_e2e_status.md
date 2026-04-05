---
name: E2E simulation status and remaining work
description: Current state of coverage path execution and what needs tuning for production readiness
type: project
---

Goal: production-ready mower with standard + complex + recovery scenarios before community testing.

**What works (as of 2026-04-04):**
- ExecuteFullCoveragePath sends full F2C path to Nav2 FollowPath (replaces broken per-swath approach)
- Transit to path start via NavigateToPose, then FollowPath with FollowCoveragePath controller
- Path subsampling (>5000 poses → ~4000) to avoid RPP segfault
- BT cycle: UNDOCKING → PLANNING → MOWING → recovery → DOCKING
- Robot moves and SLAM map grows (confirmed in E2E)
- `make sim-stop` / `make e2e-test` with proper cleanup

**Remaining issues to tune:**
1. **First-run pose matching**: Robot at (0,0) matches mid-path (pose 4392) instead of start because boustrophedon passes near origin. Fix: always transit to first pose on fresh mow start, use findClosestPoseIndex only for retry-after-stuck.
2. **FollowPath aborts after ~117s**: Likely progress_checker timeout or TF issue. Check nav2_params.yaml progress_checker config.
3. **Stuck detection too aggressive (15s)**: During slow U-turns the robot may appear stuck. Consider 30s or checking angular velocity too.
4. **Path subsampling loses turn precision**: Subsampling from 8326→4000 loses Dubins turn waypoints. Better fix: reduce F2C path discretization in coverage_planner_node instead of subsampling in BT.
5. **Dual install trees**: `ros2/install/` vs `/ros2_ws/install/` — Makefile builds into `ros2/install/` but devcontainer post-create builds into `/ros2_ws/install/`. Need to unify or ensure e2e-test uses correct one.

**How to apply:** Start next session with `make sim-stop`, then address items above in priority order.
