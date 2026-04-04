---
name: Always clean DDS/Gazebo state before sim launch
description: Must kill all processes and clean shared memory before launching simulation to avoid stale state
type: feedback
---

ALWAYS run `make sim-stop` (or the equivalent cleanup) before launching simulation or E2E tests.

**Why:** Stale Cyclone DDS shared memory and zombie Gazebo processes cause "Failed to find a free participant index", "additional publishers on /clock", and lifecycle manager crashes. These look like real bugs but are just leftover state from previous runs.

**How to apply:**
- Use `make sim` or `make e2e-test` — both auto-run `sim-stop` first
- If running ros2 launch directly: run `make sim-stop` manually first
- Never have two simulation instances running simultaneously
- The cleanup sequence is: pkill ros2/gz → rm /dev/shm/cyclone* → verify clean
