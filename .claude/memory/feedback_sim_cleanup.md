---
name: Always clean DDS/Gazebo state before sim launch
description: Must kill all processes and clean shared memory before launching simulation to avoid stale state
type: feedback
---

ALWAYS kill all processes and clean DDS shared memory before launching simulation.

**Why:** Spent hours debugging "Failed to find a free participant index" and "additional publishers on /clock" errors that were caused by stale Cyclone DDS shared memory and zombie Gazebo processes from previous runs. The symptoms look like real bugs (clock not working, lifecycle manager crashing, TF not publishing) but the root cause is just leftover state.

**How to apply:** Before every `ros2 launch`, run this cleanup sequence:
```bash
pkill -9 -f "ros2|gz|ruby" 2>/dev/null
sleep 2
rm -rf /dev/shm/cyclone* /dev/shm/dds* /dev/shm/iox* /tmp/gz-* /tmp/ign-*
pgrep -a "ros2|gz" || echo "All clean"
```
Never launch a second simulation without first killing the first one AND cleaning shared memory. Never run sim in background without a way to verify cleanup.
