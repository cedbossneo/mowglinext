#!/bin/bash
# =============================================================================
# post-create.sh — Runs after the devcontainer is created.
#
# Symlinks the monorepo's ROS2 packages into the workspace, resolves
# dependencies, and builds the full workspace.
# =============================================================================
set -e

echo "=== MowgliNext: Setting up ROS2 workspace ==="

# Source ROS2
source /opt/ros/jazzy/setup.bash

cd /ros2_ws

# ---------------------------------------------------------------------------
# Symlink ROS2 packages from monorepo into workspace src/
# The workspace is at /ros2_ws, source code is bind-mounted at
# /ros2_ws/src/mowglinext/ros2/src/<packages>
# ---------------------------------------------------------------------------
echo "Linking ROS2 packages into workspace..."

# Create src/ if it doesn't exist (workspace convention)
mkdir -p src

# Symlink each package from the monorepo
for pkg_dir in src/mowglinext/ros2/src/mowgli_*/; do
    pkg_name=$(basename "$pkg_dir")
    if [ ! -e "src/$pkg_name" ]; then
        ln -sf "/ros2_ws/$pkg_dir" "src/$pkg_name"
        echo "  Linked: $pkg_name"
    fi
done

# ---------------------------------------------------------------------------
# Resolve rosdep dependencies
# ---------------------------------------------------------------------------
echo "Resolving rosdep dependencies..."
rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro jazzy \
    -y || true

# ---------------------------------------------------------------------------
# Build the workspace
# ---------------------------------------------------------------------------
echo "Building workspace (this may take a few minutes on first run)..."
colcon build \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
    --parallel-workers "$(nproc)" \
    --symlink-install \
    --event-handlers console_cohesion+

# Source the built workspace
source install/setup.bash

echo ""
echo "=== MowgliNext workspace ready ==="
echo ""
echo "Quick start (from ros2/ directory):"
echo "  make sim          # Launch headless simulation (Foxglove ws://localhost:8765)"
echo "  make e2e-test     # Run E2E validation (sim must be running)"
echo "  make build        # Rebuild after code changes"
echo "  make format       # Format C++ code"
echo "  make help         # Show all targets"
echo ""
