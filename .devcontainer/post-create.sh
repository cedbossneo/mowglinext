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
    --skip-keys "fields2cover" \
    -y || true

# ---------------------------------------------------------------------------
# Build the workspace
# ---------------------------------------------------------------------------
echo "Building workspace (this may take a few minutes on first run)..."
colcon build \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
        -DFields2Cover_DIR=/usr/local/cmake/fields2cover \
    --parallel-workers "$(nproc)" \
    --symlink-install \
    --event-handlers console_cohesion+

# Source the built workspace
source install/setup.bash

echo ""
echo "=== MowgliNext workspace ready ==="
echo ""
echo "Quick start:"
echo "  # Run headless simulation:"
echo "  ros2 launch mowgli_bringup sim_full_system.launch.py"
echo ""
echo "  # Connect Foxglove Studio to: ws://localhost:8765"
echo ""
echo "  # Run E2E test:"
echo "  python3 /ros2_ws/src/mowglinext/ros2/src/e2e_test.py"
echo ""
echo "  # Rebuild after code changes:"
echo "  colcon build --symlink-install --cmake-args -DFields2Cover_DIR=/usr/local/cmake/fields2cover"
echo ""
