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
source /opt/ros/kilted/setup.bash

cd /ros2_ws

# ---------------------------------------------------------------------------
# Symlink ROS2 packages from monorepo into workspace src/
#
# The workspace is /ros2_ws, the monorepo is bind-mounted at
# /ros2_ws/src/mowglinext. colcon picks up packages it finds recursively
# under src/, but some repos (FusionCore) have a top-level CMakeLists
# that shadows the nested packages — so we symlink the sub-packages to
# the workspace root exactly like ros2/Dockerfile does for production.
# ---------------------------------------------------------------------------
echo "Linking ROS2 packages into workspace..."

mkdir -p src

# 1. Our own packages (mowgli_*) — straight symlinks.
for pkg_dir in src/mowglinext/ros2/src/mowgli_*/; do
    pkg_name=$(basename "$pkg_dir")
    ln -sfn "/ros2_ws/$pkg_dir" "src/$pkg_name"
    echo "  Linked: $pkg_name"
done

# 2. FusionCore sub-packages — the submodule has a top-level CMakeLists
#    that would be treated as a single cmake project and shadow its
#    internal packages. Symlink each sub-package individually.
FUSION_ROOT="src/mowglinext/ros2/src/fusioncore"
if [ -d "$FUSION_ROOT" ]; then
    for sub in fusioncore_core fusioncore_ros compass_msgs; do
        if [ -d "$FUSION_ROOT/$sub" ]; then
            ln -sfn "/ros2_ws/$FUSION_ROOT/$sub" "src/$sub"
            echo "  Linked: $sub  (fusioncore submodule)"
        fi
    done
fi

# 3. Kinematic-ICP — its cpp+ros layout has no top-level CMakeLists, so
#    colcon discovers ros/package.xml directly with no symlink needed.
#    Listed here as a sanity check.
if [ -f "src/mowglinext/ros2/src/kinematic_icp/ros/package.xml" ]; then
    echo "  Kinematic-ICP submodule present (discovered via nested package.xml)."
fi

# ---------------------------------------------------------------------------
# ARM / x86 warning. Firmware builds (PlatformIO stm32 toolchain) assume
# ARM host support or cross-compile, GUI builds cross arches fine, ROS2
# builds fine on both but the real robot is ARM.
# ---------------------------------------------------------------------------
ARCH=$(uname -m)
if [ "$ARCH" != "aarch64" ] && [ "$ARCH" != "arm64" ]; then
    echo ""
    echo "⚠  Dev container arch is ${ARCH} (not ARM)."
    echo "   ROS2 / GUI / sim build fine; firmware (pio run) won't match the"
    echo "   real robot without a cross-compile step."
    echo ""
fi

# ---------------------------------------------------------------------------
# Resolve rosdep dependencies
# ---------------------------------------------------------------------------
echo "Resolving rosdep dependencies..."
rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro kilted \
    -y || true

# ---------------------------------------------------------------------------
# Build the workspace. Skip FusionCore's monorepo wrapper package if it
# got picked up separately from the sub-package symlinks.
# ---------------------------------------------------------------------------
echo "Building workspace (this may take a few minutes on first run)..."
colcon build \
    --packages-ignore fusioncore fusioncore_gazebo kinematic-icp \
    --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF \
    --parallel-workers "$(nproc)" \
    --symlink-install \
    --event-handlers console_cohesion+

# Source the built workspace
# shellcheck disable=SC1091
source install/setup.bash

# ---------------------------------------------------------------------------
# Optional: install pre-commit hooks if the repo has a config (no-op today).
# ---------------------------------------------------------------------------
if [ -f "src/mowglinext/.pre-commit-config.yaml" ]; then
    (cd src/mowglinext && pre-commit install) || true
fi

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
echo "GUI work (from gui/ directory):"
echo "  go build -o openmower-gui          # Build the Go backend"
echo "  cd web && yarn install && yarn dev # Start the React frontend"
echo ""
