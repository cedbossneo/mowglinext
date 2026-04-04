#!/usr/bin/env bash
# Format all C++ files in ros2/src/ using the project .clang-format config.
# Usage: ./ros2/scripts/format.sh [--check]
#   --check   Dry-run mode: exit 1 if any file needs formatting (CI mode)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_DIR="$(dirname "$SCRIPT_DIR")"
STYLE="file:${ROS2_DIR}/.clang-format"

FILES=$(find "${ROS2_DIR}/src/" \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) \
  -not -path "*/opennav_coverage/*")

if [ -z "$FILES" ]; then
  echo "No C++ files found."
  exit 0
fi

if [ "${1:-}" = "--check" ]; then
  echo "$FILES" | xargs clang-format --dry-run --Werror -style="$STYLE"
  echo "All files are correctly formatted."
else
  echo "$FILES" | xargs clang-format -i -style="$STYLE"
  echo "Formatted $(echo "$FILES" | wc -l | tr -d ' ') files."
fi
