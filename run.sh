#!/bin/bash
# AFI920 ROS2 Driver — Build, Launch & RViz2
# Usage:
#   ./run.sh                    # build + launch single sensor + rviz
#   ./run.sh --no-build         # skip build, launch + rviz only
#   ./run.sh --multi            # build + launch multi sensor + rviz
#   ./run.sh --no-build --multi # skip build, multi sensor + rviz

set -e

# ── Options ──────────────────────────────────────────────
NO_BUILD=false
MULTI=false

for arg in "$@"; do
    case $arg in
        --no-build) NO_BUILD=true ;;
        --multi)    MULTI=true ;;
        *)          echo "Unknown option: $arg"; exit 1 ;;
    esac
done

# ── Paths ────────────────────────────────────────────────
# This script can be run from the repo root (standalone workspace)
# or from inside a parent workspace (e.g., ~/ros2_ws/src/afi920_ros/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ── ROS2 Environment ────────────────────────────────────
if [ -z "$ROS_DISTRO" ]; then
    for distro in humble foxy jazzy; do
        if [ -f "/opt/ros/$distro/setup.bash" ]; then
            source "/opt/ros/$distro/setup.bash"
            break
        fi
    done
fi

# ── Build ────────────────────────────────────────────────
if [ "$NO_BUILD" = false ]; then
    echo "==> Building workspace..."
    colcon build --symlink-install
    echo ""
fi

# ── Source ────────────────────────────────────────────────
echo "==> Sourcing workspace..."
source install/setup.bash

# ── Launch ────────────────────────────────────────────────
if [ "$MULTI" = true ]; then
    echo "==> Launching multi-sensor..."
    ros2 launch afi920_driver multi_sensor.launch.py &
else
    echo "==> Launching single sensor..."
    ros2 launch afi920_driver afi920.launch.py &
fi

LAUNCH_PID=$!
sleep 2

# ── RViz2 ────────────────────────────────────────────────
if [ "$MULTI" = true ]; then
    RVIZ_CONFIG="$(ros2 pkg prefix afi920_driver)/share/afi920_driver/config/multi_sensor.rviz"
else
    RVIZ_CONFIG="$(ros2 pkg prefix afi920_driver)/share/afi920_driver/config/afi920.rviz"
fi
echo "==> Starting RViz2..."
if [ -f "$RVIZ_CONFIG" ]; then
    rviz2 -d "$RVIZ_CONFIG" &
else
    echo "    (rviz config not found, launching default)"
    rviz2 &
fi
RVIZ_PID=$!

# ── Cleanup on exit ──────────────────────────────────────
trap "kill $LAUNCH_PID $RVIZ_PID 2>/dev/null; wait" SIGINT SIGTERM
echo ""
echo "Press Ctrl+C to stop."
wait
