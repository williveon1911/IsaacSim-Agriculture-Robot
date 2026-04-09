#!/bin/bash
# Run Isaac Sim with ROS2 bridge enabled.
# Usage: ./run_isaacsim.sh [--usd /abs/path/to/scene.usd] [--headless]

set -e

# Configuration
ISAACSIM_PATH="${HOME}/isaacsim"
ISAACSIM_SCRIPT="${ISAACSIM_PATH}/isaac-sim.sh"
ISAACSIM_ROS_BRIDGE_LIB="${ISAACSIM_PATH}/exts/isaacsim.ros2.bridge/humble/lib"

# Parse arguments
USD_FILE=""
HEADLESS=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --usd)
            USD_FILE="$2"
            shift 2
            ;;
        --headless)
            HEADLESS=true
            shift
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Usage: $0 [--usd /path/to/scene.usd] [--headless]"
            exit 1
            ;;
    esac
done

# Validation
if [[ ! -f "$ISAACSIM_SCRIPT" ]]; then
    echo "ERROR: Isaac Sim script not found at: $ISAACSIM_SCRIPT"
    echo "Please install Isaac Sim at ~/isaacsim"
    exit 1
fi

if [[ ! -d "$ISAACSIM_ROS_BRIDGE_LIB" ]]; then
    echo "ERROR: Isaac Sim ROS2 bridge library not found at: $ISAACSIM_ROS_BRIDGE_LIB"
    echo "Make sure isacsim.ros2.bridge extension is installed"
    exit 1
fi

# Validate USD file if provided
if [[ -n "$USD_FILE" ]]; then
    if [[ "$USD_FILE" != /* ]]; then
        echo "ERROR: --usd must be an absolute path"
        echo "Got: $USD_FILE"
        exit 1
    fi
    if [[ ! -f "$USD_FILE" ]]; then
        echo "ERROR: USD file not found at: $USD_FILE"
        exit 1
    fi
    echo "[run_isaacsim] Loading USD: $USD_FILE"
fi

# Set up environment
# Avoid inheriting conflicting ROS setup from shell startup files.
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset PYTHONPATH

export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="${ISAACSIM_ROS_BRIDGE_LIB}:${LD_LIBRARY_PATH}"

echo "[run_isaacsim] Environment configured:"
echo "  ROS_DISTRO=$ROS_DISTRO"
echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "  LD_LIBRARY_PATH includes Isaac Sim ROS bridge"

echo "[run_isaacsim] Starting Isaac Sim..."

ISAAC_ARGS=(
    "--/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge"
)
if [[ -n "$USD_FILE" ]]; then
    ISAAC_ARGS+=("--/isaac/startup/usd=${USD_FILE}")
fi
if [[ "$HEADLESS" == "true" ]]; then
    ISAAC_ARGS+=("--headless")
fi

exec "$ISAACSIM_SCRIPT" "${ISAAC_ARGS[@]}"
