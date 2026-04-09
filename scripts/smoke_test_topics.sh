#!/bin/bash
# Smoke test: verify ROS2 topics are publishing
# Checks if /odom receives a message within 5 seconds

set -e

TIMEOUT=5
RECEIVED=false

echo "[smoke_test] Checking for ROS2 topics..."
echo ""

# Check /odom topic
echo "[smoke_test] Waiting up to ${TIMEOUT}s for message on /odom..."
if timeout ${TIMEOUT} ros2 topic hz /odom --window 1 >/dev/null 2>&1; then
    echo "✓ /odom is publishing"
    RECEIVED=true
else
    echo "✗ /odom not receiving messages within ${TIMEOUT}s"
fi

echo ""
echo "[smoke_test] Topic information:"
echo ""

if ros2 topic list 2>/dev/null | grep -q "^/cmd_vel$"; then
    echo "--- /cmd_vel ---"
    ros2 topic info /cmd_vel 2>/dev/null || echo "  (topic exists but info unavailable)"
else
    echo "⚠ /cmd_vel topic not found"
fi

echo ""

if ros2 topic list 2>/dev/null | grep -q "^/odom$"; then
    echo "--- /odom ---"
    ros2 topic info /odom 2>/dev/null || echo "  (topic exists but info unavailable)"
else
    echo "⚠ /odom topic not found"
fi

echo ""

if [[ "$RECEIVED" == "true" ]]; then
    echo "[smoke_test] ✓ PASSED: Topics are available"
    exit 0
else
    echo "[smoke_test] ✗ FAILED: /odom not receiving messages"
    exit 1
fi
