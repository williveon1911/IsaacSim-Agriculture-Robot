# Testing Guide

This document describes how to test the agricultural robot tracker with different simulation backends.

## Architecture

The launch system is designed to be resilient:
- **ROS Launch** (`full_stack.launch.py`) handles tracker, RViz, data collection, and visualization
- **Isaac Sim** (optional, separate process) simulates physics with ROS2 bridge
- **Odometry source** is controlled by `use_fake_odom` parameter:
  - `true`: Pure kinematic simulation (fast, deterministic, no physics)
  - `false`: Physics simulation (requires Isaac Sim running separately)

This separation ensures that if Isaac Sim crashes or fails to start, your ROS stack continues running in RViz-only mode.

---

## Testing Workflow 1: RViz-Only (Kinematic Simulation)

**Fast, deterministic testing without physics engine.**

### Prerequisites
```bash
cd ~/agri_robot_ws
source install/setup.bash
```

### Launch
```bash
ros2 launch agri_robot_description full_stack.launch.py use_fake_odom:=true
```

### What this runs:
- ✓ Robot State Publisher (URDF → TF transforms)
- ✓ Pure Pursuit Tracker (path following control algorithm)
- ✓ Fake Odometry Publisher (kinematic simulation of robot motion)
- ✓ Data Collector (logs trajectory to `sim_log.json`)
- ✓ RViz2 (visualization)
- ✓ Tracker Visualizer (matplotlib plots after 120s delay)
- ✗ Isaac Sim (skipped - not needed)

### Monitor
In another terminal:
```bash
# Watch the tracker output
ros2 topic echo /cmd_vel

# Check odometry
ros2 topic echo /odom

# Quick health check
./scripts/smoke_test_topics.sh
```

### Expected behavior:
You should see the robot smoothly follow the waypoint path in RViz, ending at coordinate (15, 5). The path is deterministic—same input, same output every time.

---

## Testing Workflow 2: Physics Simulation with Isaac Sim

**Test algorithm with physics engine; compare Isaac vs RViz kinematic results.**

### Prerequisites
```bash
cd ~/agri_robot_ws
source install/setup.bash

# Ensure Isaac Sim is installed at ~/isaacsim
ls ~/isaacsim/isaac-sim.sh  # Should exist
```

### Terminal 1: Start Isaac Sim
```bash
./scripts/run_isaacsim.sh --usd src/agri_robot_description/isaac/agri_robot_scene.usd
```

Monitor for initialization messages:
```
[run_isaacsim] Environment configured:
  ROS_DISTRO=humble
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  LD_LIBRARY_PATH includes Isaac Sim ROS bridge
[run_isaacsim] Starting Isaac Sim...
```

Wait 10-15 seconds for Isaac Sim GUI to appear and settle. You should see:
- Isaac Sim window with the robot scene
- OmniGraph graph (if saved in USD) executing
- Console messages indicating ROS2 bridge is active

### Terminal 2: Start ROS Stack
```bash
ros2 launch agri_robot_description full_stack.launch.py use_fake_odom:=false
```

### What this runs:
- ✓ Robot State Publisher (URDF → TF transforms)
- ✓ Pure Pursuit Tracker (path following control algorithm)
- ✗ Fake Odometry Publisher (disabled - Isaac Sim provides /odom)
- ✓ Data Collector (logs trajectory to `sim_log.json`)
- ✓ RViz2 (visualization)
- ✓ Tracker Visualizer (matplotlib plots after 120s delay)

### Monitor
In a 3rd terminal:
```bash
# Watch odometry source
ros2 topic echo /odom

# Quick health check
./scripts/smoke_test_topics.sh
```

### Expected behavior:
You should see the robot move in Isaac Sim (physics-driven) and simultaneously in RViz (tracking odometry from Isaac). The motion may differ slightly from kinematic simulation due to:
- Wheel slip/friction
- Ground deformation
- Inertial effects
- Lower control frequency due to physics CPU load

### Comparison workflow:
1. Run **Workflow 1** (RViz-only), save `sim_log.json` as `sim_log_rviz.json`
2. Run **Workflow 2** (Isaac), save `sim_log.json` as `sim_log_isaac.json`
3. Visualize both:
   ```bash
   ros2 run agri_robot_description tracker_visualizer --run sim_log_rviz.json --plots-dir plots_rviz --show
   ros2 run agri_robot_description tracker_visualizer --run sim_log_isaac.json --plots-dir plots_isaac --show
   ```
4. Compare trajectory fidelity, final position, and control performance

---

## Headless Isaac Sim (for CI/automation)

If you don't need the GUI:
```bash
# Terminal 1: Headless Isaac Sim
./scripts/run_isaacsim.sh --usd src/agri_robot_description/isaac/agri_robot_scene.usd --headless

# Terminal 2: ROS Stack
ros2 launch agri_robot_description full_stack.launch.py use_fake_odom:=false
```

Headless mode runs faster but you won't see visual feedback. Use for automated testing pipelines.

---

## Troubleshooting

### "Isaac Sim script not found at: ~/isaacsim/isaac-sim.sh"
- **Problem**: Isaac Sim not installed
- **Solution**: Install Isaac Sim to `~/isaacsim` or update `run_isaacsim.sh` path

### "Isaac Sim ROS2 bridge library not found"
- **Problem**: ROS2 bridge extension not installed  
- **Solution**: In Isaac Sim, go to Window → Extensions, search and install `isaacsim.ros2.bridge`

### RViz shows '/odom' frame but no motion
- **Problem**: Incorrect odometry source selected
- **Check**: Are you running with `use_fake_odom:=true` (RViz mode)?
  - If yes: `fake_odom_publisher` should be running (`ros2 topic echo /odom` should show updates)
  - If no (Isaac mode): Isaac Sim must be running and publishing `/odom` topic

### `/odom` topic not found after 5 seconds
Run smoke test for diagnostics:
```bash
./scripts/smoke_test_topics.sh
```

If it fails:
1. Check `ros2 topic list | grep odom`
2. Which process should be publishing? 
   - RViz mode: `fake_odom_publisher`
   - Isaac mode: Isaac Sim bridge
3. Check node status: `ros2 node list`

### Tracker not following waypoints
- **Check 1**: Is `pure_pursuit_tracker` running? (`ros2 node list | grep tracker`)
- **Check 2**: Is `/odom` updating? (`ros2 topic hz /odom`)
- **Check 3**: Is tracker receiving waypoints? Check tracker log output
- **Check 4**: Are RViz and Isaac odometry aligned? Visualize both `/odom` and `/cmd_vel` in RViz

---

## Environment Variables (for reference)

These are set automatically by `run_isaacsim.sh`:

```bash
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=~/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH
```

If you source these manually:
```bash
# In your shell
source ~/isaacsim/extbindings/setup.sh
```

---

## Quick Reference

| Component | RViz-Only | Isaac Mode |
|-----------|-----------|-----------|
| Launch | `ros2 launch ... use_fake_odom:=true` | `use_fake_odom:=false` |
| Odom Source | `fake_odom_publisher` (node) | Isaac Sim ROS bridge |
| Physics | Kinematic (no) | Enabled (yes) |
| Speed | Fast | Slower |
| GUI | RViz only | RViz + Isaac |
| Deterministic | Yes | No (physics varies) |

---

## For Algorithm Development

1. **Rapid iteration**: Use RViz-only mode (`use_fake_odom:=true`)
   - Predictable, fast, no external dependencies
   
2. **Validation**: Compare against Isaac (`use_fake_odom:=false`)
   - Ensure algorithm works with real physics effects
   
3. **Visualization**: Always run tracker visualizer
   - Shows trajectory fidelity and control quality

4. **Logging**: Save trajectory JSON files
   - Compare quantitative metrics between modes
