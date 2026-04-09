# IsaacSim-Agriculture-Robot

## Current status

Warning: Isaac Sim ROS bridge integration is available but not fully verified end-to-end in this repository yet.

- Observed in current testing:
	- `/cmd_vel` publisher/subscriber are visible.
	- `/odom` has no active publisher in Isaac mode in some runs.
	- Isaac GUI can become unresponsive on startup depending on local machine/runtime state.
- Recommended default for reliable evaluation right now: RViz + fake odometry (`use_fake_odom:=true`).

## Recommended workflow (reliable)

Use this for algorithm validation and visualization today:

```bash
source /opt/ros/jazzy/setup.bash
source ~/agri_robot_ws/install/setup.bash
cd ~/agri_robot_ws
ros2 launch agri_robot_description full_stack.launch.py use_fake_odom:=true
```

This mode is the safest way to evaluate alignment in RViz and through the tracker visualizer while Isaac mode is still marked untested.

## Optional Isaac workflow (untested warning)

If you want to try Isaac mode anyway:

Terminal 1 (Isaac Sim):

```bash
cd ~/agri_robot_ws
./scripts/run_isaacsim.sh --usd /home/wilsonchandra/agri_robot_ws/src/agri_robot_description/isaac/agri_robot_scene.usd
```

Terminal 2 (ROS stack expecting external odometry):

```bash
source /opt/ros/jazzy/setup.bash
source ~/agri_robot_ws/install/setup.bash
cd ~/agri_robot_ws
ros2 launch agri_robot_description full_stack.launch.py use_fake_odom:=false
```

Terminal 3 (health check):

```bash
source /opt/ros/jazzy/setup.bash
source ~/agri_robot_ws/install/setup.bash
cd ~/agri_robot_ws
./scripts/smoke_test_topics.sh
```

If the smoke test reports `/odom` publisher count = 0, switch back to `use_fake_odom:=true` for evaluation and plotting.

## Notes

- `run_isaacsim.sh` sets required Isaac-side bridge environment internally:
	- `ROS_DISTRO=humble`
	- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
	- `LD_LIBRARY_PATH=~/isaacsim/exts/isaacsim.ros2.bridge/humble/lib:$LD_LIBRARY_PATH`
- ROS terminals can still use local Jazzy setup for CLI and launch.
