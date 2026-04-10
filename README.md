# IsaacSim Agriculture Robot

ROS 2–based agricultural robot stack with a Pure Pursuit path tracker, RViz visualization, trajectory logging, and optional Isaac Sim integration for physics‑based testing. The goal is to validate path‑following behavior in both a fast kinematic mode and a more realistic simulation mode.

> **Status:**  
> - ✅ RViz + fake odometry workflow: **reliable, tested, and recommended** for day‑to‑day development.  
> - ⚠️ Isaac Sim integration: **experimental**; end‑to‑end odometry publishing is not fully validated on this machine.

---

## Project Overview

This project implements:

- A **Pure Pursuit tracker** for following reference trajectories.
- **RViz visualization** for:
  - Robot pose and path
  - Target trajectory
  - Control points / lookahead markers
- **Trajectory logging** and plotting for analysis.
- **Optional Isaac Sim integration** to run the same stack against a physics‑based simulator.

Two primary modes are supported:

1. **Fast kinematic mode (recommended)**  
   - Uses simple kinematic odometry (fake odom).  
   - Runs deterministically and is well‑suited for:
     - Debugging the Pure Pursuit algorithm
     - Verifying control behavior
     - Visual inspection in RViz

2. **Isaac Sim mode (experimental)**  
   - Uses Nvidia Isaac Sim as a physics‑based backend.  
   - Intended for more realistic dynamic testing, but odometry integration is **not fully validated** end‑to‑end yet.

---

## Recommended Workflow (RViz + Fake Odometry)

This mode uses kinematic odometry, runs deterministically, and is the preferred mode for inspecting algorithm behavior in RViz and the tracker visualizer.

Typical setup:

1. **Build the workspace** (from your ROS Jazzy workspace):

   ```bash
   colcon build
   source install/setup.bash
   ```

2. **Launch the kinematic / fake odometry workflow** (example):

   ```bash
   ros2 launch <your_package> rviz_fake_odom.launch.py use_fake_odom:=true
   ```

3. Inspect in **RViz**:
   - Robot base frame
   - Local/global trajectory
   - Pure Pursuit lookahead point
   - Path tracking error

4. Analyze outputs (see [Outputs](#outputs)).

> Replace `<your_package>` / launch file names above with the actual ones you use in this repo.

---

## Optional Isaac Workflow (Experimental)

Isaac Sim integration is available for physics‑based validation of the controller, but should currently be treated as **experimental**.

A typical setup might look like:

### Terminal 1: ROS 2 Core / Main Launch

```bash
# ROS Jazzy environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch <your_package> tracker_with_rviz.launch.py use_fake_odom:=false
```

### Terminal 2: Isaac Sim Bridge / ROS 2 Bridge

```bash
# DO NOT source Humble globally here.
# Use the Isaac-provided launch script / bridge script.
./run_isaac_bridge.sh
# or whatever script is provided by Isaac Sim in this repo
```

### Terminal 3: Isaac Sim Itself

```bash
# Start Isaac Sim from its own environment
./isaac-sim.sh
# Load the appropriate scene / USD world for the agricultural robot
```

> The exact commands/scripts will depend on how Isaac Sim is configured in this repo.  
> Replace the placeholder commands above with the concrete ones you use locally.

---

## Warning

Isaac Sim mode is currently **not fully validated end‑to‑end** on this machine.

- If `/odom` is **not** being published from Isaac Sim, the tracker will not receive real odometry.
- In that case:
  - Fall back to `use_fake_odom:=true` in the ROS launch.
  - Verify the trajectory in **RViz** and in the **tracker visualizer**.
- Treat Isaac Sim runs as **experimental** and cross‑check behavior against the known‑good RViz + fake odom workflow.

---

## Environment Notes

- This laptop uses **ROS 2 Jazzy** for the local ROS workspace.
- The **Isaac Sim bridge** uses its **own internal ROS 2 Humble** environment, which is handled by the Isaac startup script.
- **Do not** source Humble globally in the normal ROS terminals used for Jazzy:
  - Keep Jazzy terminals “clean” (only Jazzy + your workspace overlays).
  - Let Isaac Sim manage its own Humble environment internally.

A safe pattern is:

- Terminals running your regular ROS nodes and RViz:
  - `source /opt/ros/jazzy/setup.bash`
  - `source <your_ws>/install/setup.bash`
- Terminals launching Isaac Sim or its bridges:
  - Use the Isaac Sim–provided scripts; **do not** manually source Humble in your Jazzy terminals.

---

## Outputs

The stack logs trajectory data and generates plots for analysis:

- **Logs**
  - `sim_log.json`
    - Contains trajectory and control data (e.g., timestamps, reference vs actual pose, control commands).
- **Plots**
  - Generated under `plots/`
  - Typical plots can include:
    - Reference vs actual trajectory in `x–y` space
    - Longitudinal / lateral tracking error
    - Control inputs over time (velocity, steering, etc.)

Use these outputs to:

- Compare performance between fake odom and Isaac Sim runs.
- Quantify tracking error.
- Diagnose controller tuning issues.

---

## Repository Layout (Placeholder)

_Note: update this section once the repository structure is finalized._

Example:

- `config/` – Robot and controller configuration files.
- `launch/` – ROS 2 launch files for:
  - RViz + fake odom workflow
  - Isaac Sim workflow
- `scripts/` – Helper scripts, plotting utilities.
- `src/` – Core ROS 2 nodes (tracker, visualizer, logging).
- `plots/` – Generated plots.
- `sim_log.json` – Most recent trajectory log file.

---

## Future Improvements

- Harden the Isaac Sim integration:
  - Ensure `/odom` (and/or `/tf`) is consistently published from the simulator.
  - Validate that the same Pure Pursuit configuration works under physics.
- Add automated comparisons between kinematic and Isaac modes.
- Add CI checks or basic simulation tests for regression protection.

---

## Doumentation

<img width="2494" height="1408" alt="Screenshot from 2026-04-05 17-38-26" src="https://github.com/user-attachments/assets/168145ab-b2bc-4ce2-8a2c-6e6d1ad20a2f" />

