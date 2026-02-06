# Custom DWA Local Planner for TurtleBot3 in ROS2 Humble

**Author:** Eshaan  
**Date:** February 6, 2026  
**Assignment:** Implement a Custom Dynamic Window Approach (DWA) Local Planner from Scratch

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Assignment Checklist](#assignment-checklist)
3. [System Requirements](#system-requirements)
4. [Installation & Setup](#installation--setup)
5. [How to Run](#how-to-run)
6. [Architecture & Implementation](#architecture--implementation)
7. [Development Journey](#development-journey)
8. [Problems Faced & Solutions](#problems-faced--solutions)
9. [Tuning & Optimization](#tuning--optimization)
10. [Video Demonstration](#video-demonstration)
11. [References](#references)

---

## Project Overview

This project implements a **Dynamic Window Approach (DWA) local planner** from scratch for a TurtleBot3 Burger robot in Gazebo using ROS2 Humble. The planner computes safe and efficient velocity commands (`/cmd_vel`) by sampling velocity pairs within dynamic constraints, simulating forward trajectories, and evaluating them using a multi-objective cost function that balances goal-seeking, obstacle avoidance, and smooth motion.

### Key Features

- ✅ **Real-time velocity sampling** within acceleration/deceleration limits
- ✅ **Forward trajectory prediction** using kinematic simulation
- ✅ **Multi-objective cost function** (goal distance, heading alignment, obstacle clearance, smoothness, forward speed preference, spin penalty, progress reward)
- ✅ **LaserScan-based obstacle detection** with configurable safety margins
- ✅ **RViz marker visualization** showing candidate trajectories (gray), collision trajectories (red), and the selected best trajectory (green)
- ✅ **Goal-reaching behavior** with position tolerance and final orientation alignment
- ✅ **Recovery behaviors** including reversing capability and blocked-ahead turn forcing
- ✅ **Extensive parameter tuning** for robust navigation in tight spaces

---

## Assignment Checklist

### ✅ Task 1: Set up ROS2 Humble Environment
- [x] Installed ROS2 Humble and Gazebo 11
- [x] Installed TurtleBot3 simulation packages
- [x] Configured environment variables (`TURTLEBOT3_MODEL=burger`, `use_sim_time:=true`)
- [x] Tested TurtleBot3 house world in Gazebo

### ✅ Task 2: Implement Custom DWA Local Planner
- [x] Implemented velocity sampling within dynamic constraints (velocity + acceleration limits)
- [x] Created trajectory prediction function (`simulate_trajectory`) using forward kinematics
- [x] Designed cost function with multiple terms:
  - Distance to goal
  - Heading alignment to goal
  - Obstacle avoidance with soft clearance model
  - Smoothness (penalize large velocity changes)
  - Speed preference (encourage forward motion)
  - Spin penalty (discourage excessive rotation)
  - Progress reward (prefer trajectories that reduce goal distance)
- [x] Implemented collision detection using robot radius + safety margin
- [x] Selected best velocity command based on minimum cost

### ✅ Task 3: Integrate with ROS2 Navigation Stack
- [x] Subscribed to `/odom` (odometry feedback)
- [x] Subscribed to `/scan` (LaserScan for obstacle detection)
- [x] Subscribed to `/goal_pose` (PoseStamped goal input)
- [x] Published velocity commands to `/cmd_vel`
- [x] Published RViz MarkerArray to `/dwa_trajectories` showing:
  - Robot position (blue arrow)
  - Goal position (orange arrow)
  - Candidate trajectories (gray lines)
  - Collision trajectories (red lines)
  - Selected best trajectory (green line)
- [x] Tested in Gazebo TurtleBot3 house world with static obstacles

### ✅ Task 4: Expected Output
- [x] TurtleBot navigates to goals while avoiding obstacles
- [x] Logs meaningful debugging messages:
  - Goal reception with coordinates and frame
  - TF transform failures (with warnings)
  - Cost evaluation details (via parameters)
- [x] Provided comprehensive README with setup instructions (this document)
- [x] Recorded video demonstration (see [Video Demonstration](#video-demonstration))

---

## System Requirements

### Software Dependencies

- **Operating System:** Ubuntu 22.04 LTS (or WSL2 with Ubuntu 22.04)
- **ROS2 Distribution:** Humble Hawksbill
- **Gazebo:** Version 11 (default with ROS2 Humble)
- **Python:** 3.10+

### Required ROS2 Packages

```bash
ros-humble-gazebo-ros-pkgs
ros-humble-turtlebot3
ros-humble-turtlebot3-gazebo
ros-humble-turtlebot3-simulations
```

### Python Dependencies

- `rclpy` (ROS2 Python client library)
- `tf2_ros` (TF transformations)
- Standard library: `math`

---

## Installation & Setup

### Step 1: Install ROS2 Humble

If you haven't already installed ROS2 Humble, follow the official guide:  
[https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

### Step 2: Install TurtleBot3 Packages

```bash
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs \
                    ros-humble-turtlebot3 \
                    ros-humble-turtlebot3-gazebo \
                    ros-humble-turtlebot3-simulations
```

### Step 3: Create Workspace and Clone Package

```bash
# Create workspace
mkdir -p ~/dev/ros2-humble-dwa/src
cd ~/dev/ros2-humble-dwa/src

# Create package structure
ros2 pkg create --build-type ament_python dwa_local_planner
cd dwa_local_planner/dwa_local_planner

# Copy your dwa_planner.py into this directory
# Make sure package.xml and setup.py are configured correctly
```

### Step 4: Configure `setup.py`

Ensure your `setup.py` includes the entry point:

```python
entry_points={
    'console_scripts': [
        'dwa_planner = dwa_local_planner.dwa_planner:main',
    ],
},
```

### Step 5: Build the Workspace

```bash
cd ~/dev/ros2-humble-dwa
colcon build --symlink-install --packages-select dwa_local_planner
source install/setup.bash
```

### Step 6: Set Environment Variables

Add these to your `~/.bashrc` or run in each terminal:

```bash
export TURTLEBOT3_MODEL=burger
export LIBGL_ALWAYS_SOFTWARE=1       # For WSL2/software rendering
export GALLIUM_DRIVER=llvmpipe        # For WSL2 stability
source /opt/ros/humble/setup.bash
source ~/dev/ros2-humble-dwa/install/setup.bash
```

---

## How to Run

### Terminal 1: Launch Gazebo World

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

**Note:** If Gazebo GUI crashes with "D3D12: Removing Device" (common on WSL2), run only the server:

```bash
gzserver --verbose /opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_house.world
```

Then spawn the robot manually in Terminal 2:

```bash
ros2 run gazebo_ros spawn_entity.py \
  -entity burger \
  -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
  -x -2.0 -y -0.5 -z 0.01
```

### Terminal 2: Run Custom DWA Planner

```bash
source ~/dev/ros2-humble-dwa/install/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 run dwa_local_planner dwa_planner --ros-args -p use_sim_time:=true
```

**Optional tuning parameters:**

```bash
ros2 run dwa_local_planner dwa_planner --ros-args \
  -p use_sim_time:=true \
  -p horizon:=3.5 \
  -p v_samples:=12 \
  -p w_samples:=19 \
  -p w_spin:=0.08 \
  -p alpha_max:=6.0
```

### Terminal 3: Visualize in RViz

```bash
source /opt/ros/humble/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1

rviz2
```

**RViz Configuration:**
1. Set **Fixed Frame** to `odom`
2. Add **TF** display
3. Add **LaserScan** display (topic: `/scan`)
4. Add **MarkerArray** display (topic: `/dwa_trajectories`)
5. Add **RobotModel** display (optional)

### Terminal 4: Send Goal Pose

```bash
source /opt/ros/humble/setup.bash

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 1.5, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

**Watch the robot navigate to the goal while avoiding obstacles!**

---

## Architecture & Implementation

### Core Algorithm: Dynamic Window Approach

The DWA algorithm operates in a receding-horizon fashion:

1. **Dynamic Window Construction**
   - Current velocity: `(v_curr, w_curr)` from odometry
   - Admissible velocities constrained by:
     - Maximum limits: `v_max`, `w_max`
     - Acceleration limits: `a_max`, `alpha_max`
     - Control period: `dt`
   - Window: `[v_lo, v_hi] × [w_lo, w_hi]`

2. **Velocity Sampling**
   - Sample `v_samples × w_samples` velocity pairs uniformly within the dynamic window
   - Default: 8 linear × 14 angular = 112 candidates per cycle

3. **Trajectory Prediction**
   - For each `(v, w)`, simulate forward motion for `horizon` seconds using discrete time steps `dt`
   - Kinematic model:
     ```
     x += v * cos(θ) * dt
     y += v * sin(θ) * dt
     θ += w * dt
     ```

4. **Cost Evaluation**
   - Each trajectory scored using weighted sum of costs:
     ```
     total_cost = w_goal × goal_dist
                + w_heading × heading_error
                + w_obs × obstacle_cost
                + w_smooth × smoothness_cost
                + w_speed × speed_cost
                + w_spin × spin_cost
                + 0.8 × progress_cost
     ```

5. **Collision Checking**
   - Transform LaserScan points into `base_footprint` frame using TF
   - Compute minimum distance from trajectory to obstacles
   - Mark trajectory as collision if distance < `robot_radius`

6. **Best Trajectory Selection**
   - Select trajectory with minimum cost
   - Publish first control `(v, w)` to `/cmd_vel`

### Key Design Decisions

#### Soft Obstacle Cost Model

Instead of hard binary collision rejection, I implemented a **soft clearance model**:

```python
if clearance >= preferred_clearance:
    obs_cost = 0.0  # No penalty if we have enough space
else:
    d = (preferred_clearance - clearance) / preferred_clearance
    obs_cost = d * d  # Quadratic penalty as we get closer
```

**Benefits:**
- Allows robot to pass through narrow corridors
- Smooth cost gradients prevent oscillations
- `preferred_clearance` parameter controls "safety bubble" (default: 6 cm beyond robot radius)

#### Progress Reward

To break symmetry and avoid local minima (especially in symmetric environments like corridors):

```python
dist_now = hypot(gx, gy)       # Distance from current position to goal
dist_end = goal_dist           # Distance from trajectory end to goal
progress = dist_now - dist_end # How much closer we got
progress_cost = -progress      # Negative cost = reward
```

This explicitly rewards trajectories that make forward progress toward the goal.

#### Front-Blocked Turn Forcing

When the robot detects an obstacle directly ahead (within a frontal cone), it penalizes trajectories with insufficient turning rate:

```python
if front_blocked and abs(w) < w_min_turn:
    total += w_turn_blocked  # Big penalty
```

This prevents "staring at obstacles" and forces the robot to commit to a turn.

#### Reversing Capability

In tight spaces, sometimes the only way out is to back up. The planner allows small reverse velocities when blocked:

```python
v_lo = clamp(v_curr - a_max * dt, v_min, v_max)  # v_min = -0.06 m/s
if not front_blocked:
    v_lo = clamp(v_lo, 0.0, v_max)  # Disable reverse when clear ahead
```

#### Goal Alignment

When the robot reaches the goal position (within `goal_tolerance = 0.2 m`), it performs a final rotation to match the goal orientation:

```python
if distance_to_goal < goal_tolerance:
    yaw_error = goal_yaw - robot_yaw
    if abs(yaw_error) > yaw_tolerance:
        w_cmd = k_yaw * yaw_error  # Proportional controller
```

---

## Development Journey

### Phase 1: Basic DWA Implementation

**Initial Implementation:**
- Created basic velocity sampling and trajectory simulation
- Implemented simple cost function: `w_goal × goal_dist + w_obs × (1/clearance)`
- Integrated with ROS2 topics (`/odom`, `/scan`, `/cmd_vel`)

**Early Testing Results:**
- Robot successfully reached goals in open space
- Smooth trajectories with good visualization in RViz

**Issues Identified:**
- ❌ Robot kept very large distances from obstacles (too conservative)
- ❌ Slow movement near walls
- ❌ Would spin randomly when goal was near obstacles
- ❌ Got stuck between parallel walls (corridors)

### Phase 2: Obstacle Cost Tuning

**Problem:** The `1/clearance` obstacle cost exploded as the robot approached walls, making it overly cautious.

**Solution:** Implemented soft clearance model with `preferred_clearance` threshold:
- Zero penalty when clearance > 6 cm (beyond robot radius)
- Quadratic penalty only when getting too close
- Reduced `safety_margin` from 5 cm to 2 cm

**Results:**
- ✅ Robot now passes closer to obstacles
- ✅ Faster navigation through doorways
- ⚠️ Still spinning issues near corners

### Phase 3: Speed & Spin Optimization

**Problem:** Robot favored "rotate in place" over moving forward, leading to slow progress and excessive spinning.

**Solution:** Added explicit speed and spin cost terms:
```python
speed_cost = (v_max - v)           # Reward higher forward velocity
spin_cost = abs(w) + 0.5 * abs(w - w_prev)  # Penalize rotation + oscillation
```

**Tuning:**
- `w_speed = 0.6` (moderate forward preference)
- `w_spin = 0.2` (gentle spin penalty, reduced from initial 0.4 to allow turning)

**Results:**
- ✅ Much faster forward motion
- ✅ Reduced spinning behavior
- ⚠️ Sometimes still "stares" at obstacles without turning

### Phase 4: Front-Blocked Turn Forcing

**Problem:** When facing an obstacle directly ahead, the cost function sometimes favored "do nothing" (v=0, w≈0) over committing to a turn.

**Solution:** Added explicit blocked-ahead detection and turn penalty:
```python
front_blocked = (front_min_range() < 0.55 m)
if front_blocked and abs(w) < w_min_turn:
    total += w_turn_blocked  # Large penalty
```

**Results:**
- ✅ Robot now decisively turns when blocked
- ✅ No more "staring at walls"
- ✅ Better reactive behavior in cluttered spaces

### Phase 5: Progress Reward & Horizon Tuning

**Problem:** In symmetric environments (parallel walls), the robot would sometimes oscillate or make no progress due to cost symmetry.

**Solution:** 
1. Added progress reward term to explicitly favor trajectories that reduce goal distance
2. Increased prediction horizon from 2.0s to 3.5s for better foresight
3. Increased angular velocity samples from 11 to 19 for finer turn resolution

**Results:**
- ✅ Consistent forward progress in corridors
- ✅ Earlier obstacle detection (longer horizon)
- ✅ Smoother turns (more angular samples)

### Phase 6: Goal Alignment & Polish

**Problem:** Robot would reach goal position but not align to goal orientation (orange arrow).

**Solution:** Implemented two-stage goal behavior:
1. Move to goal position (within 20 cm tolerance)
2. Rotate in place to match goal yaw (within 14° tolerance)

**Final Polish:**
- Reduced LaserScan stride from 3 to 1 (denser obstacle map)
- Increased alpha_max from 3.0 to 6.0 rad/s² (faster angular acceleration)
- Added reversing capability for recovery from tight spots

**Final Results:**
- ✅ Precise goal reaching with correct orientation
- ✅ Fast, confident navigation
- ✅ Robust handling of tight spaces
- ✅ Recovery from local minima

---

## Problems Faced & Solutions

### Problem 1: Gazebo Robot Spawn Failure

**Error:**
```
[ERROR] [spawn_entity.py-4]: Service /spawn_entity unavailable.
Was Gazebo started with GazeboRosFactory?
```

**Root Cause:** Gazebo wasn't loading ROS factory plugins, so `/spawn_entity` service was unavailable.

**Solution:**
1. Verified `ros-humble-gazebo-ros-pkgs` was installed
2. Set environment variables for WSL2 stability:
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   export GALLIUM_DRIVER=llvmpipe
   ```
3. Launched `gzserver` manually with plugins:
   ```bash
   gzserver -s libgazebo_ros_init.so -s libgazebo_ros_factory.so world_file.world
   ```

**Outcome:** Robot spawned successfully in Gazebo.

---

### Problem 2: Indentation Errors in Python

**Error:**
```
IndentationError: expected an indented block after 'if' statement
```

**Root Cause:** Mixed tabs and spaces, or incorrect nesting when adding new code blocks.

**Solution:**
1. Configured VS Code to use 4 spaces for indentation
2. Ran `autopep8` for consistent formatting
3. Carefully verified indentation when copying code snippets

**Outcome:** Clean, error-free Python code.

---

### Problem 3: Robot Too Conservative Near Obstacles

**Symptoms:**
- Robot kept 30-40 cm away from walls
- Very slow movement in corridors
- Avoided narrow passages

**Root Cause:** `1/clearance` obstacle cost exploded near walls, dominating all other costs.

**Solution:**
- Implemented soft clearance model with `preferred_clearance` threshold
- Reduced `safety_margin` from 5 cm to 2 cm
- Lowered `w_obs` weight from 2.0 to 2.0 (kept same, but soft model reduced effective penalty)

**Outcome:** Robot confidently navigates close to obstacles (maintains ~8 cm clearance).

---

### Problem 4: Excessive Spinning Near Goals

**Symptoms:**
- Robot would spin multiple times before reaching goal
- Erratic rotation behavior near corners
- Slow progress despite clear path

**Root Cause:** No explicit penalty for rotation; cost function was indifferent between "move forward" and "rotate in place."

**Solution:**
1. Added `speed_cost = (v_max - v)` to reward forward motion
2. Added `spin_cost = abs(w) + 0.5 * abs(w - w_prev)` to penalize rotation and oscillation
3. Tuned weights: `w_speed = 0.6`, `w_spin = 0.2`

**Outcome:** Robot now prefers smooth forward motion and only turns when necessary.

---

### Problem 5: Stuck Between Parallel Walls

**Symptoms:**
- Robot would stop in corridors with walls on both sides
- No forward progress despite clear path ahead
- Oscillating left-right without advancing

**Root Cause:** 
1. Cost symmetry (left and right walls created identical costs)
2. Short prediction horizon (2.0s) didn't see far enough ahead

**Solution:**
1. Added progress reward to break symmetry
2. Increased horizon from 2.0s to 3.5s
3. Allowed small reversing capability (`v_min = -0.06 m/s`)

**Outcome:** Robot consistently makes forward progress through corridors.

---

### Problem 6: Not Aligning to Goal Orientation

**Symptoms:**
- Robot reached goal position but faced wrong direction
- Orange arrow (goal) and blue arrow (robot) didn't align

**Root Cause:** Planner only optimized for position, not final orientation.

**Solution:** Implemented two-stage goal behavior:
1. Position phase: Navigate to within 20 cm of goal
2. Alignment phase: Stop and rotate to match goal yaw

**Outcome:** Robot now precisely matches goal pose (position + orientation).

---

### Problem 7: D3D12 GPU Crash on WSL2

**Error:**
```
[gzclient-2] D3D12: Removing Device.
[ERROR] [gzclient-2]: process has died
```

**Root Cause:** Gazebo GUI using DirectX 12 backend (WSLg) which is unstable.

**Solution:**
1. Force software rendering:
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   export GALLIUM_DRIVER=llvmpipe
   ```
2. Run only `gzserver` (no GUI) and use RViz for visualization

**Outcome:** Stable simulation without graphics crashes.

---

## Tuning & Optimization

### Critical Parameters

| Parameter | Default | Tuned Value | Effect |
|-----------|---------|-------------|--------|
| `horizon` | 2.0 s | **3.5 s** | Longer prediction for earlier obstacle detection |
| `dt` | 0.1 s | **0.08 s** | More accurate trajectory simulation |
| `v_samples` | 8 | **12** | Finer linear velocity resolution |
| `w_samples` | 11 | **19** | Smoother turns with more angular samples |
| `alpha_max` | 3.0 rad/s² | **6.0 rad/s²** | Faster turning response |
| `safety_margin` | 0.05 m | **0.02 m** | Closer approach to obstacles |
| `preferred_clearance` | N/A | **0.06 m** | Soft obstacle penalty threshold |
| `w_spin` | N/A | **0.08** | Gentle spin penalty (allows necessary turns) |
| `w_speed` | N/A | **0.6** | Moderate forward speed preference |
| `w_heading` | N/A | **0.8** | Strong heading alignment to goal |
| `goal_tolerance` | N/A | **0.20 m** | Position threshold for goal alignment phase |
| `v_min` | 0.0 | **-0.06 m/s** | Small reversing for recovery |

### Cost Function Weights

```python
total_cost = (
    1.0 * goal_dist          # Distance to goal
    + 0.8 * heading_error    # Angular alignment to goal
    + 2.0 * obs_cost         # Obstacle avoidance (soft model)
    + 0.2 * smooth_cost      # Velocity smoothness
    + 0.6 * speed_cost       # Forward speed preference
    + 0.08 * spin_cost       # Rotation penalty
    + 0.8 * progress_cost    # Progress reward
)
```

### Recommended Starting Points for Different Scenarios

**Open Spaces:**
- `w_spin = 0.05` (less turn penalty)
- `w_speed = 0.8` (more speed preference)
- `preferred_clearance = 0.08 m` (can be slightly more cautious)

**Tight Corridors:**
- `w_spin = 0.12` (stronger turn penalty to reduce oscillation)
- `w_obs = 1.5` (lower obstacle weight)
- `preferred_clearance = 0.05 m` (tighter clearance)
- `v_min = -0.08 m/s` (allow more reversing)

**Cluttered Spaces:**
- `horizon = 4.0 s` (longer lookahead)
- `w_samples = 23` (more angular resolution)
- `front_blocked_dist = 0.7 m` (earlier turn forcing)

---

## Video Demonstration

### 🎥 Proof of Life

**Video:** `dwa_planner_demo.mp4` (included in submission)

**Contents:**
1. **Gazebo Environment** - TurtleBot3 in house world with furniture obstacles
2. **RViz Visualization** - Real-time trajectory markers:
   - Gray lines: candidate trajectories
   - Red lines: collision trajectories
   - Green line: selected best trajectory
   - Blue arrow: robot position
   - Orange arrow: goal position
3. **Navigation Scenarios:**
   - Open space goal reaching
   - Obstacle avoidance (navigating around furniture)
   - Corridor navigation (between parallel walls)
   - Tight doorway passage
   - Final orientation alignment at goal
4. **Terminal Output** - Showing goal reception logs and planner status

**Key Observations in Video:**
- Robot smoothly navigates to multiple goals
- Maintains safe clearance from obstacles (~8-10 cm)
- Fast, decisive turning when blocked
- Consistent forward progress through corridors
- Precise goal pose matching (position + orientation)
- No spinning or oscillation issues
- RViz markers clearly show trajectory evaluation process

---

## References

1. Fox, D., Burgard, W., & Thrun, S. (1997). *The Dynamic Window Approach to Collision Avoidance*. IEEE Robotics & Automation Magazine.
   
2. ROS2 Humble Documentation: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)

3. TurtleBot3 e-Manual: [https://emanual.robotis.com/docs/en/platform/turtlebot3/](https://emanual.robotis.com/docs/en/platform/turtlebot3/)

4. Gazebo ROS Plugins: [https://classic.gazebosim.org/tutorials?tut=ros2_overview](https://classic.gazebosim.org/tutorials?tut=ros2_overview)

5. Dynamic Window Approach Tutorial: *Adaptive Agro Tech Lecture 10 - Local Path Planning with DWA*

---

## Conclusion

This project successfully demonstrates a fully functional Dynamic Window Approach local planner implemented from scratch in ROS2 Humble. The planner exhibits robust obstacle avoidance, smooth motion generation, and reliable goal-reaching behavior across diverse environments.

Through iterative development and extensive tuning, I addressed key challenges including conservative obstacle avoidance, excessive spinning, corridor navigation, and goal alignment. The final implementation balances safety and efficiency, producing fast and confident navigation while maintaining appropriate safety margins.

The visualization system provides clear insight into the planner's decision-making process, showing how it evaluates and selects trajectories in real-time. The code is well-structured, parameterized, and ready for further extension or integration into larger navigation systems.

**Future Improvements:**
- Integration with global planner for waypoint following
- Dynamic obstacle handling (moving objects)
- Costmap integration for map-based planning
- Adaptive parameter tuning based on environment
- Path tracking mode for following predefined trajectories

---

**Contact:** [Your Email/GitHub]  
**License:** MIT  
**Last Updated:** February 6, 2026
