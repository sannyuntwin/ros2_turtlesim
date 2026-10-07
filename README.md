# ROS2_MultiThreadedExecutor

A ROS2 project demonstrating a `MultiThreadedExecutor` with turtlesim.

## Packages

### `my_robot_interfaces`

**Action:** `NavigatePath`
- **Goal:** `geometry_msgs/Pose[] waypoints`, `float32 linear_speed`, `float32 angular_speed`, `float32 waypoint_timeout`, `bool resume`
- **Result:** `bool success`, `string message`, `uint32 waypoints_completed`
- **Feedback:** `uint32 current_index`, `uint32 total_waypoints`, `float32 remaining_distance`, `float32 current_x`, `float32 current_y`

**Service:** `SetSpeed`
- **Request:** `float32 linear_speed`, `float32 angular_speed`
- **Response:** `bool success`, `string message`

### `my_turtle_controllers`

**Node:** `draw_circle_node`
- Publishes circular motion to `/<turtle_name>/cmd_vel`
- Subscribes to `/<turtle_name>/pose` for current position
- Exposes `/<turtle_name>/navigate_path` action server with proportional control navigation
- Exposes `/<turtle_name>/set_speed` service to change circle speed at runtime
- Uses `MultiThreadedExecutor` so pose callbacks keep firing during action execution
- Supports `turtle_name` parameter for running multiple instances

---

## Setup (Ubuntu 24.04 / WSL)

### 1. Install ROS2 Jazzy

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-jazzy-desktop
sudo apt install -y python3-colcon-common-extensions
```

### 2. Source ROS2 permanently

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Clone and build

```bash
mkdir -p ~/ros2 && cd ~/ros2
git clone https://github.com/sannyuntwin/ros2_turtlesim.git
cd ros2_turtlesim
colcon build
source install/setup.bash
```

---

## Run

> **Important:** Start terminals in this order. Each terminal must source the workspace first.

**Terminal 1 — turtlesim:**
```bash
source ~/ros2/ros2_turtlesim/install/setup.bash
ros2 run turtlesim turtlesim_node
```

**Terminal 2 — draw_circle node:**
```bash
source ~/ros2/ros2_turtlesim/install/setup.bash
ros2 run my_turtle_controllers draw_circle
```
The node logs `Draw circle node ready for [turtle1].` and then runs silently while drawing circles.

**Terminal 3 — send a navigation goal:**
```bash
source ~/ros2/ros2_turtlesim/install/setup.bash
ros2 action send_goal --feedback /turtle1/navigate_path my_robot_interfaces/action/NavigatePath \
  "{waypoints: [{position: {x: 5.0, y: 8.0, z: 0.0}, orientation: {w: 1.0}},
                {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}],
    linear_speed: 2.0, angular_speed: 0.0, waypoint_timeout: 30.0, resume: false}"
```

---

## Testing Each Feature

> Before testing any feature: Terminal 1 (turtlesim) and Terminal 2 (draw_circle) must already be running.

### Test Feature 1 — Pose feedback
Send a goal with `--feedback`. Watch live position updates print in Terminal 3:
```bash
source ~/ros2/ros2_turtlesim/install/setup.bash
ros2 action send_goal --feedback /turtle1/navigate_path my_robot_interfaces/action/NavigatePath \
  "{waypoints: [{position: {x: 5.0, y: 8.0, z: 0.0}, orientation: {w: 1.0}}],
    linear_speed: 2.0, angular_speed: 0.0, waypoint_timeout: 30.0, resume: false}"
```
**Expected:** Feedback lines showing `current_x`, `current_y`, `remaining_distance` updating every 0.1s.

---

### Test Feature 2 — Cancel and resume
**Step 1** — Send a 5-waypoint goal at slow speed:
```bash
source ~/ros2/ros2_turtlesim/install/setup.bash
ros2 action send_goal --feedback /turtle1/navigate_path my_robot_interfaces/action/NavigatePath \
  "{waypoints: [
      {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}},
      {position: {x: 9.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}},
      {position: {x: 9.0, y: 9.0, z: 0.0}, orientation: {w: 1.0}},
      {position: {x: 1.0, y: 9.0, z: 0.0}, orientation: {w: 1.0}},
      {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}],
    linear_speed: 1.0, angular_speed: 0.0, waypoint_timeout: 60.0, resume: false}"
```
**Step 2** — While turtle is moving, press `Ctrl+C`.
**Expected in Terminal 2:** `Cancelled at waypoint X. Send goal with resume=true to continue.`

**Step 3** — Resume from the cancelled waypoint:
```bash
ros2 action send_goal --feedback /turtle1/navigate_path my_robot_interfaces/action/NavigatePath \
  "{waypoints: [], linear_speed: 0.0, angular_speed: 0.0, waypoint_timeout: 0.0, resume: true}"
```
**Expected:** Turtle continues from the waypoint it was heading to, not from the beginning.

---

### Test Feature 3 — Speed parameter in goal
Send the same goal twice with different speeds:
```bash
# Slow
ros2 action send_goal --feedback /turtle1/navigate_path my_robot_interfaces/action/NavigatePath \
  "{waypoints: [{position: {x: 8.0, y: 8.0, z: 0.0}, orientation: {w: 1.0}}],
    linear_speed: 0.5, angular_speed: 0.0, waypoint_timeout: 30.0, resume: false}"
# Fast
ros2 action send_goal --feedback /turtle1/navigate_path my_robot_interfaces/action/NavigatePath \
  "{waypoints: [{position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}],
    linear_speed: 5.0, angular_speed: 0.0, waypoint_timeout: 30.0, resume: false}"
```
**Expected:** Turtle visibly moves faster for the second goal.

---

### Test Feature 4 — Waypoint timeout
Set a very short timeout (2 seconds) for a far waypoint:
```bash
ros2 action send_goal --feedback /turtle1/navigate_path my_robot_interfaces/action/NavigatePath \
  "{waypoints: [
      {position: {x: 9.0, y: 9.0, z: 0.0}, orientation: {w: 1.0}},
      {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}],
    linear_speed: 1.0, angular_speed: 0.0, waypoint_timeout: 2.0, resume: false}"
```
**Expected in Terminal 2:** `Waypoint 1 timed out — skipping.` then turtle moves to waypoint 2.

---

### Test Feature 5 — Change circle speed at runtime
While turtle is circling (Terminal 2 running, no active goal), open a new terminal:
```bash
source ~/ros2/ros2_turtlesim/install/setup.bash
# Slow it down
ros2 service call /turtle1/set_speed my_robot_interfaces/srv/SetSpeed \
  "{linear_speed: 0.3, angular_speed: 0.3}"
# Speed it up
ros2 service call /turtle1/set_speed my_robot_interfaces/srv/SetSpeed \
  "{linear_speed: 4.0, angular_speed: 4.0}"
```
**Expected:** Circle radius and speed change immediately in the turtlesim window.

---

### Test Feature 6 — Multiple turtles
Stop Terminal 1 and Terminal 2 (Ctrl+C both). Then use the launch file instead:
```bash
source ~/ros2/ros2_turtlesim/install/setup.bash
ros2 launch my_turtle_controllers multi_turtle.launch.py
```
**Expected:** Two turtles appear and both draw circles. Send goals to each independently:
```bash
# Terminal A — navigate turtle1
ros2 action send_goal --feedback /turtle1/navigate_path my_robot_interfaces/action/NavigatePath \
  "{waypoints: [{position: {x: 2.0, y: 8.0, z: 0.0}, orientation: {w: 1.0}}],
    linear_speed: 2.0, angular_speed: 0.0, waypoint_timeout: 30.0, resume: false}"
# Terminal B — navigate turtle2 at the same time
ros2 action send_goal --feedback /turtle2/navigate_path my_robot_interfaces/action/NavigatePath \
  "{waypoints: [{position: {x: 8.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}],
    linear_speed: 2.0, angular_speed: 0.0, waypoint_timeout: 30.0, resume: false}"
```

---

## Features

### 1. Pose feedback
The action client receives `current_x` and `current_y` in every feedback message.

### 2. Cancel and resume

**Step 1** — Send a goal with many waypoints (use slow speed so there's time to cancel):
```bash
ros2 action send_goal --feedback /turtle1/navigate_path my_robot_interfaces/action/NavigatePath \
  "{waypoints: [
      {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}},
      {position: {x: 9.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}},
      {position: {x: 9.0, y: 9.0, z: 0.0}, orientation: {w: 1.0}},
      {position: {x: 1.0, y: 9.0, z: 0.0}, orientation: {w: 1.0}},
      {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}],
    linear_speed: 1.0, angular_speed: 0.0, waypoint_timeout: 60.0, resume: false}"
```

**Step 2** — While the turtle is moving, press `Ctrl+C`. Terminal 2 should log `Cancelled at waypoint X`.

**Step 3** — Resume from where it stopped:
```bash
ros2 action send_goal --feedback /turtle1/navigate_path my_robot_interfaces/action/NavigatePath \
  "{waypoints: [], linear_speed: 0.0, angular_speed: 0.0, waypoint_timeout: 0.0, resume: true}"
```

### 3. Speed parameter in goal
Set `linear_speed` in the goal (0.0 uses the node default):
```bash
"{..., linear_speed: 3.0, ...}"
```

### 4. Waypoint timeout
If the turtle cannot reach a waypoint within the timeout (seconds), it skips and continues:
```bash
"{..., waypoint_timeout: 5.0, ...}"
```

### 5. Change circle speed at runtime

While the turtle is circling (no active goal), call from a new terminal:
```bash
source ~/ros2/ros2_turtlesim/install/setup.bash
ros2 service call /turtle1/set_speed my_robot_interfaces/srv/SetSpeed \
  "{linear_speed: 0.3, angular_speed: 0.3}"
```

### 6. Multiple turtles

Instead of running Terminal 1 + Terminal 2 separately, use the launch file:
```bash
source ~/ros2/ros2_turtlesim/install/setup.bash
ros2 launch my_turtle_controllers multi_turtle.launch.py
```
This starts turtlesim and two draw_circle nodes (turtle1 and turtle2). Send goals independently:
```bash
ros2 action send_goal --feedback /turtle2/navigate_path my_robot_interfaces/action/NavigatePath \
  "{waypoints: [{position: {x: 3.0, y: 7.0, z: 0.0}, orientation: {w: 1.0}}],
    linear_speed: 1.5, angular_speed: 0.0, waypoint_timeout: 30.0, resume: false}"
```
