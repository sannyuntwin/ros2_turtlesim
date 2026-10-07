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
git clone https://github.com/sannyuntwin/ROS2_MultiThreadedExecutor.git ros2_ws
cd ros2_ws
colcon build
source install/setup.bash
```

---

## Run

> **Important:** Start terminals in this order. Each terminal must source the workspace first.

**Terminal 1 — turtlesim:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run turtlesim turtlesim_node
```

**Terminal 2 — draw_circle node:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_turtle_controllers draw_circle
```
The node logs `Draw circle node ready for [turtle1].` and then runs silently while drawing circles.

**Terminal 3 — send a navigation goal:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 action send_goal /turtle1/navigate_path my_robot_interfaces/action/NavigatePath \
  "{waypoints: [{position: {x: 5.0, y: 8.0, z: 0.0}, orientation: {w: 1.0}},
                {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}],
    linear_speed: 2.0, angular_speed: 0.0, waypoint_timeout: 10.0, resume: false}"
```

---

## Features

### 1. Pose feedback
The action client receives `current_x` and `current_y` in every feedback message.

### 2. Cancel and resume
Cancel a running goal with `Ctrl+C`, then resume from the same waypoint:
```bash
ros2 action send_goal /turtle1/navigate_path my_robot_interfaces/action/NavigatePath \
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
```bash
ros2 service call /turtle1/set_speed my_robot_interfaces/srv/SetSpeed \
  "{linear_speed: 1.0, angular_speed: 1.0}"
```

### 6. Multiple turtles
Launch two turtles at once:
```bash
ros2 launch my_turtle_controllers multi_turtle.launch.py
```
Then send goals to `/turtle1/navigate_path` or `/turtle2/navigate_path` independently.
