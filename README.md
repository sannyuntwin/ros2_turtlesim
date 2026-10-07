# ROS2_MultiThreadedExecutor

A ROS2 project demonstrating a `MultiThreadedExecutor` with turtlesim.

## Packages

### `my_robot_interfaces`
Custom action interface for path navigation.

**Action:** `NavigatePath`
- **Goal:** `geometry_msgs/Pose[] waypoints`
- **Result:** `bool success`, `string message`
- **Feedback:** `uint32 current_index`, `float32 remaining_distance`

### `my_turtle_controllers`
Node that drives turtlesim in a circle and accepts path-following action goals.

**Node:** `draw_circle_node`
- Publishes circular motion to `/turtle1/cmd_vel`
- Subscribes to `/turtle1/pose` for current position
- Exposes a `navigate_path` action server that navigates the turtle through waypoints using proportional control
- Uses `MultiThreadedExecutor` so the pose subscriber keeps firing during blocking action execution

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

### 2. Source ROS2 (add to ~/.bashrc to make permanent)

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Clone the repo

```bash
git clone https://github.com/sannyuntwin/ROS2_MultiThreadedExecutor.git ros2_ws
cd ros2_ws
```

### 4. Build

```bash
colcon build
source install/setup.bash
```

---

## Run

> **Important:** Start terminals in this order. Each terminal must source the workspace first.

**Terminal 1 — draw_circle node (start this first):**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_turtle_controllers draw_circle
```

**Terminal 2 — turtlesim:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run turtlesim turtlesim_node
```

**Terminal 3 — send a navigation goal (only after both above are running):**
```bash
source ~/ros2_ws/install/setup.bash
ros2 action send_goal /navigate_path my_robot_interfaces/action/NavigatePath \
  "{waypoints: [{position: {x: 5.0, y: 8.0, z: 0.0}, orientation: {w: 1.0}},
                {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}]}"
```

The turtle will draw circles until a goal is sent, then navigate to each waypoint using proportional control.
