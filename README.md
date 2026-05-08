# 🤖 Color-Based Navigation — Peppermint Robotics Intern Challenge

**Candidate:** Utkarsh | [@Psynox-lang](https://github.com/Psynox-lang)  
**Stack:** ROS2 Humble · TurtleBot3 Waffle · OpenCV · Gazebo  
**Platform:** Ubuntu 22.04

---

## 📽️ Demo

Robot detects green sphere → rotates to align → drives forward → stops safely via LiDAR.

```
Camera Feed → HSV Threshold → Contour Detection → P-Controller → /cmd_vel
LiDAR Scan  → Front-sector min range → Emergency Stop at < 0.30 m
```

---

## 📁 Repository Structure

```
color_follower/
├── color_follower/
│   ├── __init__.py
│   └── follower_node.py       # Main perception + control node
├── models/
│   └── green_sphere.sdf       # Gazebo sphere model (ready to spawn)
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

---

## ⚙️ Requirements

- Ubuntu 22.04
- ROS2 Humble
- TurtleBot3 packages
- Python 3, OpenCV, cv_bridge

---

## 🛠️ Installation

### 1. Install ROS2 Humble

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu \
  $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop -y

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install TurtleBot3 & Dependencies

```bash
sudo apt install -y \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-simulations \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-cv-bridge \
  python3-opencv \
  python3-colcon-common-extensions

echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

### 3. Clone & Build

```bash
mkdir -p ~/peppermint_ws/src
cd ~/peppermint_ws/src
git clone https://github.com/Psynox-lang/Peppermint-Task.git color_follower

cd ~/peppermint_ws
colcon build
source install/setup.bash
```

---

## 🚀 Running

### Terminal 1 — Launch Gazebo

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal 2 — Spawn the Green Sphere

The SDF file is included in the repo under `models/green_sphere.sdf`.

```bash
ros2 run gazebo_ros spawn_entity.py \
  -file ~/peppermint_ws/src/color_follower/models/green_sphere.sdf \
  -entity green_sphere
```

### Terminal 3 — Run the Color Follower Node

```bash
cd ~/peppermint_ws
source install/setup.bash
ros2 run color_follower follower_node
```

---

## 🧠 How It Works

### Color Detection
- Converts camera feed to HSV colorspace
- Thresholds for green: `lower=[59, 240, 95]` → `upper=[61, 255, 110]`
- Applies ROI mask — centre 50% width, lower 70% height — to ignore background green objects
- Picks the lowest valid blob in frame (sphere sits on the ground plane)

### P-Controller

| Condition | Linear (m/s) | Angular (rad/s) |
|-----------|-------------|-----------------|
| Misaligned `\|error\|` ≥ 30 px | 0.0 | `0.002 × error` |
| Aligned `\|error\|` < 30 px | 0.15 | 0.0 |
| LiDAR < 0.30 m | 0.0 | 0.0 |
| No sphere detected | 0.0 | 0.08 (recovery spin) |

### LiDAR Obstacle Stop
- Subscribes to `/scan`
- Monitors front ±30° arc (indices 0–29 and 330–359)
- Publishes zero `Twist` and halts all processing when closest reading < 0.30 m

### Recovery Behaviour
- If no green contour is found in the ROI, the robot spins slowly at `0.08 rad/s`
- Continues until the sphere re-enters the camera frame

---

## 🔧 Tuning Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `Kp_angular` | `0.002` | Proportional gain for rotation |
| `angular_threshold` | `30` px | Error below which robot drives forward |
| `min_lidar_distance` | `0.30` m | Stop distance from obstacle |
| `MIN_CONTOUR_AREA` | `200` px² | Minimum blob size to consider valid |
| `LOWER_GREEN` | `[59, 240, 95]` | HSV lower bound |
| `UPPER_GREEN` | `[61, 255, 110]` | HSV upper bound |

> **Tip:** If the sphere isn't detected in your environment, adjust HSV bounds by capturing a frame with `rqt_image_view` and using a colour picker.