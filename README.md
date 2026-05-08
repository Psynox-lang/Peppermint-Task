# Color Follower — ROS2 + OpenCV

A ROS2 node that detects a green sphere using OpenCV and navigates 
a Turtlebot3 Waffle toward it using a P controller, stopping safely 
using LiDAR.

## Demo
Robot detects green sphere → rotates to align → moves forward → 
stops when LiDAR detects obstacle within 0.30m.

## Requirements
- Ubuntu 22.04
- ROS2 Humble
- Turtlebot3 packages
- Python3, OpenCV, cv_bridge

## Installation

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

### 2. Install Turtlebot3
```bash
sudo apt install -y ros-humble-turtlebot3 \
                   ros-humble-turtlebot3-gazebo \
                   ros-humble-turtlebot3-simulations \
                   ros-humble-gazebo-ros-pkgs \
                   ros-humble-cv-bridge \
                   python3-opencv \
                   python3-colcon-common-extensions
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

### 3. Clone and build
```bash
mkdir -p ~/peppermint_ws/src
cd ~/peppermint_ws/src
git clone https://github.com/YOUR_USERNAME/color_follower.git
cd ~/peppermint_ws
colcon build
source install/setup.bash
```

### 4. Create green sphere SDF
```bash
cat > ~/green_sphere.sdf << 'EOF'
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="green_sphere">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <sphere><radius>0.2</radius></sphere>
        </geometry>
        <material>
          <ambient>0 1 0 1</ambient>
          <diffuse>0 1 0 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <sphere><radius>0.2</radius></sphere>
        </geometry>
      </collision>
    </link>
    <pose>-1.3 -0.5 0.2 0 0 0</pose>
  </model>
</sdf>
EOF
```

## Running

### Terminal 1 — Launch Gazebo
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal 2 — Spawn green sphere
```bash
ros2 run gazebo_ros spawn_entity.py -file ~/green_sphere.sdf -entity green_sphere
```

### Terminal 3 — Run color follower node
```bash
cd ~/peppermint_ws
source install/setup.bash
ros2 run color_follower follower_node
```

## How it works

### Color Detection
- Converts camera feed to HSV colorspace
- Applies HSV threshold `[59,240,95]` to `[61,255,110]` to isolate green
- Applies ROI mask — center 50% width, lower 70% height to filter out 
  background green cubes
- Picks lowest green blob in frame (sphere sits on ground plane)

### P Controller
- Computes horizontal error between sphere center and image center
- `angular_z = Kp * error` where `Kp = 0.002`
- Rotates until `|error| < 30px` then moves forward at `0.15 m/s`

### Obstacle Detection
- Subscribes to `/scan` (LiDAR)
- Monitors front 60° arc (indices 0-30 and 330-360)
- Stops when closest obstacle within `0.30m`

### Recovery Behavior
- If no green detected in ROI → spins slowly at `0.08 rad/s` to search
- Continues until sphere re-enters camera frame

## Node Architecture
