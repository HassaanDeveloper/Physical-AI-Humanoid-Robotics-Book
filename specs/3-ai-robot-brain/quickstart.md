# Quickstart: AI-Robot Brain (NVIDIA Isaac™) Development Environment

## Prerequisites

### Hardware Requirements
- NVIDIA GPU with Compute Capability 6.0+ (GTX 1060 or better)
- 16GB+ RAM recommended
- 50GB+ free disk space for Isaac Sim installation

### Software Requirements
- Ubuntu 20.04/22.04 LTS or Windows 10/11 with WSL2
- ROS 2 Humble Hawksbill
- Docker and Docker Compose
- NVIDIA GPU drivers (470+)
- NVIDIA Container Toolkit

## Environment Setup

### 1. Install ROS 2 Humble
```bash
# Ubuntu
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros-latest.list
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-buildifier
sudo rosdep init
rosdep update
source /opt/ros/humble/setup.bash
```

### 2. Install NVIDIA Isaac Software
```bash
# Install Isaac Sim (requires NVIDIA Developer Account)
# Download from https://developer.nvidia.com/isaac-sim
# Follow installation guide for your platform

# Install Isaac ROS packages
sudo apt install ros-humble-isaac-ros-dev
sudo apt install ros-humble-isaac-ros-common
sudo apt install ros-humble-isaac-ros-gems
```

### 3. Install Nav2
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-nav2-gui-tools
```

### 4. Verify Installation
```bash
# Check ROS 2
source /opt/ros/humble/setup.bash
ros2 topic list

# Check Isaac ROS
ros2 pkg list | grep isaac_ros

# Check Nav2
ros2 pkg list | grep nav2
```

## Development Workflow

### 1. Clone the Repository
```bash
git clone <repository-url>
cd book_frontend
```

### 2. Install Docusaurus Dependencies
```bash
cd book_frontend
npm install
```

### 3. Create Isaac Workspace
```bash
mkdir -p ~/isaac_ws/src
cd ~/isaac_ws
colcon build
source install/setup.bash
```

### 4. Start Development Server
```bash
cd book_frontend
npm start
```

## Running Examples

### Isaac Sim Example
```bash
# Launch Isaac Sim with a sample scene
cd ~/isaac_ws
source install/setup.bash
ros2 launch isaac_ros_apriltag_ros isaac_ros_apriltag.launch.py
```

### Isaac ROS Perception Example
```bash
# Launch Isaac ROS perception pipeline
cd ~/isaac_ws
source install/setup.bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
```

### Nav2 Navigation Example
```bash
# Launch Nav2 with a sample robot
cd ~/isaac_ws
source install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

## Troubleshooting

### Common Issues
- **Isaac Sim won't start**: Verify GPU drivers and CUDA installation
- **Isaac ROS nodes not launching**: Check Isaac extensions are enabled in Isaac Sim
- **Nav2 planning fails**: Verify costmap parameters and map availability
- **Performance issues**: Reduce simulation complexity or increase hardware resources

### Performance Tips
- Use RTX rendering in Isaac Sim for best performance
- Limit simulation real-time factor to avoid overload
- Close unnecessary applications during intensive simulations
- Use appropriate physics parameters for stable simulation

## Next Steps
1. Complete the Isaac fundamentals tutorial
2. Practice with synthetic data generation
3. Implement perception pipelines
4. Configure navigation for humanoid robots