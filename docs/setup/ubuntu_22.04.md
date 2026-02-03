# Simulation Setup for Ubuntu 22.04

**ArduPilot + Gazebo Classic + ROS 2 Humble**

This document explains how to set up a complete drone simulation environment on **Ubuntu 22.04** using **Gazebo Classic 11**, **ArduPilot SITL**, and **ROS 2 Humble**, including camera streaming and MAVLink-based control.

---

## Quick Start (TL;DR)

**Target OS:** Ubuntu 22.04 (Jammy)

**Simulation Stack**

- Gazebo Classic 11
- ArduPilot SITL
- ardupilot_gazebo
- ROS 2 Humble
- MAVProxy + MAVLink
- OpenCV + cv_bridge

**Run Order**

1. Launch Gazebo world
2. Start ArduPilot SITL
3. Verify ROS 2 camera topics

---

## Table of Contents

1. Overview of Used Software
2. Pre-installation Recommendations
3. System Requirements
4. Ubuntu 22.04 Installation
5. ROS 2 Humble Installation
6. Gazebo Classic 11 Installation
7. ardupilot_gazebo Plugin Build
8. ArduPilot SITL Build
9. MAVProxy Installation
10. Running the Simulation
11. Validation & Testing
12. Troubleshooting
13. Final Notes

---

## 1. Overview of Used Software

### Gazebo 11 (Classic)

- Simulates drone and world physics
- Provides GUI visualization
- Loads models and sensors

### ardupilot_gazebo

- Bridge between Gazebo and ArduPilot
- Provides Gazebo-compatible ArduPilot drone models

### gazebo_ros_camera

- Publishes ROS 2 camera topics:
  - /iris_camera/image_raw
  - /iris_camera/camera_info
- Camera orientation is defined in the model SDF file

### ROS 2 Humble

- Robot middleware and communication layer

### cv_bridge

- Converts ROS 2 image messages to OpenCV (NumPy) format

### OpenCV

- Image processing and computer vision

### ArduPilot SITL

- Software-in-the-loop flight controller
- No real hardware required
- Handles arming, flight modes, navigation
- Starts MAVProxy console and map

### MAVLink

- Communication protocol between FCU and ground station

### MAVProxy

- Simulated ground control station

### pymavlink

- Python MAVLink message interface

---

## 2. Pre-installation Recommendations

NOTE:
These steps prevent most common setup issues.

- Learn basic Linux terminal usage
- Install a code editor (VS Code, Vim, Neovim)
- Use a capable terminal emulator (Terminator or Konsole)
- English resources are limited; official docs and Turkish videos are helpful
- Learn ROS 2 basics beforehand

IMPORTANT:

- Read ArduPilot SITL documentation
- Inspect Gazebo world and model source files

---

## 3. System Requirements

Minimum:

- Ubuntu 22.04
- 8 GB RAM
- Integrated GPU

Recommended:

- 16 GB RAM
- Dedicated GPU
- SSD storage

WARNING:
Do NOT install on USB drives or virtual machines.

---

## 4. Ubuntu 22.04 Installation

Ubuntu 22.04 is recommended for Windows users.  
Kubuntu 22.04 is also suitable for KDE users.

External disk installation guide (Turkish):  
https://www.youtube.com/watch?v=j2RYqahtkNc

---

## 5. ROS 2 Humble Installation

### 5.1 System preparation

```bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
```

### 5.2 Add ROS 2 key

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo tee /etc/apt/keyrings/ros-archive-keyring.asc > /dev/null
```

### 5.3 Add ROS 2 repository

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.asc] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list
```

### 5.4 Install ROS 2

```bash
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool
```

### 5.5 Initialize rosdep

```bash
sudo rosdep init || true
rosdep update
```

### 5.6 Source ROS environment

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 6. Gazebo Classic 11 Installation

```bash
sudo add-apt-repository -y universe
sudo apt update
sudo apt install -y gazebo libgazebo-dev ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

### Verification

```bash
gazebo --version
```

Expected output: Gazebo 11.x

---

## 7. ardupilot_gazebo Plugin Build

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo.git ~/ardupilot_gazebo
cd ~/ardupilot_gazebo
git submodule update --init --recursive

mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

### Environment variables

```bash
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$HOME/ardupilot_gazebo/build' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/ardupilot_gazebo/models' >> ~/.bashrc
source ~/.bashrc
```

---

## 8. ArduPilot SITL Build

```bash
sudo apt install -y git python3 python3-pip python3-dev pkg-config wget flex bison g++ make cmake libncurses-dev libffi-dev libgmp-dev libmpc-dev libmpfr-dev libeigen3-dev libxml2-dev
```

```bash
python3 -m pip install --upgrade pip
```

```bash
git clone https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
git submodule update --init --recursive

Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

./waf configure --board sitl
./waf build -j$(nproc)
```

```bash
ls ~/ardupilot/build/sitl/bin/arducopter
```

---

## 9. MAVProxy Installation

```bash
python3 -m pip install --user --upgrade MAVProxy future pyserial pymavlink geographiclib python-dateutil PyQt5 matplotlib
```

```bash
echo 'export PATH=$PATH:$HOME/.local/bin' >> ~/.bashrc
source ~/.bashrc
```

```bash
mavproxy.py --version
```

---

## 10. Running the Simulation

### Start Gazebo

```bash
cd ~/ardupilot_gazebo/worlds
gazebo iris_arducopter_runway.world
```

### Start ArduPilot SITL

```bash
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

---

## 11. Validation & Testing

```bash
source /opt/ros/humble/setup.bash
ros2 topic list | grep iris_camera
```

Expected:

- /iris_camera/image_raw
- /iris_camera/camera_info

```bash
ros2 topic hz /iris_camera/image_raw
```

---

## 12. Troubleshooting

Camera topics not visible:

- Check model SDF for gazebo_ros_camera
- Verify GAZEBO_PLUGIN_PATH
- Restart Gazebo

SITL not connecting:

- Verify world file
- Check ports
- Re-source .bashrc

mavproxy.py not found:

- Ensure ~/.local/bin is in PATH
- Restart terminal

---

## 13. Final Notes

- ChatGPT is effective for debugging ArduPilot + Gazebo issues
- Arch-based systems are possible but complex
- Reading model and world source code saves days later

As we say in turkish, "Kolay Gelsin!"
