# Simulation Setup for Kubuntu 24.04 (Noble)

**ArduPilot SITL + Gazebo Harmonic (gz-sim) + ROS 2 Jazzy**

This document explains how to set up a **complete and stable** drone simulation environment on **Kubuntu 24.04**
using **Gazebo Harmonic (gz-sim)**, **ArduPilot SITL**, and **ROS 2 Jazzy**, including **camera streaming** and
**MAVLink-based control**.

It is **revised based on real setup issues** we actually hit (MAVProxy not found, missing Python deps, PEP 668,
map/console not opening, Gazebo ↔ SITL not linked, etc.).

---

## Quick Start (TL;DR)

**Target OS:** Kubuntu 24.04 (Noble)  
**Python:** 3.12 (PEP 668 “externally managed”)

**Simulation Stack**

- Gazebo Harmonic (gz-sim)
- ArduPilot SITL (ArduCopter)
- ardupilot_gazebo (Gazebo Harmonic plugin)
- ROS 2 Jazzy
- MAVProxy + MAVLink
- OpenCV + cv_bridge

**Run Order (Correct)**

1. Launch Gazebo world (make sure it is **not paused**)
2. Start ArduPilot SITL with **`-f gazebo-iris --model JSON`**
3. Bridge Gazebo camera topic to ROS 2
4. Visualize camera stream (image_tools / rqt)

---

## Table of Contents

1. Overview of Used Software
2. Pre-installation Recommendations
3. System Requirements
4. Kubuntu 24.04 Preparation
5. ROS 2 Jazzy Installation
6. Gazebo Harmonic Installation
7. ardupilot_gazebo Plugin Build
8. ArduPilot SITL Build
9. MAVProxy Installation (PEP 668 Safe)
10. Running the Simulation
11. Camera Streaming (Gazebo → ROS 2 → OpenCV)
12. Validation & Testing
13. Troubleshooting (Real Errors + Fixes)
14. Final Notes

---

## 1. Overview of Used Software

### Gazebo Harmonic (gz-sim)

- Simulates physics and sensors (camera, IMU)
- Runs SDF worlds/models
- Provides topics via Gazebo Transport

### ardupilot_gazebo

- Bridge between Gazebo Harmonic and ArduPilot SITL
- Provides Gazebo-compatible vehicle models
- Uses **JSON** model interface for lockstep simulation

### ROS 2 Jazzy

- Middleware for sensor and control pipelines
- Used to consume camera frames and run OpenCV

### ArduPilot SITL (ArduCopter)

- Software-in-the-loop flight controller
- Handles arming, flight modes, EKF, navigation
- Exposes MAVLink endpoints (TCP 5760 by default)

### MAVProxy + pymavlink

- MAVProxy: console + map, lightweight GCS for SITL
- pymavlink: Python MAVLink interface used by MAVProxy and custom scripts

### OpenCV + cv_bridge

- Real-time vision processing on ROS 2 Image messages

---

## 2. Pre-installation Recommendations

IMPORTANT:

- Use separate terminals (Konsole/Terminator) for Gazebo / SITL / ROS tools
- Read ArduPilot SITL basics (GUIDED needs TAKEOFF to lift)
- Expect EKF/GPS to settle for ~10–30 seconds after start

NOTE (Ubuntu 24.04 / PEP 668):

- `pip install --user ...` may be blocked by the OS
- Prefer **apt** for system libraries and **pipx** for Python CLI apps (MAVProxy)

---

## 3. System Requirements

Minimum:

- Kubuntu 24.04
- 8 GB RAM

Recommended:

- 16 GB RAM
- Dedicated GPU
- SSD

---

## 4. Kubuntu 24.04 Preparation

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y git curl wget build-essential
```

---

## 5. ROS 2 Jazzy Installation

```bash
sudo apt install -y ros-jazzy-desktop
```

Auto-source ROS:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

ROS packages for Gazebo bridge + vision:

```bash
sudo apt install -y   ros-jazzy-ros-gz-bridge   ros-jazzy-ros-gz-image   ros-jazzy-image-tools   ros-jazzy-rqt-image-view   ros-jazzy-cv-bridge   python3-opencv
```

---

## 6. Gazebo Harmonic Installation

```bash
sudo apt install -y gz-harmonic
```

Verify:

```bash
gz sim --versions
```

---

## 7. ardupilot_gazebo Plugin Build

```bash
mkdir -p ~/gz_ws/src
cd ~/gz_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ~/gz_ws
mkdir -p build
cd build
cmake ../src/ardupilot_gazebo
make -j$(nproc)
```

### Environment variables (CRITICAL)

```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/build:$GZ_SIM_SYSTEM_PLUGIN_PATH' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
source ~/.bashrc
```

---

## 8. ArduPilot SITL Build

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

Install ArduPilot prereqs:

```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

Build Copter SITL:

```bash
./waf configure --board sitl
./waf copter
```

Verify binary:

```bash
ls ~/ardupilot/build/sitl/bin/arducopter
```

---

## 9. MAVProxy Installation (PEP 668 Safe)

### Why pipx?

On Ubuntu 24.04, Python is **externally managed** (PEP 668).  
This often blocks `pip install --user ...`. MAVProxy is a Python CLI app, so pipx is ideal.

### Install pipx

```bash
sudo apt install -y pipx
pipx ensurepath
source ~/.profile
```

### Install MAVProxy (upstream) + required deps

```bash
pipx install "git+https://github.com/ArduPilot/MAVProxy.git"
pipx inject MAVProxy future pymavlink
```

> Real issue we hit: `ModuleNotFoundError: No module named 'future'`  
> Fix is included above (inject `future`).

### GUI deps for map/console

```bash
sudo apt install -y python3-wxgtk4.0 python3-matplotlib python3-tk
```

Verify:

```bash
mavproxy.py --version
```

---

## 10. Running the Simulation

### Terminal A – Start Gazebo world

```bash
gz sim -v4 -r iris_runway.sdf
```

IMPORTANT:

- Ensure simulation is **not paused** (▶ button / Space key)
- SDF warnings like `gz_frame_id not defined in SDF` are typically **harmless**

### Terminal B – Start ArduPilot SITL (Gazebo-linked)

```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py   -v ArduCopter   -f gazebo-iris   --model JSON   --console --map
```

CRITICAL CHECK (Gazebo link):

```bash
ps aux | grep arducopter | grep -- --model
```

Expected:

- `--model JSON` ✅ Gazebo-linked
  Not expected:
- `--model +` ❌ SITL internal model only (Gazebo won’t move)

---

## 11. Camera Streaming (Gazebo → ROS 2 → OpenCV)

### 11.1 Find Gazebo camera topics

```bash
gz topic -l | grep -i -E "camera|image"
```

Example topics we saw:

- `/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image`
- `/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/camera_info`

### 11.2 Bridge the image to ROS 2

```bash
ros2 run ros_gz_bridge parameter_bridge /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image@sensor_msgs/msg/Image@gz.msgs.Image
```

(Optional) Bridge camera_info if needed:

```bash
ros2 run ros_gz_bridge parameter_bridge /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

If you ever get a “type mismatch”, inspect types:

```bash
gz topic -i -t /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
```

### 11.3 Visualize camera stream

**Most stable (recommended):**

```bash
ros2 run image_tools showimage --ros-args   -r image:=/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
```

**rqt_image_view (Jazzy-safe invocation):**

```bash
ros2 run rqt_image_view rqt_image_view
```

NOTE:

- Running `rqt_image_view` directly may fail due to PATH; use `ros2 run ...`.

Optional (only if stream is disabled):

- If your model uses an enable topic, you may need:
  ```bash
  gz topic -t /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming     -m gz.msgs.Boolean -p "data: true"
  ```

---

## 12. Validation & Testing

### 12.1 MAVProxy flight test (GUIDED requires TAKEOFF)

In MAVProxy:

```text
mode guided
arm throttle
takeoff 5
```

If you only set a guided target without TAKEOFF, the drone may **not lift** (normal behavior).

### 12.2 Check altitude / state

```text
alt
status
```

---

## 13. Troubleshooting (Real Errors + Fixes)

### A) `mavproxy.py: No such file or directory`

Cause: MAVProxy not in PATH / not installed  
Fix:

```bash
pipx ensurepath
source ~/.profile
pipx install "git+https://github.com/ArduPilot/MAVProxy.git"
```

### B) `ModuleNotFoundError: No module named 'future'`

Fix:

```bash
pipx inject MAVProxy future
```

### C) `Failed to load module: No module named 'map' / 'console'`

Cause: GUI dependencies missing  
Fix:

```bash
sudo apt install -y python3-wxgtk4.0 python3-matplotlib python3-tk
```

### D) “Takeoff started” but Gazebo drone does not move

Cause: SITL not linked to Gazebo (using `--model +`)  
Fix: start SITL with:

```bash
../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map
```

### E) “Waiting for connection”

Common causes:

- Gazebo not started first
- MAVProxy crashed (missing deps)
- Port conflicts (old processes still running)

Quick cleanup:

```bash
pkill -f arducopter || true
pkill -f mavproxy || true
pkill -f sim_vehicle || true
pkill -f "gz sim" || true
```

### F) PreArm warnings (EKF attitude bad / accels inconsistent)

Often normal right after boot. Wait ~10–30 seconds.  
If it persists, verify Gazebo is running (not paused) and time is advancing.

---

## 14. Final Notes

This setup has been verified end-to-end on Kubuntu 24.04:

Gazebo ✔ ArduPilot SITL ✔ MAVProxy ✔ ROS 2 ✔ Camera ✔ OpenCV ✔

You are now ready to implement:

- Vision-based gate detection
- Autonomous waypoint + vision correction
- Full competition track completion

As we say in Turkish: **Kolay gelsin!**
