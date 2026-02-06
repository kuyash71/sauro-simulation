# Python venv Setup (Ubuntu 24.04 + ROS 2 Jazzy)

For this project, the compatible model is:

- Install ROS/OpenCV dependencies via `apt`
- Create a standard Python virtual environment under `~/venvs/<venv_name>`
- Use `--system-site-packages` so ROS Python modules are visible inside the venv

This is required for imports such as `rclpy`, `sensor_msgs`, `cv_bridge`, and `cv2`.

## 1) Install system packages (one-time)

```bash
sudo apt update
sudo apt install -y \
  python3-venv \
  ros-jazzy-rclpy \
  ros-jazzy-sensor-msgs \
  ros-jazzy-cv-bridge \
  python3-opencv
```

## 2) Create a standard venv in `~/venvs`

```bash
mkdir -p ~/venvs
python3 -m venv ~/venvs/<venv_name> --system-site-packages
```

Example:

```bash
python3 -m venv ~/venvs/sauro-iha --system-site-packages
```

## 3) Activate the environment (classic flow)

If you use `zsh`:

```bash
source /opt/ros/jazzy/setup.zsh
source ~/venvs/<venv_name>/bin/activate
```

If you use `bash`:

```bash
source /opt/ros/jazzy/setup.bash
source ~/venvs/<venv_name>/bin/activate
```

## 4) Verify imports

```bash
python -c "import rclpy; from rclpy.node import Node; from sensor_msgs.msg import Image; from cv_bridge import CvBridge; import cv2; print('OK')"
```

## 5) Run project OpenCV test

```bash
cd /home/kuyash/Project\ Repos/SAURO/sauro-simulation
python docs/tmp/opencvtest.py
```

With OpenCV windows:

```bash
python docs/tmp/opencvtest.py --display
```

## Notes

- Do not install `rclpy`, `sensor_msgs`, or `cv_bridge` via `pip` for Jazzy. Use `apt`.
- `MAVProxy` should stay outside this project venv and be managed with `pipx` as documented in `docs/setup/ubuntu_24.04.md`.
- In VS Code, select `~/venvs/<venv_name>/bin/python` as the interpreter.
