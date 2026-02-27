# ROS2 Workspace

**ROS2 distribution: Jazzy Jalisco** (LTS 2024–2029)

> **Why Docker?** ROS2 Jazzy officially supports Ubuntu 24.04 only — our RPi 5 runs Ubuntu 25.10,
> which has no pre-built ROS2 packages. Docker runs `ros:jazzy-ros-base` (Ubuntu 24.04 inside)
> regardless of the host OS. Docker is the standard workflow for all platforms.

---

## Packages

| Package | Status | Description |
|---------|--------|-------------|
| `nuevo_msgs` | ✅ Ready | Custom message definitions (voltage, motor status, servo status, IMU cal, etc.) |
| `nuevo_bridge` | ✅ Ready | Arduino ↔ ROS2 ↔ WebSocket bridge (`nuevo_ui/backend/`, mounted via Docker volume) |
| `robot_bringup` | Planned | Launch files and system configuration |
| `robot_control` | Planned | Velocity control, trajectory planning, pure pursuit |
| `sensor_drivers` | Planned | Pi camera (YOLO/OpenCV), GPS receiver |
| `robot_description` | Planned | URDF models and visualization |

---

## 0. Build the frontend (one-time)

The WebSocket UI must be compiled to static files before the bridge can serve it.
Skip if you only need ROS2 topics without the web UI.

```bash
cd nuevo_ui/frontend
npm install        # first time only
npm run build
cd ..
cp -r frontend/dist/. backend/static/
```

See [`nuevo_ui/README.md`](../nuevo_ui/README.md) for full frontend development instructions.

---

## 1. Install Docker (one-time per machine)

### Raspberry Pi 5 (Ubuntu 25.10)
```bash
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER
newgrp docker
```

### macOS
1. Install [Docker Desktop for Mac](https://docs.docker.com/desktop/install/mac-install/)
2. Open **Docker Desktop** and wait for the whale icon in the menu bar to stop animating

### Windows
1. Install [Docker Desktop for Windows](https://docs.docker.com/desktop/install/windows-install/) with the **WSL 2** backend
2. Open **Docker Desktop** and wait for the whale icon in the taskbar to stop animating

---

## 2. Set your compose file

Two compose files are provided — pick the one for your platform:

| Platform | Compose file | Mode |
|----------|-------------|------|
| **Raspberry Pi** | `ros2_ws/docker/docker-compose.rpi.yml` | Real Arduino on `/dev/ttyAMA0` |
| **macOS / Windows** | `ros2_ws/docker/docker-compose.vm.yml` | Mock (simulated data, no Arduino) |

Set a shell variable so you can copy-paste all commands below without changes:

### Raspberry Pi (real target)
```bash
COMPOSE=ros2_ws/docker/docker-compose.rpi.yml
```

### macOS (VM)
```bash
COMPOSE=ros2_ws/docker/docker-compose.vm.yml
```

### Windows
> **Windows PowerShell:** use `$env:COMPOSE = "ros2_ws/docker/docker-compose.vm.yml"` instead.

---

## 3. Build the Docker image (one-time, or after pip dependency changes)

```bash
# From the repository root (MAE_162_2026/)
docker compose -f $COMPOSE build
```

---

## Option A — Manual shell (recommended for learning)

Starts a bare shell inside the container with nothing running. Build the ROS2
packages and launch the bridge yourself — good for understanding each step.

```bash
docker compose -f $COMPOSE run --rm --entrypoint bash robot
```

Inside the container:

```bash
# 1. Source the ROS2 base install
source /opt/ros/jazzy/setup.bash

# 2. Build the packages (nuevo_msgs + nuevo_bridge)
colcon build --packages-select nuevo_msgs nuevo_bridge --symlink-install --cmake-args -DBUILD_TESTING=OFF

# 3. Source the built packages
source install/setup.bash

# 4. Verify packages are visible
ros2 pkg list | grep nuevo
ros2 interface show nuevo_msgs/msg/Voltage

# 5. Start the bridge
uvicorn nuevo_bridge.app:app --host 0.0.0.0 --port 8000
```

Press `Ctrl+C` to stop. Type `exit` or press `Ctrl+D` to leave (container is removed automatically).

The WebSocket UI is accessible at `http://localhost:8000`.

> **macOS / Windows — ROS2 topics:** The container uses bridge networking, so `ros2 topic list`
> from a Mac/Windows terminal will not see container topics. Use the shell inside the container.

---

## Option B — Automated

The entrypoint handles the colcon build automatically on first start.

**Terminal 1 — start the stack**
```bash
docker compose -f $COMPOSE up
```

**Terminal 2 — open a shell into the running container**
```bash
docker compose -f $COMPOSE exec robot bash
# then:
ros2 topic list
ros2 topic echo /nuevo/voltage
```

The WebSocket UI is accessible at `http://localhost:8000`.

---

## Common Docker commands

```bash
# Stop the stack
docker compose -f $COMPOSE down

# Restart after Python source or .msg changes
docker compose -f $COMPOSE restart

# Rebuild image after pip dependency changes
docker compose -f $COMPOSE build

# Force a clean colcon build (wipes build/install volumes)
docker compose -f $COMPOSE down -v
docker compose -f $COMPOSE up

# View live logs
docker compose -f $COMPOSE logs -f
```

### Useful commands inside the container

```bash
ros2 pkg list | grep nuevo           # confirm packages are built
ros2 interface show nuevo_msgs/msg/Voltage # inspect a message type
ros2 topic list                      # list active topics (bridge must be running)
ros2 topic echo /odom
ros2 topic hz /nuevo/dc_status       # check publish rate

# Send commands (bridge must be running)
ros2 topic pub --once /nuevo/sys_cmd nuevo_msgs/msg/SysCommand "{command: 2}"
ros2 topic pub --once /nuevo/cmd_vel nuevo_msgs/msg/MotorVelocities \
    "{velocity_ticks_per_sec: [500, 500, 500, 500], mode: [2, 2, 2, 2]}"
```

---

## 4. Ubuntu 24.04 — native install (optional, dev machines only)

> Only applicable to machines running Ubuntu 24.04 (Noble). **Not for the RPi 5 (Ubuntu 25.10).**

### Install ROS2 Jazzy

```bash
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu noble main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update && sudo apt install -y \
    ros-jazzy-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip

sudo rosdep init && rosdep update
```

### Build and run

```bash
cd nuevo_ui/backend && pip3 install -e . && cd ../..

cd ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

export PYTHONPATH="$(pwd)/../nuevo_ui/backend:$PYTHONPATH"
NUEVO_ROS2=1 uvicorn nuevo_bridge.app:app --host 0.0.0.0 --port 8000
```

### Clean build

```bash
rm -rf build/ install/ log/ && colcon build --symlink-install
```

---

## Raspberry Pi 5 GPIO Reference

| RPi GPIO | Function | Connected to |
|----------|----------|--------------|
| GPIO2 (SDA1) | I2C Data | 2× Qwiic connectors (RPi bus) |
| GPIO3 (SCL1) | I2C Clock | 2× Qwiic connectors (RPi bus) |
| GPIO14 (TXD0) | UART TX → Arduino | Via 5V↔3.3V level shifter |
| GPIO15 (RXD0) | UART RX ← Arduino | Via 5V↔3.3V level shifter |
| Other | GPIO breakout | Unused pins to screw terminals |

UART baud rate: **1,000,000 bps**

---

## ROS2 Topic Reference

| Topic | Type | Direction | Rate |
|-------|------|-----------|------|
| `/odom` | `nav_msgs/Odometry` | Bridge → nodes | 100 Hz |
| `/imu/data` | `sensor_msgs/Imu` | Bridge → nodes | 100 Hz |
| `/imu/mag` | `sensor_msgs/MagneticField` | Bridge → nodes | 100 Hz |
| `/nuevo/voltage` | `nuevo_msgs/Voltage` | Bridge → nodes | 10 Hz |
| `/nuevo/sys_status` | `nuevo_msgs/SystemStatus` | Bridge → nodes | 1–10 Hz |
| `/nuevo/dc_status` | `nuevo_msgs/DCStatusAll` | Bridge → nodes | 100 Hz |
| `/nuevo/step_status` | `nuevo_msgs/StepStatusAll` | Bridge → nodes | 100 Hz |
| `/nuevo/servo_status` | `nuevo_msgs/ServoStatusAll` | Bridge → nodes | 50 Hz |
| `/nuevo/io_status` | `nuevo_msgs/IOStatus` | Bridge → nodes | 100 Hz |
| `/nuevo/mag_cal` | `nuevo_msgs/MagCalStatus` | Bridge → nodes | 10 Hz |
| `/nuevo/cmd_vel` | `nuevo_msgs/MotorVelocities` | Nodes → bridge | control loop rate |
| `/nuevo/step_cmd` | `nuevo_msgs/StepCommand` | Nodes → bridge | on demand |
| `/nuevo/servo_cmd` | `nuevo_msgs/ServoCommand` | Nodes → bridge | on demand |
| `/nuevo/sys_cmd` | `nuevo_msgs/SysCommand` | Nodes → bridge | on demand |
| `/nuevo/mag_cal_cmd` | `nuevo_msgs/MagCalCmd` | Nodes → bridge | on demand |
