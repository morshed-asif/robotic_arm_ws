# 🦾 6-DOF Robotic Arm — MoveIt 2 Full Stack

A complete ROS 2 + MoveIt 2 implementation of a 6-axis robotic arm built from scratch, including URDF modelling, motion planning, C++ and Python commander APIs, a parallel gripper, and a fully dockerised environment for instant deployment.

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt%202-Humble-orange)](https://moveit.picknik.ai/)
[![Docker](https://img.shields.io/badge/Docker-morshedasif%2Frobotic__arm__moveit2-2496ED?logo=docker&logoColor=white)](https://hub.docker.com/repository/docker/morshedasif/robotic_arm_moveit2/general)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)


## ✨ Features

- **6-DOF robotic arm** fully modelled in URDF/XACRO with visual and collision geometry
- **Parallel gripper** with mimic joint — single motor controls both fingers
- **MoveIt 2** — OMPL planning, KDL inverse kinematics, self-collision matrix
- **Named, joint-space, pose, and Cartesian path** goal types
- **C++ Commander node** — full MoveIt 2 C++ API with subscribers for programmatic control
- **Custom ROS 2 interfaces** — `PoseCommand.msg` for Cartesian commands over a topic
- **ROS 2 Control** — `joint_trajectory_controller` and `gripper_controller` wired up and verified
- **Fully Dockerised** — pull and run with no manual dependency setup

---

## 📦 Package Structure

```
robotic_arm_ws/
├── my_robot_description/      # URDF/XACRO, launch, RViz config
├── my_robot_moveit_config/    # MoveIt 2 config (SRDF, kinematics, controllers)
├── my_robot_bringup/          # Master launch file + ros2_controllers.yaml
├── my_robot_commander_cpp/    # C++ API commander node
├── my_robot_interfaces/       # Custom ROS 2 message definitions
└── Tf Tree.pdf                # Full kinematic TF tree reference
```

### Kinematic Chain

```
world → base_link → shoulder_link → arm_link → elbow_link
                                              → forearm_link → wrist_link → hand_link → tool_link
                                                                                       → gripper_base_link
                                                                                           ├── gripper_left_finger_link
                                                                                           └── gripper_right_finger_link
```

---

## 🐳 Docker — Quickest Way to Run

The entire stack is pre-built and published on Docker Hub.

**Pull the image**

```bash
docker pull morshedasif/robotic_arm_moveit2:latest
```

Docker Hub: [morshedasif/robotic_arm_moveit2](https://hub.docker.com/repository/docker/morshedasif/robotic_arm_moveit2/general)

**Allow GUI forwarding (for RViz)**

```bash
xhost +local:docker
```

**Run the container**

```bash
docker run -it \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --network host \
  morshedasif/robotic_arm_moveit2:latest
```

**Inside the container — launch the full stack**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot_bringup my_robot.launch.xml
```

---

## 🛠️ Manual Installation (Native Ubuntu 22.04)

### System Requirements

| Requirement | Version |
|-------------|---------|
| OS | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| MoveIt 2 | `ros-humble-moveit` |
| DDS | CycloneDDS (recommended) |
| Build tool | colcon |

### Step 1 — Install ROS 2 Humble

Follow the official guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Then source it:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2 — Switch DDS to CycloneDDS

MoveIt 2 works more reliably with CycloneDDS than the default FastDDS.

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

### Step 3 — Install MoveIt 2

```bash
sudo apt install ros-humble-moveit
```

### Step 4 — Install Python MoveIt API (optional — for Python commander)

```bash
echo "deb [trusted=yes] https://raw.githubusercontent.com/moveit/moveit2_packages/jammy-humble/ ./" | \
  sudo tee /etc/apt/sources.list.d/moveit_moveit2_packages.list
sudo apt update
sudo apt install ros-humble-moveit-py
```

### Step 5 — Install additional dependencies

```bash
sudo apt install \
  ros-humble-urdf-tutorial \
  ros-humble-joint-state-publisher-gui \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  python3-colcon-common-extensions
```

### Step 6 — Clone and build the workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/morshed-asif/robotic_arm_ws.git .

cd ~/ros2_ws
colcon build
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

> **Note:** If you see `max_velocity has invalid type: expected double got integer`, the `joint_limits.yaml` needs float values — all values in this repo are already fixed.

---

## 🚀 Running the Stack

### Launch everything (simulation with mock hardware)

```bash
ros2 launch my_robot_bringup my_robot.launch.xml
```

This starts — in order:

1. `robot_state_publisher` — publishes TF from URDF + joint states
2. `ros2_control_node` — loads mock hardware interface
3. Controller spawners — `joint_state_broadcaster`, `arm_controller`, `gripper_controller`
4. `move_group` — MoveIt 2 planning server
5. `rviz2` — visualization and motion planning UI

---

## 🎮 Commander Nodes

### C++ Commander

```bash
# Terminal 1
ros2 launch my_robot_bringup my_robot.launch.xml

# Terminal 2
ros2 run my_robot_commander_cpp commander
```

## 📡 Control Topics

| Topic | Message Type | Effect |
|-------|-------------|--------|
| `/open_gripper` | `example_interfaces/Bool` | `true` = open, `false` = close |
| `/joint_command` | `example_interfaces/Float64MultiArray` | Move arm to joint positions (6 values) |
| `/pose_command` | `my_robot_interfaces/PoseCommand` | Move arm to Cartesian pose |

### Example commands

**Close the gripper**
```bash
ros2 topic pub --once /open_gripper example_interfaces/msg/Bool "data: false"
```

**Open the gripper**
```bash
ros2 topic pub --once /open_gripper example_interfaces/msg/Bool "data: true"
```

**Joint-space command**
```bash
ros2 topic pub --once /joint_command example_interfaces/msg/Float64MultiArray \
  "data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]"
```

**Cartesian pose command**
```bash
ros2 topic pub --once /pose_command my_robot_interfaces/msg/PoseCommand \
  "{x: 0.7, y: 0.0, z: 0.4, roll: 3.14, pitch: 0.0, yaw: 0.0, cartesian_path: false}"
```

**Cartesian path (straight-line motion)**
```bash
ros2 topic pub --once /pose_command my_robot_interfaces/msg/PoseCommand \
  "{x: 0.7, y: 0.0, z: 0.2, roll: 3.14, pitch: 0.0, yaw: 0.0, cartesian_path: true}"
```

---

## 🔄 Pick-and-Place Sequence (RViz UI)

Run these steps in the **MotionPlanning** panel in RViz:

1. Planning Group: `arm` → Goal: `pose_one` → **Plan & Execute**
2. Planning Group: `gripper` → Goal: `gripper_open` → **Plan & Execute**
3. Planning Group: `gripper` → Goal: `gripper_half_closed` → **Plan & Execute** *(grip object)*
4. Planning Group: `arm` → Goal: `pose_two` → **Plan & Execute** *(move to drop)*
5. Planning Group: `gripper` → Goal: `gripper_open` → **Plan & Execute** *(release)*

---

## 📐 Robot Specifications

### Arm Links

| Link | Geometry | Size |
|------|----------|------|
| base_link | box | 0.4 × 0.4 × 0.1 m |
| shoulder_link | cylinder | r=0.1, l=0.5 m |
| arm_link | cylinder | r=0.05, l=0.6 m |
| elbow_link | cylinder | r=0.05, l=0.1 m |
| forearm_link | cylinder | r=0.05, l=0.5 m |
| wrist_link | box | 0.1 × 0.1 × 0.05 m |
| hand_link | box | 0.1 × 0.1 × 0.02 m |
| tool_link | — | virtual mount point |

### Arm Joints

| Joint | Type | Axis | Limits |
|-------|------|------|--------|
| joint_1 | revolute | Z | ±3.14 rad |
| joint_2 | revolute | Y | 0 → 2.5 rad |
| joint_3 | revolute | Y | 0 → 2.5 rad |
| joint_4 | revolute | Z | ±3.14 rad |
| joint_5 | revolute | Y | ±1.57 rad |
| joint_6 | continuous | Z | unlimited |

### Gripper Joints

| Joint | Type | Axis | Limits |
|-------|------|------|--------|
| gripper_left_finger_joint | prismatic | X | 0 → 0.06 m |
| gripper_right_finger_joint | prismatic (mimic) | X | -0.06 → 0 m |

---

## 🐛 Known Issues & Fixes

**`max_velocity has invalid type: expected double got integer`**
Open `my_robot_moveit_config/config/joint_limits.yaml` and ensure all limit values are floats (e.g. `1.0` not `1`). Already fixed in this repo.

**`TypeError: Expected 'value' to be one of [...], but got '()' of type tuple`**
The MoveIt Setup Assistant leaves joints lists empty (`[]`) in controller YAML files. Fill them with the explicit joint names. Already fixed in this repo.

**`Action client not connected to action server`**
Controller name mismatch between `ros2_controllers.yaml` and `moveit_controllers.yaml`. Both must use the same name (`arm_controller`). Already fixed in this repo.

**`no ros2_control tag` crash**
The controller manager needs the `robot_description` parameter passed explicitly — it is not auto-discovered. Already handled in the bringup launch file.

---

## 🗺️ Roadmap / What to Learn Next

- [ ] Add collision objects to the planning scene
- [ ] Integrate a depth camera for perception pipeline
- [ ] Port to a real Dynamixel-based arm using a custom hardware interface plugin
- [ ] Add a grasping pipeline (MoveIt Task Constructor)
- [ ] 7-axis arm for improved Cartesian path coverage

---

## 🤝 Contributing

Pull requests are welcome. For major changes, open an issue first to discuss what you would like to change.

---
