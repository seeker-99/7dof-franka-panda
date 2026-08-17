# 🤖 Franka Panda Color Sorting Robot

A **ROS 2-based intelligent color sorting system** powered by the **Franka Emika Panda robotic arm**. This project seamlessly integrates **OpenCV computer vision**, **MoveIt 2 motion planning**, and **Gazebo simulation** to create an autonomous pick-and-place system that detects and sorts colored objects with precision.

---

## 🎥 Pick and Place Demo  

https://github.com/user-attachments/assets/0813eb4e-310d-4b68-b538-3c8a857079a4

---

## ✨ Features

- 🎨 **Color Detection**: Real-time OpenCV-based vision system for Red, Green, and Blue objects
- 🦾 **Motion Planning**: Advanced trajectory planning using MoveIt 2
- 🎯 **Autonomous Operation**: Complete pick-and-place automation with gripper control
- 🔄 **Dynamic Target Selection**: Switch between colors without restarting the system
- 📊 **Visual Feedback**: Integrated RViz visualization for motion planning and execution
---

```bash
# Inside Docker container
source install/setup.bash
ros2 launch panda_bringup pick_and_place.launch.py
```

This launches:
- ✅ Gazebo simulation with Panda robot
- ✅ RViz motion planning visualization
- ✅ Camera and color detection node
- ✅ MoveIt 2 motion planning server
- ✅ Robot controllers

**Wait for all nodes to initialize** (you'll see "Ready to plan" messages)

---

### Terminal 2: Run Pick-and-Place Node

Open a new terminal and attach to the running container:

```bash
docker exec -it franka_panda_color_sorter bash
```

Inside the new terminal:

```bash
source install/setup.bash
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=R
```

**Color Options:**
- `target_color:=R` → Sort Red objects
- `target_color:=G` → Sort Green objects  
- `target_color:=B` → Sort Blue objects

🎯 **Watch the robot detect, pick, and place objects automatically!**

---

## 🔄 Step 5: Managing the Container

### Switch Target Color:

Stop the pick-and-place node (`Ctrl+C`) and rerun with a new color:

```bash
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=G
```

```

---

# 💻 Manual Installation on Local PC

If you prefer complete control or need to modify the source code, follow this comprehensive setup guide for a local installation.

## Prerequisites

- **Ubuntu 22.04 LTS** (recommended for ROS 2 Humble)
- **16GB+ RAM** (recommended for Gazebo simulation)
- **20GB+ free disk space**
- **Stable internet connection**

---

```


---

# 📂 Project Structure

```
panda_ws/
├── src/
│   ├── panda_description/        # Robot URDF, meshes, visuals
│   ├── panda_controller/         # Joint/gripper controllers
│   ├── panda_moveit/            # MoveIt 2 configuration
│   ├── panda_vision/            # OpenCV color detection
│   ├── panda_bringup/           # Launch files for full system
│   └── pymoveit2/               # Python MoveIt 2 interface
│       └── examples/
│           └── pick_and_place.py  # Main pick-and-place logic
├── build/                        # Build artifacts
├── install/                      # Installed packages
└── log/                         # Build and runtime logs
```

---

# 🧩 Package Overview

| Package | Description | Key Files |
|---------|-------------|-----------|
| **panda_description** | Robot model and visualization | `panda.urdf.xacro`, meshes |
| **panda_controller** | Controller configurations | `panda_controllers.yaml` |
| **panda_moveit** | Motion planning setup | MoveIt configs, SRDF |
| **panda_vision** | Computer vision system | `color_detector.py` |
| **panda_bringup** | System launcher | `pick_and_place.launch.py` |
| **pymoveit2** | High-level control | `pick_and_place.py` |

---

# 🚀 How It Works

## System Architecture

```
Camera Feed → Color Detection → Object Localization
                                        ↓
                                 Motion Planning (MoveIt 2)
                                        ↓
                               Trajectory Execution
                                        ↓
                         Pick Object → Move to Bin → Place
```

## Detailed Workflow

1. **Vision System** continuously monitors the workspace
2. **Color detector** identifies target color and computes 3D position
3. **PyMoveIt2** receives object coordinates
4. **MoveIt 2** plans collision-free trajectory
5. **Robot controller** executes motion
6. **Gripper** closes to grasp object
7. **Motion planner** computes path to sorting bin
8. **Robot** moves to designated area
9. **Gripper** opens to release object
10. **System** returns to home position and repeats
