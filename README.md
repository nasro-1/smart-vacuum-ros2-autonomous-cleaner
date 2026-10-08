# SmartVacuum — Autonomous Intelligent Robotic Vacuum Cleaner

<p align="center">
  <strong>ROS 2 Jazzy · Gazebo Harmonic · TurtleBot3 Burger · SLAM · YOLOv8 · Computer Vision · Autonomous Coverage</strong>
</p>

<p align="center">
  A simulation-based autonomous vacuum robot that maps an indoor environment,
  understands floor conditions, detects dirt/trash, avoids obstacles and performs systematic coverage.
</p>

---

## Overview

**SmartVacuum** is an autonomous robotic vacuum simulation built with **ROS 2 Jazzy** and **Gazebo Harmonic (Gz Sim)**.

The supplied project implements a perception–mapping–decision–action pipeline around a **TurtleBot3 Burger**. It combines:

- online SLAM with `slam_toolbox`;
- HSV-based semantic floor classification;
- YOLOv8-based dirt/trash detection;
- LiDAR and odometry for navigation;
- systematic coverage planning in `coverage_explorer`;
- Gazebo house/world models;
- ROS 2 launch orchestration;
- an optional YOLO training workflow.

> **Important implementation status:** the supplied source explicitly shows that `coverage_explorer` replaced the earlier `autonomous_explorer` in the active launch sequence. `autonomous_explorer.py` is retained as a legacy/reference implementation. The supplied source archive does **not** contain a deployed Diffusion Policy controller; any Diffusion Policy discussion should therefore be treated as a research direction, not as the active controller.

---

## System Architecture

```text
                         Gazebo Harmonic
                       Indoor House World
                              │
                ┌─────────────┴─────────────┐
                │                           │
          RGB Camera                      LiDAR
                │                           │
       ┌────────┴────────┐                  │
       │                 │                  │
       ▼                 ▼                  ▼
 semantic_detector   dirt_detector_node   SLAM Toolbox
       │                 │                  │
       │                 │                  ├── /map
       │                 │                  │
       └────────────┬────┴──────────────────┘
                    │
                    ▼
             coverage_explorer
          Coverage + Navigation FSM
                    │
                    ▼
                 /cmd_vel
                    │
                    ▼
             TurtleBot3 Burger
```

---

## Core Components

### `semantic_detector`

Processes the camera stream:

```text
/camera/image_raw
        │
        ▼
  OpenCV / HSV
        │
        ├── water
        ├── dirt
        ├── carpet
        └── clean
        │
        ├── /semantic_class
        ├── /semantic_area
        └── /semantic_debug
```

### `dirt_detector_node`

Runs YOLOv8 on the camera stream and publishes target information:

```text
/camera/image_raw
        │
        ▼
      YOLOv8
        │
        ├── /dirt_target
        └── /camera/dirt_debug
```

### `coverage_explorer`

This is the **active navigation controller** registered by the ROS 2 package.

It receives LiDAR, odometry, semantic information and dirt targets, then publishes:

```text
/cmd_vel
```

The launch file starts it after SLAM and perception initialization.

### `autonomous_explorer`

The repository also contains an older reactive exploration implementation.

It is **not the active console entry point** in the supplied `setup.py`; the active entry point is:

```text
coverage_explorer = dirt_cleaner.coverage_explorer:main
```

---

## ROS 2 Topic Interface

| Topic | Direction | Purpose |
|---|---|---|
| `/scan` | input | LiDAR |
| `/odom` | input | Robot odometry |
| `/camera/image_raw` | input | RGB camera |
| `/dirt_target` | input/output | Dirt/trash target information |
| `/semantic_class` | input | Semantic floor class |
| `/semantic_area` | input | Semantic region information |
| `/semantic_debug` | output | Semantic perception visualization |
| `/camera/dirt_debug` | output | YOLO detection visualization |
| `/cmd_vel` | output | Robot velocity commands |
| `/map` | output | SLAM occupancy grid |

---

## Project Structure

```text
SmartVacuum/
├── README.md
├── SETUP.txt
├── YOLO_DIRT_WORKFLOW.md
├── requirements_yolo.txt
├── .gitignore
│
├── config/
│   ├── cartographer.lua
│   ├── headless_sim.launch.py
│   └── slam_params.yaml
│
├── datasets/
│   └── dirt_detection/
│       └── data.yaml.example
│
├── nodes/
│   ├── autonomous_explorer.py
│   └── dirt_detector_node.py
│
├── scripts/
│   ├── capture_yolo_images.sh
│   ├── run_vacuum.sh
│   ├── start_cartographer.sh
│   └── start_slam.sh
│
├── src/
│   └── dirt_cleaner/
│       ├── dirt_cleaner/
│       │   ├── __init__.py
│       │   ├── autonomous_explorer.py
│       │   ├── coverage_explorer.py
│       │   ├── dirt_detector_node.py
│       │   └── semantic_detector.py
│       ├── launch/
│       │   └── smart_vacuum.launch.py
│       ├── models/
│       │   └── dirt_patch/
│       ├── resource/
│       ├── test/
│       ├── worlds/
│       ├── package.xml
│       ├── setup.cfg
│       └── setup.py
│
├── training/
│   └── train_dirt_yolo.py
│
└── worlds/
    ├── my_house.sdf
    └── turtlebot3_world.world
```

> The supplied archive contains both the ROS 2 package implementation under `src/dirt_cleaner/` and some root-level helper/legacy node files under `nodes/`. They are preserved rather than deleted so the original project remains reproducible.

---

# Installation

## Requirements

The supplied project targets:

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3.12
- Git
- TurtleBot3 Jazzy packages

---

## 1. Install TurtleBot3 Dependencies

Create the external TurtleBot3 workspace:

```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src

git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git

cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 2. Install Python Dependencies

The supplied project defines:

```text
requirements_yolo.txt
```

with:

```text
ultralytics
opencv-python
```

The original setup guide additionally specifies the CPU PyTorch installation used by the project.

Follow the environment requirements appropriate for your machine rather than blindly mixing incompatible PyTorch builds.

---

## 3. Build SmartVacuum

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash

colcon build --symlink-install
source install/setup.bash
```

---

# Running the Simulation

Configure TurtleBot3:

```bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-01
```

Then launch:

```bash
ros2 launch dirt_cleaner smart_vacuum.launch.py
```

The supplied launch file starts:

1. Gazebo server;
2. Gazebo client;
3. TurtleBot3 state publisher;
4. TurtleBot3 spawn;
5. `slam_toolbox`;
6. `semantic_detector`;
7. `dirt_detector_node`;
8. `coverage_explorer`.

The launch file intentionally uses startup delays so that SLAM and perception resources are available before the coverage controller starts.

---

# SLAM

The active launch file starts:

```text
slam_toolbox
```

using its online asynchronous launch.

The purpose is to build a map while the robot operates rather than requiring a pre-existing map.

Configuration files are provided under:

```text
config/
```

including:

```text
slam_params.yaml
cartographer.lua
```

---

# YOLOv8 Dirt Detection

The project includes an optional custom YOLO training workflow.

## Dataset layout

```text
datasets/dirt_detection/
├── images/
│   ├── train/
│   └── val/
├── labels/
│   ├── train/
│   └── val/
└── data.yaml
```

Start from:

```bash
cp datasets/dirt_detection/data.yaml.example \
   datasets/dirt_detection/data.yaml
```

Then configure the dataset.

## Training

```bash
python3 training/train_dirt_yolo.py \
  --data datasets/dirt_detection/data.yaml \
  --epochs 60
```

Generated model artifacts should normally remain outside the Git repository unless there is a deliberate reason to version them.

The runtime launch expects weights at the documented project location:

```text
models/dirt_yolo/weights/best.pt
```

or an explicitly supplied launch parameter.

---

# Research / Diffusion Policy Status

The project materials may discuss modern learning-based policies, but the **uploaded source archive itself does not contain a Diffusion Policy implementation**.

Therefore this repository deliberately does **not** claim:

```text
"Diffusion Policy is the deployed robot controller."
```

The active implementation is:

```text
Camera + LiDAR + Odometry
          ↓
Perception + SLAM
          ↓
coverage_explorer
          ↓
Classical / FSM navigation control
          ↓
/cmd_vel
```

A future research extension could introduce:

```text
Camera ──► Visual Encoder ─┐
LiDAR ───► LiDAR Encoder ──┼──► Fusion ──► Diffusion Policy ──► (v, ω)
State ───► State Encoder ──┘
```

Any future diffusion implementation should be added as a clearly separated experimental module and benchmarked against the current `coverage_explorer` baseline.

---

# Academic Context

## Institution

**École Nationale Polytechnique d'Alger (ENP)**

## Student Team

- Iskender Yahiaoui
- Lamia Koudri
- Nourane Bouchama
- Raihana Hiba Bouray
- Bouras Manar

## Academic Supervisor

**Nasr-Eddine Mellah**  
Academic Supervisor  
Specialization: Electronics  
École Nationale Polytechnique d'Alger (ENP)

---

# Project Status

| Component | Status |
|---|---|
| ROS 2 Jazzy integration | Present |
| Gazebo Harmonic simulation | Present |
| TurtleBot3 Burger | Present / external dependency |
| Semantic HSV perception | Implemented |
| YOLOv8 dirt detection | Implemented |
| YOLO training workflow | Included |
| Online SLAM | Integrated |
| `coverage_explorer` | Active navigation implementation |
| `autonomous_explorer` | Legacy/reference |
| Diffusion Policy | Not implemented in supplied source |
| Physical robot deployment | Not included |

---

# Reproducibility

This repository was organized from the supplied `SmartVacuum-main(1).zip`.

The original implementation files are preserved. Documentation and repository metadata have been added around them without replacing the source implementation.

For a clean reproduction:

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash

export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-01

ros2 launch dirt_cleaner smart_vacuum.launch.py
```

---

# Documentation

- [`SETUP.txt`](SETUP.txt) — original setup instructions
- [`YOLO_DIRT_WORKFLOW.md`](YOLO_DIRT_WORKFLOW.md) — original YOLO workflow
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — organized architecture description
- [`docs/GITHUB_METADATA.md`](docs/GITHUB_METADATA.md) — repository name, description and topics
- [`docs/DIFFUSION_POLICY_STATUS.md`](docs/DIFFUSION_POLICY_STATUS.md) — implementation-status clarification

---

# License

The supplied project states that it is for **academic use only** and contains third-party robotics software dependencies.

This repository therefore does not add a new permissive open-source license without first confirming the ownership and redistribution terms of the bundled material.

---

# Citation

If this project is used in academic work:

```text
SmartVacuum — Autonomous Intelligent Robotic Vacuum Cleaner.
École Nationale Polytechnique d'Alger (ENP).
Academic Supervisor: Nasr-Eddine Mellah.
```

---

## Acknowledgements

This project builds on the ROS 2, Gazebo, TurtleBot3, SLAM Toolbox, OpenCV and Ultralytics ecosystems.


### Diffusion Policy — Research Direction

The project is also considering the use of **Diffusion Policy** as a future learning-based control component. The current deployed navigation and coverage system remains based on ROS 2, SLAM, semantic perception, and systematic coverage planning. Diffusion Policy should therefore be understood as a research direction being investigated for future integration rather than the primary controller of the current release.

