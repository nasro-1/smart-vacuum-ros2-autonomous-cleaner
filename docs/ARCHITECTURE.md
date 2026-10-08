# SmartVacuum Architecture

## Active pipeline

```text
Gazebo
  │
  ├── RGB Camera ──► semantic_detector
  │                       │
  │                       └── /semantic_class
  │
  ├── RGB Camera ──► dirt_detector_node
  │                       │
  │                       └── /dirt_target
  │
  ├── LiDAR ─────────────► slam_toolbox ──► /map
  │
  └── Odometry ──────────► coverage_explorer
                              │
                              ▼
                           /cmd_vel
                              │
                              ▼
                       TurtleBot3 Burger
```

## Active controller

The supplied `src/dirt_cleaner/setup.py` registers:

```text
semantic_detector
dirt_detector_node
coverage_explorer
```

The `autonomous_explorer` console entry is commented out.

The supplied `smart_vacuum.launch.py` starts `coverage_explorer` after a 13-second delay.

## Perception

`semantic_detector.py` performs HSV-based classification from the camera.

`dirt_detector_node.py` performs YOLO-based targeted detection.

## Mapping

`slam_toolbox` is launched in online asynchronous mode.

## Navigation

`coverage_explorer.py` is the main active navigation implementation and combines mapping, coverage behaviour, obstacle handling and dirt-related behaviour.

## Legacy code

The repository contains `autonomous_explorer.py` in both the package and root `nodes/` area. These files are retained because they are part of the supplied archive, but they are not the active launch entry point.
