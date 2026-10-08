# Diffusion Policy Status

## Source-of-truth result

The uploaded `SmartVacuum-main(1).zip` was inspected.

The source archive contains:

- `coverage_explorer.py`
- `semantic_detector.py`
- `dirt_detector_node.py`
- `autonomous_explorer.py`
- SLAM configuration
- Gazebo worlds/models
- YOLO training code

It does **not** contain a recognizable deployed Diffusion Policy implementation, diffusion U-Net/Transformer controller, diffusion training loop, or diffusion-policy checkpoint.

## Correct repository wording

Diffusion Policy should therefore be presented as a **research/future direction**, not as the current deployed navigation controller.

The active source implementation is centered on:

```text
SLAM
+
Semantic perception
+
YOLOv8
+
coverage_explorer
+
classical/FSM navigation
```

This distinction is maintained deliberately to keep the GitHub repository technically accurate and reproducible.
