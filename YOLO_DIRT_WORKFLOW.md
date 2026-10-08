# YOLO Dirt Detection Workflow

This project now includes:

- `dirt_detector_node.py`: ROS2 node that runs YOLO on `/camera/image_raw` and publishes `/dirt_target`.
- `autonomous_explorer.py`: now listens to `/dirt_target` and chases detected dirt/trash.
- `train_dirt_yolo.py`: helper script to train a YOLO model.

## 1) Install dependencies

```bash
python3 -m pip install -r /home/lxmix/turtlebot3_ws/requirements_yolo.txt
sudo apt install ros-jazzy-rqt-image-view
```

## 2) Collect images

Run simulation:

```bash
bash /home/lxmix/turtlebot3_ws/run_vacuum.sh
```

In another terminal, save images:

```bash
bash /home/lxmix/turtlebot3_ws/capture_yolo_images.sh /home/lxmix/turtlebot3_ws/datasets/dirt_detection
```

## 3) Label data in YOLO format

Directory structure expected by YOLO:

```text
datasets/dirt_detection/
  images/train/*.jpg
  images/val/*.jpg
  labels/train/*.txt
  labels/val/*.txt
  data.yaml
```

Copy template config:

```bash
cp /home/lxmix/turtlebot3_ws/datasets/dirt_detection/data.yaml.example /home/lxmix/turtlebot3_ws/datasets/dirt_detection/data.yaml
```

Use class name `trash` for your dirt/trash patches.

## 4) Train YOLO

```bash
python3 /home/lxmix/turtlebot3_ws/train_dirt_yolo.py \
  --data /home/lxmix/turtlebot3_ws/datasets/dirt_detection/data.yaml \
  --epochs 60
```

Best model path:

```text
/home/lxmix/turtlebot3_ws/models/dirt_yolo/weights/best.pt
```

## 5) Run detection + pursuit

`run_vacuum.sh` already starts the detector with:

```text
-p weights:=/home/lxmix/turtlebot3_ws/models/dirt_yolo/weights/best.pt
```

To debug detections:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/camera/dirt_debug`.
