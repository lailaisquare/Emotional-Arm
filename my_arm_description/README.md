# my_arm_description tools

## Real eye-in-hand calibration

Start the real robot driver(s) and camera, then launch:

```bash
ros2 launch my_arm_description eyeinhand_real.launch.py
```

Common overrides:

```bash
ros2 launch my_arm_description eyeinhand_real.launch.py \
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info \
  robot_base_frame:=base_link \
  robot_effector_frame:=lamp_link_1
```

## Camera calibration (chessboard)

```bash
ros2 launch my_arm_description camera_calib.launch.py \
  board_size:=8x6 square_size:=0.025
```

## Generate calibration targets

Chessboard:

```bash
python3 scripts/generate_chessboard.py --size 8x6 --square-mm 25 --dpi 300 --output chessboard.png
```

ArUco marker (with optional white border):

```bash
python3 scripts/generate_aruco.py --dict DICT_6X6_250 --id 2 --side-px 600 --border-px 60 --output aruco.png
```

## Face tracking (real robot)

```bash
ros2 launch my_arm_description face_tracking_real.launch.py
```

Useful overrides:

```bash
ros2 launch my_arm_description face_tracking_real.launch.py \
  servo_port:=/dev/ttyUSB0 \
  pan_id:=1 tilt_id:=2 \
  gain_pan:=3.0 gain_tilt:=3.0
```

## Face tracking (MediaPipe + MoveIt2)

This mode uses MoveIt to respect joint limits and collision constraints.

```bash
ros2 launch my_arm_description face_tracking_moveit.launch.py
```

Download a MediaPipe face detector model (short range) and pass it in:

```bash
mkdir -p ~/.cache/mediapipe
curl -L -o ~/.cache/mediapipe/face_detector_short_range.tflite \
  https://storage.googleapis.com/mediapipe-models/face_detector/short_range/float16/1/face_detector_short_range.tflite
```

Then run:

```bash
ros2 launch my_arm_description face_tracking_moveit.launch.py \
  model_path:=~/.cache/mediapipe/face_detector_short_range.tflite
```

Common overrides:

```bash
ros2 launch my_arm_description face_tracking_moveit.launch.py \
  pan_joint:=l1 tilt_joint:=l2 \
  pan_min:=-1.57 pan_max:=1.57 \
  gain_pan:=0.4 gain_tilt:=0.4
```

Add a table collision object under base_link:

```bash
ros2 launch my_arm_description face_tracking_moveit.launch.py \
  add_table:=true \
  table_size_x:=1.2 table_size_y:=0.8 table_size_z:=0.05 \
  table_center_x:=0.0 table_center_y:=0.0 table_center_z:=-0.25
```
