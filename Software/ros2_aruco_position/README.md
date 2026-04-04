# ros2_aruco_position

ROS 2 Python package that detects ArUco markers from a local camera device and publishes one TF frame per detected marker.

## Features

- Uses the OpenCV ArUco detector pattern from `aruco_detect.py`
- Reads frames directly from `/dev/video0` or another user-specified device
- Loads camera intrinsics and distortion from the packaged `camera_calibration.yaml` by default
- Estimates pose with `cv2.solvePnP`
- Publishes TF frames named `aruco_<id>` for every marker detected in the current frame
- Publishes the full list of currently detected marker IDs on a ROS topic
- If no calibration file is provided, assumes the camera image is already corrected and uses zero distortion

## Build

Place this folder in your ROS 2 workspace `src/` directory, then build:

```bash
colcon build --packages-select ros2_aruco_position
```

## Docker

Build the ROS 2 Humble image:

```bash
docker build -f docker/Dockerfile -t ros2_aruco_position:humble .
```

The container starts as root only long enough to install missing package dependencies, then drops to a normal user named `ros` before building and running the node.
The `ros` user has passwordless `sudo` inside the container for interactive debugging and package checks.

If you want the container user to match your host UID and GID:

```bash
docker build \
  --build-arg USER_UID=$(id -u) \
  --build-arg USER_GID=$(id -g) \
  -f docker/Dockerfile \
  -t ros2_aruco_position:humble .
```

When the container starts, it installs package dependencies from `package.xml` with `rosdep`, then builds the mounted workspace before launching the node. The container therefore needs network access when it starts.
It also detects mounted `/dev/video*` devices and adds the runtime user to the matching video-device groups before dropping root privileges.

Run it with this package directory mounted into the container and a camera device passed through:

```bash
docker run --rm -it --net=host --device=/dev/video0 \
  -v "$(pwd)":/workspace/ros2_aruco_position \
  ros2_aruco_position:humble
```

Run with a different device or extra ROS parameters:

```bash
docker run --rm -it --net=host --device=/dev/video2 \
  -v "$(pwd)":/workspace/ros2_aruco_position \
  ros2_aruco_position:humble \
  ros2 run ros2_aruco_position aruco_tf_node --ros-args -p video_device:=/dev/video2
```

Run with X11 forwarding so GUI apps like `rviz2` can display on the host:

```bash
xhost +local:docker
docker run --rm -it --net=host --device=/dev/video0 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$(pwd)":/workspace/ros2_aruco_position \
  ros2_aruco_position:humble \
  bash
```

Then inside the container:

```bash
rviz2
```

## Run

```bash
ros2 launch ros2_aruco_position aruco_tf.launch.py
```

Or run the node directly:

```bash
ros2 run ros2_aruco_position aruco_tf_node
```

Direct run with another device:

```bash
ros2 run ros2_aruco_position aruco_tf_node --ros-args -p video_device:=/dev/video2
```

You can also pass an integer device index:

```bash
ros2 run ros2_aruco_position aruco_tf_node --ros-args -p video_device:=0
```

Run with a calibration file:

```bash
ros2 run ros2_aruco_position aruco_tf_node --ros-args -p camera_calibration_file:=/path/to/camera.yaml
```

If `camera_calibration_file` is left empty, the node automatically loads the packaged calibration file from `share/ros2_aruco_position/camera_calibration.yaml`.

## Key Parameters

- `video_device`: device path like `/dev/video0` or an index like `0`
- `camera_frame`: TF parent frame
- `camera_calibration_file`: OpenCV-style calibration YAML with `camera_matrix` and distortion coefficients; defaults to the packaged `camera_calibration.yaml`
- `detected_ids_topic`: topic publishing `std_msgs/msg/Int32MultiArray` of visible marker IDs
- `marker_length`: marker size in meters
- `aruco_dictionary`: OpenCV dictionary name such as `DICT_4X4_50`
- `fallback_fx` and `fallback_fy`: approximate focal lengths used without calibration
- `capture_width` and `capture_height`: requested camera resolution
- `capture_fps`: requested polling rate for the camera
- `processing_scale`: downscale factor for marker detection, between `0.0` and `1.0`
- `frame_skip`: process every `frame_skip + 1` frames
- `use_grayscale`: detect markers on grayscale images to reduce CPU load

Marker poses are published directly from each raw `solvePnP` result without Kalman smoothing.
