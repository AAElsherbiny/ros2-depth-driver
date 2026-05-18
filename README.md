# ros2-depth-driver

A ROS 2 depth estimation package that computes per-pixel depth from **Kinect v1**, **Kinect v2**, **mono**, and **stereo** cameras.

- Works as a companion to [ros2-perception-stack](https://github.com/AA-Elsherbiny/ros2-perception-stack) — consumes its camera driver objects and adds depth processing on top
- Provides a unified API: `get_depth(y, x)` → returns a depth frame (with optional marker) and the depth value at that pixel
- Stereo depth is computed via **SGBM disparity mapping** and published as a ROS 2 topic
- Mono depth is estimated using **known-object-width** triangulation with HSV-based object detection
- Kinect depth is read directly from the sensor's depth stream

---

## Table of Contents

- [Overview](#overview)
- [Depth Estimation Methods](#depth-estimation-methods)
- [Class Hierarchy](#class-hierarchy)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
  - [Kinect v1 Depth](#kinect-v1-depth)
  - [Kinect v2 Depth](#kinect-v2-depth)
  - [Mono Depth](#mono-depth)
  - [Stereo Depth (Publisher)](#stereo-depth-publisher)
  - [Stereo Depth (Subscriber)](#stereo-depth-subscriber)
- [API Reference](#api-reference)
  - [depth_driver (Base Class)](#depth_driver-base-class)
  - [kinect_v1_depth / kinect_v2_depth](#kinect_v1_depth--kinect_v2_depth)
  - [monoDepth](#monodepth)
  - [stereoDepthPub](#stereodepthpub)
  - [StereoDepthDriver](#stereodepthdriver)
- [Camera Intrinsics](#camera-intrinsics)
- [QoS Configuration](#qos-configuration)
- [Project Structure](#project-structure)

---

## Overview

This package sits on top of the camera driver layer (`camera_drivers` from `ros2-perception-stack`) and adds depth estimation capabilities. Each depth driver wraps a camera driver object and extracts depth information using the appropriate method for that sensor type.

```
┌──────────────────────────────────────────────────────────────┐
│                     depth_driver package                     │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐    │
│  │              depth_driver (base class)               │    │
│  │   normalize() · get_depth() · display() · spin()     │    │
│  └────────┬───────────┬──────────────┬──────────────────┘    │
│           │           │              │                       │
│  ┌────────▼───┐ ┌─────▼──────┐ ┌─────▼─────────────┐         │
│  │ kinect_v1  │ │ kinect_v2  │ │   monoDepth       │         │
│  │  _depth    │ │  _depth    │ │ (object-width     │         │
│  │(raw depth) │ │(raw depth) │ │  triangulation)   │         │
│  └────────────┘ └────────────┘ └───────────────────┘         │
│                                                              │
│  ┌─────────────────────┐  ┌───────────────────────┐          │
│  │   stereoDepthPub    │  │  StereoDepthDriver    │          │
│  │ (SGBM disparity →   │  │ (subscribes to the    │          │
│  │  depth → publish)   │  │  published depth map) │          │
│  └─────────────────────┘  └───────────────────────┘          │
└──────────────────────────────────────────────────────────────┘
         ▲                          ▲
         │   uses camera objects    │
         │                          │
┌────────┴──────────────────────────┴────────────────────────┐
│              camera_drivers (ros2-perception-stack)        │
│   kinect_v1Driver · kinect_v2Driver · monoDriver · stereo  │
└────────────────────────────────────────────────────────────┘
```

---

## Depth Estimation Methods

| Driver | Sensor | Method | Output Unit |
|--------|--------|--------|-------------|
| `kinect_v1_depth` | Kinect v1 (Xbox 360) | Raw depth stream from sensor | mm |
| `kinect_v2_depth` | Kinect v2 (Xbox One) | Raw depth stream from sensor | mm |
| `monoDepth` | USB mono camera | Known-object-width triangulation: `depth = (W × f) / pixel_width` | proportional (set by calibration) |
| `stereoDepthPub` | USB stereo camera | SGBM disparity → `depth = (baseline × focal) / disparity` | mm |

---

## Class Hierarchy

```
depth_driver (Node)               cameradriver (from ros2-perception-stack)
├── kinect_v1_depth                    │
├── kinect_v2_depth                    └── StereoDepthDriver
├── monoDepth
└── stereoDepthPub
```

- **`depth_driver`** — Base class providing `normalize()`, `get_depth()`, `display()`, and `get_timestamp()`
- **`kinect_v1_depth` / `kinect_v2_depth`** — Wrap a Kinect driver, spin it, and expose the raw depth frame
- **`monoDepth`** — Detects a known-width object via HSV color thresholding and computes distance
- **`stereoDepthPub`** — Subscribes to left/right stereo topics, computes SGBM disparity, converts to depth, and publishes the depth map on a ROS 2 topic
- **`StereoDepthDriver`** — Subscribes to the depth topic published by `stereoDepthPub` for downstream use

---

## Prerequisites

| Dependency | Notes |
|------------|-------|
| **ROS 2** (Jazzy / Humble) | Core middleware |
| **ros2-perception-stack** | Must be built in the same workspace — provides `camera_drivers` and `srv_pkg` |
| **OpenCV** ≥ 4.x | `python3-opencv` |
| **NumPy** | `python3-numpy` |
| **imutils** | `pip install imutils` (used by mono depth) |
| **cv_bridge** | `ros-<distro>-cv-bridge` |

---

## Installation

### 1. Ensure ros2-perception-stack is in your workspace

The `depth_driver` package depends on `camera_drivers` from `ros2-perception-stack`. Both must live in the same ROS 2 workspace:

```
ros2_ws/src/
├── ros2-perception-stack/   # camera_drivers, kinect_ros2, etc.
└── ros2-depth-driver/       # this package
```

### 2. Install Python dependencies

```bash
pip install imutils
```

### 3. Install ROS 2 dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Usage

### Kinect v1 Depth

First, start the Kinect v1 camera driver (from `ros2-perception-stack`):

```bash
ros2 launch kinect_ros2 showimage.launch.py
```

Then run the depth node:

```bash
ros2 run depth_driver kinect_v1_depth
```

**What it does:** subscribes to the Kinect v1 depth stream, normalizes the frame, queries the depth at the center pixel, draws a circle marker, and prints the depth in mm.

```python
from camera_drivers.cameradriver import kinect_v1Driver
from depth_driver.kinect1_depth_driver import kinect_v1_depth
import rclpy

rclpy.init()
kinect1_subscriber = kinect_v1Driver(mode=2)          # mode=2 → depth only
kinect1_depth_node = kinect_v1_depth(kinect_object=kinect1_subscriber)

while rclpy.ok():
    kinect1_depth_node.spin(timeout=0.1)

    depth_frame = kinect1_depth_node.get_depth_frame()
    if depth_frame is None:
        continue

    h, w = depth_frame.shape

    # Returns (frame_with_marker, depth_value_in_mm)
    frame, depth_value = kinect1_depth_node.get_depth(h // 2, w // 2, normalize=True)
    kinect1_depth_node.display(frame)
    print(depth_value)
```

---

### Kinect v2 Depth

Start the Kinect v2 camera driver:

```bash
ros2 run kinect2_bridge sherbiny
```

Then run the depth node:

```bash
ros2 run depth_driver kinect_v2_depth
```

Usage is identical to Kinect v1 — just swap `kinect_v1Driver` → `kinect_v2Driver` and `kinect_v1_depth` → `kinect_v2_depth`.

---

### Mono Depth

Mono depth estimates distance using **known-object-width triangulation**. It detects a target object (by default: black color in HSV space) in the frame, measures its pixel width, and computes distance:

```
depth = (known_width × focal_length) / pixel_width
```

First, start a mono camera publisher (from `ros2-perception-stack`):

```bash
ros2 run camera_drivers mono_publisher
```

Then run the mono depth node:

```bash
ros2 run depth_driver mono_depth
```

**Configuration — you must set these before running:**

```python
from depth_driver.mono_depth_driver import monoDepth
from camera_drivers.cameradriver import monoDriver
import rclpy

rclpy.init()
mono_subscriber = monoDriver()
mono_depth = monoDepth(mono_object=mono_subscriber)

mono_depth.set_KNOWNWIDTH(15)                    # real-world width of target object (cm)
mono_depth.set_FOCAL_LENGTH(227.62354385 * 3)    # camera focal length (pixels)

while rclpy.ok():
    mono_depth.spin(timeout=0.1)
    mono_depth.spin()

    frame, depth_value = mono_depth.get_depth()
    mono_depth.display(frame, "mono depth")
    print(depth_value)
```

**Detection parameters** (configured as constants in `mono_depth_driver.py`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TARGET_POINT` | `(320, 240)` | Pixel coordinate of the target center |
| `MAX_DISTANCE_PX` | `200` | Max pixel distance a contour center can be from the target |
| `AREA_THRESHOLD` | `500` | Min contour area to consider |
| `LOWER_COLOR` | `[0, 0, 0]` | HSV lower bound for color detection |
| `UPPER_COLOR` | `[180, 255, 50]` | HSV upper bound for color detection |

---

### Stereo Depth (Publisher)

The stereo depth pipeline subscribes to left and right camera topics, computes an **SGBM disparity map**, converts it to a depth map using:

```
depth = (baseline × focal_length) / disparity
```

…and publishes the depth map on a ROS 2 topic (`stereo_depth` by default).

First, start the stereo camera publisher (from `ros2-perception-stack`):

```bash
ros2 run camera_drivers stereo_publisher
```

Then run the stereo depth publisher:

```bash
ros2 run depth_driver stereo_depth_publisher
```

```python
from depth_driver.stereo_depth_driver import stereoDepthPub
from camera_drivers.cameradriver import stereoDriver
import rclpy

rclpy.init()
left_sub  = stereoDriver(subscriber=True, topic="left_stereo")
right_sub = stereoDriver(subscriber=True, topic="right_stereo")
stereo = stereoDepthPub(
    left_stereo_object=left_sub,
    right_stereo_object=right_sub,
    compressed=False    # set True for JPEG-compressed depth topic
)

while rclpy.ok():
    left_sub.spin(timeout=0.1)
    right_sub.spin(timeout=0.1)
    stereo.spin()

    depth_frame = stereo.get_depth_frame()
    if depth_frame is None:
        continue

    h, w = depth_frame.shape

    # Query depth at a specific pixel
    frame, depth_value = stereo.get_depth(h // 2, w // 2, normalize=True, draw_circle=True)
    stereo.display(frame)
    print(depth_value)
```

**Stereo calibration constants** (configured in `stereo_depth_driver.py`):

| Constant | Default | Description |
|----------|---------|-------------|
| `BASELINE` | `6.3` | Distance between stereo camera centers (cm) |
| `FOCAL_LENGTH` | `194` | Camera focal length (pixels) |
| `CX`, `CY` | `320`, `180` | Principal point |

---

### Stereo Depth (Subscriber)

If the stereo depth publisher is running, any other node can subscribe to the published depth map:

```bash
ros2 run depth_driver stereo_depth_subscriber
```

```python
from depth_driver.stereo_depth_driver import StereoDepthDriver
import rclpy

rclpy.init()
stereo_depth_sub = StereoDepthDriver(subscriber=True, topic="stereo_depth")

while rclpy.ok():
    stereo_depth_sub.spin(timeout=0.1)

    depth_frame = stereo_depth_sub.get_frame(normalize=False)
    if depth_frame is None:
        continue

    h, w = depth_frame.shape[:2]
    frame, depth_value = stereo_depth_sub.get_depth(h // 2, w // 2, normalize=False, draw_circle=True)
    stereo_depth_sub.display(frame=frame, frame_name="depth_frame")
    print(depth_value)
```

---

## API Reference

### `depth_driver` (Base Class)

| Method | Signature | Description |
|--------|-----------|-------------|
| `normalize(frame)` | `frame → np.ndarray` | Normalizes a depth frame to 0–255 for visualization |
| `get_depth(y, x, normalize, draw_circle)` | `→ (frame, depth_value)` | Returns the depth frame (optionally normalized, with a circle marker) and the raw depth value at pixel `(y, x)` |
| `get_depth_frame()` | `→ np.ndarray or None` | Returns the latest raw depth frame |
| `get_timestamp()` | `→ stamp or None` | Returns the timestamp of the latest depth frame |
| `display(frame, frame_name)` | | Displays a frame with OpenCV. Press `q` to close |
| `spin()` | | Spins the node once |
| `get_internsic_values()` | `→ (Fx, Fy, Cx, Cy)` | Returns the camera intrinsic parameters |

---

### `kinect_v1_depth` / `kinect_v2_depth`

```python
kinect_v1_depth(kinect_object, node_name="kinect1_depth_node")
kinect_v2_depth(kinect_object, node_name="kinect2_depth_node")
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `kinect_object` | `kinect_v1Driver` / `kinect_v2Driver` | A camera driver object from `ros2-perception-stack` |
| `node_name` | `str` | ROS 2 node name |

**Additional method:**

| Method | Description |
|--------|-------------|
| `spin(timeout)` | Spins the wrapped Kinect driver with a timeout, then updates the internal depth frame |

---

### `monoDepth`

```python
monoDepth(node_name="mono_depth_node", mono_object=None)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `mono_object` | `monoDriver` | A mono camera subscriber from `ros2-perception-stack` |

| Method | Description |
|--------|-------------|
| `set_KNOWNWIDTH(width)` | Set the real-world width of the target object |
| `set_FOCAL_LENGTH(focal)` | Set the camera focal length (in pixels) |
| `get_depth()` | Returns `(frame, depth_value)` — the annotated frame and estimated distance |
| `find_marker(image)` | Detects the target object via HSV thresholding and returns its rotated bounding rect |
| `distance_to_camera(knownWidth, focalLength, perWidth)` | Computes distance from known object width |
| `FOV(sensor_width, focal)` | Computes the field of view in degrees |

---

### `stereoDepthPub`

```python
stereoDepthPub(
    node_name="stereo_depth_node",
    left_stereo_object=None,
    right_stereo_object=None,
    topic_name="stereo_depth",
    compressed=False
)
```

| Parameter | Type | Description |
|-----------|------|-------------|
| `left_stereo_object` | `stereoDriver` | Left camera subscriber |
| `right_stereo_object` | `stereoDriver` | Right camera subscriber |
| `topic_name` | `str` | Topic to publish the depth map on |
| `compressed` | `bool` | Use JPEG-compressed transport |

| Method | Description |
|--------|-------------|
| `publish()` | Publishes the current depth frame to the configured topic |
| `get_depth(y, x, normalize, draw_circle)` | Returns `(frame, depth_value)` with optional visualization |
| `normalize(frame)` | Returns the inverted disparity canvas |
| `spin()` | Spins all three nodes (self + left + right subscribers) |

**Published topic:** `stereo_depth` — `sensor_msgs/Image` (`64FC1`) or `sensor_msgs/CompressedImage`

---

### `StereoDepthDriver`

```python
StereoDepthDriver(
    subscriber=True,
    topic="stereo_depth",
    node_name="stereoDepth_node_sub",
    service="stereoDepth_service"
)
```

Inherits from `cameradriver` (ros2-perception-stack). Subscribes to the depth topic published by `stereoDepthPub`.

| Method | Description |
|--------|-------------|
| `get_depth(y, x, normalize, draw_circle)` | Returns `(frame, depth_value)` from the subscribed depth map |
| `get_frame(normalize)` | Returns the latest depth frame, optionally normalized |
| `normalize(frame)` | Normalizes the depth frame to 0–255 |
| `display(frame, frame_name)` | Displays with OpenCV |

---

## Camera Intrinsics

Each driver has built-in default intrinsic values that can be retrieved via `get_internsic_values()`:

| Driver | Fx | Fy | Cx | Cy |
|--------|-----|-----|------|------|
| `kinect_v1_depth` | 572.99 | 542.74 | 314.65 | 240.17 |
| `kinect_v2_depth` | 365.60 | 365.36 | 248.82 | 208.63 |
| `monoDepth` | 682.87 | 682.87 | 178.53 | 187.09 |
| `stereoDepthPub` | 194.00 | 194.00 | 320.00 | 180.00 |

---

## QoS Configuration

The stereo depth publisher uses:

```python
QoSProfile(
    reliability = BEST_EFFORT,
    durability  = VOLATILE,
    history     = KEEP_LAST,
    depth       = 1
)
```

---

## Project Structure

```
ros2-depth-driver/
│
├── depth_driver/                          # ROS 2 Python package
│   ├── depth_driver/
│   │   ├── depth_driver.py                #   Base class — normalize, get_depth, display
│   │   ├── kinect1_depth_driver.py        #   Kinect v1 depth wrapper
│   │   ├── kinect2_depth_driver.py        #   Kinect v2 depth wrapper
│   │   ├── mono_depth_driver.py           #   Mono depth via object-width triangulation
│   │   ├── stereo_depth_driver.py         #   Stereo SGBM disparity → depth + subscriber
│   │   └── examples/
│   │       ├── kinect_v1_depth_ex.py      #   Kinect v1 depth example
│   │       ├── kinect_v2_depth_ex.py      #   Kinect v2 depth example
│   │       ├── mono_depth_ex.py           #   Mono depth example
│   │       ├── stereo_depth_publisher_ex.py   #   Stereo depth publisher example
│   │       └── stereo_depth_subscriber_ex.py  #   Stereo depth subscriber example
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
│
└── README.md
```

---

## Quick-Start Commands

| Command | Description |
|---------|-------------|
| `ros2 run depth_driver kinect_v1_depth` | Kinect v1 depth estimation |
| `ros2 run depth_driver kinect_v2_depth` | Kinect v2 depth estimation |
| `ros2 run depth_driver mono_depth` | Mono camera depth estimation |
| `ros2 run depth_driver stereo_depth_publisher` | Stereo depth computation + publishing |
| `ros2 run depth_driver stereo_depth_subscriber` | Subscribe to stereo depth topic |
