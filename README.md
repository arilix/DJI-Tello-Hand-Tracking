# DJI Tello Hand Tracking

ROS 2 Humble workspace for controlling a DJI Tello drone with hand tracking.
The project combines a Tello UDP/video driver, keyboard control, and a
MediaPipe-based hand vision node.

## Features

- DJI Tello command, state, and video communication over UDP
- ROS 2 topics for takeoff, landing, and RC control
- Hand tracking with OpenCV and MediaPipe
- Automatic lateral, vertical, and distance tracking
- Debug image output for viewing detected landmarks and tracking state

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- DJI Tello connected over Wi-Fi
- OpenCV, `cv_bridge`, and the dependencies listed in each package

## Repository Layout

```text
workspace/
└── src/
    ├── tello_driver/    # Tello UDP driver and video stream
    ├── tello_control/   # Keyboard control
    ├── tello_msg/       # Custom ROS 2 messages
    ├── hand_vision/     # Hand tracking and automatic control
    └── launch.py        # Launches the complete system
```

## Build

```bash
cd workspace
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

To build only the hand vision package:

```bash
colcon build --packages-select hand_vision \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Run

Connect to the Tello Wi-Fi network, then run the complete system:

```bash
source /opt/ros/humble/setup.bash
source workspace/install/setup.bash
ros2 launch workspace/src/launch.py
```

The hand control window uses:

| Key | Action |
| --- | --- |
| `Q` | Take off from idle |
| `E` | Land while tracking |
| `ESC` | Emergency land |

The debug stream is published on `/hand_vision/debug_image` and can be
viewed with:

```bash
ros2 run rqt_image_view rqt_image_view /hand_vision/debug_image
```

## Configuration

Detailed configuration, ROS 2 topics, troubleshooting, and per-node launch
instructions are available in [workspace/README.md](workspace/README.md).

## Safety

Test with the propellers removed first. Keep the emergency landing path clear,
maintain a safe distance from people and objects, and stop the node immediately
if tracking behaves unexpectedly.