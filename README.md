# UR5e Vision-Guided Positioning

Python visual servoing for a Universal Robots UR5e using the Robotiq wrist camera. The robot detects a coloured box on the table and moves the tool centre point until the box is centred in the camera view.

Developed at the Laboratory for Advanced Manufacturing Simulation and Robotics, UCD School of Mechanical and Materials Engineering.

## What it does

The control loop runs continuously and does the following on each pass:

1. Pulls a JPEG frame from the wrist camera over HTTP
2. Detects an ArUco marker in the frame and uses its known physical size to work out a millimetres-per-pixel scale
3. Detects the target box using HSV colour segmentation
4. Measures how far the box centre sits from the image centre, converts that pixel error into millimetres, and issues a small linear move to reduce it
5. Repeats until the error falls inside the deadband

The ArUco marker is what makes the pixel error meaningful. Without a known reference in the scene, a 40 pixel offset could be 4 mm or 40 mm depending on camera height. The marker gives the loop a real-world scale on every frame.

## Requirements

- Python 3.11, 64-bit. `ur-rtde` will not install or run correctly on other versions.
- A UR5e reachable over the network with the RTDE interface enabled
- Robotiq wrist camera serving frames on port 4242

```
pip install ur-rtde opencv-python numpy requests
```

## Setup

Edit the constants at the top of `CameraControl.py` before the first run.

| Constant | Meaning |
|---|---|
| `ROBOT_IP` | Robot address. Also used to build the camera URL. |
| `MARKER_SIZE` | Physical side length of the ArUco marker in mm. Must match the printed marker. |
| `MARKER_ID` | Specific marker to track. `None` tracks the lowest ID found. |
| `BOX_HSV_LOWER` / `BOX_HSV_UPPER` | HSV range for the target colour. Retune under the actual lab lighting. |
| `BOX_MIN_AREA` | Minimum contour area in pixels. Filters out noise. |
| `CAMERA_TO_TOOL_DEG` | Rotation between the camera frame and the tool frame. Set this if the robot moves in the wrong direction. |
| `TABLE_Z_BASE_MM` | Table height in the robot base frame. Leave as `None` to hold the current height. |

The ArUco dictionary is set in code as `DICT_4X4_50`. Change it if your markers use a different dictionary, and check that `MARKER_SIZE` matches what you actually printed, since a wrong value scales every move.

## Running

```
python CameraControl.py
```

Two windows open, one showing the live frame with detections drawn on it and one showing the colour mask. Press ESC in either window to stop. The console prints the current error and the commanded step on every iteration.

Position the robot manually so the marker and the box are both in view before starting. The loop does nothing until it sees both.

## Safety limits

Several guards are built in. They are conservative by default and worth keeping that way.

- **Step cap.** No single move exceeds `MAX_STEP_MM` (20 mm), regardless of how large the error is.
- **Deadband.** Below `DEADBAND_MM` (3 mm) the robot holds position rather than hunting.
- **Proportional gain.** `KP` is 0.5, so the robot corrects half the error per step and approaches smoothly.
- **Tilt check.** If the tool is more than `MAX_TILT_DEG` from vertical, moves are blocked. The pixel-to-millimetre conversion assumes the camera is looking straight down and stops being valid at an angle.
- **Workspace limits.** `X_LIMITS_MM` and `Y_LIMITS_MM` block moves outside a set envelope. These are `None` by default. Set them before running near anything you would rather not hit.
- **Z limits and rate limit.** Height is clamped to `Z_LIMITS_MM` and can only change by `MAX_Z_STEP_MM` per iteration.
- **Camera dropout.** After `MAX_CAMERA_FAILURES` consecutive failed frames the loop exits rather than continuing blind.

The `finally` block calls `stopScript` and disconnects both interfaces, so the robot is released even if the script crashes.

## Known limitations

**The marker pose is calculated but barely used.** `solvePnP` returns both a rotation and a translation for the marker. The rotation is discarded and the translation is only used to print a distance readout. All actual movement comes from the millimetres-per-pixel scale derived from the marker's apparent side length. This works while the camera looks roughly straight down at a flat table, which is why the tilt check exists. Using the full pose would make the system tolerant of an angled view.

**Camera intrinsics are guessed.** `FX` and `FY` are hardcoded to 800 and distortion is assumed to be zero. This is fine for the scale-based control the loop actually uses, but the printed distance figure should not be trusted as a measurement. A proper calibration would fix this.

**The box is assumed to be at marker height.** The scale comes from the marker. If the box sits noticeably higher or lower, the conversion is off.

**No gripper action.** This centres the tool over the target. Picking and placing is the next step.

## Files

```
CameraControl.py    Detection and control loop
```
