# Camera calibration

## Overview

Raiden uses ChArUco boards to calibrate cameras. Two types of calibration are
supported:

- **Hand-eye calibration** (wrist cameras) - determines the rigid transform
  from the camera frame to the gripper frame (`T_camera→gripper`).
- **Scene camera calibration** - determines the transform from the camera
  frame to the robot base frame (`T_camera→base`).

![Calibration setup](../assets/calibration.jpg)

## Step 1 - Print a ChArUco board

Print a ChArUco board and mount it rigidly (e.g. on a flat aluminium plate).
For best results, consider an industry-grade printed target from
[calib.io](https://calib.io/products/charuco-targets). These are manufactured
on dimensionally stable material and give lower reprojection errors than
paper prints.

## Step 2 - Record calibration poses

Move the robot to a series of diverse poses while the board is visible. At
least **5 poses** are required; 7–10 is recommended for good coverage.

```bash
rd record_calibration_poses
```

A live Rerun viewer opens automatically in your browser so you can verify that
the board is visible from all cameras before recording each pose. The URL and
an SSH tunnel command are printed to the terminal:

```
Rerun viewer:    http://localhost:9877?url=...
SSH tunnel:      ssh -L 9877:localhost:9877 -L 9878:localhost:9878 <host>
```

Images are refreshed at **1 Hz**. The right wrist camera image is
automatically rotated 180° to correct for its upside-down mounting.

Keyboard controls:

- **R** - record current pose
- **D** - delete last recorded pose
- **L** - list all recorded poses
- **Q** - quit (saves if enough poses recorded)

Poses are saved to `~/.config/raiden/calibration_poses.json`.

### Recording without leader arms

By default the followers are posed by moving the leader arms. If you have no
leader arms connected, use hand-guide mode:

```bash
rd record_calibration_poses --control handguide
```

The follower arms are put into zero-stiffness gravity-compensation mode so you
can position them directly by hand, and poses are recorded with the **R** key.

A small amount of velocity damping is retained so that cable tension does not
slowly drag the arms out of position. Tune it with `--handguide-damping`, given
as a fraction of the arm's nominal `kd`:

| Value | Effect |
|---|---|
| `0.0` | Fully released — moves with the least effort, but drifts under cable pull |
| `0.2` | Default — resists drift, still easy to reposition |
| `0.4`+ | Noticeably stiff to move; use only if the arms still creep |

Damping resists *speed*, not position, so it holds the arm against a steady
tug without fighting you when you move it deliberately.

!!! warning

    In hand-guide mode the arms hold no position of their own — only gravity
    compensation resists the load. Support both arms by hand before the
    "HAND-GUIDE MODE ACTIVE" banner appears, and keep clear of anything the
    arms could fall onto.

## Step 3 - Run calibration

```bash
rd calibrate
```

If your board has different dimensions or a different dictionary, pass them here:

```bash
rd calibrate \
    --squares-x 14 \
    --squares-y 9 \
    --square-length 0.02 \
    --marker-length 0.015 \
    --dictionary DICT_5X5_250
```

| Parameter | Default | Description |
|---|---|---|
| `--squares-x` | `14` | Number of squares along X |
| `--squares-y` | `9` | Number of squares along Y |
| `--square-length` | `0.02` | Checker square side length in metres |
| `--marker-length` | `0.015` | ArUco marker side length in metres |
| `--dictionary` | `DICT_5X5_250` | ArUco dictionary |

Raiden reads `~/.config/raiden/camera.json` and calibrates every camera whose
`role` is `"left_wrist"`, `"right_wrist"`, or `"scene"` automatically - no
explicit camera IDs are required.

### If the board is not detected

ChArUco detection needs roughly **3-4 pixels per marker bit cell**, i.e. a
marker at least ~28 px across in the image. A 15 mm marker at 640x480 is only
decodable within ~20 cm of a wrist camera, which is usually too close to be
practical. Raise the colour resolution per camera in `camera.json`:

```json
"scene_camera": {
  "serial": "254622075770", "type": "realsense", "role": "scene",
  "color_size": [1280, 720],
  "depth_size": [640, 480]
}
```

`color_size` doubles the usable working distance at 1280x720. `depth_size`
defaults to `color_size`; set it explicitly when the two must differ:

- **D405** derives colour and depth from the same stereo pair and *rejects*
  mismatched resolutions - leave `depth_size` unset so it follows `color_size`.
- **D435** accepts a mismatch, and cannot sustain 1280x720 on both streams at
  30 fps - pin `depth_size` to `[640, 480]` as above.

Check the board dimensions too: `--squares-x` is the number of squares along
the board's **long** edge if the board is wider than it is tall. A transposed
board (9x14 vs 14x9) detects individual ArUco markers fine but yields **zero**
ChArUco corners, since the marker IDs no longer land where the layout expects.

The runner will:

1. Load the recorded poses from `~/.config/raiden/calibration_poses.json`
2. Move the robot to each pose
3. Capture frames from all target cameras
4. Detect the ChArUco board in each frame
5. Run hand-eye / scene calibration
6. Save results to `~/.config/raiden/calibration_results.json`

## Output

`~/.config/raiden/calibration_results.json` contains one entry per camera:

```json
{
  "version": "1.0",
  "timestamp": "2026-03-12T22:00:00.000000",
  "coordinate_frame": "left_arm_base",
  "cameras": {
    "left_wrist_camera": {
      "type": "hand_eye",
      "serial_number": 16522755,
      "intrinsics": {
        "camera_matrix": [[526.7, 0, 319.4], [0, 526.7, 178.5], [0, 0, 1]],
        "distortion_coeffs": [-0.041, 0.013, 0.0, 0.0, 0.0],
        "image_size": [672, 376],
        "source": "zed_sdk_factory_calibration"
      },
      "num_poses_used": 8,
      "hand_eye_calibration": {
        "success": true,
        "rotation_matrix": [[0.999, -0.012, 0.008], [0.012, 0.999, -0.003], [-0.008, 0.003, 0.999]],
        "translation_vector": [0.012, -0.034, 0.061],
        "rotation_vector": [-0.003, 0.008, 0.012],
        "method": "Tsai"
      }
    },
    "scene_camera": {
      "type": "scene",
      "serial_number": 37038161,
      "intrinsics": {
        "camera_matrix": [[700.1, 0, 659.8], [0, 700.1, 371.2], [0, 0, 1]],
        "distortion_coeffs": [-0.052, 0.021, 0.0, 0.0, 0.0],
        "image_size": [1280, 720],
        "source": "zed_sdk_factory_calibration"
      },
      "num_poses_used": 8
    }
  },
  "bimanual_transform": {
    "right_base_to_left_base": [[1, 0, 0, -0.612], [0, 1, 0, 0.0], [0, 0, 1, 0.0], [0, 0, 0, 1]],
    "computed_from_calibration": true,
    "description": "Transform from right arm base frame to left arm base frame"
  }
}
```

