# SO101 Robot - URDF and MuJoCo Description

This repository contains the URDF and MuJoCo (MJCF) files for the SO101 robot.

## Overview

- The robot model files were generated using the [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) plugin from a CAD model designed in Onshape.
- The generated URDFs were modified to allow meshes with relative paths instead of `package://...`.
- Base collision meshes were removed due to problematic collision behavior during simulation and planning.

## Calibration Methods

The MuJoCo file `scene.xml` supports two differenly calibrated SO101 robot files:

- **New Calibration (Default)**: Each joint's virtual zero is set to the **middle** of its joint range. Use -> `so101_new_calib.xml`. 
- **Old Calibration**: Each joint's virtual zero is set to the configuration where the robot is **fully extended horizontally**. Use -> `so101_old_calib.xml`.

To switch between calibration methods, modify the included robot file in `scene.xml`.

## LeKiwi Extension

For `pylekiwi` runtime FK, use `lekiwi_so101_new_calib.xml`. If you switch the MuJoCo
scene to the old calibration, use `lekiwi_so101_old_calib.xml`. These derived MJCF
files keep the original SO101 arm kinematics intact and add LeKiwi-specific fixed frames:

- `lekiwi_arm_mount`: the SO101 arm mounting frame used by runtime FK. It is kept
  coincident with the original SO101 `base` body.
- `lekiwi_chassis`: a CAD-derived chassis frame aligned from the public LeKiwi URDF
  by matching the shoulder-pan joint frame to the SO101 MJCF.
- `lekiwi_base_camera_mount`: a CAD-derived chassis-side camera frame taken from the
  upstream `Camera-Model-v3` link in the LeKiwi URDF.
- `lekiwi_base_camera_optical`: an approximate optical-center frame placed at the
  CAD lens-front center of `Camera-Model-v3` and oriented with the camera CAD mesh.

`lekiwi_base_camera_mount` is the camera-body CAD frame. `lekiwi_base_camera_optical`
is the frame to use when you want a camera-origin pose, but it is still a CAD
approximation rather than a hand-calibrated optical center. To regenerate these
values from the upstream URDF, use:

```bash
uv run python examples/lekiwi_cad_transforms.py /path/to/LeKiwi.urdf
```

Referenced LeKiwi CAD sources:
- Public Fusion 360 CAD: <https://a360.co/4k1P8yO>
- Upstream LeKiwi URDF exported from CAD: <https://github.com/SIGRobotics-UIUC/LeKiwi/blob/main/URDF/LeKiwi.urdf>

## Motor Parameters

Motor properties for the STS3215 motors used in the robot are adapted from the [Open Duck Mini project](https://github.com/apirrone/Open_Duck_Mini).

## Gripper Note

In LeRobot, the gripper is represented as a **linear joint**, where:

* `0` = fully closed
* `100` = fully open

This mapping is **not yet reflected** in the current URDF and MuJoCo files. 

---

Feel free to open an issue or contribute improvements!
