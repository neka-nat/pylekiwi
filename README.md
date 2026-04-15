# pylekiwi
[![PyPI version](https://badge.fury.io/py/pylekiwi.svg)](https://badge.fury.io/py/pylekiwi)

Python package for controlling the LeKiwi robot.

![lekiwi](assets/lekiwi.jpg)

## Quick Start

### Web UI

Log into the robot and run the following command:

```bash
ssh <your robot ip>
sudo chmod 666 <your_follower_robot_serial_port>
uvx pylekiwi webui --serial-port <your_follower_robot_serial_port>
```

Then, open a web browser and navigate to `http://<your robot ip>:8080` to see the web UI.

![web ui](assets/webui.png)

### Leader and Follower/Client Nodes

Run the following command to start the follower node (host) on the robot (Respberry Pi):

```bash
ssh <your robot ip>
sudo chmod 666 <your_follower_robot_serial_port>
uvx pylekiwi host --serial-port <your_follower_robot_serial_port>
```

Run the following command to start the leader node (client) on the remote machine:

```bash
sudo chmod 666 <your_leader_robot_serial_port>
uvx --from 'pylekiwi[client]' pylekiwi leader --serial-port <your_leader_robot_serial_port>
```

Use `pylekiwi client` on the remote machine for one-off remote operations without
starting the continuous `leader` controller.

Available command groups:

- `state`: read the current remote robot state
- `capture`: save one JPEG from the `base` or `arm` camera
- `pose`: go to, save, list, or delete named arm poses
- `position` / `inching`: move the end effector in the base frame
- `grasp` / `release`: close or open the gripper
- `arm`: inspect modeled link frames or toggle arm torque
- `calibrate`: inspect, back up, zero, or restore remote arm calibration

Common examples:

Inspect state or capture images:

```bash
uvx pylekiwi client state
uvx pylekiwi client capture --camera base --output photo.jpg
uvx pylekiwi client capture --camera arm --output wrist.jpg
```

Move the arm or gripper:

```bash
uvx pylekiwi client pose go <preset_name>
uvx pylekiwi client pose go "10,20,30,40,50"
uvx pylekiwi client position --x-mm 180 --y-mm 0 --z-mm 120
uvx pylekiwi client inching --x-mm 10 --z-mm -5
uvx pylekiwi client grasp
uvx pylekiwi client release
```

Manage presets or maintenance:

```bash
uvx pylekiwi client pose save <name>
uvx pylekiwi client pose list
uvx pylekiwi client pose delete <name>
uvx pylekiwi client arm off
uvx pylekiwi client arm on
uvx pylekiwi client calibrate status
uvx pylekiwi client calibrate backup
```

Inspect modeled frames:

```bash
uvx pylekiwi client arm links actual --frame lekiwi_chassis
uvx pylekiwi client arm links actual --frame lekiwi_base_camera_mount
uvx pylekiwi client arm links actual --frame lekiwi_base_camera_optical
uvx pylekiwi client arm links actual --frame wrist_camera_mount
```

The `lekiwi_chassis` and `lekiwi_base_camera_mount` frames are now seeded from the
public LeKiwi CAD/URDF rather than placeholders. `lekiwi_base_camera_mount` tracks
the upstream CAD camera-body link, and `lekiwi_base_camera_optical` adds an
approximate lens-center frame derived from the CAD mesh.

Referenced LeKiwi CAD sources:
- Public Fusion 360 CAD: <https://a360.co/4k1P8yO>
- Upstream LeKiwi URDF exported from CAD: <https://github.com/SIGRobotics-UIUC/LeKiwi/blob/main/URDF/LeKiwi.urdf>

If automatic discovery does not work across machines, you can connect explicitly to the robot:

```bash
# On the robot (Raspberry Pi)
uvx pylekiwi host --serial-port <your_follower_robot_serial_port> --listen-host 0.0.0.0 --listen-port 7447

# On the remote machine, add --host/--port before the client subcommand
uvx --from 'pylekiwi[client]' pylekiwi leader --serial-port <your_leader_robot_serial_port> --host <your robot ip> --port 7447
uvx pylekiwi client --host <your robot ip> --port 7447 state
uvx pylekiwi client --host <your robot ip> --port 7447 capture --camera base --output photo.jpg
uvx pylekiwi client --host <your robot ip> --port 7447 position --x-mm 180 --y-mm 0 --z-mm 120
```
