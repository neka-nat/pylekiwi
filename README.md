# pylekiwi

Python package for controlling the LeKiwi robot.

## Quick Start

Log into the robot and run the following command:

```bash
ssh <your robot ip>
sudo chmod 666 /dev/ttyACM0
uvx launch-lekiwi-host --serial-port /dev/ttyACM0
```
