# pylekiwi

Python package for controlling the LeKiwi robot.

## Quick Start

Log into the robot and run the following command:

```bash
ssh <your robot ip>
sudo chmod 666 /dev/ttyACM0
uvx launch-lekiwi-webui --serial-port /dev/ttyACM0
```

Then, open a web browser and navigate to `http://<your robot ip>:8080` to see the web UI.
