# pylekiwi

Python package for controlling the LeKiwi robot.

## Quick Start

### Web UI

Log into the robot and run the following command:

```bash
ssh <your robot ip>
sudo chmod 666 /dev/ttyACM0
uvx launch-lekiwi-webui --serial-port /dev/ttyACM0
```

Then, open a web browser and navigate to `http://<your robot ip>:8080` to see the web UI.

### Leader and Follower Nodes

Run the following command to start the follower node (host) on the robot (Respberry Pi):

```bash
uvx launch-lekiwi-host --serial-port /dev/ttyACM0
```

Run the following command to start the leader node (client) on the remote machine:

```bash
uvx launch-lekiwi-client-leader --serial-port /dev/ttyACM0
```
