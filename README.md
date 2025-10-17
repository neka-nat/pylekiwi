# pylekiwi

Python package for controlling the LeKiwi robot.

## Quick Start

### Web UI

Log into the robot and run the following command:

```bash
ssh <your robot ip>
sudo chmod 666 <your_follower_robot_serial_port>
uvx pylekiwi webui --serial-port <your_follower_robot_serial_port>
```

Then, open a web browser and navigate to `http://<your robot ip>:8080` to see the web UI.

### Leader and Follower Nodes

Run the following command to start the follower node (host) on the robot (Respberry Pi):

```bash
ssh <your robot ip>
sudo chmod 666 <your_follower_robot_serial_port>
uvx pylekiwi host --serial-port <your_follower_robot_serial_port>
```

Run the following command to start the leader node (client) on the remote machine:

```bash
sudo chmod 666 <your_leader_robot_serial_port>
uvx pylekiwi leader --serial-port <your_leader_robot_serial_port>
```
