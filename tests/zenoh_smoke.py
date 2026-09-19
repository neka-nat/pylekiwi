"""Loopback transport smoke test with fake motors/camera; never opens hardware.

Run: uv run python tests/zenoh_smoke.py
"""

import socket
import threading
import time

import numpy as np

from test_control import fake_host
from pylekiwi.models import ArmJointCommand, BaseCommand, ControlEnvelope, LekiwiCommand
from pylekiwi.nodes import ClientControllerWithCameraNode
from pylekiwi.settings import Settings


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("Timed out waiting for loopback test condition")


def main():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    endpoint = f"tcp/127.0.0.1:{port}"
    host = fake_host()
    host._settings = Settings(
        zenoh_mode="peer",
        zenoh_listen_endpoints=[endpoint],
        zenoh_enable_multicast=False,
        base_camera_id=0,
        arm_camera_id=None,
    )
    host._camera_controller.get_base_frame.return_value = np.zeros(
        (12, 16, 3), dtype=np.uint8
    )
    thread = threading.Thread(target=host.run, daemon=True)
    thread.start()
    node = None
    try:
        wait_for(lambda: host._base_controller.stop.call_count == 1)
        node = ClientControllerWithCameraNode(
            Settings(
                zenoh_mode="client",
                zenoh_connect_endpoints=[endpoint],
                zenoh_enable_multicast=False,
                view_camera=False,
            )
        )
        wait_for(lambda: node.get_observation("base", max_age_s=1.0) is not None)
        observation = node.get_observation("base", max_age_s=1.0)
        assert observation.metadata.arm_state is not None
        assert observation.metadata.host_id == host._command_guard.host_id
        command = LekiwiCommand(
            base_command=BaseCommand(x_vel=0.1, y_vel=0, theta_deg_vel=0),
            arm_command=ArmJointCommand(joint_angles=(0.2,) * 5, gripper_position=0.3),
        )
        node.send_command(command, validity_s=0.3)
        wait_for(lambda: host._base_controller.send_action.call_count == 1)
        wait_for(lambda: host._arm_controller.send_joint_action.call_count > 0)
        replay = command.model_copy(
            update={"envelope": ControlEnvelope(lease=node._lease, sequence=0)}
        )
        node.publisher.put(replay.model_dump_json())
        node.close()  # No stop command: the host must expire the active lease.
        node = None
        wait_for(
            lambda: host._base_deadline_ns is None and host._arm_deadline_ns is None
        )
        assert host._base_controller.stop.call_count >= 2
        assert host._base_controller.send_action.call_count == 1
        host._arm_controller.disable_torque.assert_not_called()
        print(
            "PASS: actual Zenoh lease, command, duplicate rejection, timestamped JPEG, disconnect expiry"
        )
    finally:
        if node is not None:
            node.close()
        host.stop()
        thread.join(3.0)
        assert not thread.is_alive(), "Host thread did not stop"


if __name__ == "__main__":
    main()
