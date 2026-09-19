import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from pylekiwi.control import CommandGuard
from pylekiwi.models import (
    ArmEEPositionCommand,
    ArmJointCommand,
    ArmState,
    BaseCommand,
    CommandLeaseRequest,
    ControlEnvelope,
    LekiwiCommand,
)
from pylekiwi.nodes import HostControllerNode
from pylekiwi.settings import Settings


def fake_host():
    host = HostControllerNode.__new__(HostControllerNode)
    host._settings = Settings(base_camera_id=None, arm_camera_id=None)
    host._arm_lock = threading.RLock()
    host._command_guard = CommandGuard()
    host._base_controller = Mock()
    host._arm_controller = Mock()
    host._arm_controller.get_current_state.return_value = ArmState(
        joint_angles=(0.0,) * 5, gripper_position=0.4
    )
    host._camera_controller = Mock()
    host._maintenance_active = False
    host._target_arm_command = None
    host._arm_smoother = None
    host._base_deadline_ns = None
    host._arm_deadline_ns = None
    host._arm_generation = 0
    host._dt = 0.01
    host._stop_event = threading.Event()
    host._reset_arm_smoother_locked()
    return host


def leased(host, *, sequence=0, client_id="test", duration=0.5, base=False, arm=None):
    lease = host._command_guard.issue(
        CommandLeaseRequest(client_id=client_id, validity_s=duration)
    )
    return LekiwiCommand(
        envelope=ControlEnvelope(lease=lease, sequence=sequence),
        base_command=BaseCommand(x_vel=0.1, y_vel=0, theta_deg_vel=0) if base else None,
        arm_command=arm,
    )


class LeaseTests(unittest.TestCase):
    def setUp(self):
        self.now = 1_000_000_000
        self.guard = CommandGuard(clock=lambda: self.now)
        self.lease = self.guard.issue(
            CommandLeaseRequest(client_id="test", validity_s=0.5)
        )

    def command(self, sequence=0):
        return LekiwiCommand(
            envelope=ControlEnvelope(lease=self.lease, sequence=sequence),
            base_command=BaseCommand(x_vel=0.1, y_vel=0, theta_deg_vel=0),
        )

    def test_deadline_is_not_extended_by_delivery(self):
        self.now += 400_000_000
        self.assertEqual(self.guard.validate(self.command()), 1_500_000_000)
        self.now += 100_000_000
        with self.assertRaisesRegex(ValueError, "expired"):
            self.guard.validate(self.command(1))

    def test_duplicate_and_reordered_commands_rejected(self):
        self.guard.validate(self.command(3))
        for sequence in (3, 2):
            with self.assertRaisesRegex(ValueError, "Duplicate"):
                self.guard.validate(self.command(sequence))

    def test_previous_host_and_forged_deadline_rejected(self):
        self.lease = self.lease.model_copy(
            update={"expires_at_monotonic_ns": 9999999999}
        )
        with self.assertRaises(ValueError):
            self.guard.validate(self.command())
        other = CommandGuard(clock=lambda: self.now)
        with self.assertRaises(ValueError):
            other.validate(self.command())

    def test_replaced_lease_and_superseded_preparation_rejected(self):
        self.guard.validate(self.command(0))
        self.guard.validate(self.command(1))
        with self.assertRaisesRegex(ValueError, "superseded"):
            self.guard.validate(self.command(0), reserve=False)
        self.guard.issue(CommandLeaseRequest(client_id="test", validity_s=0.5))
        with self.assertRaisesRegex(ValueError, "replaced"):
            self.guard.validate(self.command(2))

    def test_unbounded_legacy_command_rejected(self):
        with self.assertRaisesRegex(ValueError, "lease required"):
            self.guard.validate(LekiwiCommand())

    def test_arm_lease_cannot_authorize_long_base_motion(self):
        self.lease = self.guard.issue(
            CommandLeaseRequest(client_id="test", validity_s=5)
        )
        with self.assertRaisesRegex(ValueError, "Base command lease"):
            self.guard.validate(self.command())

    def test_client_limit_and_expiry_cleanup(self):
        guard = CommandGuard(clock=lambda: self.now, max_clients=1)
        guard.issue(CommandLeaseRequest(client_id="one", validity_s=0.1))
        with self.assertRaises(ValueError):
            guard.issue(CommandLeaseRequest(client_id="two", validity_s=0.1))
        self.now += 100_000_000
        guard.issue(CommandLeaseRequest(client_id="two", validity_s=0.1))



class HostTests(unittest.TestCase):
    def setUp(self):
        self.host = fake_host()
        self.arm = ArmJointCommand(joint_angles=(0.3,) * 5, gripper_position=0.2)

    def test_expiry_stops_base_and_holds_last_sent_arm_without_torque_off(self):
        command = leased(self.host, base=True, arm=self.arm)
        self.host._handle_command(command)
        self.host._control_step()
        last_sent = self.host._arm_controller.send_joint_action.call_args.args[0]
        with patch(
            "pylekiwi.nodes.time.monotonic_ns",
            return_value=command.envelope.lease.expires_at_monotonic_ns,
        ):
            self.host._control_step()
        self.host._base_controller.stop.assert_called_once()
        self.assertEqual(
            self.host._arm_controller.send_joint_action.call_args.args[0], last_sent
        )
        self.assertEqual(self.host._arm_smoother.v.joint_angles, (0.0,) * 5)
        self.host._arm_controller.disable_torque.assert_not_called()
        calls = self.host._arm_controller.send_joint_action.call_count
        self.host._control_step()
        self.assertEqual(self.host._arm_controller.send_joint_action.call_count, calls)

    def test_arm_command_does_not_refresh_base_expiry(self):
        self.host._handle_command(leased(self.host, base=True))
        base_deadline = self.host._base_deadline_ns
        self.host._handle_command(
            leased(self.host, sequence=1, duration=5, arm=self.arm)
        )
        self.assertEqual(self.host._base_deadline_ns, base_deadline)
        self.assertGreater(self.host._arm_deadline_ns, base_deadline)

    def test_expired_ik_result_never_sets_target(self):
        command = leased(self.host, arm=ArmEEPositionCommand(xyz=(0.1, 0, 0.1)))

        def solve(*args, **kwargs):
            self.host._command_guard.clock = (
                lambda: command.envelope.lease.expires_at_monotonic_ns
            )
            return self.arm

        self.host._arm_controller.resolve_ee_position_action.side_effect = solve
        before = self.host._target_arm_command
        with self.assertRaisesRegex(ValueError, "expired"):
            self.host._handle_command(command)
        self.assertEqual(self.host._target_arm_command, before)
        self.host._arm_controller.send_joint_action.assert_not_called()

    def test_watchdog_runs_while_ik_is_blocked(self):
        base = leased(self.host, base=True, client_id="base")
        self.host._handle_command(base)
        started, release = threading.Event(), threading.Event()

        def solve(*args, **kwargs):
            started.set()
            release.wait(2)
            return self.arm

        self.host._arm_controller.resolve_ee_position_action.side_effect = solve
        command = leased(
            self.host, duration=5, arm=ArmEEPositionCommand(xyz=(0.1, 0, 0.1))
        )
        errors = []

        def run():
            try:
                self.host._handle_command(command)
            except Exception as error:
                errors.append(error)

        thread = threading.Thread(target=run)
        thread.start()
        try:
            self.assertTrue(started.wait(1))
            with patch(
                "pylekiwi.nodes.time.monotonic_ns",
                return_value=base.envelope.lease.expires_at_monotonic_ns,
            ):
                self.host._control_step()
            self.host._base_controller.stop.assert_called_once()
        finally:
            release.set()
            thread.join(2)
        self.assertFalse(errors)

    def test_camera_stall_does_not_block_expiry(self):
        started, release = threading.Event(), threading.Event()

        def read():
            started.set()
            release.wait(2)
            return None

        self.host._camera_controller.get_base_frame.side_effect = read
        command = leased(self.host, base=True)
        self.host._handle_command(command)
        thread = threading.Thread(
            target=self.host._camera_loop, args=("base", Mock(), Mock())
        )
        thread.start()
        try:
            self.assertTrue(started.wait(1))
            with patch(
                "pylekiwi.nodes.time.monotonic_ns",
                return_value=command.envelope.lease.expires_at_monotonic_ns,
            ):
                self.host._control_step()
            self.host._base_controller.stop.assert_called_once()
        finally:
            self.host.stop()
            release.set()
            thread.join(2)

    def test_invalid_ik_does_not_apply_combined_base_command(self):
        self.host._arm_controller.resolve_ee_position_action.side_effect = ValueError(
            "unreachable"
        )
        with self.assertRaisesRegex(ValueError, "unreachable"):
            self.host._handle_command(
                leased(self.host, base=True, arm=ArmEEPositionCommand(xyz=(9, 9, 9)))
            )
        self.host._base_controller.send_action.assert_not_called()

    def test_failed_stop_retries_and_still_holds_arm(self):
        command = leased(self.host, base=True, arm=self.arm)
        self.host._handle_command(command)
        self.host._base_controller.stop.side_effect = [OSError("serial error"), None]
        with (
            patch(
                "pylekiwi.nodes.time.monotonic_ns",
                return_value=command.envelope.lease.expires_at_monotonic_ns,
            ),
            patch("pylekiwi.nodes.logger"),
        ):
            self.host._control_step()
            self.assertIsNone(self.host._arm_deadline_ns)
            self.host._control_step()
        self.assertIsNone(self.host._base_deadline_ns)
        self.assertEqual(self.host._base_controller.stop.call_count, 2)

    def test_maintenance_and_malformed_messages_do_not_move(self):
        self.host._maintenance_active = True
        with self.assertRaisesRegex(ValueError, "maintenance"):
            self.host._handle_command(leased(self.host, base=True, arm=self.arm))
        self.host._listener(
            SimpleNamespace(payload=SimpleNamespace(to_string=lambda: "not-json"))
        )
        self.host._base_controller.send_action.assert_not_called()
        self.host._arm_controller.send_joint_action.assert_not_called()


if __name__ == "__main__":
    unittest.main()
