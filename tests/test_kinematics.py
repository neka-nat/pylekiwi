import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np
from kinpy import Transform, build_serial_chain_from_mjcf

from pylekiwi.arm_controller import ArmController, _MODEL_FILE
from pylekiwi.models import ArmEEInchingCommand, ArmState


def make_controller(motor=None):
    # Test real body/joint transforms independently of optional display meshes.
    # The production MJCF currently references a missing camera mesh; changing
    # model loading is outside the gripper IK regression being tested here.
    model = ET.parse(_MODEL_FILE)
    for parent in model.iter():
        for child in list(parent):
            if child.tag in ("asset", "geom"):
                parent.remove(child)
    chain = build_serial_chain_from_mjcf(
        ET.tostring(model.getroot()), "moving_jaw_so101_v1",
        model_dir=str(Path(_MODEL_FILE).parent),
    )
    with patch("pylekiwi.arm_controller.build_serial_chain_from_mjcf", return_value=chain):
        return ArmController(motor if motor is not None else Mock())


class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller = make_controller()  # No serial connection.
        cls.q = np.array([0.2, -0.4, 0.5, -0.3, 0.1])

    def assert_pose_close(self, expected, actual):
        self.assertLess(np.linalg.norm(expected.pos - actual.pos), 0.001)
        rotation = expected.matrix()[:3, :3].T @ actual.matrix()[:3, :3]
        error = np.arccos(np.clip((np.trace(rotation) - 1) / 2, -1, 1))
        self.assertLess(error, np.deg2rad(1))

    def test_roundtrip_at_different_openings(self):
        for opening in (0.0, 0.4, 1.2):
            with self.subTest(opening=opening):
                target = self.controller.forward_kinematics(tuple(self.q), opening)
                result = self.controller.inverse_kinematics(
                    target, self.q + 0.02, gripper_position=opening
                )
                self.assertEqual(len(result), 5)
                self.assert_pose_close(
                    target, self.controller.forward_kinematics(result, opening)
                )

    def test_inexact_one_cm_solution_is_rejected(self):
        target = self.controller.forward_kinematics(tuple(self.q))
        target.pos += [0.01, 0, 0]
        with self.assertRaisesRegex(ValueError, "1 mm / 1 degree"):
            self.controller.inverse_kinematics(target, self.q, gripper_position=0.0)

    def test_solver_uses_five_joints_without_mutating_the_original_chain(self):
        from kinpy.chain import SerialChain
        original = SerialChain.inverse_kinematics
        calls = []

        def solve(chain, target, initial_state=None):
            calls.append(chain.get_joint_parameter_names())
            return original(chain, target, initial_state)

        target = self.controller.forward_kinematics(tuple(self.q), 0.4)
        before = self.controller.chain.forward_kinematics([*self.q, 0.9])
        with patch.object(SerialChain, "inverse_kinematics", solve):
            self.controller.inverse_kinematics(target, self.q, gripper_position=0.4)
        self.assertEqual(len(calls), 1)
        self.assertEqual(len(calls[0]), 5)
        self.assertNotIn("gripper", calls[0])
        self.assertIn("gripper", self.controller.chain.get_joint_parameter_names())
        after = self.controller.chain.forward_kinematics([*self.q, 0.9])
        np.testing.assert_array_equal(before.matrix(), after.matrix())

    def test_none_initial_state_and_old_six_element_seed(self):
        target = self.controller.forward_kinematics((0.0,) * 5, 0.3)
        for seed in (None, np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.7])):
            result = self.controller.inverse_kinematics(
                target, seed, gripper_position=0.3
            )
            self.assert_pose_close(
                target, self.controller.forward_kinematics(result, 0.3)
            )

    def test_unreachable_or_invalid_target_raises(self):
        for target in (Transform(pos=[10, 0, 0]), Transform(pos=[float("nan"), 0, 0])):
            with self.assertRaises(ValueError):
                self.controller.inverse_kinematics(target, self.q)
        target = self.controller.forward_kinematics(tuple(self.q))
        with self.assertRaises(ValueError):
            self.controller.inverse_kinematics(target, self.q, gripper_position=float("nan"))
        with self.assertRaises(ValueError):
            self.controller.inverse_kinematics(target, np.array([1.0, 2.0]))

    def test_opening_change_does_not_compensate_with_arm_rotation(self):
        state = ArmState(joint_angles=tuple(self.q), gripper_position=0.4)
        command = self.controller.resolve_ee_inching_action(
            ArmEEInchingCommand(delta_xyz=(0, 0, 0), gripper_position=0.9),
            current_state=state,
        )
        np.testing.assert_allclose(command.joint_angles, self.q, atol=1e-7)
        self.assertEqual(command.gripper_position, 0.9)

    def test_inching_uses_one_state_sample(self):
        motor = Mock()
        motor.sync_read_present_position.return_value = [*self.q, 0.4]
        motor.sync_read_torque_enable.return_value = [True] * 5
        motor.read_torque_enable.return_value = True
        controller = make_controller(motor)
        controller.resolve_ee_inching_action(ArmEEInchingCommand(delta_xyz=(0, 0, 0)))
        motor.sync_read_present_position.assert_called_once()
        motor.sync_write_goal_position.assert_not_called()


if __name__ == "__main__":
    unittest.main()
