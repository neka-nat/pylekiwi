import os
from copy import deepcopy

import numpy as np
from loguru import logger
from rustypot import Sts3215PyController

from kinpy import Transform, build_serial_chain_from_mjcf

from pylekiwi.models import (
    ArmEEInchingCommand,
    ArmLinkPose,
    ArmEEPositionCommand,
    ArmJointCommand,
    ArmState,
)
from pylekiwi.settings import Settings


_MODEL_FILE = os.path.join(
    os.path.dirname(__file__), "data/SO101/lekiwi_so101_new_calib.xml"
)


class ArmController:
    JOINT_IDS = (1, 2, 3, 4, 5)
    GRIPPER_ID = 6

    @staticmethod
    def _unwrap_single_value(value):
        if isinstance(value, (list, tuple)):
            if len(value) != 1:
                raise ValueError(f"Expected a single value, got {value!r}")
            return value[0]
        return value

    def __init__(self, motor_controller: Sts3215PyController | Settings | None = None):
        if motor_controller is None:
            settings = (
                Settings()
                if not isinstance(motor_controller, Settings)
                else motor_controller
            )
            motor_controller = Sts3215PyController(
                serial_port=settings.serial_port,
                baudrate=settings.baudrate,
                timeout=settings.timeout,
            )
        elif isinstance(motor_controller, Settings):
            motor_controller = Sts3215PyController(
                serial_port=motor_controller.serial_port,
                baudrate=motor_controller.baudrate,
                timeout=motor_controller.timeout,
            )
        self.motor_controller = motor_controller

        self.chain = build_serial_chain_from_mjcf(
            open(_MODEL_FILE, "rb").read(),
            "moving_jaw_so101_v1",
            model_dir=os.path.dirname(_MODEL_FILE),
        )
        self._chain_joint_names = tuple(self.chain.get_joint_parameter_names())

    def forward_kinematics(
        self,
        joint_angles: tuple[float, float, float, float, float],
        gripper_position: float = 0.0,
    ) -> Transform:
        return self.chain.forward_kinematics([*joint_angles, gripper_position])

    def inverse_kinematics(
        self,
        transform: Transform,
        initial_state: np.ndarray | None = None,
        *,
        gripper_position: float = 0.0,
    ) -> tuple[float, float, float, float, float]:
        """Use kinpy IK with the gripper fixed at the commanded opening (radians)."""
        if initial_state is not None:
            initial_state = np.asarray(initial_state, dtype=float)
            if initial_state.shape == (6,):
                initial_state = initial_state[:5]
            if initial_state.shape != (5,) or not np.isfinite(initial_state).all():
                raise ValueError("IK initial_state must contain five finite arm angles.")
        if not np.isfinite(gripper_position) or not np.isfinite(transform.matrix()).all():
            raise ValueError("IK target and gripper position must be finite.")

        # Use a separate chain so FK/link queries retain the real gripper joint.
        ik_chain = deepcopy(self.chain)
        gripper = next(frame for frame in ik_chain if frame.joint.name == "gripper")
        gripper.joint.offset = gripper.get_transform(gripper_position)
        gripper.joint.joint_type = "fixed"
        result = ik_chain.inverse_kinematics(transform, initial_state=initial_state)

        # kinpy returns an approximate solution even when a pose is unreachable.
        actual = ik_chain.forward_kinematics(result)
        rotation = transform.matrix()[:3, :3].T @ actual.matrix()[:3, :3]
        angle_error = np.arccos(np.clip((np.trace(rotation) - 1) / 2, -1, 1))
        if (not np.isfinite(result).all()
                or np.linalg.norm(actual.pos - transform.pos) > 0.001
                or angle_error > np.deg2rad(1.0)):
            raise ValueError("IK target cannot be reached within 1 mm / 1 degree.")
        return tuple(float(v) for v in result)

    def set_torque(self):
        for i in self.JOINT_IDS:
            self.motor_controller.write_torque_enable(i, True)
        self.motor_controller.write_torque_enable(self.GRIPPER_ID, True)

    def disable_torque(self):
        for i in self.JOINT_IDS:
            self.motor_controller.write_torque_enable(i, False)
        self.motor_controller.write_torque_enable(self.GRIPPER_ID, False)

    def get_current_state(self) -> ArmState:
        all_ids = list(self.JOINT_IDS) + [self.GRIPPER_ID]
        joint_angles = self.motor_controller.sync_read_present_position(all_ids)
        gripper_position = joint_angles[-1]
        joint_angles = joint_angles[:-1]
        logger.debug(
            f"Joint angles: {joint_angles}, Gripper position: {gripper_position}"
        )
        return ArmState(
            joint_angles=tuple(joint_angles),
            gripper_position=gripper_position,
            torque_enabled=self.is_arm_torque_enabled(),
        )

    def get_link_frame_names(self) -> list[str]:
        transforms = self.chain.forward_kinematics({}, end_only=False)
        return list(transforms.keys())

    def get_link_poses(
        self,
        joint_angles: tuple[float, float, float, float, float],
        gripper_position: float | None,
        frame_names: list[str] | None = None,
    ) -> list[ArmLinkPose]:
        if gripper_position is None:
            raise ValueError("gripper_position is required to compute link poses.")

        joint_values = dict(
            zip(
                self._chain_joint_names,
                [*joint_angles, gripper_position],
                strict=True,
            )
        )
        transforms = self.chain.forward_kinematics(joint_values, end_only=False)
        selected_frame_names = frame_names or list(transforms.keys())
        missing_frame_names = [
            frame_name
            for frame_name in selected_frame_names
            if frame_name not in transforms
        ]
        if missing_frame_names:
            raise ValueError(
                "Unknown frame names: " + ", ".join(missing_frame_names)
            )

        return [
            ArmLinkPose(
                frame_name=frame_name,
                xyz_m=tuple(float(v) for v in transforms[frame_name].pos),
                quat_wxyz=tuple(float(v) for v in transforms[frame_name].rot),
            )
            for frame_name in selected_frame_names
        ]

    def read_joint_positions(self) -> tuple[float, float, float, float, float]:
        positions = self.motor_controller.sync_read_present_position(list(self.JOINT_IDS))
        return tuple(float(v) for v in positions)

    def read_raw_joint_positions(self) -> tuple[int, int, int, int, int]:
        positions = self.motor_controller.sync_read_raw_present_position(
            list(self.JOINT_IDS)
        )
        return tuple(int(v) for v in positions)

    def read_joint_offsets(self) -> tuple[float, float, float, float, float]:
        offsets = self.motor_controller.sync_read_offset(list(self.JOINT_IDS))
        return tuple(float(v) for v in offsets)

    def read_joint_offset(self, joint_id: int) -> float:
        return float(self._unwrap_single_value(self.motor_controller.read_offset(joint_id)))

    def write_joint_offset(self, joint_id: int, offset: float) -> None:
        self.motor_controller.write_offset(joint_id, offset)

    def read_joint_lock(self, joint_id: int) -> bool:
        return bool(self._unwrap_single_value(self.motor_controller.read_lock(joint_id)))

    def write_joint_lock(self, joint_id: int, locked: bool) -> None:
        self.motor_controller.write_lock(joint_id, locked)

    def read_joint_torque_enabled(self) -> tuple[bool, bool, bool, bool, bool]:
        enabled = self.motor_controller.sync_read_torque_enable(list(self.JOINT_IDS))
        return tuple(bool(v) for v in enabled)

    def read_gripper_torque_enabled(self) -> bool:
        return bool(
            self._unwrap_single_value(
                self.motor_controller.read_torque_enable(self.GRIPPER_ID)
            )
        )

    def is_arm_torque_enabled(self) -> bool:
        return all(self.read_joint_torque_enabled()) and self.read_gripper_torque_enabled()

    def send_joint_action(self, action: ArmJointCommand):
        target_ids = list(self.JOINT_IDS)
        if action.gripper_position is not None:
            target_ids += [self.GRIPPER_ID]
        command = list(action.require_joint_angles())
        if action.gripper_position is not None:
            command += [action.gripper_position]
        self.motor_controller.sync_write_goal_position(target_ids, command)

    def _resolve_ee_target(
        self,
        *,
        target_xyz: np.ndarray,
        gripper_position: float | None,
        current_state: ArmState | None = None,
    ) -> ArmJointCommand:
        current_state = current_state or self.get_current_state()
        opening = (
            gripper_position
            if gripper_position is not None
            else current_state.gripper_position
        )
        if opening is None:
            raise ValueError("Gripper state is required for Cartesian commands.")
        # Preserve the arm's orientation at the commanded opening. Opening the
        # jaw alone must not cause compensating wrist/arm motion.
        current_ee = self.forward_kinematics(current_state.joint_angles, opening)
        target_ee = Transform(
            rot=np.asarray(current_ee.rot, dtype=float),
            pos=target_xyz,
        )
        target_joints = self.inverse_kinematics(
            target_ee,
            initial_state=np.asarray(current_state.joint_angles, dtype=float),
            gripper_position=opening,
        )
        return ArmJointCommand(
            joint_angles=target_joints,
            gripper_position=opening,
        )

    def resolve_ee_position_action(
        self, action: ArmEEPositionCommand, *, current_state: ArmState | None = None
    ) -> ArmJointCommand:
        return self._resolve_ee_target(
            target_xyz=np.asarray(action.xyz, dtype=float),
            gripper_position=action.gripper_position,
            current_state=current_state,
        )

    def send_ee_position_action(self, action: ArmEEPositionCommand):
        self.send_joint_action(self.resolve_ee_position_action(action))

    def resolve_ee_inching_action(
        self, action: ArmEEInchingCommand, *, current_state: ArmState | None = None
    ) -> ArmJointCommand:
        current_state = current_state or self.get_current_state()
        if current_state.gripper_position is None:
            raise ValueError("Gripper state is required for Cartesian commands.")
        current_ee = self.forward_kinematics(
            current_state.joint_angles, current_state.gripper_position
        )
        return self._resolve_ee_target(
            target_xyz=np.asarray(current_ee.pos, dtype=float)
            + np.asarray(action.delta_xyz, dtype=float),
            gripper_position=action.gripper_position,
            current_state=current_state,
        )

    def send_ee_inching_action(self, action: ArmEEInchingCommand):
        self.send_joint_action(self.resolve_ee_inching_action(action))
