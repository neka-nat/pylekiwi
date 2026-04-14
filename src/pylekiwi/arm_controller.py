import os

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

    def forward_kinematics(self, joint_angles: tuple[float, float, float, float, float]) -> Transform:
        return self.chain.forward_kinematics(list(joint_angles) + [0.0])  # add gripper joint angle

    def inverse_kinematics(
        self,
        transform: Transform,
        initial_state: np.ndarray | None = None,
    ) -> tuple[float, float, float, float, float]:
        if len(initial_state) == len(self.JOINT_IDS):
            initial_state = np.r_[initial_state, 0]
        return tuple(
            float(v)
            for v in self.chain.inverse_kinematics(
                transform,
                initial_state=initial_state,
            )[:-1]   # exclude gripper joint
        )

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
    ) -> ArmJointCommand:
        current_state = self.get_current_state()
        current_ee = self.forward_kinematics(current_state.joint_angles)
        target_ee = Transform(
            rot=np.asarray(current_ee.rot, dtype=float),
            pos=target_xyz,
        )
        target_joints = self.inverse_kinematics(
            target_ee,
            initial_state=np.asarray(current_state.joint_angles, dtype=float),
        )
        return ArmJointCommand(
            joint_angles=target_joints,
            gripper_position=(
                gripper_position
                if gripper_position is not None
                else current_state.gripper_position
            ),
        )

    def resolve_ee_position_action(
        self, action: ArmEEPositionCommand
    ) -> ArmJointCommand:
        return self._resolve_ee_target(
            target_xyz=np.asarray(action.xyz, dtype=float),
            gripper_position=action.gripper_position,
        )

    def send_ee_position_action(self, action: ArmEEPositionCommand):
        self.send_joint_action(self.resolve_ee_position_action(action))

    def resolve_ee_inching_action(
        self, action: ArmEEInchingCommand
    ) -> ArmJointCommand:
        current_state = self.get_current_state()
        current_ee = self.forward_kinematics(current_state.joint_angles)
        return self._resolve_ee_target(
            target_xyz=np.asarray(current_ee.pos, dtype=float)
            + np.asarray(action.delta_xyz, dtype=float),
            gripper_position=action.gripper_position,
        )

    def send_ee_inching_action(self, action: ArmEEInchingCommand):
        self.send_joint_action(self.resolve_ee_inching_action(action))
