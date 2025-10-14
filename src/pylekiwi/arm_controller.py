import os

import numpy as np
from loguru import logger
from rustypot import Sts3215PyController

from kinpy import build_serial_chain_from_mjcf

from pylekiwi.models import ArmJointCommand, ArmState
from pylekiwi.settings import Settings


_MODEL_FILE = os.path.join(os.path.dirname(__file__), "data/SO101/so101_new_calib.xml")


class ArmController:
    JOINT_IDS = (1, 2, 3, 4, 5)
    GRIPPER_ID = 6

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

        # self.chain = build_serial_chain_from_mjcf(
        #     open(_MODEL_FILE, "rb").read(),
        #     "gripper",
        #     model_dir=os.path.dirname(_MODEL_FILE),
        # )

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
            joint_angles=tuple(joint_angles), gripper_position=gripper_position
        )

    def send_joint_action(self, action: ArmJointCommand):
        target_ids = list(self.JOINT_IDS)
        if action.gripper_position is not None:
            target_ids += [self.GRIPPER_ID]
        command = list(action.joint_angles)
        if action.gripper_position is not None:
            command += [action.gripper_position]
        self.motor_controller.sync_write_goal_position(target_ids, command)
