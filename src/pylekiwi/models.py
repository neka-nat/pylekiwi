from typing import Literal

from pydantic import BaseModel


class BaseCommand(BaseModel):
    x_vel: float
    y_vel: float
    theta_deg_vel: float


class ArmState(BaseModel):
    joint_angles: tuple[float, float, float, float, float]
    gripper_position: float | None = None


class ArmJointCommand(BaseModel):
    command_type: Literal["joint"] = "joint"
    joint_angles: tuple[float, float, float, float, float]
    gripper_position: float | None = None


class ArmEEPositionCommand(BaseModel):
    command_type: Literal["ee_position"] = "ee_position"
    xyz: tuple[float, float, float]
    gripper_position: float | None = None


class ArmEEInchingCommand(BaseModel):
    command_type: Literal["ee_inching"] = "ee_inching"
    delta_xyz: tuple[float, float, float]
    gripper_position: float | None = None


class LekiwiCommand(BaseModel):
    base_command: BaseCommand | None = None
    arm_command: ArmJointCommand | ArmEEPositionCommand | ArmEEInchingCommand | None = None
