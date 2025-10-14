from typing import Literal

from pydantic import BaseModel


class BaseState(BaseModel):
    x_vel: float
    y_vel: float
    theta_deg_vel: float


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

    def __add__(self, other: "ArmJointCommand") -> "ArmJointCommand":
        return ArmJointCommand(
            command_type=self.command_type,
            joint_angles=tuple(
                a + b for a, b in zip(self.joint_angles, other.joint_angles)
            ),
            gripper_position=(
                self.gripper_position + other.gripper_position
                if (
                    self.gripper_position is not None
                    and other.gripper_position is not None
                )
                else self.gripper_position
            ),
        )

    def __sub__(self, other: "ArmJointCommand") -> "ArmJointCommand":
        return ArmJointCommand(
            command_type=self.command_type,
            joint_angles=tuple(
                a - b for a, b in zip(self.joint_angles, other.joint_angles)
            ),
            gripper_position=(
                self.gripper_position - other.gripper_position
                if (
                    self.gripper_position is not None
                    and other.gripper_position is not None
                )
                else self.gripper_position
            ),
        )

    def __mul__(self, other: float) -> "ArmJointCommand":
        return ArmJointCommand(
            command_type=self.command_type,
            joint_angles=tuple(a * other for a in self.joint_angles),
            gripper_position=self.gripper_position * other
            if self.gripper_position is not None
            else self.gripper_position
        )

    def __truediv__(self, other: float) -> "ArmJointCommand":
        return ArmJointCommand(
            command_type=self.command_type,
            joint_angles=tuple(a / other for a in self.joint_angles),
            gripper_position=self.gripper_position / other
            if self.gripper_position is not None
            else self.gripper_position
        )

    def clip(self, lo: "ArmJointCommand", hi: "ArmJointCommand") -> "ArmJointCommand":
        return ArmJointCommand(
            command_type=self.command_type,
            joint_angles=tuple(
                max(lo, min(a, hi))
                for a, lo, hi in zip(
                    self.joint_angles, lo.joint_angles, hi.joint_angles
                )
            ),
            gripper_position=(
                max(
                    lo.gripper_position, min(self.gripper_position, hi.gripper_position)
                )
                if self.gripper_position is not None
                else self.gripper_position
            ),
        )


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
    arm_command: ArmJointCommand | ArmEEPositionCommand | ArmEEInchingCommand | None = (
        None
    )
