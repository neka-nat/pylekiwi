from typing import Literal

from pydantic import BaseModel, Field


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


class ArmCalibrationJointState(BaseModel):
    joint_id: int
    raw_present_position: int
    present_angle_deg: float
    offset_deg: float


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

    def __add__(self, other: "ArmEEInchingCommand") -> "ArmEEInchingCommand":
        return ArmEEInchingCommand(
            command_type=self.command_type,
            delta_xyz=tuple(a + b for a, b in zip(self.delta_xyz, other.delta_xyz)),
            gripper_position=self.gripper_position + other.gripper_position
            if self.gripper_position is not None
            else self.gripper_position
        )

    def __sub__(self, other: "ArmEEInchingCommand") -> "ArmEEInchingCommand":
        return ArmEEInchingCommand(
            command_type=self.command_type,
            delta_xyz=tuple(a - b for a, b in zip(self.delta_xyz, other.delta_xyz)),
            gripper_position=self.gripper_position - other.gripper_position
            if self.gripper_position is not None
            else self.gripper_position
        )

    def __mul__(self, other: float) -> "ArmEEInchingCommand":
        return ArmEEInchingCommand(
            command_type=self.command_type,
            delta_xyz=tuple(a * other for a in self.delta_xyz),
            gripper_position=self.gripper_position * other
            if self.gripper_position is not None
            else self.gripper_position
        )

    def __truediv__(self, other: float) -> "ArmEEInchingCommand":
        return ArmEEInchingCommand(
            command_type=self.command_type,
            delta_xyz=tuple(a / other for a in self.delta_xyz),
            gripper_position=self.gripper_position / other
            if self.gripper_position is not None
            else self.gripper_position
        )

    def clip(self, lo: "ArmEEInchingCommand", hi: "ArmEEInchingCommand") -> "ArmEEInchingCommand":
        return ArmEEInchingCommand(
            command_type=self.command_type,
            delta_xyz=tuple(max(lo, min(a, hi)) for a, lo, hi in zip(self.delta_xyz, lo.delta_xyz, hi.delta_xyz)),
            gripper_position=max(lo.gripper_position, min(self.gripper_position, hi.gripper_position))
            if self.gripper_position is not None
            else self.gripper_position
        )


class ArmCalibrationRequest(BaseModel):
    action: Literal["status", "backup", "zero", "restore"]
    reference_joint_angles_deg: tuple[float, float, float, float, float] | None = None
    backup_path: str | None = None


class ArmCalibrationResponse(BaseModel):
    ok: bool
    message: str
    serial_port: str | None = None
    joint_states: list[ArmCalibrationJointState] = Field(default_factory=list)
    backup_path: str | None = None
    before_offsets_deg: tuple[float, float, float, float, float] | None = None
    after_offsets_deg: tuple[float, float, float, float, float] | None = None
    present_joint_angles_deg: tuple[float, float, float, float, float] | None = None
    reference_joint_angles_deg: tuple[float, float, float, float, float] | None = None
    verified: bool | None = None


class LekiwiCommand(BaseModel):
    base_command: BaseCommand | None = None
    arm_command: ArmJointCommand | ArmEEPositionCommand | ArmEEInchingCommand | None = (
        None
    )
