from typing import Literal

from pydantic import BaseModel, Field, model_validator


JointAngles = tuple[float, float, float, float, float]


class BaseState(BaseModel):
    x_vel: float
    y_vel: float
    theta_deg_vel: float
    torque_enabled: bool | None = None


class BaseCommand(BaseModel):
    x_vel: float
    y_vel: float
    theta_deg_vel: float


class ArmState(BaseModel):
    joint_angles: JointAngles
    gripper_position: float | None = None
    torque_enabled: bool | None = None


class ArmLinkPose(BaseModel):
    frame_name: str
    xyz_m: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]


class ArmCalibrationJointState(BaseModel):
    joint_id: int
    raw_present_position: int
    present_angle_deg: float
    offset_deg: float


class ArmJointCommand(BaseModel):
    command_type: Literal["joint"] = "joint"
    joint_angles: JointAngles | None
    gripper_position: float | None = None

    def resolved(
        self,
        *,
        joint_angles: JointAngles,
        gripper_position: float | None,
    ) -> "ArmJointCommand":
        return ArmJointCommand(
            command_type=self.command_type,
            joint_angles=(
                self.joint_angles if self.joint_angles is not None else joint_angles
            ),
            gripper_position=(
                self.gripper_position
                if self.gripper_position is not None
                else gripper_position
            ),
        )

    def require_joint_angles(self) -> JointAngles:
        if self.joint_angles is None:
            raise ValueError(
                "ArmJointCommand joint_angles must be resolved before use."
            )
        return self.joint_angles

    def __add__(self, other: "ArmJointCommand") -> "ArmJointCommand":
        self_joint_angles = self.require_joint_angles()
        other_joint_angles = other.require_joint_angles()
        return ArmJointCommand(
            command_type=self.command_type,
            joint_angles=tuple(
                a + b for a, b in zip(self_joint_angles, other_joint_angles)
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
        self_joint_angles = self.require_joint_angles()
        other_joint_angles = other.require_joint_angles()
        return ArmJointCommand(
            command_type=self.command_type,
            joint_angles=tuple(
                a - b for a, b in zip(self_joint_angles, other_joint_angles)
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
        self_joint_angles = self.require_joint_angles()
        return ArmJointCommand(
            command_type=self.command_type,
            joint_angles=tuple(a * other for a in self_joint_angles),
            gripper_position=self.gripper_position * other
            if self.gripper_position is not None
            else self.gripper_position
        )

    def __truediv__(self, other: float) -> "ArmJointCommand":
        self_joint_angles = self.require_joint_angles()
        return ArmJointCommand(
            command_type=self.command_type,
            joint_angles=tuple(a / other for a in self_joint_angles),
            gripper_position=self.gripper_position / other
            if self.gripper_position is not None
            else self.gripper_position
        )

    def clip(self, lo: "ArmJointCommand", hi: "ArmJointCommand") -> "ArmJointCommand":
        self_joint_angles = self.require_joint_angles()
        lo_joint_angles = lo.require_joint_angles()
        hi_joint_angles = hi.require_joint_angles()
        return ArmJointCommand(
            command_type=self.command_type,
            joint_angles=tuple(
                max(lo, min(a, hi))
                for a, lo, hi in zip(
                    self_joint_angles, lo_joint_angles, hi_joint_angles
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
    action: Literal["status", "backup", "zero", "restore", "torque_off", "torque_on"]
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
    maintenance_active: bool | None = None
    torque_enabled: bool | None = None
    verified: bool | None = None


class RobotStateRequest(BaseModel):
    pass


class ArmLinksRequest(BaseModel):
    source: Literal["actual", "command"]
    frame_names: list[str] = Field(default_factory=list)


class ArmLinksResponse(BaseModel):
    ok: bool
    source: Literal["actual", "command"] | None = None
    link_poses: list[ArmLinkPose] = Field(default_factory=list)
    error: str | None = None


class RobotStateResponse(BaseModel):
    ok: bool
    message: str
    serial_port: str | None = None
    maintenance_active: bool | None = None
    arm_state: ArmState | None = None
    base_state: BaseState | None = None


class CommandLeaseRequest(BaseModel):
    client_id: str = Field(min_length=1, max_length=128)
    validity_s: float = Field(gt=0, le=60, allow_inf_nan=False)


class CommandLease(BaseModel):
    host_id: str = Field(min_length=1, max_length=128)
    client_id: str = Field(min_length=1, max_length=128)
    lease_id: str = Field(min_length=1, max_length=128)
    issued_at_monotonic_ns: int = Field(ge=0)
    expires_at_monotonic_ns: int = Field(gt=0)

    @model_validator(mode="after")
    def ordered_times(self):
        if self.expires_at_monotonic_ns <= self.issued_at_monotonic_ns:
            raise ValueError("Lease expiry must follow issue time.")
        return self


class CommandLeaseResponse(BaseModel):
    ok: bool
    lease: CommandLease | None = None
    error: str | None = None


class ControlEnvelope(BaseModel):
    lease: CommandLease
    sequence: int = Field(ge=0)


class CameraFrameMetadata(BaseModel):
    schema_version: Literal[1] = 1
    host_id: str
    camera: Literal["base", "arm"]
    frame_id: int = Field(ge=0)
    # OpenCV does not expose a reliable sensor exposure timestamp. These are
    # host-side read times, and do not bound time spent in driver buffering.
    timestamp_source: Literal["opencv_read"] = "opencv_read"
    read_started_monotonic_ns: int = Field(ge=0)
    read_completed_monotonic_ns: int = Field(ge=0)
    arm_state: ArmState | None = None
    arm_state_started_monotonic_ns: int | None = Field(default=None, ge=0)
    arm_state_completed_monotonic_ns: int | None = Field(default=None, ge=0)

    @model_validator(mode="after")
    def ordered_times(self):
        if self.read_completed_monotonic_ns < self.read_started_monotonic_ns:
            raise ValueError("Camera read timestamps are out of order.")
        times = (
            self.arm_state_started_monotonic_ns,
            self.arm_state_completed_monotonic_ns,
        )
        if self.arm_state is None:
            if any(value is not None for value in times):
                raise ValueError("State timestamps require an arm state.")
        elif any(value is None for value in times) or times[0] > times[1]:
            raise ValueError("Arm state requires ordered sampling timestamps.")
        return self


class LekiwiCommand(BaseModel):
    envelope: ControlEnvelope | None = None
    base_command: BaseCommand | None = None
    arm_command: ArmJointCommand | ArmEEPositionCommand | ArmEEInchingCommand | None = (
        None
    )
