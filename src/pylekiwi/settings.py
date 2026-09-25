from pydantic import BaseModel, Field

from pylekiwi.models import ArmJointCommand


class Settings(BaseModel):
    command_validity_s: float = Field(default=5.0, gt=0, le=60, allow_inf_nan=False)
    base_command_validity_s: float = Field(default=0.5, gt=0, le=5, allow_inf_nan=False)
    command_query_timeout_s: float = Field(default=3.0, gt=0, allow_inf_nan=False)
    publish_legacy_camera_frames: bool = True
    serial_port: str = "/dev/ttyACM0"
    baudrate: int = 1000000
    timeout: float = 0.5
    base_camera_id: int | str | None = 0
    arm_camera_id: int | str | None = 2
    view_camera: bool = True
    rerun_spawn: bool = True
    zenoh_mode: str | None = None
    zenoh_connect_endpoints: list[str] = Field(default_factory=list)
    zenoh_listen_endpoints: list[str] = Field(default_factory=list)
    zenoh_enable_multicast: bool | None = None
    zenoh_match_timeout: float = 3.0


class Constants(BaseModel):
    DT: float = 0.01
    JOINT_V_MAX: ArmJointCommand = ArmJointCommand(
        joint_angles=(80.0, 80.0, 80.0, 80.0, 80.0),
        gripper_position=80.0,
    )
    JOINT_A_MAX: ArmJointCommand = ArmJointCommand(
        joint_angles=(600.0, 600.0, 600.0, 600.0, 600.0),
        gripper_position=600.0,
    )
    COMMAND_KEY: str = "lekiwi/command"
    COMMAND_LEASE_KEY: str = "lekiwi/control/lease"
    BASE_OBSERVATION_KEY: str = "lekiwi/camera/base/observation"
    ARM_OBSERVATION_KEY: str = "lekiwi/camera/arm/observation"
    ROBOT_STATE_KEY: str = "lekiwi/state"
    ARM_LINKS_KEY: str = "lekiwi/arm/links"
    BASE_CAMERA_KEY: str = "lekiwi/camera/base"
    ARM_CAMERA_KEY: str = "lekiwi/camera/arm"
    ARM_CALIBRATION_KEY: str = "lekiwi/maintenance/arm/calibration"
    CALIBRATION_BACKUP_DIRNAME: str = "calibration-backups"


constants = Constants()
