from pydantic import BaseModel

from pylekiwi.models import ArmJointCommand


class Settings(BaseModel):
    serial_port: str = "/dev/ttyACM0"
    baudrate: int = 1000000
    timeout: float = 0.5
    base_camera_id: int | None = 0
    arm_camera_id: int | None = 2
    view_camera: bool = False
    rerun_spawn: bool = True


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
    BASE_CAMERA_KEY: str = "lekiwi/camera/base"
    ARM_CAMERA_KEY: str = "lekiwi/camera/arm"


constants = Constants()
