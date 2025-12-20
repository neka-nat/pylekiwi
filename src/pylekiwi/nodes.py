import time

import cv2
import numpy as np
import zenoh
try:
    import rerun as rr
except ImportError:
    rr = None
from collections import deque
from loguru import logger
from rustypot import Sts3215PyController

from pylekiwi.arm_controller import ArmController
from pylekiwi.base_controller import BaseController
from pylekiwi.camera_controller import CameraController, encode_jpeg
from pylekiwi.models import ArmJointCommand, BaseCommand, LekiwiCommand
from pylekiwi.settings import Settings, constants
from pylekiwi.smoother import AccelLimitedSmoother


class HostControllerNode:
    """Host controller node that receives commands and sends them to the base and arm controllers.
    """

    def __init__(self, settings: Settings | None = None):
        settings = settings or Settings()
        motor_controller = Sts3215PyController(
            serial_port=settings.serial_port,
            baudrate=settings.baudrate,
            timeout=settings.timeout,
        )
        self._base_controller = BaseController(motor_controller=motor_controller)
        self._arm_controller = ArmController(motor_controller=motor_controller)
        self._camera_controller = CameraController(
            base_camera_id=settings.base_camera_id,
            arm_camera_id=settings.arm_camera_id,
        )
        self._target_arm_command: ArmJointCommand | None = None
        self._dt = constants.DT

    def _listener(self, msg: zenoh.Sample) -> zenoh.Reply:
        command: LekiwiCommand = LekiwiCommand.model_validate_json(msg.payload.to_string())
        logger.debug(f"Received command: {command}")
        if command.base_command is not None:
            self._base_controller.send_action(command.base_command)
        if (
            command.arm_command is not None
            and command.arm_command.command_type == "joint"
        ):
            self._target_arm_command = command.arm_command

    def run(self):
        with zenoh.open(zenoh.Config()) as session:
            sub = session.declare_subscriber(constants.COMMAND_KEY, self._listener)
            pub_base_cam = session.declare_publisher(constants.BASE_CAMERA_KEY)
            pub_arm_cam = session.declare_publisher(constants.ARM_CAMERA_KEY)
            try:
                current_arm_state = self._arm_controller.get_current_state()
                current_arm_command = ArmJointCommand(
                    joint_angles=current_arm_state.joint_angles,
                    gripper_position=current_arm_state.gripper_position,
                )
                self._arm_smoother = AccelLimitedSmoother(
                    q=current_arm_command,
                    v_max=constants.JOINT_V_MAX,
                    a_max=constants.JOINT_A_MAX,
                    dt=self._dt,
                )
                self._target_arm_command = current_arm_command
            except Exception as e:
                logger.error(f"Error initializing arm smoother: {e}")
                sub.undeclare()
                return
            logger.info("Starting host controller node...")
            try:
                while True:
                    start_time = time.time()
                    if self._target_arm_command is not None:
                        q, _ = self._arm_smoother.step(self._target_arm_command)
                        self._arm_controller.send_joint_action(q)
                    # Publish camera frames
                    base_frame = self._camera_controller.get_base_frame()
                    arm_frame = self._camera_controller.get_arm_frame()
                    if base_frame is not None:
                        pub_base_cam.put(encode_jpeg(base_frame))
                    if arm_frame is not None:
                        pub_arm_cam.put(encode_jpeg(arm_frame))
                    time.sleep(max(0, self._dt - (time.time() - start_time)))
            except KeyboardInterrupt:
                pass
            finally:
                sub.undeclare()


class ClientControllerNode:
    """Controller node that publishes commands to the host node.
    """

    def __init__(self):
        self.session = zenoh.open(zenoh.Config())
        self.publisher = self.session.declare_publisher(constants.COMMAND_KEY)

    def send_command(self, command: LekiwiCommand):
        self.publisher.put(command.model_dump_json())

    def send_base_command(self, command: BaseCommand):
        self.send_command(LekiwiCommand(base_command=command))

    def send_arm_joint_command(self, command: ArmJointCommand):
        self.send_command(LekiwiCommand(arm_command=command))


class ClientControllerWithCameraNode(ClientControllerNode):
    """Controller node that publishes commands to the host node and receives camera frames.
    """

    def __init__(self, settings: Settings):
        super().__init__()
        self.settings = settings
        self.sub_base_cam = self.session.declare_subscriber(constants.BASE_CAMERA_KEY, self._listener_base_cam)
        self.sub_arm_cam = self.session.declare_subscriber(constants.ARM_CAMERA_KEY, self._listener_arm_cam)
        self.base_frame_queue = deque(maxlen=5)
        self.arm_frame_queue = deque(maxlen=5)
        if rr is not None and settings.view_camera:
            rr.init("lekiwi_client_camera", spawn=settings.rerun_spawn)

    def _listener_base_cam(self, msg: zenoh.Sample) -> zenoh.Reply:
        binary_data = bytes(msg.payload)
        image = cv2.imdecode(np.frombuffer(binary_data, dtype=np.uint8), cv2.IMREAD_COLOR)
        self.base_frame_queue.append(image)

    def _listener_arm_cam(self, msg: zenoh.Sample) -> zenoh.Reply:
        binary_data = bytes(msg.payload)
        image = cv2.imdecode(np.frombuffer(binary_data, dtype=np.uint8), cv2.IMREAD_COLOR)
        self.arm_frame_queue.append(image)

    def get_base_frame(self) -> np.ndarray | None:
        return self.base_frame_queue[-1] if len(self.base_frame_queue) > 0 else None

    def get_arm_frame(self) -> np.ndarray | None:
        return self.arm_frame_queue[-1] if len(self.arm_frame_queue) > 0 else None

    def view_camera(self):
        if (
            rr is not None
            and self.settings.view_camera
            and len(self.base_frame_queue) > 0
            and len(self.arm_frame_queue) > 0
        ):
            rr.log("base_camera", rr.Image(self.base_frame_queue[-1][..., ::-1]))
            rr.log("arm_camera", rr.Image(self.arm_frame_queue[-1][..., ::-1]))


class LeaderControllerNode(ClientControllerWithCameraNode):
    """Leader controller node that publishes commands to the host node.
    Arm commands are based on the current leader's arm state.
    Base commands are from the keyboard.
    """

    def __init__(self, settings: Settings | None = None):
        settings = settings or Settings()
        super().__init__(settings=settings)
        try:
            motor_controller = Sts3215PyController(
                serial_port=settings.serial_port,
                baudrate=settings.baudrate,
                timeout=settings.timeout,
            )
            self.arm_controller = ArmController(motor_controller=motor_controller)
        except OSError as e:
            logger.error(f"Error initializing arm controller: {e}")
            self.arm_controller = None

        from pylekiwi.key_listener import KeyListener
        self.key_listener = KeyListener()

    def send_leader_command(self, base_command: BaseCommand | None = None):
        if self.arm_controller is not None:
            arm_state = self.arm_controller.get_current_state()
            arm_command = ArmJointCommand(
                joint_angles=arm_state.joint_angles,
                gripper_position=arm_state.gripper_position,
            )
            if base_command is not None:
                self.send_command(LekiwiCommand(base_command=base_command, arm_command=arm_command))
            else:
                self.send_arm_joint_command(arm_command)
        elif base_command is not None:
            self.send_base_command(base_command)

    def run(self):
        from pynput import keyboard

        with keyboard.Listener(
            on_press=self.key_listener.on_key_press,
            on_release=self.key_listener.on_key_release,
        ):
            while True:
                start_time = time.time()
                self.view_camera()
                self.send_leader_command(base_command=self.key_listener.current_command)
                time.sleep(max(0, constants.DT - (time.time() - start_time)))
