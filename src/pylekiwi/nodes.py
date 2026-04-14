import time
import threading

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
from pylekiwi.calibration import (
    CalibrationError,
    backup_offsets,
    get_status,
    restore_offsets,
    torque_off_for_manual_pose,
    torque_on_after_manual_pose,
    zero_to_reference_pose,
)
from pylekiwi.camera_controller import CameraController, encode_jpeg
from pylekiwi.models import (
    ArmCalibrationRequest,
    ArmCalibrationResponse,
    ArmJointCommand,
    ArmLinksRequest,
    ArmLinksResponse,
    BaseCommand,
    LekiwiCommand,
    RobotStateResponse,
)
from pylekiwi.settings import Settings, constants
from pylekiwi.smoother import AccelLimitedSmoother
from pylekiwi.zenoh_config import create_zenoh_config


class HostControllerNode:
    """Host controller node that receives commands and sends them to the base and arm controllers.
    """

    def __init__(self, settings: Settings | None = None):
        settings = settings or Settings()
        self._settings = settings
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
        self._arm_smoother: AccelLimitedSmoother | None = None
        self._dt = constants.DT
        self._maintenance_active = False
        self._arm_lock = threading.RLock()

    def _build_status_response_locked(self, message: str) -> ArmCalibrationResponse:
        response = get_status(
            self._arm_controller, serial_port=self._settings.serial_port
        )
        response.message = message
        response.maintenance_active = self._maintenance_active
        return response

    def _build_robot_state_response_locked(self, message: str) -> RobotStateResponse:
        return RobotStateResponse(
            ok=True,
            message=message,
            serial_port=self._settings.serial_port,
            maintenance_active=self._maintenance_active,
            arm_state=self._arm_controller.get_current_state(),
            base_state=self._base_controller.get_current_state(),
        )

    def _build_arm_links_response_locked(
        self, request: ArmLinksRequest
    ) -> ArmLinksResponse:
        current_arm_state = self._arm_controller.get_current_state()
        if request.source == "actual":
            return ArmLinksResponse(
                ok=True,
                source=request.source,
                link_poses=self._arm_controller.get_link_poses(
                    current_arm_state.joint_angles,
                    current_arm_state.gripper_position,
                    request.frame_names or None,
                ),
            )
        if self._target_arm_command is None:
            raise ValueError("No arm command is currently available.")
        return ArmLinksResponse(
            ok=True,
            source=request.source,
            link_poses=self._arm_controller.get_link_poses(
                self._target_arm_command.require_joint_angles(),
                self._target_arm_command.gripper_position
                if self._target_arm_command.gripper_position is not None
                else current_arm_state.gripper_position,
                request.frame_names or None,
            ),
        )

    def _enter_arm_maintenance_locked(self) -> ArmCalibrationResponse:
        self._maintenance_active = True
        response = torque_off_for_manual_pose(
            self._arm_controller, serial_port=self._settings.serial_port
        )
        response.maintenance_active = True
        return response

    def _exit_arm_maintenance_locked(self) -> ArmCalibrationResponse:
        self._reset_arm_smoother_locked()
        response = torque_on_after_manual_pose(
            self._arm_controller, serial_port=self._settings.serial_port
        )
        self._maintenance_active = False
        response.maintenance_active = False
        return response

    def _reset_arm_smoother_locked(self) -> None:
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

    def _resolve_joint_command_locked(
        self, command: ArmJointCommand
    ) -> ArmJointCommand:
        if self._target_arm_command is not None:
            return command.resolved(
                joint_angles=self._target_arm_command.require_joint_angles(),
                gripper_position=self._target_arm_command.gripper_position,
            )

        current_arm_state = self._arm_controller.get_current_state()
        return command.resolved(
            joint_angles=current_arm_state.joint_angles,
            gripper_position=current_arm_state.gripper_position,
        )

    def _handle_arm_calibration_request(
        self, request: ArmCalibrationRequest
    ) -> ArmCalibrationResponse:
        with self._arm_lock:
            if request.action == "torque_off":
                return self._enter_arm_maintenance_locked()

            if request.action == "torque_on":
                return self._exit_arm_maintenance_locked()

            if request.action == "status":
                return self._build_status_response_locked(
                    "Read arm calibration status."
                )

            if request.action == "backup":
                response = backup_offsets(
                    self._arm_controller, serial_port=self._settings.serial_port
                )
                response.maintenance_active = self._maintenance_active
                return response

            if request.action == "zero":
                if request.reference_joint_angles_deg is None:
                    raise CalibrationError(
                        "reference_joint_angles_deg is required for zero."
                    )
                if not self._maintenance_active:
                    raise CalibrationError(
                        "Arm torque must be off before zero calibration. "
                        "Run 'pylekiwi client arm off' first."
                    )
                response = zero_to_reference_pose(
                    self._arm_controller,
                    serial_port=self._settings.serial_port,
                    reference_angles_deg=request.reference_joint_angles_deg,
                )
                response.maintenance_active = self._maintenance_active
                return response

            if request.action == "restore":
                if request.backup_path is None:
                    raise CalibrationError("backup_path is required for restore.")
                if not self._maintenance_active:
                    raise CalibrationError(
                        "Arm torque must be off before restoring offsets. "
                        "Run 'pylekiwi client arm off' first."
                    )
                response = restore_offsets(
                    self._arm_controller,
                    serial_port=self._settings.serial_port,
                    backup_path=request.backup_path,
                )
                response.maintenance_active = self._maintenance_active
                return response

            raise CalibrationError(f"Unsupported calibration action: {request.action}")

    def _listener_arm_calibration_query(self, query: zenoh.Query) -> None:
        with query:
            try:
                if query.payload is None:
                    raise CalibrationError("Missing calibration request payload.")
                request = ArmCalibrationRequest.model_validate_json(
                    query.payload.to_string()
                )
                response = self._handle_arm_calibration_request(request)
            except Exception as e:
                logger.exception("Arm calibration request failed")
                response = ArmCalibrationResponse(
                    ok=False,
                    message=str(e),
                    serial_port=self._settings.serial_port,
                )

            query.reply(
                constants.ARM_CALIBRATION_KEY,
                response.model_dump_json(),
                encoding=zenoh.Encoding.APPLICATION_JSON,
            )

    def _listener_robot_state_query(self, query: zenoh.Query) -> None:
        with query:
            try:
                with self._arm_lock:
                    response = self._build_robot_state_response_locked(
                        "Read current robot state."
                    )
            except Exception as e:
                logger.exception("Robot state request failed")
                response = RobotStateResponse(
                    ok=False,
                    message=str(e),
                    serial_port=self._settings.serial_port,
                    maintenance_active=self._maintenance_active,
                )

            query.reply(
                constants.ROBOT_STATE_KEY,
                response.model_dump_json(),
                encoding=zenoh.Encoding.APPLICATION_JSON,
            )

    def _listener_arm_links_query(self, query: zenoh.Query) -> None:
        with query:
            request: ArmLinksRequest | None = None
            try:
                if query.payload is None:
                    raise ValueError("Missing arm links request payload.")
                request = ArmLinksRequest.model_validate_json(
                    query.payload.to_string()
                )
                with self._arm_lock:
                    response = self._build_arm_links_response_locked(request)
            except Exception as e:
                logger.exception("Arm links request failed")
                response = ArmLinksResponse(
                    ok=False,
                    source=request.source if request is not None else None,
                    error=str(e),
                )

            query.reply(
                constants.ARM_LINKS_KEY,
                response.model_dump_json(),
                encoding=zenoh.Encoding.APPLICATION_JSON,
            )

    def _listener(self, msg: zenoh.Sample) -> zenoh.Reply:
        command: LekiwiCommand = LekiwiCommand.model_validate_json(msg.payload.to_string())
        logger.debug(f"Received command: {command}")
        with self._arm_lock:
            if command.base_command is not None:
                self._base_controller.send_action(command.base_command)
            if command.arm_command is not None:
                if self._maintenance_active:
                    logger.warning("Ignoring arm command while arm maintenance is active.")
                    return
                if command.arm_command.command_type == "joint":
                    self._target_arm_command = self._resolve_joint_command_locked(
                        command.arm_command
                    )
                elif command.arm_command.command_type == "ee_position":
                    self._target_arm_command = (
                        self._arm_controller.resolve_ee_position_action(
                            command.arm_command
                        )
                    )
                elif command.arm_command.command_type == "ee_inching":
                    self._target_arm_command = (
                        self._arm_controller.resolve_ee_inching_action(
                            command.arm_command
                        )
                    )
                else:
                    logger.warning(
                        f"Unsupported arm command type: {command.arm_command.command_type}"
                    )

    def run(self):
        with zenoh.open(create_zenoh_config(self._settings)) as session:
            sub = session.declare_subscriber(constants.COMMAND_KEY, self._listener)
            state_queryable = session.declare_queryable(
                constants.ROBOT_STATE_KEY, self._listener_robot_state_query
            )
            arm_links_queryable = session.declare_queryable(
                constants.ARM_LINKS_KEY, self._listener_arm_links_query
            )
            calibration_queryable = session.declare_queryable(
                constants.ARM_CALIBRATION_KEY, self._listener_arm_calibration_query
            )
            pub_base_cam = session.declare_publisher(constants.BASE_CAMERA_KEY)
            pub_arm_cam = session.declare_publisher(constants.ARM_CAMERA_KEY)
            try:
                with self._arm_lock:
                    self._reset_arm_smoother_locked()
            except Exception as e:
                logger.error(f"Error initializing arm smoother: {e}")
                sub.undeclare()
                state_queryable.undeclare()
                arm_links_queryable.undeclare()
                calibration_queryable.undeclare()
                return
            logger.info("Starting host controller node...")
            try:
                while True:
                    start_time = time.time()
                    with self._arm_lock:
                        if (
                            not self._maintenance_active
                            and self._target_arm_command is not None
                            and self._arm_smoother is not None
                        ):
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
                state_queryable.undeclare()
                arm_links_queryable.undeclare()
                calibration_queryable.undeclare()


class ClientControllerNode:
    """Controller node that publishes commands to the host node.
    """

    def __init__(
        self,
        settings: Settings | None = None,
        *,
        wait_for_matching: bool = False,
    ):
        self.settings = settings or Settings()
        self.session = zenoh.open(create_zenoh_config(self.settings))
        self.publisher = self.session.declare_publisher(constants.COMMAND_KEY)
        self._wait_for_matching = wait_for_matching
        self._matching_checked = not wait_for_matching

    def _ensure_matching(self) -> None:
        if self._matching_checked or self.settings.zenoh_match_timeout <= 0:
            self._matching_checked = True
            return
        deadline = time.time() + self.settings.zenoh_match_timeout
        while time.time() < deadline:
            if self.publisher.matching_status.matching:
                self._matching_checked = True
                return
            time.sleep(0.05)
        raise RuntimeError(
            f"Timed out waiting for a host subscriber on '{constants.COMMAND_KEY}'."
        )

    def send_command(self, command: LekiwiCommand):
        if self._wait_for_matching:
            self._ensure_matching()
        self.publisher.put(command.model_dump_json())

    def send_base_command(self, command: BaseCommand):
        self.send_command(LekiwiCommand(base_command=command))

    def send_arm_joint_command(self, command: ArmJointCommand):
        self.send_command(LekiwiCommand(arm_command=command))

    def close(self) -> None:
        publisher = getattr(self, "publisher", None)
        if publisher is not None:
            try:
                publisher.undeclare()
            except Exception:
                pass
        session = getattr(self, "session", None)
        if session is not None:
            try:
                if not session.is_closed():
                    session.close()
            except Exception:
                pass

    def __del__(self):
        self.close()


class ClientControllerWithCameraNode(ClientControllerNode):
    """Controller node that publishes commands to the host node and receives camera frames.
    """

    def __init__(self, settings: Settings):
        super().__init__(settings=settings)
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

    def close(self) -> None:
        sub_base_cam = getattr(self, "sub_base_cam", None)
        if sub_base_cam is not None:
            try:
                sub_base_cam.undeclare()
            except Exception:
                pass
        sub_arm_cam = getattr(self, "sub_arm_cam", None)
        if sub_arm_cam is not None:
            try:
                sub_arm_cam.undeclare()
            except Exception:
                pass
        super().close()


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
