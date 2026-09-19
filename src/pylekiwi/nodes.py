import time
import threading
from uuid import uuid4

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
    CameraFrameMetadata,
    CommandLeaseRequest,
    CommandLeaseResponse,
    ControlEnvelope,
    LekiwiCommand,
    RobotStateResponse,
)
from pylekiwi.control import CommandGuard
from pylekiwi.observation import (
    CameraObservation,
    decode_observation,
    encode_observation,
)
from pylekiwi.settings import Settings, constants
from pylekiwi.smoother import AccelLimitedSmoother
from pylekiwi.zenoh_config import create_zenoh_config


class HostControllerNode:
    """Host controller node that receives commands and sends them to the base and arm controllers."""

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
        # All serial I/O (arm AND base) shares this lock. Camera reads and IK
        # optimization must never hold it.
        self._arm_lock = threading.RLock()
        self._command_guard = CommandGuard(
            max_validity_s=settings.command_validity_s,
            base_max_validity_s=settings.base_command_validity_s,
        )
        self._base_deadline_ns: int | None = None
        self._arm_deadline_ns: int | None = None
        self._arm_generation = 0
        self._stop_event = threading.Event()

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
        self._arm_deadline_ns = None
        self._target_arm_command = None
        self._arm_generation += 1
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
        self._last_arm_command = current_arm_command
        self._arm_deadline_ns = None
        self._arm_generation += 1

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

    def _listener_command_lease_query(self, query: zenoh.Query) -> None:
        with query:
            try:
                if query.payload is None:
                    raise ValueError("Missing command lease request.")
                request = CommandLeaseRequest.model_validate_json(
                    query.payload.to_string()
                )
                with self._arm_lock:
                    lease = self._command_guard.issue(request)
                response = CommandLeaseResponse(ok=True, lease=lease)
            except Exception as error:
                response = CommandLeaseResponse(ok=False, error=str(error))
            query.reply(
                constants.COMMAND_LEASE_KEY,
                response.model_dump_json(),
                encoding=zenoh.Encoding.APPLICATION_JSON,
            )

    def _hold_arm_locked(self) -> None:
        # Retain the last commanded position and reset smoother velocity. Do not
        # disable torque (which could drop a held object), or continue to the old target.
        if self._arm_smoother is not None and not self._maintenance_active:
            hold = self._last_arm_command
            self._arm_smoother.q = hold
            self._target_arm_command = hold
            self._arm_smoother.v = hold - hold
            self._arm_controller.send_joint_action(hold)
        self._arm_deadline_ns = None
        self._arm_generation += 1

    def _expire_commands_locked(self) -> None:
        now = time.monotonic_ns()
        if self._base_deadline_ns is not None and now >= self._base_deadline_ns:
            try:
                self._base_controller.stop()
                self._base_deadline_ns = None
            except Exception:
                logger.exception("Failed to stop expired base command; will retry")
        if self._arm_deadline_ns is not None and now >= self._arm_deadline_ns:
            try:
                self._hold_arm_locked()
            except Exception:
                logger.exception("Failed to hold expired arm command; will retry")

    def _handle_command(self, command: LekiwiCommand) -> None:
        with self._arm_lock:
            if self._stop_event.is_set():
                raise ValueError("Host is stopping.")
            self._expire_commands_locked()
            self._command_guard.validate(command)
            if command.arm_command is not None and self._maintenance_active:
                raise ValueError("Arm maintenance is active.")
            generation = self._arm_generation
            arm_command = command.arm_command
            state = None
            if arm_command is not None:
                if arm_command.command_type == "joint":
                    arm_command = self._resolve_joint_command_locked(arm_command)
                else:
                    state = self._arm_controller.get_current_state()

        # The five-variable optimizer can take longer than a command's lease.
        # Motor control/watchdog continues while this callback computes.
        if arm_command is not None and state is not None:
            if arm_command.command_type == "ee_position":
                arm_command = self._arm_controller.resolve_ee_position_action(
                    arm_command, current_state=state
                )
            else:
                arm_command = self._arm_controller.resolve_ee_inching_action(
                    arm_command, current_state=state
                )

        with self._arm_lock:
            if self._stop_event.is_set():
                raise ValueError("Host is stopping.")
            self._expire_commands_locked()
            deadline = self._command_guard.validate(command, reserve=False)
            if command.arm_command is not None and (
                self._maintenance_active or generation != self._arm_generation
            ):
                raise ValueError("Arm state changed while preparing the command.")
            if command.base_command is not None:
                # Set deadline BEFORE I/O so a partially failed write is stopped too.
                self._base_deadline_ns = deadline
                self._base_controller.send_action(command.base_command)
            # Serial I/O itself may have consumed the remainder of the lease.
            if time.monotonic_ns() >= deadline:
                self._expire_commands_locked()
                raise ValueError("Command expired during motor I/O.")
            if arm_command is not None:
                self._target_arm_command = arm_command
                self._arm_deadline_ns = deadline
                self._arm_generation += 1

    def _listener(self, msg: zenoh.Sample) -> None:
        try:
            command = LekiwiCommand.model_validate_json(msg.payload.to_string())
            self._handle_command(command)
        except Exception as error:
            logger.warning(f"Rejected robot command: {error}")

    def _control_step(self) -> None:
        with self._arm_lock:
            self._expire_commands_locked()
            if (
                not self._maintenance_active
                and self._arm_deadline_ns is not None
                and time.monotonic_ns() < self._arm_deadline_ns
                and self._target_arm_command is not None
                and self._arm_smoother is not None
            ):
                q, _ = self._arm_smoother.step(self._target_arm_command)
                self._arm_controller.send_joint_action(q)
                self._last_arm_command = q

    def _camera_loop(self, camera: str, publisher, legacy_publisher) -> None:
        get_frame = (
            self._camera_controller.get_base_frame
            if camera == "base"
            else self._camera_controller.get_arm_frame
        )
        frame_id = 0
        while not self._stop_event.is_set():
            try:
                # A nearby, explicitly timestamped state sample, NOT an assertion
                # that this is the exact arm pose at sensor exposure time.
                with self._arm_lock:
                    state_start = time.monotonic_ns()
                    state = self._arm_controller.get_current_state()
                    state_end = time.monotonic_ns()
                read_start = time.monotonic_ns()
                frame = get_frame()
                read_end = time.monotonic_ns()
                if frame is not None:
                    metadata = CameraFrameMetadata(
                        host_id=self._command_guard.host_id,
                        camera=camera,
                        frame_id=frame_id,
                        read_started_monotonic_ns=read_start,
                        read_completed_monotonic_ns=read_end,
                        arm_state=state,
                        arm_state_started_monotonic_ns=state_start,
                        arm_state_completed_monotonic_ns=state_end,
                    )
                    frame_id += 1
                    jpeg = encode_jpeg(frame)
                    publisher.put(encode_observation(metadata, jpeg))
                    if self._settings.publish_legacy_camera_frames:
                        legacy_publisher.put(jpeg)
            except Exception:
                logger.exception(f"Failed to publish {camera} camera observation")
            self._stop_event.wait(0.03)

    def run(self):
        self._stop_event.clear()
        with zenoh.open(create_zenoh_config(self._settings)) as session:
            handles = []
            camera_threads = []
            try:
                with self._arm_lock:
                    self._base_controller.stop()
                    self._reset_arm_smoother_locked()
                handles.append(
                    session.declare_subscriber(constants.COMMAND_KEY, self._listener)
                )
                for key, callback in (
                    (constants.COMMAND_LEASE_KEY, self._listener_command_lease_query),
                    (constants.ROBOT_STATE_KEY, self._listener_robot_state_query),
                    (constants.ARM_LINKS_KEY, self._listener_arm_links_query),
                    (
                        constants.ARM_CALIBRATION_KEY,
                        self._listener_arm_calibration_query,
                    ),
                ):
                    handles.append(session.declare_queryable(key, callback))
                for camera, key, legacy_key, enabled in (
                    (
                        "base",
                        constants.BASE_OBSERVATION_KEY,
                        constants.BASE_CAMERA_KEY,
                        self._settings.base_camera_id is not None,
                    ),
                    (
                        "arm",
                        constants.ARM_OBSERVATION_KEY,
                        constants.ARM_CAMERA_KEY,
                        self._settings.arm_camera_id is not None,
                    ),
                ):
                    if not enabled:
                        continue
                    publisher = session.declare_publisher(key)
                    legacy = session.declare_publisher(legacy_key)
                    handles.extend((publisher, legacy))
                    thread = threading.Thread(
                        target=self._camera_loop,
                        args=(camera, publisher, legacy),
                        daemon=True,
                    )
                    thread.start()
                    camera_threads.append(thread)
                logger.info("Starting host controller node...")
                while not self._stop_event.is_set():
                    started = time.monotonic()
                    try:
                        self._control_step()
                    except Exception:
                        logger.exception("Motor control step failed")
                    self._stop_event.wait(
                        max(0, self._dt - (time.monotonic() - started))
                    )
            except KeyboardInterrupt:
                pass
            finally:
                self._stop_event.set()
                with self._arm_lock:
                    try:
                        self._base_controller.stop()
                    except Exception:
                        logger.exception("Failed to stop base during shutdown")
                    try:
                        self._hold_arm_locked()
                    except Exception:
                        logger.exception("Failed to hold arm during shutdown")
                for thread in camera_threads:
                    thread.join(timeout=1.0)
                for handle in reversed(handles):
                    handle.undeclare()

    def stop(self) -> None:
        self._stop_event.set()


class ClientControllerNode:
    """Controller node that publishes commands to the host node."""

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
        self._client_id = str(uuid4())
        self._command_sequence = 0
        self._lease = None
        self._lease_refresh_at = 0.0
        self._lease_validity_s = None
        self._send_lock = threading.Lock()

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

    def _get_command_lease(self, validity_s: float):
        now = time.monotonic()
        if (
            self._lease is not None
            and now < self._lease_refresh_at
            and self._lease_validity_s == validity_s
        ):
            return self._lease
        request = CommandLeaseRequest(client_id=self._client_id, validity_s=validity_s)
        started = time.monotonic()
        replies = self.session.get(
            constants.COMMAND_LEASE_KEY,
            payload=request.model_dump_json(),
            encoding=zenoh.Encoding.APPLICATION_JSON,
            timeout=self.settings.command_query_timeout_s,
        )
        error = "No command-lease response; update the host to a lease-capable version."
        for reply in replies:
            if not reply.ok:
                continue
            response = CommandLeaseResponse.model_validate_json(
                reply.ok.payload.to_string()
            )
            if response.ok and response.lease is not None:
                lease = response.lease
                if lease.client_id != self._client_id:
                    raise RuntimeError("Host returned a lease for a different client.")
                # Local duration only; never compare client and host clock values.
                # A delayed reply is discarded without publishing a motor command.
                remaining = (
                    lease.expires_at_monotonic_ns - lease.issued_at_monotonic_ns
                ) / 1e9
                if time.monotonic() - started >= remaining:
                    raise RuntimeError(
                        "Command lease expired while waiting for the host."
                    )
                self._lease = lease
                self._lease_validity_s = validity_s
                self._lease_refresh_at = started + remaining / 2
                return lease
            error = response.error or "Host refused command lease."
        raise RuntimeError(error)

    def send_command(self, command: LekiwiCommand, *, validity_s: float | None = None):
        """Publish with a host-issued deadline; repeated calls are new commands.

        The publication is not an execution/completion acknowledgement. Reused
        leases retain their original expiry, so validity_s is an upper bound.
        """
        with self._send_lock:
            if self._wait_for_matching:
                self._ensure_matching()
            if command.envelope is not None:
                raise ValueError(
                    "send_command creates its own envelope; pass a plain command."
                )
            duration = (
                validity_s
                if validity_s is not None
                else (
                    self.settings.base_command_validity_s
                    if command.base_command is not None
                    else self.settings.command_validity_s
                )
            )
            lease = self._get_command_lease(duration)
            envelope = ControlEnvelope(lease=lease, sequence=self._command_sequence)
            self._command_sequence += 1
            self.publisher.put(
                command.model_copy(update={"envelope": envelope}).model_dump_json()
            )

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
    """Controller node that publishes commands to the host node and receives camera frames."""

    def __init__(self, settings: Settings):
        super().__init__(settings=settings)
        self.settings = settings
        self.base_frame_queue = deque(maxlen=5)
        self.arm_frame_queue = deque(maxlen=5)
        self._observations: dict[str, CameraObservation] = {}
        self._observation_lock = threading.Lock()
        self.sub_base_cam = self.session.declare_subscriber(
            constants.BASE_CAMERA_KEY, self._listener_base_cam
        )
        self.sub_arm_cam = self.session.declare_subscriber(
            constants.ARM_CAMERA_KEY, self._listener_arm_cam
        )
        self.sub_base_observation = self.session.declare_subscriber(
            constants.BASE_OBSERVATION_KEY,
            lambda msg: self._listener_observation("base", msg),
        )
        self.sub_arm_observation = self.session.declare_subscriber(
            constants.ARM_OBSERVATION_KEY,
            lambda msg: self._listener_observation("arm", msg),
        )
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

    def _listener_observation(self, camera: str, msg: zenoh.Sample) -> None:
        try:
            observation = decode_observation(bytes(msg.payload))
            if observation.metadata.camera != camera:
                raise ValueError("Observation camera does not match its topic.")
            with self._observation_lock:
                previous = self._observations.get(camera)
                if (
                    previous is not None
                    and previous.metadata.host_id == observation.metadata.host_id
                    and previous.metadata.frame_id >= observation.metadata.frame_id
                ):
                    return  # Duplicates must not reset receipt age.
                self._observations[camera] = observation
            queue = self.base_frame_queue if camera == "base" else self.arm_frame_queue
            queue.append(observation.image)
        except Exception as error:
            logger.warning(f"Invalid {camera} camera observation: {error}")

    def get_observation(
        self,
        camera: str,
        *,
        max_age_s: float | None = None,
        after: tuple[str, int] | None = None,
    ) -> CameraObservation | None:
        """Get metadata + image; `after` is (host_id, frame_id), age is local receipt age."""
        if camera not in ("base", "arm"):
            raise ValueError("camera must be 'base' or 'arm'.")
        with self._observation_lock:
            observation = self._observations.get(camera)
        if observation is None:
            return None
        if max_age_s is not None and not observation.is_fresh(max_age_s):
            return None
        if (
            after is not None
            and observation.metadata.host_id == after[0]
            and observation.metadata.frame_id <= after[1]
        ):
            return None
        return observation

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
        for name in ("sub_base_observation", "sub_arm_observation"):
            subscriber = getattr(self, name, None)
            if subscriber is not None:
                try:
                    subscriber.undeclare()
                except Exception:
                    pass
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
