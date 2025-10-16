import time

import zenoh
from loguru import logger
from rustypot import Sts3215PyController

from pylekiwi.arm_controller import ArmController
from pylekiwi.base_controller import BaseController
from pylekiwi.models import ArmJointCommand, BaseCommand, LekiwiCommand
from pylekiwi.settings import Settings, constants
from pylekiwi.smoother import AccelLimitedSmoother


class HostControllerNode:
    def __init__(self, settings: Settings | None = None):
        settings = settings or Settings()
        motor_controller = Sts3215PyController(
            serial_port=settings.serial_port,
            baudrate=settings.baudrate,
            timeout=settings.timeout,
        )
        self._base_controller = BaseController(motor_controller=motor_controller)
        self._arm_controller = ArmController(motor_controller=motor_controller)
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
                    time.sleep(self._dt - (time.time() - start_time))
            except KeyboardInterrupt:
                pass
            finally:
                sub.undeclare()


class ClientControllerNode:
    def __init__(self):
        self.session = zenoh.open(zenoh.Config())
        self.publisher = self.session.declare_publisher(constants.COMMAND_KEY)

    def send_command(self, command: LekiwiCommand):
        self.publisher.put(command.model_dump_json())

    def send_base_command(self, command: BaseCommand):
        self.send_command(LekiwiCommand(base_command=command))

    def send_arm_joint_command(self, command: ArmJointCommand):
        self.send_command(LekiwiCommand(arm_command=command))


class LeaderControllerNode(ClientControllerNode):
    def __init__(self, settings: Settings | None = None):
        super().__init__()
        settings = settings or Settings()
        motor_controller = Sts3215PyController(
            serial_port=settings.serial_port,
            baudrate=settings.baudrate,
            timeout=settings.timeout,
        )
        self.arm_controller = ArmController(motor_controller=motor_controller)

    def send_leader_command(self):
        arm_state = self.arm_controller.get_current_state()
        arm_command = ArmJointCommand(
            joint_angles=arm_state.joint_angles,
            gripper_position=arm_state.gripper_position,
        )
        self.send_arm_joint_command(arm_command)

    def run(self):
        while True:
            start_time = time.time()
            self.send_leader_command()
            time.sleep(constants.DT - (time.time() - start_time))
