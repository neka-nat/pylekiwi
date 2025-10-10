import time

import zenoh
from loguru import logger
from rustypot import Sts3215PyController

from pylekiwi.arm_controller import ArmController
from pylekiwi.base_controller import BaseController
from pylekiwi.models import ArmJointCommand, BaseCommand, LekiwiCommand
from pylekiwi.settings import Settings


_COMMAND_KEY = "lekiwi/command"


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

    def _listener(self, msg: zenoh.Message) -> zenoh.Reply:
        command: LekiwiCommand = LekiwiCommand.from_json(msg.payload)
        if command.base_command is not None:
            self._base_controller.send_action(command.base_command)
        if command.arm_command is not None and command.arm_command.command_type == "joint":
            self._arm_controller.send_joint_action(command.arm_command)
        return zenoh.Reply.ok()

    def run(self):
        with zenoh.open() as session:
            sub = session.declare_subscriber(_COMMAND_KEY, self._listener)
            logger.info("Starting host controller node...")
            try:
                while True:
                    time.sleep(0.01)
            except KeyboardInterrupt:
                pass
            finally:
                sub.undeclare()


class ClientControllerNode:
    def __init__(self):
        self.session = zenoh.open()
        self.publisher = self.session.declare_publisher(_COMMAND_KEY)

    def send_command(self, command: LekiwiCommand):
        self.publisher.put(command.to_json())

    def send_base_command(self, command: BaseCommand):
        self.send_command(LekiwiCommand(base_command=command))

    def send_arm_joint_command(self, command: ArmJointCommand):
        self.send_command(LekiwiCommand(arm_command=command))
