import time

import zenoh
from loguru import logger

from pylekiwi.base_controller import BaseController
from pylekiwi.models import BaseCommand
from pylekiwi.settings import Settings


_COMMAND_KEY = "lekiwi/command"


class HostControllerNode:
    def __init__(self, settings: Settings | None = None):
        settings = settings or Settings()
        self._base_controller = BaseController(motor_controller=settings)

    def _listener(self, msg: zenoh.Message) -> zenoh.Reply:
        command = BaseCommand.from_json(msg.payload)
        self._base_controller.send_action(command)
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

    def send_action(self, command: BaseCommand):
        self.publisher.put(command.to_json())
