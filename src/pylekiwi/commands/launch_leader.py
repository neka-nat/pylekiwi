from typer import Typer

from loguru import logger

from pylekiwi.nodes import LeaderControllerNode
from pylekiwi.settings import Settings


app = Typer(help="Launch the leader controller", invoke_without_command=True)


@app.callback()
def leader(serial_port: str = "/dev/ttyACM0"):
    logger.info("Starting leader controller node")
    logger.info(f"Serial port: {serial_port}")
    logger.info("Base command is from the following keys:")
    logger.info("| w  | forward        |")
    logger.info("| s  | backward       |")
    logger.info("| a  | left           |")
    logger.info("| d  | right          |")
    logger.info("| -> | left rotation  |")
    logger.info("| <- | right rotation |")
    leader_node = LeaderControllerNode(settings=Settings(serial_port=serial_port))
    leader_node.run()


if __name__ == "__main__":
    app()
