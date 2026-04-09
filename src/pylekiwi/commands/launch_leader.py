from typing import Annotated

import typer

from loguru import logger

from pylekiwi.nodes import LeaderControllerNode
from pylekiwi.settings import Settings
from pylekiwi.zenoh_config import describe_zenoh_settings, tcp_endpoint


app = typer.Typer(help="Launch the leader controller", invoke_without_command=True)


@app.callback()
def leader(
    serial_port: str = "/dev/ttyACM0",
    host: str | None = None,
    port: int = 7447,
    no_multicast: Annotated[
        bool,
        typer.Option("--no-multicast", is_flag=True, help="Disable Zenoh multicast scouting."),
    ] = False,
):
    logger.info("Starting leader controller node")
    logger.info(f"Serial port: {serial_port}")
    logger.info("Base command is from the following keys:")
    logger.info("| w  | forward        |")
    logger.info("| s  | backward       |")
    logger.info("| a  | left           |")
    logger.info("| d  | right          |")
    logger.info("| -> | left rotation  |")
    logger.info("| <- | right rotation |")
    settings_kwargs = {"serial_port": serial_port}
    if host is not None:
        settings_kwargs.update(
            {
                "zenoh_mode": "client",
                "zenoh_connect_endpoints": [tcp_endpoint(host, port)],
                "zenoh_enable_multicast": False,
            }
        )
    elif no_multicast:
        settings_kwargs["zenoh_enable_multicast"] = False
    settings = Settings(**settings_kwargs)
    logger.info(f"Zenoh settings: {describe_zenoh_settings(settings)}")
    leader_node = LeaderControllerNode(settings=settings)
    leader_node.run()


if __name__ == "__main__":
    app()
