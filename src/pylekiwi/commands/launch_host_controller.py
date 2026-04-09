from typing import Annotated

import typer

from loguru import logger

from pylekiwi.nodes import HostControllerNode
from pylekiwi.settings import Settings
from pylekiwi.zenoh_config import describe_zenoh_settings, tcp_endpoint


app = typer.Typer(help="Launch the host controller", invoke_without_command=True)

@app.callback()
def host(
    serial_port: str = "/dev/ttyACM0",
    listen_host: str = "0.0.0.0",
    listen_port: int = 7447,
    no_multicast: Annotated[
        bool,
        typer.Option("--no-multicast", is_flag=True, help="Disable Zenoh multicast scouting."),
    ] = False,
):
    logger.info("Starting host controller node")
    logger.info(f"Serial port: {serial_port}")
    settings = Settings(
        serial_port=serial_port,
        zenoh_mode="peer",
        zenoh_listen_endpoints=[tcp_endpoint(listen_host, listen_port)],
        zenoh_enable_multicast=False if no_multicast else None,
    )
    logger.info(f"Zenoh settings: {describe_zenoh_settings(settings)}")
    host_controller_node = HostControllerNode(settings)
    host_controller_node.run()


if __name__ == "__main__":
    app()
