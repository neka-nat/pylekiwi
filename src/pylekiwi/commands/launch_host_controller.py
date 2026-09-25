from typing import Annotated

import typer

from loguru import logger

from pylekiwi.nodes import HostControllerNode
from pylekiwi.settings import Settings
from pylekiwi.zenoh_config import describe_zenoh_settings, tcp_endpoint


app = typer.Typer(help="Launch the host controller", invoke_without_command=True)


def _parse_camera_id(value: str, option: str) -> int | str:
    if not value.strip():
        raise typer.BadParameter("Specify a camera index or device path.", param_hint=option)
    try:
        camera_id = int(value)
    except ValueError:
        return value
    if camera_id < 0:
        raise typer.BadParameter("Camera index must be non-negative.", param_hint=option)
    return camera_id


@app.callback()
def host(
    serial_port: str = "/dev/ttyACM0",
    listen_host: str = "0.0.0.0",
    listen_port: int = 7447,
    no_multicast: Annotated[
        bool,
        typer.Option("--no-multicast", is_flag=True, help="Disable Zenoh multicast scouting."),
    ] = False,
    base_camera_id: Annotated[
        str,
        typer.Option(help="Base camera index or device path (e.g. /dev/v4l/by-id/...)."),
    ] = "0",
    arm_camera_id: Annotated[
        str,
        typer.Option(help="Arm camera index or device path (e.g. /dev/v4l/by-id/...)."),
    ] = "2",
):
    logger.info("Starting host controller node")
    logger.info(f"Serial port: {serial_port}")
    settings = Settings(
        serial_port=serial_port,
        base_camera_id=_parse_camera_id(base_camera_id, "--base-camera-id"),
        arm_camera_id=_parse_camera_id(arm_camera_id, "--arm-camera-id"),
        zenoh_mode="peer",
        zenoh_listen_endpoints=[tcp_endpoint(listen_host, listen_port)],
        zenoh_enable_multicast=False if no_multicast else None,
    )
    logger.info(f"Cameras: base={settings.base_camera_id}, arm={settings.arm_camera_id}")
    logger.info(f"Zenoh settings: {describe_zenoh_settings(settings)}")
    host_controller_node = HostControllerNode(settings)
    host_controller_node.run()


if __name__ == "__main__":
    app()
