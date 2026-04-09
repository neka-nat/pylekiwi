import json
import os

import zenoh

from pylekiwi.settings import Settings


def tcp_endpoint(host: str, port: int) -> str:
    return f"tcp/{host}:{port}"


def create_zenoh_config(settings: Settings) -> zenoh.Config:
    """Build a Zenoh config from Settings, optionally layering on ZENOH_CONFIG."""
    if os.getenv(zenoh.Config.DEFAULT_CONFIG_PATH_ENV):
        config = zenoh.Config.from_env()
    else:
        config = zenoh.Config()

    if settings.zenoh_mode is not None:
        config.insert_json5("mode", json.dumps(settings.zenoh_mode))
    if settings.zenoh_connect_endpoints:
        config.insert_json5(
            "connect/endpoints", json.dumps(settings.zenoh_connect_endpoints)
        )
    if settings.zenoh_listen_endpoints:
        config.insert_json5(
            "listen/endpoints",
            json.dumps({"peer": settings.zenoh_listen_endpoints}),
        )
    if settings.zenoh_enable_multicast is not None:
        config.insert_json5(
            "scouting/multicast/enabled",
            json.dumps(settings.zenoh_enable_multicast),
        )
    return config


def describe_zenoh_settings(settings: Settings) -> str:
    mode = settings.zenoh_mode or "default"
    connect = (
        ", ".join(settings.zenoh_connect_endpoints)
        if settings.zenoh_connect_endpoints
        else "auto"
    )
    listen = (
        ", ".join(settings.zenoh_listen_endpoints)
        if settings.zenoh_listen_endpoints
        else "default"
    )
    if settings.zenoh_enable_multicast is None:
        multicast = "default"
    else:
        multicast = "enabled" if settings.zenoh_enable_multicast else "disabled"
    return (
        f"mode={mode} connect={connect} listen={listen} multicast={multicast}"
    )
