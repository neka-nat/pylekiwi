import math
import time
from typing import Annotated

import typer
import zenoh
from loguru import logger

from pylekiwi.models import (
    ArmCalibrationRequest,
    ArmCalibrationResponse,
    ArmEEInchingCommand,
    ArmEEPositionCommand,
    ArmJointCommand,
    LekiwiCommand,
)
from pylekiwi.preset import delete_preset, list_presets, load_presets, save_preset
from pylekiwi.settings import Settings, constants
from pylekiwi.zenoh_config import (
    create_zenoh_config,
    describe_zenoh_settings,
    tcp_endpoint,
)

app = typer.Typer(
    help="Client utilities (capture, pose, position, inching, grasp, release)",
    no_args_is_help=True,
)
arm_app = typer.Typer(help="Remote arm maintenance commands", no_args_is_help=True)
pose_app = typer.Typer(help="Arm pose presets", no_args_is_help=True)
calibrate_app = typer.Typer(
    help="Remote arm calibration utilities",
    no_args_is_help=True,
)
app.add_typer(arm_app, name="arm")
app.add_typer(pose_app, name="pose")
app.add_typer(calibrate_app, name="calibrate")


@app.callback()
def client(
    ctx: typer.Context,
    host: Annotated[
        str | None,
        typer.Option(help="Robot host/IP for an explicit Zenoh TCP connection."),
    ] = None,
    port: Annotated[
        int,
        typer.Option(help="Zenoh TCP port on the robot."),
    ] = 7447,
    no_multicast: Annotated[
        bool,
        typer.Option(
            "--no-multicast",
            is_flag=True,
            help="Disable Zenoh multicast scouting.",
        ),
    ] = False,
) -> None:
    settings_kwargs: dict[str, object] = {}
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
    ctx.obj = Settings(**settings_kwargs)
    logger.info(f"Zenoh settings: {describe_zenoh_settings(ctx.obj)}")


def _get_client_settings(ctx: typer.Context, **updates: object) -> Settings:
    current: typer.Context | None = ctx
    while current is not None:
        if isinstance(current.obj, Settings):
            return current.obj.model_copy(update=updates)
        current = current.parent
    return Settings(**updates)


def _request_calibration(
    request: ArmCalibrationRequest,
    *,
    timeout: float,
    settings: Settings,
) -> ArmCalibrationResponse:
    with zenoh.open(create_zenoh_config(settings)) as session:
        replies = session.get(
            constants.ARM_CALIBRATION_KEY,
            timeout=timeout,
            payload=request.model_dump_json(),
            encoding=zenoh.Encoding.APPLICATION_JSON,
        )
        errors: list[str] = []
        for reply in replies:
            if reply.ok is not None:
                return ArmCalibrationResponse.model_validate_json(
                    reply.ok.payload.to_string()
                )
            if reply.err is not None:
                errors.append(reply.err.payload.to_string())

    if errors:
        raise RuntimeError("; ".join(errors))
    raise RuntimeError("No calibration reply received from host.")


def _parse_reference_pose(target: str) -> tuple[float, float, float, float, float]:
    parts = [float(x.strip()) for x in target.split(",")]
    if len(parts) != 5:
        raise ValueError("Expected 5 comma-separated joint angles.")
    return parts[0], parts[1], parts[2], parts[3], parts[4]


def _print_calibration_response(response: ArmCalibrationResponse) -> None:
    if response.serial_port is not None:
        typer.echo(f"serial_port: {response.serial_port}")
    typer.echo(response.message)
    if response.backup_path is not None:
        typer.echo(f"backup_path: {response.backup_path}")
    if response.before_offsets_deg is not None:
        typer.echo(f"before_offsets_deg: {list(response.before_offsets_deg)}")
    if response.after_offsets_deg is not None:
        typer.echo(f"after_offsets_deg: {list(response.after_offsets_deg)}")
    if response.present_joint_angles_deg is not None:
        typer.echo(
            f"present_joint_angles_deg: {list(response.present_joint_angles_deg)}"
        )
    if response.reference_joint_angles_deg is not None:
        typer.echo(
            f"reference_joint_angles_deg: {list(response.reference_joint_angles_deg)}"
        )
    if response.maintenance_active is not None:
        typer.echo(f"maintenance_active: {response.maintenance_active}")
    if response.torque_enabled is not None:
        typer.echo(f"torque_enabled: {response.torque_enabled}")
    if response.verified is not None:
        typer.echo(f"verified: {response.verified}")
    if response.joint_states:
        typer.echo("joint_states:")
        for joint in response.joint_states:
            typer.echo(
                "  "
                f"J{joint.joint_id}: raw={joint.raw_present_position} "
                f"present_deg={joint.present_angle_deg:.2f} "
                f"offset_deg={joint.offset_deg:.2f}"
            )


def _ensure_confirmed(
    *,
    yes: bool,
    message: str,
) -> None:
    if yes:
        return
    if not typer.confirm(message):
        raise typer.Exit(code=1)


# ---------------------------------------------------------------------------
# arm maintenance
# ---------------------------------------------------------------------------


@arm_app.command("on")
def arm_on(ctx: typer.Context):
    """Enable remote arm torque and resume arm control."""
    try:
        response = _request_calibration(
            ArmCalibrationRequest(action="torque_on"),
            timeout=10.0,
            settings=_get_client_settings(ctx),
        )
    except RuntimeError as e:
        logger.error(str(e))
        raise typer.Exit(code=1)
    _print_calibration_response(response)
    if not response.ok:
        raise typer.Exit(code=1)


@arm_app.command("off")
def arm_off(ctx: typer.Context):
    """Disable remote arm torque so the arm can be moved by hand."""
    try:
        response = _request_calibration(
            ArmCalibrationRequest(action="torque_off"),
            timeout=10.0,
            settings=_get_client_settings(ctx),
        )
    except RuntimeError as e:
        logger.error(str(e))
        raise typer.Exit(code=1)
    _print_calibration_response(response)
    if not response.ok:
        raise typer.Exit(code=1)


# ---------------------------------------------------------------------------
# capture
# ---------------------------------------------------------------------------

@app.command()
def capture(
    ctx: typer.Context,
    camera: Annotated[str, typer.Option(help="Camera to capture: base or arm")] = "base",
    output: Annotated[str, typer.Option(help="Output file path")] = "capture.jpg",
):
    """Capture a single frame from the host camera and save as JPEG."""
    import cv2

    from pylekiwi.nodes import ClientControllerWithCameraNode

    node = ClientControllerWithCameraNode(
        settings=_get_client_settings(ctx, view_camera=False)
    )
    logger.info(f"Waiting for {camera} camera frame...")

    try:
        deadline = time.time() + 10.0
        frame = None
        while time.time() < deadline:
            if camera == "base":
                frame = node.get_base_frame()
            else:
                frame = node.get_arm_frame()
            if frame is not None:
                break
            time.sleep(0.05)

        if frame is None:
            logger.error("Timed out waiting for camera frame")
            raise typer.Exit(code=1)

        cv2.imwrite(output, frame)
        logger.info(f"Saved {camera} camera frame to {output}")
    finally:
        node.close()


# ---------------------------------------------------------------------------
# pose go
# ---------------------------------------------------------------------------

@pose_app.command("go")
def pose_go(
    ctx: typer.Context,
    target: Annotated[str, typer.Argument(help="Preset name or comma-separated angles in degrees (e.g. '10,20,30,40,50')")],
):
    """Move the arm to a preset pose or specified joint angles."""
    from pylekiwi.nodes import ClientControllerNode

    presets = load_presets()
    if target in presets:
        cmd = presets[target]
        logger.info(f"Using preset '{target}'")
    else:
        try:
            angles_deg = [float(x.strip()) for x in target.split(",")]
            if len(angles_deg) != 5:
                logger.error("Expected 5 comma-separated joint angles")
                raise typer.Exit(code=1)
            cmd = ArmJointCommand(
                joint_angles=tuple(math.radians(a) for a in angles_deg),
            )
        except ValueError:
            logger.error(f"Unknown preset '{target}' and could not parse as angles")
            raise typer.Exit(code=1)

    node = ClientControllerNode(
        settings=_get_client_settings(ctx),
        wait_for_matching=True,
    )
    try:
        node.send_arm_joint_command(cmd)
        logger.info(f"Sent arm command: {cmd}")
    except RuntimeError as e:
        logger.error(str(e))
        raise typer.Exit(code=1)
    finally:
        node.close()


# ---------------------------------------------------------------------------
# pose save
# ---------------------------------------------------------------------------

@pose_app.command("save")
def pose_save(
    name: Annotated[str, typer.Argument(help="Preset name to save")],
    serial_port: Annotated[str, typer.Option(help="Serial port for reading current arm state")] = "/dev/ttyACM0",
):
    """Save the current arm pose as a named preset."""
    from pylekiwi.arm_controller import ArmController
    from pylekiwi.settings import Settings

    settings = Settings(serial_port=serial_port)
    arm = ArmController(settings)
    state = arm.get_current_state()
    cmd = ArmJointCommand(
        joint_angles=state.joint_angles,
        gripper_position=state.gripper_position,
    )
    save_preset(name, cmd)
    angles_deg = [f"{math.degrees(a):.1f}" for a in cmd.joint_angles]
    grip_deg = f"{math.degrees(cmd.gripper_position):.1f}" if cmd.gripper_position is not None else "N/A"
    logger.info(f"Saved preset '{name}': joints={angles_deg} gripper={grip_deg}")


# ---------------------------------------------------------------------------
# pose list
# ---------------------------------------------------------------------------

@pose_app.command("list")
def pose_list():
    """List all saved arm pose presets."""
    presets = list_presets()
    if not presets:
        typer.echo("No presets found.")
        return
    for name, cmd in presets.items():
        angles_deg = [f"{math.degrees(a):.1f}" for a in cmd.joint_angles]
        grip_deg = f"{math.degrees(cmd.gripper_position):.1f}" if cmd.gripper_position is not None else "N/A"
        typer.echo(f"  {name}: joints={angles_deg} gripper={grip_deg}")


# ---------------------------------------------------------------------------
# pose delete
# ---------------------------------------------------------------------------

@pose_app.command("delete")
def pose_delete(
    name: Annotated[str, typer.Argument(help="Preset name to delete")],
):
    """Delete a saved arm pose preset."""
    try:
        delete_preset(name)
        logger.info(f"Deleted preset '{name}'")
    except (KeyError, ValueError) as e:
        logger.error(str(e))
        raise typer.Exit(code=1)


# ---------------------------------------------------------------------------
# position / inching / grasp / release
# ---------------------------------------------------------------------------

@app.command()
def position(
    ctx: typer.Context,
    x_mm: Annotated[
        float,
        typer.Option("--x-mm", help="Absolute X position in mm in the base frame."),
    ],
    y_mm: Annotated[
        float,
        typer.Option("--y-mm", help="Absolute Y position in mm in the base frame."),
    ],
    z_mm: Annotated[
        float,
        typer.Option("--z-mm", help="Absolute Z position in mm in the base frame."),
    ],
    gripper_deg: Annotated[
        float | None,
        typer.Option(
            "--gripper-deg",
            help="Optional gripper target in degrees. If omitted, keep the current value.",
        ),
    ] = None,
):
    """Move the end effector to an absolute position in the base frame."""
    from pylekiwi.nodes import ClientControllerNode

    node = ClientControllerNode(
        settings=_get_client_settings(ctx),
        wait_for_matching=True,
    )
    cmd = ArmEEPositionCommand(
        xyz=(x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0),
        gripper_position=(
            math.radians(gripper_deg) if gripper_deg is not None else None
        ),
    )
    try:
        node.send_command(LekiwiCommand(arm_command=cmd))
        logger.info(f"Sent base-frame position command: {cmd}")
    except RuntimeError as e:
        logger.error(str(e))
        raise typer.Exit(code=1)
    finally:
        node.close()


@app.command()
def inching(
    ctx: typer.Context,
    x_mm: Annotated[
        float,
        typer.Option("--x-mm", help="X-axis increment in mm in the base frame."),
    ] = 0.0,
    y_mm: Annotated[
        float,
        typer.Option("--y-mm", help="Y-axis increment in mm in the base frame."),
    ] = 0.0,
    z_mm: Annotated[
        float,
        typer.Option("--z-mm", help="Z-axis increment in mm in the base frame."),
    ] = 0.0,
    gripper_deg: Annotated[
        float | None,
        typer.Option(
            "--gripper-deg",
            help="Optional gripper target in degrees. If omitted, keep the current value.",
        ),
    ] = None,
):
    """Move the end effector by a small delta in the base frame."""
    from pylekiwi.nodes import ClientControllerNode

    delta_xyz = (x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0)
    gripper_position = (
        math.radians(gripper_deg) if gripper_deg is not None else None
    )

    if delta_xyz == (0.0, 0.0, 0.0) and gripper_position is None:
        logger.error("Specify a non-zero delta or --gripper-deg")
        raise typer.Exit(code=1)

    node = ClientControllerNode(
        settings=_get_client_settings(ctx),
        wait_for_matching=True,
    )
    cmd = ArmEEInchingCommand(
        delta_xyz=delta_xyz,
        gripper_position=gripper_position,
    )
    try:
        node.send_command(LekiwiCommand(arm_command=cmd))
        logger.info(f"Sent base-frame inching command: {cmd}")
    except RuntimeError as e:
        logger.error(str(e))
        raise typer.Exit(code=1)
    finally:
        node.close()


GRIPPER_CLOSED = 0.0
GRIPPER_OPEN = math.radians(60.0)  # ~1.047 rad


@app.command()
def grasp(ctx: typer.Context):
    """Close the gripper."""
    from pylekiwi.nodes import ClientControllerNode

    node = ClientControllerNode(
        settings=_get_client_settings(ctx),
        wait_for_matching=True,
    )
    cmd = LekiwiCommand(
        arm_command=ArmJointCommand(
            joint_angles=(0.0, 0.0, 0.0, 0.0, 0.0),
            gripper_position=GRIPPER_CLOSED,
        ),
    )
    try:
        node.send_command(cmd)
        logger.info("Sent grasp command")
    except RuntimeError as e:
        logger.error(str(e))
        raise typer.Exit(code=1)
    finally:
        node.close()


@app.command()
def release(ctx: typer.Context):
    """Open the gripper."""
    from pylekiwi.nodes import ClientControllerNode

    node = ClientControllerNode(
        settings=_get_client_settings(ctx),
        wait_for_matching=True,
    )
    cmd = LekiwiCommand(
        arm_command=ArmJointCommand(
            joint_angles=(0.0, 0.0, 0.0, 0.0, 0.0),
            gripper_position=GRIPPER_OPEN,
        ),
    )
    try:
        node.send_command(cmd)
        logger.info("Sent release command")
    except RuntimeError as e:
        logger.error(str(e))
        raise typer.Exit(code=1)
    finally:
        node.close()


# ---------------------------------------------------------------------------
# calibrate
# ---------------------------------------------------------------------------


@calibrate_app.command("status")
def calibrate_status(ctx: typer.Context):
    """Read the remote arm calibration status."""
    try:
        response = _request_calibration(
            ArmCalibrationRequest(action="status"),
            timeout=5.0,
            settings=_get_client_settings(ctx),
        )
    except RuntimeError as e:
        logger.error(str(e))
        raise typer.Exit(code=1)
    _print_calibration_response(response)
    if not response.ok:
        raise typer.Exit(code=1)


@calibrate_app.command("backup")
def calibrate_backup(ctx: typer.Context):
    """Back up the current remote arm offsets."""
    try:
        response = _request_calibration(
            ArmCalibrationRequest(action="backup"),
            timeout=10.0,
            settings=_get_client_settings(ctx),
        )
    except RuntimeError as e:
        logger.error(str(e))
        raise typer.Exit(code=1)
    _print_calibration_response(response)
    if not response.ok:
        raise typer.Exit(code=1)


@calibrate_app.command("zero")
def calibrate_zero(
    ctx: typer.Context,
    reference_deg: Annotated[
        str,
        typer.Option(
            help="Reference joint angles in degrees, e.g. '0,0,0,0,0'.",
        ),
    ] = "0,0,0,0,0",
    yes: Annotated[
        bool,
        typer.Option(
            "--yes",
            help="Skip the confirmation prompt before writing offsets.",
        ),
    ] = False,
):
    """Set the current remote arm pose as the reference pose after `client arm off`."""
    try:
        reference_pose = _parse_reference_pose(reference_deg)
    except ValueError as e:
        logger.error(str(e))
        raise typer.Exit(code=1)

    _ensure_confirmed(
        yes=yes,
        message=(
            "This will disable arm torque on the host and write EEPROM offsets. "
            "Continue?"
        ),
    )
    try:
        response = _request_calibration(
            ArmCalibrationRequest(
                action="zero",
                reference_joint_angles_deg=reference_pose,
            ),
            timeout=30.0,
            settings=_get_client_settings(ctx),
        )
    except RuntimeError as e:
        logger.error(str(e))
        raise typer.Exit(code=1)
    _print_calibration_response(response)
    if not response.ok:
        raise typer.Exit(code=1)


@calibrate_app.command("restore")
def calibrate_restore(
    ctx: typer.Context,
    backup_path: Annotated[
        str,
        typer.Argument(
            help="Backup path on the host, typically returned by calibrate backup/zero.",
        ),
    ],
    yes: Annotated[
        bool,
        typer.Option(
            "--yes",
            help="Skip the confirmation prompt before restoring offsets.",
        ),
    ] = False,
):
    """Restore remote arm offsets from a host-side backup file after `client arm off`."""
    _ensure_confirmed(
        yes=yes,
        message=(
            "This will disable arm torque on the host and restore EEPROM offsets. "
            "Continue?"
        ),
    )
    try:
        response = _request_calibration(
            ArmCalibrationRequest(
                action="restore",
                backup_path=backup_path,
            ),
            timeout=30.0,
            settings=_get_client_settings(ctx),
        )
    except RuntimeError as e:
        logger.error(str(e))
        raise typer.Exit(code=1)
    _print_calibration_response(response)
    if not response.ok:
        raise typer.Exit(code=1)
