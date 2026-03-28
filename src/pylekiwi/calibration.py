import math
from datetime import datetime, timezone
from pathlib import Path

from pydantic import BaseModel

from pylekiwi.arm_controller import ArmController
from pylekiwi.models import ArmCalibrationJointState, ArmCalibrationResponse
from pylekiwi.settings import constants


_VERIFY_TOLERANCE_RAD = math.radians(2.0)


class CalibrationError(RuntimeError):
    pass


class ArmCalibrationBackup(BaseModel):
    created_at: str
    serial_port: str
    joint_ids: tuple[int, int, int, int, int]
    offsets_deg: tuple[float, float, float, float, float]


def _backup_dir() -> Path:
    return Path.home() / ".pylekiwi" / constants.CALIBRATION_BACKUP_DIRNAME


def _angles_rad_to_deg(
    values: tuple[float, float, float, float, float],
) -> tuple[float, float, float, float, float]:
    return tuple(math.degrees(v) for v in values)


def _angles_deg_to_rad(
    values: tuple[float, float, float, float, float],
) -> tuple[float, float, float, float, float]:
    return tuple(math.radians(v) for v in values)


def _joint_states(arm: ArmController) -> list[ArmCalibrationJointState]:
    raw_positions = arm.read_raw_joint_positions()
    present_angles = arm.read_joint_positions()
    offsets = arm.read_joint_offsets()
    return [
        ArmCalibrationJointState(
            joint_id=joint_id,
            raw_present_position=raw_position,
            present_angle_deg=math.degrees(present_angle),
            offset_deg=math.degrees(offset),
        )
        for joint_id, raw_position, present_angle, offset in zip(
            arm.JOINT_IDS, raw_positions, present_angles, offsets
        )
    ]


def _write_backup(
    arm: ArmController,
    *,
    serial_port: str,
) -> tuple[Path, tuple[float, float, float, float, float]]:
    offsets_rad = arm.read_joint_offsets()
    backup = ArmCalibrationBackup(
        created_at=datetime.now(timezone.utc).isoformat(),
        serial_port=serial_port,
        joint_ids=arm.JOINT_IDS,
        offsets_deg=_angles_rad_to_deg(offsets_rad),
    )
    backup_dir = _backup_dir()
    backup_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    backup_path = backup_dir / f"arm-offsets-{timestamp}.json"
    backup_path.write_text(backup.model_dump_json(indent=2) + "\n")
    return backup_path, offsets_rad


def _apply_joint_offsets(
    arm: ArmController,
    offsets_rad: tuple[float, float, float, float, float],
) -> tuple[float, float, float, float, float]:
    for joint_id, new_offset in zip(arm.JOINT_IDS, offsets_rad):
        was_locked = arm.read_joint_lock(joint_id)
        try:
            if was_locked:
                arm.write_joint_lock(joint_id, False)
            arm.write_joint_offset(joint_id, new_offset)
        finally:
            if was_locked:
                arm.write_joint_lock(joint_id, True)

        readback = arm.read_joint_offset(joint_id)
        if not math.isclose(readback, new_offset, abs_tol=1e-6):
            raise CalibrationError(
                f"Offset readback mismatch on joint {joint_id}: "
                f"expected {new_offset:.6f}, got {readback:.6f}"
            )

    return arm.read_joint_offsets()


def get_status(arm: ArmController, *, serial_port: str) -> ArmCalibrationResponse:
    positions_rad = arm.read_joint_positions()
    offsets_rad = arm.read_joint_offsets()
    return ArmCalibrationResponse(
        ok=True,
        message="Read arm calibration status.",
        serial_port=serial_port,
        joint_states=_joint_states(arm),
        before_offsets_deg=_angles_rad_to_deg(offsets_rad),
        present_joint_angles_deg=_angles_rad_to_deg(positions_rad),
    )


def backup_offsets(arm: ArmController, *, serial_port: str) -> ArmCalibrationResponse:
    backup_path, offsets_rad = _write_backup(arm, serial_port=serial_port)
    return ArmCalibrationResponse(
        ok=True,
        message="Backed up arm offsets.",
        serial_port=serial_port,
        joint_states=_joint_states(arm),
        backup_path=str(backup_path),
        before_offsets_deg=_angles_rad_to_deg(offsets_rad),
        present_joint_angles_deg=_angles_rad_to_deg(arm.read_joint_positions()),
    )


def zero_to_reference_pose(
    arm: ArmController,
    *,
    serial_port: str,
    reference_angles_deg: tuple[float, float, float, float, float],
) -> ArmCalibrationResponse:
    arm.disable_torque()
    backup_path, before_offsets_rad = _write_backup(arm, serial_port=serial_port)
    present_positions_rad = arm.read_joint_positions()
    reference_angles_rad = _angles_deg_to_rad(reference_angles_deg)
    target_offsets_rad = tuple(
        old_offset + (reference_angle - present_position)
        for old_offset, reference_angle, present_position in zip(
            before_offsets_rad, reference_angles_rad, present_positions_rad
        )
    )
    after_offsets_rad = _apply_joint_offsets(arm, target_offsets_rad)
    present_after_rad = arm.read_joint_positions()
    verified = all(
        math.isclose(actual, expected, abs_tol=_VERIFY_TOLERANCE_RAD)
        for actual, expected in zip(present_after_rad, reference_angles_rad)
    )

    return ArmCalibrationResponse(
        ok=verified,
        message=(
            "Applied arm zero calibration."
            if verified
            else "Applied arm zero calibration, but verification exceeded tolerance."
        ),
        serial_port=serial_port,
        joint_states=_joint_states(arm),
        backup_path=str(backup_path),
        before_offsets_deg=_angles_rad_to_deg(before_offsets_rad),
        after_offsets_deg=_angles_rad_to_deg(after_offsets_rad),
        present_joint_angles_deg=_angles_rad_to_deg(present_after_rad),
        reference_joint_angles_deg=reference_angles_deg,
        verified=verified,
    )


def restore_offsets(
    arm: ArmController,
    *,
    serial_port: str,
    backup_path: str,
) -> ArmCalibrationResponse:
    backup = ArmCalibrationBackup.model_validate_json(Path(backup_path).read_text())
    target_offsets_rad = _angles_deg_to_rad(backup.offsets_deg)
    arm.disable_torque()
    before_offsets_rad = arm.read_joint_offsets()
    restored_offsets_rad = _apply_joint_offsets(arm, target_offsets_rad)
    present_positions_rad = arm.read_joint_positions()
    verified = all(
        math.isclose(actual, expected, abs_tol=1e-6)
        for actual, expected in zip(restored_offsets_rad, target_offsets_rad)
    )

    return ArmCalibrationResponse(
        ok=verified,
        message=(
            "Restored arm offsets from backup."
            if verified
            else "Restored arm offsets, but readback verification failed."
        ),
        serial_port=serial_port,
        joint_states=_joint_states(arm),
        backup_path=backup_path,
        before_offsets_deg=_angles_rad_to_deg(before_offsets_rad),
        after_offsets_deg=_angles_rad_to_deg(restored_offsets_rad),
        present_joint_angles_deg=_angles_rad_to_deg(present_positions_rad),
        verified=verified,
    )
