import json
import math
from pathlib import Path

from pylekiwi.models import ArmJointCommand

PRESET_DIR = Path.home() / ".pylekiwi"
PRESET_FILE = PRESET_DIR / "presets.json"

DEFAULT_PRESETS: dict[str, dict] = {
    "home": {
        "joint_angles": [0.0, 0.0, 0.0, 0.0, 0.0],
        "gripper_position": 0.0,
    },
    "ready": {
        "joint_angles": [0.0, -70.0, 40.0, 30.0, 0.0],
        "gripper_position": 0.0,
    },
}


def _read_raw() -> dict[str, dict]:
    if not PRESET_FILE.exists():
        return dict(DEFAULT_PRESETS)
    data = json.loads(PRESET_FILE.read_text())
    merged = dict(DEFAULT_PRESETS)
    merged.update(data)
    return merged


def _write_raw(data: dict[str, dict]) -> None:
    PRESET_DIR.mkdir(parents=True, exist_ok=True)
    # Don't persist built-in defaults
    to_save = {k: v for k, v in data.items() if k not in DEFAULT_PRESETS}
    PRESET_FILE.write_text(json.dumps(to_save, indent=2) + "\n")


def _deg_to_command(entry: dict) -> ArmJointCommand:
    return ArmJointCommand(
        joint_angles=tuple(math.radians(a) for a in entry["joint_angles"]),
        gripper_position=math.radians(entry["gripper_position"]),
    )


def _command_to_deg(cmd: ArmJointCommand) -> dict:
    return {
        "joint_angles": [round(math.degrees(a), 2) for a in cmd.joint_angles],
        "gripper_position": round(math.degrees(cmd.gripper_position), 2)
        if cmd.gripper_position is not None
        else 0.0,
    }


def load_presets() -> dict[str, ArmJointCommand]:
    return {name: _deg_to_command(entry) for name, entry in _read_raw().items()}


def list_presets() -> dict[str, ArmJointCommand]:
    return load_presets()


def save_preset(name: str, command: ArmJointCommand) -> None:
    data = _read_raw()
    data[name] = _command_to_deg(command)
    _write_raw(data)


def delete_preset(name: str) -> None:
    data = _read_raw()
    if name in DEFAULT_PRESETS:
        raise ValueError(f"Cannot delete built-in preset '{name}'")
    if name not in data:
        raise KeyError(f"Preset '{name}' not found")
    del data[name]
    _write_raw(data)
