"""Atomic frame metadata + JPEG wire format, independent of motor control."""

import struct
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

from pylekiwi.models import CameraFrameMetadata

_MAGIC = b"LKF1"
_MAX_HEADER = 65536


def encode_observation(metadata: CameraFrameMetadata, jpeg: bytes) -> bytes:
    header = metadata.model_dump_json().encode("utf-8")
    if len(header) > _MAX_HEADER:
        raise ValueError("Camera metadata is too large.")
    return _MAGIC + struct.pack("!I", len(header)) + header + jpeg


@dataclass(frozen=True)
class CameraObservation:
    metadata: CameraFrameMetadata
    image: np.ndarray
    received_monotonic_ns: int = field(default_factory=time.monotonic_ns)

    def is_fresh(self, max_age_s: float, *, now_ns: int | None = None) -> bool:
        """Local receipt age only; this does not measure sensor exposure age."""
        if not np.isfinite(max_age_s) or max_age_s < 0:
            raise ValueError("max_age_s must be finite and nonnegative.")
        now = time.monotonic_ns() if now_ns is None else now_ns
        return 0 <= now - self.received_monotonic_ns <= max_age_s * 1e9


def decode_observation(payload: bytes) -> CameraObservation:
    received = time.monotonic_ns()
    if len(payload) < 8 or payload[:4] != _MAGIC:
        raise ValueError("Invalid camera observation header.")
    length = struct.unpack("!I", payload[4:8])[0]
    if not 0 < length <= _MAX_HEADER or len(payload) <= 8 + length:
        raise ValueError("Invalid camera observation metadata length.")
    metadata = CameraFrameMetadata.model_validate_json(payload[8 : 8 + length])
    image = cv2.imdecode(
        np.frombuffer(payload[8 + length :], dtype=np.uint8), cv2.IMREAD_COLOR
    )
    if image is None:
        raise ValueError("Invalid JPEG image.")
    return CameraObservation(metadata, image, received)
