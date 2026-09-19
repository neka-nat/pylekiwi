import struct
import threading
import unittest
from collections import deque
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from pylekiwi.camera_controller import encode_jpeg
from pylekiwi.models import CameraFrameMetadata
from pylekiwi.nodes import ClientControllerWithCameraNode
from pylekiwi.observation import (
    CameraObservation,
    decode_observation,
    encode_observation,
)


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.image = np.zeros((12, 16, 3), dtype=np.uint8)
        self.metadata = CameraFrameMetadata(
            host_id="host-a",
            camera="base",
            frame_id=3,
            read_started_monotonic_ns=100,
            read_completed_monotonic_ns=200,
        )

    def packet(self, **updates):
        return encode_observation(
            self.metadata.model_copy(update=updates), encode_jpeg(self.image)
        )

    def client(self):
        node = ClientControllerWithCameraNode.__new__(ClientControllerWithCameraNode)
        node._observations = {}
        node._observation_lock = threading.Lock()
        node.base_frame_queue = deque(maxlen=5)
        node.arm_frame_queue = deque(maxlen=5)
        return node

    def test_atomic_roundtrip(self):
        decoded = decode_observation(self.packet())
        self.assertEqual(decoded.metadata, self.metadata)
        np.testing.assert_array_equal(decoded.image, self.image)

    def test_malformed_or_truncated_payload_rejected(self):
        for packet in (
            b"",
            b"LKF1",
            b"LKF1" + struct.pack("!I", 999999),
            self.packet()[:20],
            encode_observation(self.metadata, b"invalid-jpeg"),
        ):
            with self.assertRaises(ValueError):
                decode_observation(packet)

    def test_receipt_age_does_not_compare_host_and_client_clocks(self):
        observation = CameraObservation(
            self.metadata, self.image, received_monotonic_ns=9_000_000_000
        )
        self.assertTrue(observation.is_fresh(0.5, now_ns=9_400_000_000))
        self.assertFalse(observation.is_fresh(0.5, now_ns=9_600_000_000))

    def test_repeated_or_older_frame_does_not_refresh_cache_age(self):
        node = self.client()
        with patch(
            "pylekiwi.observation.time.monotonic_ns", return_value=1_000_000_000
        ):
            node._listener_observation("base", SimpleNamespace(payload=self.packet()))
        with patch(
            "pylekiwi.observation.time.monotonic_ns", return_value=3_000_000_000
        ):
            node._listener_observation("base", SimpleNamespace(payload=self.packet()))
            node._listener_observation(
                "base", SimpleNamespace(payload=self.packet(frame_id=2))
            )
            self.assertIsNone(node.get_observation("base", max_age_s=0.5))
        self.assertEqual(node.get_observation("base").metadata.frame_id, 3)
        self.assertIsNone(node.get_observation("base", after=("host-a", 3)))

    def test_new_host_frame_ids_start_again(self):
        node = self.client()
        node._listener_observation("base", SimpleNamespace(payload=self.packet()))
        node._listener_observation(
            "base", SimpleNamespace(payload=self.packet(host_id="host-b", frame_id=0))
        )
        self.assertEqual(
            node.get_observation("base", after=("host-a", 3)).metadata.host_id, "host-b"
        )

    def test_wrong_camera_and_bad_timestamps_rejected(self):
        node = self.client()
        node._listener_observation("arm", SimpleNamespace(payload=self.packet()))
        self.assertIsNone(node.get_observation("arm"))
        with self.assertRaises(ValueError):
            CameraFrameMetadata.model_validate(
                {**self.metadata.model_dump(), "read_completed_monotonic_ns": 0}
            )


if __name__ == "__main__":
    unittest.main()
