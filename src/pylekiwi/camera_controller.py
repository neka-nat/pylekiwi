import cv2
import numpy as np


def encode_jpeg(frame: np.ndarray, quality: int = 80) -> bytes:
    ok, enc = cv2.imencode(
        ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    )
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return enc.tobytes()


class CameraController:
    def __init__(self, base_camera_id: int | None = None, arm_camera_id: int | None = None):
        self.cap_base = None
        self.cap_arm = None
        if base_camera_id is not None:
            self.cap_base = cv2.VideoCapture(base_camera_id)
        if arm_camera_id is not None:
            self.cap_arm = cv2.VideoCapture(arm_camera_id)

    def __del__(self):
        if self.cap_base is not None:
            self.cap_base.release()
        if self.cap_arm is not None:
            self.cap_arm.release()

    def get_base_frame(self) -> np.ndarray | None:
        if self.cap_base is None:
            return None
        ret, frame = self.cap_base.read()
        if not ret:
            return None
        return frame

    def get_arm_frame(self) -> np.ndarray | None:
        if self.cap_arm is None:
            return None
        ret, frame = self.cap_arm.read()
        if not ret:
            return None
        return frame
