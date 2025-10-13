import numpy as np
from loguru import logger
from rustypot import Sts3215PyController

from pylekiwi.models import BaseCommand
from pylekiwi.settings import Settings


class BaseController:
    LEFT_WHEEL_ID = 7
    BACK_WHEEL_ID = 8
    RIGHT_WHEEL_ID = 9

    # 角度φの定義（+x 前方を 0°、反時計回り）
    _PHI_DEG = np.array([60.0, 180.0, 300.0])  # [LEFT, BACK, RIGHT]

    def __init__(self, motor_controller: Sts3215PyController | Settings | None = None):
        if motor_controller is None:
            settings = Settings() if not isinstance(motor_controller, Settings) else motor_controller
            motor_controller = Sts3215PyController(
                serial_port=settings.serial_port,
                baudrate=settings.baudrate,
                timeout=settings.timeout,
            )
        elif isinstance(motor_controller, Settings):
            motor_controller = Sts3215PyController(
                serial_port=motor_controller.serial_port,
                baudrate=motor_controller.baudrate,
                timeout=motor_controller.timeout,
            )
        self.motor_controller = motor_controller

    def set_torque(self):
        for i in (self.LEFT_WHEEL_ID, self.BACK_WHEEL_ID, self.RIGHT_WHEEL_ID):
            self.motor_controller.write_torque_enable(i, True)

    def disable_torque(self):
        for i in (self.LEFT_WHEEL_ID, self.BACK_WHEEL_ID, self.RIGHT_WHEEL_ID):
            self.motor_controller.write_torque_enable(i, False)

    @staticmethod
    def _saturate_radps(w_radps: np.ndarray, max_radps: float = 10.0) -> np.ndarray:
        m = np.max(np.abs(w_radps))
        return w_radps if m <= max_radps else (w_radps * (max_radps / m))

    @classmethod
    def _body_to_wheel_radps(
        cls,
        vx: float,  # m/s
        vy: float,  # m/s
        vtheta_deg: float,  # deg/s
        wheel_radius: float = 0.05,
        base_radius: float = 0.125,
        max_radps: float = 10.0,   # 速度上限（rad/s）
    ) -> list[float]:
        # 角速度を rad/s に
        omega = np.deg2rad(vtheta_deg)
        phi = np.deg2rad(cls._PHI_DEG)  # [LEFT, BACK, RIGHT] の順

        # 標準形のキネマ行列（接線方向ベクトル）
        M = np.column_stack((-np.sin(phi), np.cos(phi), np.full(3, base_radius)))
        body = np.array([vx, vy, omega])

        # 車輪接線速度 -> 角速度（rad/s）
        wheel_radps = (M @ body) / wheel_radius

        # 安全のため物理上限で正規化
        wheel_radps = cls._saturate_radps(wheel_radps, max_radps=max_radps)
        return wheel_radps.tolist()

    @classmethod
    def _wheel_radps_to_body(
        cls,
        left_wheel_radps: float,
        back_wheel_radps: float,
        right_wheel_radps: float,
        wheel_radius: float = 0.05,
        base_radius: float = 0.125,
    ) -> BaseCommand:
        phi = np.deg2rad(cls._PHI_DEG)  # [LEFT, BACK, RIGHT]
        M = np.column_stack((-np.sin(phi), np.cos(phi), np.full(3, base_radius)))
        # 角速度 -> 接線速度（m/s）
        wheel_linear = wheel_radius * np.array([left_wheel_radps, back_wheel_radps, right_wheel_radps])
        # 逆運動学
        v = np.linalg.inv(M) @ wheel_linear
        vx, vy, omega = v
        return BaseCommand(x_vel=vx, y_vel=vy, theta_deg_vel=np.rad2deg(omega))

    def send_action(self, action: BaseCommand):
        wheel_radps = self._body_to_wheel_radps(
            action.x_vel, action.y_vel, action.theta_deg_vel
        )
        logger.debug(f"Wheel radps: {wheel_radps}")
        self.motor_controller.sync_write_goal_speed(
            [self.LEFT_WHEEL_ID, self.BACK_WHEEL_ID, self.RIGHT_WHEEL_ID],
            wheel_radps  # rad/s
        )

    def stop(self):
        self.motor_controller.sync_write_goal_speed(
            [self.LEFT_WHEEL_ID, self.BACK_WHEEL_ID, self.RIGHT_WHEEL_ID], [0.0, 0.0, 0.0]
        )
