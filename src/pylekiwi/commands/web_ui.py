import os
import time
import threading
from functools import partial

import numpy as np
import mesop as me
from pydantic import BaseModel
from rustypot import Sts3215PyController

from pylekiwi.models import BaseCommand, ArmJointCommand
from pylekiwi.base_controller import BaseController
from pylekiwi.arm_controller import ArmController
from pylekiwi.settings import Settings


JOINT_LIMITS_DEG = [
    (-150.0, 150.0),  # J1
    (-120.0, 120.0),  # J2
    (-120.0, 120.0),  # J3
    (-150.0, 150.0),  # J4
    (-150.0, 150.0),  # J5
]
GRIPPER_LIMIT_DEG = (0.0, 60.0)
BORDER_LIGHT_SIDE = me.BorderSide(width=1, style="solid", color="#e5e7eb")
BORDER_LIGHT = me.Border(
    top=BORDER_LIGHT_SIDE,
    right=BORDER_LIGHT_SIDE,
    bottom=BORDER_LIGHT_SIDE,
    left=BORDER_LIGHT_SIDE,
)
PAD_N = lambda n: me.Padding(
    top=n,
    right=n,
    bottom=n,
    left=n,
)

# カードの共通スタイル
CARD = me.Style(
    display="flex",
    flex_direction="column",
    gap=16,
    padding=PAD_N(16),
    margin=me.Margin(top=12, bottom=12),
    border=BORDER_LIGHT,
    border_radius=16,
)

# 小さなバッジ風ラベル
def badge(text: str):
    with me.box(style=me.Style(
        display="inline-flex",
        align_items="center",
        padding=PAD_N(6),
        border_radius=9999,
        border=BORDER_LIGHT,
        gap=6
    )):
        me.text(text, type="caption")


class SharedBase(BaseModel):
    vx: float = 0.0            # m/s
    vy: float = 0.0            # m/s
    theta_deg: float = 0.0     # deg/s
    loop_enabled: bool = True
    armed: bool = False
    stop_requested: bool = True


class SharedArm(BaseModel):
    joints_deg: list[float] | None = None  # len=5
    gripper_deg: float = 0.0
    armed: bool = False


shared_base = SharedBase()
shared_arm = SharedArm(joints_deg=[0.0]*5, gripper_deg=0.0)

_controller: BaseController | None = None  # 台車
_arm: ArmController | None = None          # アーム

_base_loop_started = False


def _start_base_loop():
    global _base_loop_started
    if _base_loop_started:
        return
    _base_loop_started = True

    def worker():
        last_sent_stop = True
        while True:
            try:
                if _controller and shared_base.armed and shared_base.loop_enabled and not shared_base.stop_requested:
                    cmd = BaseCommand(
                        x_vel=shared_base.vx,
                        y_vel=shared_base.vy,
                        theta_deg_vel=shared_base.theta_deg,
                    )
                    _controller.send_action(cmd)
                    last_sent_stop = False
                else:
                    if _controller and not last_sent_stop:
                        _controller.stop()
                        last_sent_stop = True
                time.sleep(0.05)  # 20Hz
            except Exception as e:
                print(f"[base-loop] {e}")
                time.sleep(0.2)
    threading.Thread(target=worker, daemon=True).start()


@me.stateclass
class State:
    # ステータス
    connected: bool = False
    msg: str = ""

    # 台車パラメータ
    max_v: float = 0.25            # [m/s]
    max_omega_deg: float = 45.0    # [deg/s]
    base_armed: bool = False
    base_auto: bool = True

    # アームトグル
    arm_armed: bool = False

    # アーム関節値（deg表示）
    j1: float = 0.0
    j2: float = 0.0
    j3: float = 0.0
    j4: float = 0.0
    j5: float = 0.0
    grip: float = 0.0  # gripper


# ------------ INIT / LOAD ------------
def on_load(e: me.LoadEvent):
    me.set_theme_mode("system")
    global _controller, _arm

    try:
        if "LEKIWI_SERIAL_PORT" in os.environ:
            settings = Settings(serial_port=os.getenv("LEKIWI_SERIAL_PORT"))
        else:
            settings = Settings()
        motor_controller = Sts3215PyController(
            serial_port=settings.serial_port,
            baudrate=settings.baudrate,
            timeout=settings.timeout,
        )
        _controller = BaseController(motor_controller=motor_controller)
        _arm = ArmController(motor_controller=motor_controller)
        s = me.state(State)
        s.connected = True
        s.msg = "Controller connected"

        # 現在姿勢で初期化
        try:
            st = _arm.get_current_state()
            j_deg = [float(np.rad2deg(v)) for v in st.joint_angles]
            g_deg = float(np.rad2deg(st.gripper_position)) if st.gripper_position is not None else 0.0
            _apply_arm_to_state(j_deg, g_deg)
        except Exception:
            pass

    except Exception as ex:
        s = me.state(State)
        s.connected = False
        s.msg = f"Init failed: {ex}"

    _start_base_loop()


def _apply_arm_to_state(joints_deg: list[float], grip_deg: float):
    s = me.state(State)
    s.j1, s.j2, s.j3, s.j4, s.j5 = joints_deg
    s.grip = grip_deg
    shared_arm.joints_deg = joints_deg[:]
    shared_arm.gripper_deg = grip_deg


def _send_arm_once_from_state():
    if not _arm or not shared_arm.armed:
        return
    s = me.state(State)
    joints_deg = [s.j1, s.j2, s.j3, s.j4, s.j5]
    grip_deg = s.grip
    shared_arm.joints_deg = joints_deg[:]
    shared_arm.gripper_deg = grip_deg
    _arm.send_joint_action(
        ArmJointCommand(
            joint_angles=tuple(np.deg2rad(joints_deg)),
            gripper_position=float(np.deg2rad(grip_deg)),
        )
    )


def _stop_base():
    shared_base.vx = 0.0
    shared_base.vy = 0.0
    shared_base.theta_deg = 0.0
    shared_base.stop_requested = True
    if _controller:
        _controller.stop()


def _preset_speed(v: float, omega: float):
    s = me.state(State)
    s.max_v = v
    s.max_omega_deg = omega


@me.page(
    path="/",
    on_load=on_load,
    title="pylekiwi Web UI",
    security_policy=me.SecurityPolicy(
        allowed_script_srcs=[
            "'self'",
            "https://cdn.jsdelivr.net",
        ],
    )
)
def page():
    s = me.state(State)

    # --- Header bar ---
    with me.box(style=me.Style(
        display="flex", justify_content="space-between", align_items="center",
        padding=PAD_N(16), border=BORDER_LIGHT, border_radius=16, margin=me.Margin(bottom=12)
    )):
        with me.box(style=me.Style(display="flex", align_items="center", gap=12)):
            me.text("LeKiwi Teleop", type="headline-5")
            badge("CONNECTED" if s.connected else "DISCONNECTED")
        me.text(s.msg or "", type="body-1")
        with me.content_button(type="raised", on_click=on_estop, color="warn"):
            me.text("E-STOP")

    # --- Main grid: Base | Arm ---
    with me.box(style=me.Style(display="grid", gap=16, grid_template_columns="1fr 1fr")):

        # ===== Base card =====
        with me.box(style=CARD):
            me.text("Base", type="headline-2")

            # Row: toggles + presets
            with me.box(style=me.Style(display="flex", gap=12, align_items="center", flex_wrap="wrap")):
                me.slide_toggle(label="Torque", checked=s.base_armed, on_change=on_toggle_base_arm, color="accent")
                me.slide_toggle(label="Auto 20Hz", checked=s.base_auto, on_change=on_toggle_base_auto)
                # Presets
                with me.content_button(type="stroked", on_click=lambda e: _preset_speed(0.15, 60)):
                    me.text("Slow")
                with me.content_button(type="stroked", on_click=lambda e: _preset_speed(0.25, 120)):
                    me.text("Normal")
                with me.content_button(type="stroked", on_click=lambda e: _preset_speed(0.40, 180)):
                    me.text("Fast")

            # Row: limits
            with me.box(style=me.Style(display="grid", grid_template_columns="1fr 1fr", gap=12)):
                with me.box():
                    me.text(f"max_v = {s.max_v:.2f} m/s", type="headline-4")
                    me.slider(min=0.05, max=0.6, step=0.01, value=s.max_v, on_value_change=on_change_max_v, show_tick_marks=True)
                with me.box():
                    me.text(f"max_ω = {s.max_omega_deg:.0f} deg/s", type="headline-4")
                    me.slider(min=10, max=300, step=1, value=s.max_omega_deg, on_value_change=on_change_max_omega, show_tick_marks=True)

            # D-pad + rotation (3x3)
            me.text("D-pad", type="headline-4")
            with me.box(style=me.Style(display="grid", grid_template_columns="repeat(3, 110px)", gap=10, justify_content="start")):
                me.box()
                with me.content_button(type="flat", on_click=on_dpad_up):    me.text("↑")
                with me.content_button(type="flat", on_click=on_rot_left):   me.text("⟲")
                with me.content_button(type="flat", on_click=on_dpad_left):  me.text("←")
                with me.content_button(type="stroked",  on_click=on_dpad_stop):  me.text("■ STOP")
                with me.content_button(type="flat", on_click=on_dpad_right): me.text("→")
                me.box()
                with me.content_button(type="flat", on_click=on_dpad_down):  me.text("↓")
                with me.content_button(type="flat", on_click=on_rot_right):  me.text("⟳")

            # Current command snapshot
            me.text("Current command", type="headline-4")
            with me.box(style=me.Style(display="flex", gap=12, flex_wrap="wrap")):
                badge(f"vx {shared_base.vx:+.2f} m/s")
                badge(f"vy {shared_base.vy:+.2f} m/s")
                badge(f"ω  {shared_base.theta_deg:+.0f} deg/s")

        # ===== Arm card =====
        with me.box(style=CARD):
            me.text("Arm", type="headline-2")
            with me.box(style=me.Style(display="flex", gap=12, flex_wrap="wrap", align_items="center")):
                me.slide_toggle(label="Torque", checked=s.arm_armed, on_change=on_toggle_arm_arm, color="accent")
                with me.content_button(type="stroked", on_click=on_arm_send_once): me.text("Send")
                with me.content_button(type="stroked", on_click=on_arm_refresh):   me.text("Refresh")

            # Sliders (2 columns for joints + 1 row for gripper)
            with me.box(style=me.Style(display="grid", grid_template_columns="1fr 1fr", gap=12)):
                for i, (lo, hi) in enumerate(JOINT_LIMITS_DEG):
                    val = getattr(s, f"j{i + 1}")
                    with me.box(style=me.Style(display="flex", flex_direction="column", gap=6)):
                        me.text(f"J{i + 1}  {val:+.0f}°", type="headline-4")
                        me.slider(
                            min=lo,
                            max=hi,
                            step=1.0,
                            value=val,
                            on_value_change=partial(on_change_joint, i + 1),
                            style=me.Style(width="100%")
                        )
            with me.box(style=me.Style(display="flex", gap=12, align_items="center", margin=me.Margin(top=8))):
                me.text(f"Gripper  {s.grip:.0f}°", type="headline-4")
                me.slider(
                    min=GRIPPER_LIMIT_DEG[0],
                    max=GRIPPER_LIMIT_DEG[1],
                    step=1.0,
                    value=s.grip,
                    on_value_change=on_change_gripper,
                    style=me.Style(width="100%")
                )


def on_toggle_base_arm(e: me.SlideToggleChangeEvent):
    s = me.state(State)
    s.base_armed = not s.base_armed
    shared_base.armed = s.base_armed
    shared_base.stop_requested = not s.base_armed
    if _controller:
        if s.base_armed:
            _controller.set_torque()
        else:
            _stop_base()
            # トルクOFF（必要なら個別IDに対して）
            try:
                mc = _controller.motor_controller
                for wid in (_controller.LEFT_WHEEL_ID, _controller.BACK_WHEEL_ID, _controller.RIGHT_WHEEL_ID):
                    mc.write_torque_enable(wid, False)
            except Exception:
                pass


def on_toggle_base_auto(e: me.SlideToggleChangeEvent):
    s = me.state(State)
    s.base_auto = not s.base_auto
    shared_base.loop_enabled = s.base_auto
    if not s.base_auto:
        shared_base.stop_requested = True


def on_change_max_v(e: me.SliderValueChangeEvent):
    me.state(State).max_v = e.value


def on_change_max_omega(e: me.SliderValueChangeEvent):
    me.state(State).max_omega_deg = e.value


def on_dpad_up(e: me.ClickEvent):
    s = me.state(State)
    shared_base.vx, shared_base.vy, shared_base.theta_deg = s.max_v, 0.0, 0.0
    shared_base.stop_requested = False


def on_dpad_down(e: me.ClickEvent):
    s = me.state(State)
    shared_base.vx, shared_base.vy, shared_base.theta_deg = -s.max_v, 0.0, 0.0
    shared_base.stop_requested = False


def on_dpad_left(e: me.ClickEvent):
    s = me.state(State)
    shared_base.vx, shared_base.vy, shared_base.theta_deg = 0.0, +s.max_v, 0.0
    shared_base.stop_requested = False


def on_dpad_right(e: me.ClickEvent):
    s = me.state(State)
    shared_base.vx, shared_base.vy, shared_base.theta_deg = 0.0, -s.max_v, 0.0
    shared_base.stop_requested = False


def on_rot_left(e: me.ClickEvent):
    s = me.state(State)
    shared_base.vx, shared_base.vy, shared_base.theta_deg = 0.0, 0.0, +s.max_omega_deg
    shared_base.stop_requested = False


def on_rot_right(e: me.ClickEvent):
    s = me.state(State)
    shared_base.vx, shared_base.vy, shared_base.theta_deg = 0.0, 0.0, -s.max_omega_deg
    shared_base.stop_requested = False


def on_dpad_stop(e: me.ClickEvent):
    _stop_base()


def on_estop(e: me.ClickEvent):
    # Base 停止＋トルクOFF
    s = me.state(State)
    s.base_armed = False
    shared_base.armed = False
    _stop_base()
    if _controller:
        try:
            _controller.disable_torque()
        except Exception:
            pass
    # Arm もOFF
    s.arm_armed = False
    shared_arm.armed = False
    if _arm:
        try:
            _arm.disable_torque()
        except Exception:
            pass


def on_toggle_arm_arm(e: me.SlideToggleChangeEvent):
    s = me.state(State)
    s.arm_armed = not s.arm_armed
    shared_arm.armed = s.arm_armed
    if _arm:
        try:
            if s.arm_armed:
                _arm.set_torque()
            else:
                _arm.disable_torque()
        except Exception as ex:
            print(f"[arm-torque] {ex}")


def on_arm_send_once(e: me.ClickEvent):
    _send_arm_once_from_state()


def on_arm_refresh(e: me.ClickEvent):
    if not _arm:
        return
    try:
        st = _arm.get_current_state()
        j_deg = [float(np.rad2deg(v)) for v in st.joint_angles]
        g_deg = float(np.rad2deg(st.gripper_position)) if st.gripper_position is not None else 0.0
        _apply_arm_to_state(j_deg, g_deg)
    except Exception as ex:
        print(f"[arm-refresh] {ex}")


def on_change_joint(idx: int, e: me.SliderValueChangeEvent):
    s = me.state(State)
    setattr(s, f"j{idx}", e.value)
    if shared_arm.armed:
        _send_arm_once_from_state()


def on_change_gripper(e: me.SliderValueChangeEvent):
    s = me.state(State)
    s.grip = e.value
    if shared_arm.armed:
        _send_arm_once_from_state()
