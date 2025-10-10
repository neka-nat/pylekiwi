import time
import threading
from dataclasses import dataclass

import numpy as np
import mesop as me

from pylekiwi.models import BaseCommand, ArmJointCommand
from pylekiwi.base_controller import BaseController
from pylekiwi.arm_controller import ArmController


JOINT_LIMITS_DEG = [
    (-150.0, 150.0),  # J1
    (-120.0, 120.0),  # J2
    (-120.0, 120.0),  # J3
    (-150.0, 150.0),  # J4
    (-150.0, 150.0),  # J5
]
GRIPPER_LIMIT_DEG = (0.0, 60.0)


@dataclass
class SharedBase:
    vx: float = 0.0            # m/s
    vy: float = 0.0            # m/s
    theta_deg: float = 0.0     # deg/s
    loop_enabled: bool = True
    armed: bool = False
    stop_requested: bool = True

@dataclass
class SharedArm:
    joints_deg: list[float] = None  # len=5
    gripper_deg: float = 0.0
    loop_enabled: bool = False
    armed: bool = False

shared_base = SharedBase()
shared_arm = SharedArm(joints_deg=[0.0]*5, gripper_deg=0.0)

_controller: BaseController | None = None     # 台車
_arm: ArmController | None = None             # アーム

_base_loop_started = False
_arm_loop_started = False


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


def _start_arm_loop():
    global _arm_loop_started
    if _arm_loop_started:
        return
    _arm_loop_started = True

    def worker():
        while True:
            try:
                if _arm and shared_arm.armed and shared_arm.loop_enabled:
                    # deg -> rad に変換して送信
                    joints_rad = tuple(np.deg2rad(shared_arm.joints_deg))
                    grip_rad = float(np.deg2rad(shared_arm.gripper_deg))
                    _arm.send_joint_action(
                        ArmJointCommand(
                            joint_angles=joints_rad,
                            gripper_position=grip_rad,
                        )
                    )
                time.sleep(0.1)  # 10Hz（関節はやや低頻度でもOK）
            except Exception as e:
                print(f"[arm-loop] {e}")
                time.sleep(0.2)
    threading.Thread(target=worker, daemon=True).start()


@me.stateclass
class State:
    # ステータス
    connected: bool = False
    msg: str = ""

    # 台車パラメータ
    max_v: float = 0.25            # [m/s]
    max_omega_deg: float = 90.0    # [deg/s]
    base_armed: bool = False
    base_auto: bool = True

    # アームトグル
    arm_armed: bool = False
    arm_auto: bool = False

    # アーム関節値（deg表示）
    j0: float = 0.0
    j1: float = 0.0
    j2: float = 0.0
    j3: float = 0.0
    j4: float = 0.0
    grip: float = 0.0  # gripper


# ========= 起動時 =========
def on_load(e: me.LoadEvent):
    me.set_theme_mode("system")
    global _controller, _arm

    try:
        _controller = BaseController()   # あなたの Settings に従って接続
        _arm = ArmController()           # 同上
        s = me.state(State)
        s.connected = True
        s.msg = "Controller connected"

        # 現在姿勢を読み取って初期表示へ（rad -> deg）
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
    _start_arm_loop()


def _apply_arm_to_state(joints_deg: list[float], grip_deg: float):
    """UIのStateへ反映"""
    s = me.state(State)
    s.j0, s.j1, s.j2, s.j3, s.j4 = joints_deg
    s.grip = grip_deg
    # 共有側にもコピー
    shared_arm.joints_deg = joints_deg[:]
    shared_arm.gripper_deg = grip_deg

def _send_arm_once_from_state():
    if not _arm or not shared_arm.armed:
        return
    s = me.state(State)
    joints_deg = [s.j0, s.j1, s.j2, s.j3, s.j4]
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


@me.page(
    path="/",
    on_load=on_load,
    title="pylekiwi Web UI",
)
def page():
    s = me.state(State)

    me.text("LeKiwi Teleop", type="headline-4")
    me.text(f"Status: {'CONNECTED' if s.connected else 'DISCONNECTED'} / {s.msg}", type="body-1")

    # --- Base（台車） ---
    me.divider()
    me.text("Base", type="headline-6")

    with me.box(style=me.Style(display="flex", gap=16, margin=me.Margin(top=8, bottom=8))):
        me.slide_toggle(label="Torque", checked=s.base_armed, on_change=on_toggle_base_arm, color="accent")
        me.slide_toggle(label="Auto Send 20Hz", checked=s.base_auto, on_change=on_toggle_base_auto)
        with me.content_button(type="raised", on_click=on_estop, color="warn"):
            me.text("E-STOP")

    with me.box(style=me.Style(display="flex", gap=24, margin=me.Margin(bottom=8))):
        me.slider(min=0.05, max=0.6, step=0.01,
                  value=s.max_v, on_value_change=on_change_max_v, show_tick_marks=True)
        me.slider(min=10, max=300, step=1,
                  value=s.max_omega_deg, on_value_change=on_change_max_omega, show_tick_marks=True)

    # D-pad: 3x3 グリッド
    with me.box(style=me.Style(display="grid", grid_template_columns="repeat(3, 120px)", gap=8)):
        me.box()
        with me.content_button(type="flat", on_click=on_dpad_up):    me.text("↑")
        with me.content_button(type="stroked", on_click=on_rot_left):   me.text("⟲")
        with me.content_button(type="flat", on_click=on_dpad_left):  me.text("←")
        with me.content_button(type="stroked", on_click=on_dpad_stop):   me.text("■ STOP")
        with me.content_button(type="flat", on_click=on_dpad_right): me.text("→")
        me.box()
        with me.content_button(type="flat", on_click=on_dpad_down):  me.text("↓")
        with me.content_button(type="stroked", on_click=on_rot_right):  me.text("⟳")

    # --- Arm（アーム） ---
    me.divider()
    me.text("Arm", type="headline-6")

    with me.box(style=me.Style(display="flex", gap=16, margin=me.Margin(top=8, bottom=8))):
        me.slide_toggle(label="Torque", checked=s.arm_armed, on_change=on_toggle_arm_arm, color="accent")
        me.slide_toggle(label="Auto Send 10Hz", checked=s.arm_auto, on_change=on_toggle_arm_auto)
        with me.content_button(type="stroked", on_click=on_arm_send_once): me.text("Send Arm")
        with me.content_button(type="stroked", on_click=on_arm_home):      me.text("Home (0°)")
        with me.content_button(type="stroked", on_click=on_arm_refresh):   me.text("Refresh (read)")

    # 5関節 + グリッパ
    for i, (lo, hi) in enumerate(JOINT_LIMITS_DEG):
        val = getattr(s, f"j{i}")
        me.slider(min=lo, max=hi, step=1.0,
                  value=val, on_value_change=lambda e, idx=i: on_change_joint(idx, e))
    me.slider(min=GRIPPER_LIMIT_DEG[0], max=GRIPPER_LIMIT_DEG[1], step=1.0,
              value=s.grip, on_value_change=on_change_gripper)


# ========= イベント（Base） =========
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


# ========= イベント（Arm） =========
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

def on_toggle_arm_auto(e: me.SlideToggleChangeEvent):
    s = me.state(State)
    s.arm_auto = not s.arm_auto
    shared_arm.loop_enabled = s.arm_auto

def on_arm_send_once(e: me.ClickEvent):
    _send_arm_once_from_state()

def on_arm_home(e: me.ClickEvent):
    # すべて 0deg に
    s = me.state(State)
    s.j0 = s.j1 = s.j2 = s.j3 = s.j4 = 0.0
    s.grip = 0.0
    if not shared_arm.loop_enabled:
        _send_arm_once_from_state()
    else:
        _apply_arm_to_state([0, 0, 0, 0, 0], 0.0)

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
    if shared_arm.armed and not shared_arm.loop_enabled:
        _send_arm_once_from_state()  # Auto Off時は即時一発送信にしてもよい

def on_change_gripper(e: me.SliderValueChangeEvent):
    s = me.state(State)
    s.grip = e.value
    if shared_arm.armed and not shared_arm.loop_enabled:
        _send_arm_once_from_state()
