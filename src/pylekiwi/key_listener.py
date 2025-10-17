from pynput import keyboard

from pylekiwi.models import BaseCommand


class KeyListener:
    def __init__(self, vel: float = 0.25, omega_deg: float = 45.0):
        self.current_command = BaseCommand(x_vel=0, y_vel=0, theta_deg_vel=0)
        self.vel = vel
        self.omega_deg = omega_deg

    def on_key_press(self, key: keyboard.Key) -> None:
        if hasattr(key, "char"):
            if key.char == "w":
                self.current_command = BaseCommand(x_vel=self.vel, y_vel=0, theta_deg_vel=0)
            elif key.char == "s":
                self.current_command = BaseCommand(x_vel=-self.vel, y_vel=0, theta_deg_vel=0)
            elif key.char == "a":
                self.current_command = BaseCommand(x_vel=0, y_vel=self.vel, theta_deg_vel=0)
            elif key.char == "d":
                self.current_command = BaseCommand(x_vel=0, y_vel=-self.vel, theta_deg_vel=0)
        elif key == keyboard.Key.left:
            self.current_command = BaseCommand(x_vel=0, y_vel=0, theta_deg_vel=self.omega_deg)
        elif key == keyboard.Key.right:
            self.current_command = BaseCommand(x_vel=0, y_vel=0, theta_deg_vel=-self.omega_deg)
    
    def on_key_release(
        self,
        key: keyboard.Key,
    ) -> None:
        self.current_command = BaseCommand(x_vel=0, y_vel=0, theta_deg_vel=0)
