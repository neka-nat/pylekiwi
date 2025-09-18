import time
from pylekiwi import BaseController, BaseCommand


if __name__ == "__main__":
    base_controller = BaseController()
    base_controller.set_torque()
    base_controller.send_action(BaseCommand(x_vel=0.1, y_vel=0.0, theta_deg_vel=0.0))
    time.sleep(5)
    base_controller.send_action(BaseCommand(x_vel=0.0, y_vel=0.1, theta_deg_vel=0.0))
    time.sleep(5)
    base_controller.send_action(BaseCommand(x_vel=0.0, y_vel=0.0, theta_deg_vel=5.0))
    time.sleep(5)
    base_controller.stop()
