from pylekiwi import ArmController


if __name__ == "__main__":
    arm_controller = ArmController()
    state = arm_controller.get_current_state()
    print(state)
