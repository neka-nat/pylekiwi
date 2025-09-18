import argparse

from pylekiwi.nodes import HostControllerNode
from pylekiwi.settings import Settings


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial-port", type=str, default="/dev/ttyACM0")
    args = parser.parse_args()
    host_controller_node = HostControllerNode(Settings(serial_port=args.serial_port))
    host_controller_node.run()


if __name__ == "__main__":
    main()
