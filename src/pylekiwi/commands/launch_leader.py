import argparse

from pylekiwi.nodes import LeaderControllerNode
from pylekiwi.settings import Settings


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial-port", type=str, default="/dev/ttyACM0")
    args = parser.parse_args()
    leader_node = LeaderControllerNode(settings=Settings(serial_port=args.serial_port))
    leader_node.run()


if __name__ == "__main__":
    main()
