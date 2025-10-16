from typer import Typer

from pylekiwi.nodes import HostControllerNode
from pylekiwi.settings import Settings


app = Typer(help="Launch the host controller", invoke_without_command=True)

@app.callback()
def host(serial_port: str = "/dev/ttyACM0"):
    host_controller_node = HostControllerNode(Settings(serial_port=serial_port))
    host_controller_node.run()


if __name__ == "__main__":
    app()
