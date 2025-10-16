from typer import Typer

from pylekiwi.nodes import LeaderControllerNode
from pylekiwi.settings import Settings


app = Typer(help="Launch the leader controller", invoke_without_command=True)


@app.callback()
def leader(serial_port: str = "/dev/ttyACM0"):
    leader_node = LeaderControllerNode(settings=Settings(serial_port=serial_port))
    leader_node.run()


if __name__ == "__main__":
    app()
