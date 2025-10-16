from typer import Typer

from pylekiwi.commands import launch_host_controller, launch_leader, launch_web_ui


app = Typer(help="pylekiwi: Python package for controlling the LeKiwi robot", no_args_is_help=True)


app.add_typer(launch_host_controller.app, name="host")
app.add_typer(launch_leader.app, name="leader")
app.add_typer(launch_web_ui.app, name="webui")
