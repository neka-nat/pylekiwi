from typer import Typer
import subprocess


app = Typer(help="Launch the web UI", invoke_without_command=True)

@app.callback()
def web_ui(port: int = 8080, serial_port: str = "/dev/ttyACM0"):
    proc = subprocess.run(
        [
            "gunicorn",
            "--bind",
            f"0.0.0.0:{port}",
            "--env",
            f"LEKIWI_SERIAL_PORT={serial_port}",
            "pylekiwi.commands.web_ui:me",
        ]
    )
    return proc.returncode
