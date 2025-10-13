import argparse
import subprocess

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--serial-port", type=str, default="/dev/ttyACM0")
    args = parser.parse_args()
    port = args.port
    proc = subprocess.run(
        ["gunicorn", "--bind", f"0.0.0.0:{port}", "--env", f"LEKIWI_SERIAL_PORT={args.serial_port}", "pylekiwi.commands.web_ui:me"]
    )
    return proc.returncode
