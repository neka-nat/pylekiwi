from pydantic import BaseModel


class Settings(BaseModel):
    serial_port: str = "/dev/ttyACM0"
    baudrate: int = 1000000
    timeout: float = 0.5
