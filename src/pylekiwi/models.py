from pydantic import BaseModel


class BaseCommand(BaseModel):
    x_vel: float
    y_vel: float
    theta_deg_vel: float
