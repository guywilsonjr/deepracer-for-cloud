import math

from pydantic import BaseModel

from custom_files.reward.params import Params


class PrevyData(BaseModel):
    dx: float
    dy: float
    dt: float
    dheading360: float
    dspeed: float
    dsteering_angle: float
    dprogress_percentage: float
    x_velocity: float
    y_velocity: float
    velocity: float


class PrevDataProcessor(BaseModel):
    params: Params
    prev_params: Params

    @property
    def prevy_data(self):
        dt = self.params.sim_time - self.prev_params.sim_time
        dx = self.params.x - self.prev_params.x
        dy = self.params.y - self.prev_params.y
        dheading360 = self.params.heading360 - self.prev_params.heading360
        dspeed = self.params.speed - self.prev_params.speed
        dsteering_angle = self.params.steering_angle - self.prev_params.steering_angle
        dprogress_percentage = self.params.progress_percentage - self.prev_params.progress_percentage
        angular_velocity = dheading360 / dt
        speed_acceleration = dspeed / dt
        steering_angle_velocity = dsteering_angle / dt
        progress_percentage_velocity = dprogress_percentage / dt
        x_velocity = dx / dt
        y_velocity = dy / dt
        velocity = math.sqrt(x_velocity ** 2 + y_velocity ** 2)
        return PrevyData(dx=dx, dy=dy, dt=dt, x_velocity=x_velocity, y_velocity=y_velocity, velocity=velocity)
