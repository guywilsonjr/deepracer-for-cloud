from typing import Optional
from pydantic import BaseModel

from .constants import INF
from .models import HistoricData
from .params import Params


class HistoryProcessor(BaseModel):
    params: Params
    prev_params: Optional[Params]


    def get_historic_data(self) -> HistoricData:
        if not self.prev_params:
            return HistoricData(prev_speed=0, velocity=0, distance=0, dx=0, dy=0, dt=0)
        dx = self.params.x - self.prev_params.x
        dy = self.params.y - self.prev_params.y
        dt = self.params.sim_time - self.prev_params.sim_time
        dist = (dx ** 2 + dy ** 2) ** 0.5
        if dt == 0:
            velocity = INF
        else:
            velocity = dist / dt
        return HistoricData(
            prev_speed=self.prev_params.speed,
            velocity=velocity,
            distance=dist,
            dx=dx,
            dy=dy,
            dt=dt
        )
