from typing import List, Optional
from pydantic import BaseModel

from .params import Params


class HistoryProcessor(BaseModel):
    prev_speed: Optional[float]
    velocity: Optional[float]

    @classmethod
    def process_history(cls, params: Params, previous_run_states: List['RunState']):
        prev_run_state = previous_run_states[-1]
        if not prev_run_state:
            return cls(prev_speed=None, velocity=None)
        prev_speed = prev_run_state.params.speed
        dx = params.x - prev_run_state.params.x
        dy = params.y - prev_run_state.params.y
        velocity = (dx ** 2 + dy ** 2) ** 0.5
        return cls(prev_speed=prev_speed, velocity=velocity)
