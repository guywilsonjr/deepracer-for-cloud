import math
from typing import Dict, List, Optional

from pydantic import BaseModel

from .constants import MAX_SPEED_DIFF
from .models import Reward


class SpeedRewardProcessor(BaseModel):
    speed: float
    curve_factors: dict
    steering_reward: float
    max_speed: float
    min_speed: float
    prev_speed: Optional[float]
    curve_metrics: List[Dict[str, float]]

    @property
    def exp_reward(self):
        curve_param = 1
        if 'CURVE' in self.curve_factors:
            curve_param *= .5
        if 'CURVE_ENTER' in self.curve_factors:
            curve_param *= .5
        if 'CURVE_EXIT' in self.curve_factors:
            curve_param *= 1.5
        target_speed = self.min_speed + (self.max_speed - self.min_speed) * curve_param
        target_speed = min(target_speed, self.max_speed)
        print(f"target_speed: {target_speed}")
        speed_diff_factor = abs(self.speed - target_speed) / MAX_SPEED_DIFF
        reward = (math.exp(-speed_diff_factor) - math.exp(-1)) / (1 - math.exp(-1))
        # noinspection PyChainedComparisons
        if self.speed == target_speed or (target_speed == self.max_speed and self.speed > target_speed) or (target_speed == self.min_speed and self.speed < target_speed):
            return 1
        return reward



    @property
    def reward(self):
        return Reward(self.exp_reward)
