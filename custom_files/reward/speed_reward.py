import math
from typing import Optional

from pydantic import BaseModel

from .constants import MAX_SPEED_DIFF


class SpeedReward(BaseModel):
    speed: float
    curve_factor: float
    steering_reward: float
    max_speed: float
    min_speed: float
    prev_speed: Optional[float]

    @property
    def exp_reward(self):
        curve_param = self.curve_factor * self.steering_reward
        target_speed = self.min_speed + (self.max_speed - self.min_speed) * curve_param
        speed_diff_factor = abs(self.speed - target_speed) / MAX_SPEED_DIFF
        reward = math.exp(-speed_diff_factor)
        # noinspection PyChainedComparisons
        if self.speed == target_speed:
            return reward
        elif self.prev_speed:
            if target_speed > self.speed:
                if self.prev_speed < self.speed:
                    speed_factor = 1 + speed_diff_factor
                else:
                    speed_factor = 1 - speed_diff_factor
                return reward * speed_factor / 2
            elif target_speed < self.speed:
                if self.prev_speed > self.speed:
                    speed_factor = 1 + speed_diff_factor
                else:
                    speed_factor = 1 - speed_diff_factor
                return reward * speed_factor / 2
        else:
            return reward / 2

    @property
    def reward(self):
        return self.exp_reward