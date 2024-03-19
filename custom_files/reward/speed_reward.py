import math
from typing import Optional

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

    @property
    def exp_reward(self):
        if 'CURVE_ENTER' in self.curve_factors:
            curve_param = 0
            # curve_param = self.curve_factors['CURVE_ENTER'] / CURVE_ANGLE_THRESHOLD
        else:
            curve_param = 1
        target_speed = self.min_speed + (self.max_speed - self.min_speed) * curve_param
        speed_diff_factor = abs(self.speed - target_speed) / MAX_SPEED_DIFF
        reward = (math.exp(-speed_diff_factor) - math.exp(-1)) / (1 - math.exp(-1))
        # noinspection PyChainedComparisons
        if self.speed == target_speed or (target_speed == self.max_speed and self.speed > target_speed) or (target_speed == self.min_speed and self.speed < target_speed):
            return 1
        return reward

    '''
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
    '''

    @property
    def reward(self):
        return Reward(self.exp_reward)
