import math
from typing import Optional

from pydantic import BaseModel, Field

from .constants import MAX_SPEED_DIFF
from .models import CurveInfo, InputSpeed, Reward, SpeedReward, SpeedSetting


class SpeedRewardProcessor(BaseModel):
    speed: InputSpeed
    curve_info: CurveInfo = Field(exclude=True)
    max_speed: SpeedSetting
    min_speed: SpeedSetting
    prev_speed: Optional[InputSpeed]

    @property
    def exp_reward(self) -> SpeedReward:
        curve_param = 1.0
        '''
        Straights can be a 0 degree angle therefore check if it's not none
        '''
        '''
        if self.curve_info.straight is not None and self.curve_info.straight_enter is not None:
            curve_param = 1.0
        elif self.curve_info.straight_enter is not None:
            curve_param *= 1.0
        elif self.curve_info.straight is not None:
            curve_param *= 1.0
        if self.curve_info.straight_exit is not None:
            curve_param *= 1.0
        if self.curve_info.curve_exit:
            curve_param = 1.0
        if self.curve_info.curve:
            curve_param = 0.5
        if self.curve_info.curve_enter:
            curve_param = 0.25
        '''
        target_speed = self.min_speed + (self.max_speed - self.min_speed) * curve_param
        target_speed = min(target_speed, self.max_speed)
        speed_diff_factor = abs(self.speed - target_speed) / MAX_SPEED_DIFF
        reward = (math.exp(-speed_diff_factor) - math.exp(-1)) / (1 - math.exp(-1))
        # noinspection PyChainedComparisons
        if self.speed == target_speed or (target_speed == self.max_speed and self.speed > target_speed) or (target_speed == self.min_speed and self.speed < target_speed):
            return SpeedReward(reward=Reward(1), speed=self.speed, curve_info=self.curve_info, max_speed=self.max_speed, min_speed=self.min_speed, prev_speed=self.prev_speed, curve_param=curve_param, target_speed=target_speed)
        return SpeedReward(reward=Reward(reward), speed=self.speed, curve_info=self.curve_info, max_speed=self.max_speed, min_speed=self.min_speed, prev_speed=self.prev_speed, curve_param=curve_param, target_speed=target_speed)


    def get_reward(self) -> SpeedReward:
        return self.exp_reward
