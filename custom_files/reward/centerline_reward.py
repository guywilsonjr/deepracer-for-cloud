import math
import sys
print(sys.version)
print(sys.version_info)
print(sys.executable)
print('ARGV')
print(sys.argv)
from pydantic import BaseModel

from .reward_utils import SubReward


class CenterlineReward(BaseModel):
    distance_from_center: float
    track_width: float

    @property
    def exp_reward(self):
        distance_factor = 2 * self.distance_from_center / self.track_width
        mod_distance_factor = max(min(1.0, distance_factor), 0.5)
        return SubReward(math.exp(-mod_distance_factor)).root

    @property
    def linear_reward(self):
        distance_factor = 2 * self.distance_from_center / self.track_width
        mod_distance_factor = max(min(1.0, distance_factor), 0.5)
        return SubReward(2 * (1 - mod_distance_factor)).root

    @property
    def reward(self):
        return self.exp_reward

