import math

from pydantic import BaseModel

from .models import DistanceFromCenter, TrackWidth
from .reward_utils import SubReward


class CenterlineRewardProcessor(BaseModel):
    distance_from_center: DistanceFromCenter
    track_width: TrackWidth

    @property
    def exp_reward(self):
        distance_factor = 2 * self.distance_from_center / self.track_width
        mod_distance_factor = max(min(1.0, distance_factor), 0.5)
        return SubReward(reward=math.exp(-mod_distance_factor))

    @property
    def linear_reward(self):
        distance_factor = 2 * self.distance_from_center / self.track_width
        mod_distance_factor = max(min(1.0, distance_factor), 0.5)
        return SubReward(reward=2 * (1 - mod_distance_factor))

    @property
    def reward(self):
        return self.exp_reward

