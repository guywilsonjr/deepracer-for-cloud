import math

from pydantic import BaseModel

from .models import CenterlineReward, Distance, Reward


class CenterlineRewardProcessor(BaseModel):
    distance_from_center: Distance
    track_width: Distance

    @property
    def exp_reward(self) -> float:
        worst_distance_factor = 1
        best_distance_factor = 0.25
        #distance_factor = max(self.distance_from_center / (0.5 * self.track_width), best_distance_factor)
        distance_factor = min(self.distance_from_center / (0.5 * self.track_width), worst_distance_factor)
        return math.sqrt(1 - distance_factor)

    @property
    def linear_reward(self) -> float:
        distance_factor = 2 * self.distance_from_center / self.track_width
        mod_distance_factor = max(min(1.0, distance_factor), 0.5)
        return 2 * (1 - mod_distance_factor)

    def get_reward(self) -> CenterlineReward:
        return CenterlineReward(reward=Reward(self.exp_reward))
