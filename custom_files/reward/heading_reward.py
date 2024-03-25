import math

from pydantic import BaseModel

from .constants import HEADING_TOLERANCE, MAX_HEADING_ERROR
from .geometry import TrackPoint
from .models import Angle360, HeadingReward, Index, Reward
from .target_direction import TargetData
from .track import TrackWaypoints
from .util import to360


class HeadingRewardProcessor(BaseModel):
    track_waypoints: TrackWaypoints
    location: TrackPoint
    closest_ahead_waypoint_index: Index
    heading360: Angle360
    target_data: TargetData

    class Config:
        arbitrary_types_allowed = True

    @property
    def cos_reward(self) -> float:
        angle_to_target = math.atan2(
            self.target_data.target_point.y - self.location.y,
            self.target_data.target_point.x - self.location.x
        )
        angle_to_target = to360(math.degrees(angle_to_target))
        heading_error = to360(abs(angle_to_target - self.heading360))
        heading_error = min(heading_error, MAX_HEADING_ERROR)
        heading_error = max(heading_error - HEADING_TOLERANCE, 0)
        heading_factor = (math.cos(math.radians(heading_error)) - math.cos(math.radians(MAX_HEADING_ERROR))) / (1 - math.cos(math.radians(MAX_HEADING_ERROR)))
        return heading_factor

    def get_reward(self) -> HeadingReward:
        return HeadingReward(reward=Reward(self.cos_reward), heading360=self.heading360)
