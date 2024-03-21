import math
from typing import Optional

from pydantic import BaseModel

from .constants import HEADING_TOLERANCE, MAX_HEADING_ERROR
from .geometry import TrackPoint
from .models import Heading360, Index, Reward
from .target_direction import TargetData
from .track import TrackWaypoints


class HeadingRewardProcessor(BaseModel):
    track_waypoints: TrackWaypoints
    location: TrackPoint
    closest_ahead_waypoint_index: Index
    heading360: Heading360
    target_data: TargetData
    velocity: Optional[float]
    max_speed: float

    class Config:
        arbitrary_types_allowed = True

    @property
    def cos_reward(self) -> float:
        angle_to_target = math.atan2(
            self.target_data.target_point.y - self.location.y,
            self.target_data.target_point.x - self.location.x
        )
        angle_to_target = (math.degrees(angle_to_target) + 360) % 360
        heading_error = abs(angle_to_target - self.heading360)
        heading_error = min(heading_error, MAX_HEADING_ERROR)
        heading_error = max(heading_error - HEADING_TOLERANCE, 0)
        heading_factor = (math.cos(math.radians(heading_error)) - math.cos(math.radians(MAX_HEADING_ERROR))) / (1 - math.cos(math.radians(MAX_HEADING_ERROR)))
        return heading_factor

    @property
    def reward(self) -> Reward:
        return Reward(self.cos_reward)
