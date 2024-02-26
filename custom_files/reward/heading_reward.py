import math

from pydantic import BaseModel

from .constants import MAX_HEADING_ERROR
from .geometry import Point
from .models import Heading360, Index
from .target_direction import TargetData
from .track import TrackWaypoints


class HeadingRewardProcessor(BaseModel):
    track_waypoints: TrackWaypoints
    location: Point
    closest_ahead_waypoint_index: Index
    heading360: Heading360
    target_data: TargetData

    class Config:
        arbitrary_types_allowed = True

    @property
    def cos_reward(self) -> float:
        heading_error = min(abs(self.target_data.target_line.angle - self.heading360), MAX_HEADING_ERROR)
        heading_factor = math.cos(math.radians(heading_error))
        return heading_factor

    @property
    def reward(self) -> float:
        return self.cos_reward
