import math

from pydantic import BaseModel

from .constants import MAX_HEADING_ERROR, WAYPOINT_LOOKAHEAD_DISTANCE
from .geometry import LinearFunction, LineSegment, Point
from .track import TrackWaypoints


class HeadingReward(BaseModel):
    track_waypoints: TrackWaypoints
    location: Point
    closest_ahead_waypoint_index: int
    heading360: float
    class Config:
        arbitrary_types_allowed = True

    @property
    def cos_reward(self) -> float:
        next_wp = self.track_waypoints.waypoints_map[self.closest_ahead_waypoint_index]
        start_x, start_y = self.location.x, self.location.y
        end_x, end_y = next_wp.x, next_wp.y
        for i in range(WAYPOINT_LOOKAHEAD_DISTANCE):
            start_x, start_y = end_x, end_y
            next_wp = next_wp.next_waypoint
            end_x, end_y = next_wp.x, next_wp.y

        segment = LinearFunction.from_points(start_x, start_y, end_x, end_y)
        perp_waypoint_func = LinearFunction.get_perp_func(end_x, end_y, segment.slope)
        target_point = perp_waypoint_func.get_closest_point_on_line(self.location.x, self.location.y)
        end_point = Point(target_point.x, target_point.y)
        target_line = LineSegment(self.location, end_point)

        heading_error = min(abs(target_line.angle - self.heading360), MAX_HEADING_ERROR)
        heading_factor = math.cos(math.radians(heading_error))
        return heading_factor

    @property
    def reward(self) -> float:
        return self.cos_reward
