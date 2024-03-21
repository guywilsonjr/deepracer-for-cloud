from pydantic import BaseModel
import shapely
print(shapely.__version__)
import shapely.ops
from shapely.geometry import LineString

from .constants import WAYPOINT_LOOKAHEAD_DISTANCE
from .geometry import LineSegment, TrackPoint
from .models import Index
from .track import TrackWaypoints


class TargetData(BaseModel):
    target_point: TrackPoint
    target_line: LineSegment

    class Config:
        arbitrary_types_allowed = True


class TargetProcessor(BaseModel):
    track_waypoints: TrackWaypoints
    location: TrackPoint
    closest_ahead_waypoint_index: Index

    class Config:
        arbitrary_types_allowed = True

    @property
    def target_data(self) -> TargetData:
        next_wp = self.track_waypoints.waypoints_map[self.closest_ahead_waypoint_index]
        start_x, start_y = self.location.x, self.location.y
        end_x, end_y = next_wp.x, next_wp.y
        for i in range(WAYPOINT_LOOKAHEAD_DISTANCE):
            start_x, start_y = end_x, end_y
            next_wp = next_wp.next_waypoint
            end_x, end_y = next_wp.x, next_wp.y

        tpls = LineString([(start_x, start_y), (end_x, end_y)])
        loc_point = shapely.geometry.Point(self.location.x, self.location.y)
        cp_0, cp_1 = shapely.ops.nearest_points(tpls, loc_point)
        if loc_point.almost_equals(cp_0):
            target_point = TrackPoint(x=cp_1.x, y=cp_1.y)
        elif loc_point.almost_equals(cp_1):
            target_point = TrackPoint(x=cp_0.x, y=cp_0.y)
        else:
            raise ValueError("No closest point found on line")

        target_line = LineSegment(self.location, target_point)
        return TargetData(target_point=target_point, target_line=target_line)
