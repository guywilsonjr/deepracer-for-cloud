import math

from pydantic import BaseModel

from .constants import CURVE_ANGLE_THRESHOLD, LOOKAHEAD_TRACK_WIDTH_FACTOR, STRAIGHT_ANGLE_THRESHOLD
from .models import Index, TrackWidth
from .track import TrackWaypoints


class CurveProcessor(BaseModel):
    track_waypoints: TrackWaypoints
    closest_ahead_waypoint_index: Index
    closest_behind_waypoint_index: Index
    track_width: TrackWidth
    x: float
    y: float

    class Config:
        arbitrary_types_allowed = True

    @property
    def curve_factors(self):
        curve_factors = {}
        behind_curve, ahead_curve = self.new_curve_factor
        total_curve = abs(ahead_curve) + abs(behind_curve)
        if ahead_curve > CURVE_ANGLE_THRESHOLD:
            curve_factors['CURVE_ENTER'] = ahead_curve
        elif ahead_curve < STRAIGHT_ANGLE_THRESHOLD:
            curve_factors['STRAIGHT_ENTER'] = ahead_curve
        if behind_curve > CURVE_ANGLE_THRESHOLD:
            curve_factors['CURVE_EXIT'] = behind_curve
        elif behind_curve < STRAIGHT_ANGLE_THRESHOLD:
            curve_factors['STRAIGHT_EXIT'] = behind_curve
        if total_curve > CURVE_ANGLE_THRESHOLD:
            curve_factors['TOTAL_CURVE'] = total_curve
        elif total_curve < STRAIGHT_ANGLE_THRESHOLD:
            curve_factors['TOTAL_STRAIGHT'] = total_curve
        return curve_factors


    @property
    def curve_factor(self):
        return 1

    @property
    def new_curve_factor(self):
        prev_wp = self.track_waypoints.waypoints[self.closest_behind_waypoint_index]
        next_wp = self.track_waypoints.waypoints[self.closest_ahead_waypoint_index]

        lookbehind_length = math.sqrt((prev_wp.x - self.x) ** 2 + (prev_wp.y - self.y) ** 2)
        lookahead_length = math.sqrt((next_wp.x - self.x) ** 2 + (next_wp.y - self.y) ** 2)
        look_behind_start = prev_wp
        look_ahead_start = next_wp
        look_behind_end = prev_wp.prev_waypoint
        look_ahead_end = next_wp.next_waypoint

        start_lookahead_angle = math.degrees(math.atan2(look_ahead_end.y - look_ahead_start.y, look_ahead_end.x - look_ahead_start.x))
        start_lookbehind_angle = math.degrees(math.atan2(look_behind_start.y - look_behind_end.y, look_behind_start.x - look_behind_end.x))
        start_lookbehind_angle = (start_lookbehind_angle + 360) % 360
        start_lookahead_angle = (start_lookahead_angle + 360) % 360

        while lookbehind_length < self.track_width * LOOKAHEAD_TRACK_WIDTH_FACTOR:
            lookbehind_length += math.sqrt((look_behind_end.x - look_behind_start.x) ** 2 + (look_behind_end.y - look_behind_start.y) ** 2)
            look_behind_start = look_behind_end
            look_behind_end = look_behind_end.prev_waypoint

        while lookahead_length < self.track_width * LOOKAHEAD_TRACK_WIDTH_FACTOR:
            lookahead_length += math.sqrt((look_ahead_end.x - look_ahead_start.x) ** 2 + (look_ahead_end.y - look_ahead_start.y) ** 2)
            look_ahead_start = look_ahead_end
            look_ahead_end = look_ahead_end.next_waypoint

        look_behind_angle = math.degrees(math.atan2(look_behind_start.y - look_behind_end.y, look_behind_start.x - look_behind_end.x))
        look_ahead_angle = math.degrees(math.atan2(look_ahead_end.y - look_ahead_start.y, look_ahead_end.x - look_ahead_start.x))
        end_look_behind_angle = (look_behind_angle + 360) % 360
        end_look_ahead_angle = (look_ahead_angle + 360) % 360

        end_lookbehind_angle_diff = abs(start_lookbehind_angle - end_look_behind_angle)
        end_lookahead_angle_diff = abs(start_lookahead_angle - end_look_ahead_angle)

        return end_lookbehind_angle_diff, end_lookahead_angle_diff
