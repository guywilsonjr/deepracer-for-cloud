import math

from pydantic import BaseModel

from .constants import CURVE_ANGLE_THRESHOLD, CURVE_DISTANCE_RATIO_THRESHOLD, LOOKAHEAD_TRACK_WIDTH_FACTOR
from .models import Heading360, Index, TrackWidth
from .track import TrackSegments


class CurveProcessor(BaseModel):
    track_segments: TrackSegments
    closest_ahead_waypoint_index: Index
    track_width: TrackWidth
    heading360: Heading360
    x: float
    y: float

    class Config:
        arbitrary_types_allowed = True
    @property
    def curve_factor(self):
        segment = self.track_segments.get_closest_segment(self.closest_ahead_waypoint_index)
        next_segment = segment.next_segment

        lookahead_segment = next_segment
        lookahead_length = next_segment.length
        while lookahead_length < self.track_width * LOOKAHEAD_TRACK_WIDTH_FACTOR:
            lookahead_segment = lookahead_segment.next_segment
            lookahead_length += lookahead_segment.length
        lookahead_start = lookahead_segment.start
        lookahead_distance = math.sqrt((lookahead_start.x - self.x) ** 2 + (lookahead_start.y - self.y) ** 2)
        curve_distance_ratio = lookahead_distance / self.track_width

        max_angle_diff = 90
        angle_diff = min(abs(lookahead_segment.angle - self.heading360), max_angle_diff)
        angle_diff_radians = math.radians(angle_diff)
        curve_factor = math.cos(angle_diff_radians)
        if curve_distance_ratio < CURVE_DISTANCE_RATIO_THRESHOLD and angle_diff > CURVE_ANGLE_THRESHOLD:
            return curve_factor
        else:
            return 1