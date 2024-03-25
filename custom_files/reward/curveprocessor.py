import math

import numpy as np
from pydantic import BaseModel

from .constants import CURVE_ANGLE_THRESHOLD, LOOKAHEAD_TRACK_WIDTH_FACTOR, STRAIGHT_ANGLE_THRESHOLD
from .geometry import TrackWaypoint
from .models import CurveInfo, Distance
from .util import to360


class CurveProcessor(BaseModel):
    prev_wp: TrackWaypoint
    next_wp: TrackWaypoint
    track_width: Distance
    x: float
    y: float

    @property
    def curve_info(self):

        behind_curve, ahead_curve = self.new_curve_factor
        total_curve = abs(ahead_curve) + abs(behind_curve)
        total_curve = (total_curve + 360) % 360

        return CurveInfo(
            curve=total_curve if total_curve > CURVE_ANGLE_THRESHOLD else None,
            curve_enter=ahead_curve if ahead_curve > CURVE_ANGLE_THRESHOLD else None,
            curve_exit=behind_curve if behind_curve > CURVE_ANGLE_THRESHOLD else None,
            straight=total_curve if total_curve < STRAIGHT_ANGLE_THRESHOLD else None,
            straight_enter=ahead_curve if ahead_curve < STRAIGHT_ANGLE_THRESHOLD else None,
            straight_exit=behind_curve if behind_curve < STRAIGHT_ANGLE_THRESHOLD else None
        )



    @property
    def new_curve_factor(self):
        wp_xs = np.array([self.prev_wp.x, self.next_wp.x])
        wp_ys = np.array([self.prev_wp.y, self.next_wp.y])
        dxs = wp_xs - np.array([self.x, self.x])
        dys = wp_ys - np.array([self.y, self.y])
        sqs = np.square(np.array([dxs, dys]))
        sums = np.sum(sqs, axis=1)
        sum_squares = np.sqrt(sums)
        lookbehind_length = sum_squares[0]
        lookahead_length = sum_squares[1]
        look_behind_start = self.prev_wp
        look_ahead_start = self.next_wp
        look_behind_end = self.prev_wp.prev_waypoint
        look_ahead_end = self.next_wp.next_waypoint

        start_lookahead_angle = to360(math.degrees(math.atan2(look_ahead_end.y - look_ahead_start.y, look_ahead_end.x - look_ahead_start.x)))
        start_lookbehind_angle = to360(math.degrees(math.atan2(look_behind_start.y - look_behind_end.y, look_behind_start.x - look_behind_end.x)))

        while lookbehind_length < self.track_width * LOOKAHEAD_TRACK_WIDTH_FACTOR:
            lookbehind_length += math.sqrt((look_behind_end.x - look_behind_start.x) ** 2 + (look_behind_end.y - look_behind_start.y) ** 2)
            look_behind_start = look_behind_end
            look_behind_end = look_behind_end.prev_waypoint

        while lookahead_length < self.track_width * LOOKAHEAD_TRACK_WIDTH_FACTOR:
            lookahead_length += math.sqrt((look_ahead_end.x - look_ahead_start.x) ** 2 + (look_ahead_end.y - look_ahead_start.y) ** 2)
            look_ahead_start = look_ahead_end
            look_ahead_end = look_ahead_end.next_waypoint

        look_behind_angle = to360(math.degrees(math.atan2(look_behind_start.y - look_behind_end.y, look_behind_start.x - look_behind_end.x)))
        look_ahead_angle = to360(math.degrees(math.atan2(look_ahead_end.y - look_ahead_start.y, look_ahead_end.x - look_ahead_start.x)))


        end_lookbehind_angle_diff = to360(abs(start_lookbehind_angle - look_behind_angle))
        end_lookahead_angle_diff = to360(abs(start_lookahead_angle - look_ahead_angle))

        return end_lookbehind_angle_diff, end_lookahead_angle_diff
