import math
from typing import Optional

from pydantic import BaseModel

from .constants import INF, NINF


def get_nan(numerator):
    return INF if numerator >= 0 else NINF


class Point(BaseModel):
    x: float
    y: float


class Waypoint(Point):
    index: int
    prev_waypoint: Optional['Waypoint']
    next_waypoint: Optional['Waypoint']

    def set_prev_waypoint(self, waypoint):
        self.prev_waypoint = waypoint

    def set_next_waypoint(self, waypoint):
        self.next_waypoint = waypoint


class LineSegment:
    __slots__ = 'start', 'end', 'slope', 'angle', 'length'

    def __init__(self, start, end):
        self.start = start
        self.end = end
        numerator = (self.end.y - self.start.y)
        self.slope = numerator / (self.end.x - self.start.x) if self.end.x != self.start.x else get_nan(numerator)
        radians = math.atan2(self.end.y - self.start.y, self.end.x - self.start.x)
        self.angle = math.degrees(radians)
        self.length = math.sqrt((self.end.x - self.start.x) ** 2 + (self.end.y - self.start.y) ** 2)


class LinearWaypointSegment(LineSegment):
    __slots__ = 'start', 'end', 'waypoints', 'waypoint_indices', 'prev_segment', 'next_segment'

    def __init__(self, start, end, prev_segment):
        super().__init__(start, end)
        self.waypoint_indices = {start.index, end.index}
        self.prev_segment = prev_segment
        self.next_segment = None

    def add_waypoint(self, waypoint):
        self.end = waypoint
        self.waypoint_indices.add(waypoint.index)

    def set_next_segment(self, segment):
        self.next_segment = segment

    def set_prev_segment(self, segment):
        self.prev_segment = segment


class LinearFunction:
    __slots__ = 'slope', 'intercept', 'A', 'B', 'C', 'ref_point'

    # noinspection PyUnusedFunction
    def __init__(self, slope, intercept, ref_point=None):
        self.slope = slope
        self.intercept = intercept
        self.B = 1
        self.A = -self.slope
        self.C = -self.intercept
        self.ref_point = ref_point

    def get_closest_point_on_line(self, x, y):
        if not math.isfinite(self.slope) or not math.isfinite(self.A) or not math.isfinite(self.C) or not math.isfinite(self.intercept):
            return Point(x=self.ref_point.x, y=y)
        else:
            x = (self.B * (self.B * x - self.A * y) - self.A * self.C) / (self.A ** 2 + self.B ** 2)
            y = (self.A * (-self.B * x + self.A * y) - self.B * self.C) / (self.A ** 2 + self.B ** 2)
            return Point(x=x, y=y)

    @staticmethod
    def get_slope_intercept(x1, y1, m):
        return y1 - m * x1

    @classmethod
    def from_points(cls, x1, y1, x2, y2):
        slope = (y2 - y1) / (x2 - x1) if x2 != x1 else get_nan(y2 - y1)
        intercept = y1 - slope * x1
        return cls(slope, intercept, Point(x=x1, y=y1))

    @classmethod
    def get_perp_func(cls, x1, y1, slope):
        perp_slope = -1 / slope if slope != 0 else -slope
        perp_intercept = cls.get_slope_intercept(x1, y1, perp_slope)
        return cls(perp_slope, perp_intercept, Point(x=x1, y=y1))
