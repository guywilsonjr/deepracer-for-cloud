import math
from typing import Optional

from pydantic import BaseModel

from .constants import INF, NINF


def get_nan(numerator):
    return INF if numerator >= 0 else NINF


class TrackPoint(BaseModel):
    x: float
    y: float


class TrackWaypoint(TrackPoint):
    index: int
    prev_waypoint: Optional['TrackWaypoint']
    next_waypoint: Optional['TrackWaypoint']

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
        self.angle = (self.angle + 360) % 360
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

