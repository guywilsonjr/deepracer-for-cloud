import math
from typing import Optional

from shapely import LinearRing

from .constants import STRAIGHT_ANGLE_THRESHOLD
from .geometry import LinearWaypointSegment, TrackWaypoint


class TrackWaypoints:
    __slots__ = 'waypoints', 'waypoints_map', 'center_ring', 'track_geometry', 'outer_ring', 'inner_ring'

    def __init__(self):
        self.center_ring = self.track_geometry = self.outer_ring = self.inner_ring = None
        self.waypoints = []
        self.waypoints_map = {}

    def create_waypoints(self, waypoints, track_width):
        prev_waypoint: Optional[TrackWaypoint] = None

        for i, wp in enumerate(waypoints):
            waypoint = TrackWaypoint(x=wp[0], y=wp[1], index=i, prev_waypoint=prev_waypoint, next_waypoint=None)

            if prev_waypoint:
                prev_waypoint.set_next_waypoint(waypoint)

            self.waypoints.append(waypoint)
            self.waypoints_map[i] = waypoint
            prev_waypoint = waypoint

        last_wp = prev_waypoint
        first_wp = self.waypoints[0]
        last_wp.set_next_waypoint(first_wp)
        first_wp.set_prev_waypoint(last_wp)
        self.waypoints = tuple(self.waypoints)
        self.center_ring = LinearRing(waypoints)
        self.track_geometry = self.center_ring.buffer(track_width / 2)
        self.outer_ring = self.track_geometry.exterior
        self.inner_ring = self.track_geometry.interiors[0]


class TrackSegments:
    __slots__ = 'segments'

    def __init__(self):
        self.segments = []

    def add_waypoint_segment(self, start, end):
        radians = math.atan2(end.y - start.y, end.x - start.x)
        angle = math.degrees(radians)
        angle = (angle + 360) % 360
        prev_segment = self.segments[-1] if self.segments else None

        if prev_segment and abs(prev_segment.angle - angle) < STRAIGHT_ANGLE_THRESHOLD:
            prev_segment.add_waypoint(end)
        else:
            segment = LinearWaypointSegment(start, end, prev_segment)
            if self.segments:
                prev_segment.set_next_segment(segment)
            self.segments.append(segment)

    def create_segments(self, waypoints):
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            self.add_waypoint_segment(start, end)
        self.add_waypoint_segment(waypoints[-1], waypoints[0])
        first_segment = self.segments[0]
        last_segment = self.segments[-1]
        last_segment.set_next_segment(first_segment)
        first_segment.set_prev_segment(last_segment)
        self.segments = tuple(self.segments)

    def get_closest_segment(self, closest_ahead_waypoint_index):
        for segment in self.segments:
            if closest_ahead_waypoint_index in segment.waypoint_indices:
                return segment
        raise Exception('No segment found for closest waypoint index: {}'.format(closest_ahead_waypoint_index))

