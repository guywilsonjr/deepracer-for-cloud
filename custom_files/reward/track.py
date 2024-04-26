from typing import Dict, List, Optional, Tuple

from pydantic import BaseModel
from shapely.geometry import LinearRing, Polygon

from .geometry import LinearWaypointSegment, TrackPoint, TrackWaypoint


class TpProcessor(BaseModel):
    waypoints: List[TrackWaypoint]
    waypoints_map: Dict[int, TrackWaypoint]
    track_width: float

    @property
    def center_ring(self) -> LinearRing:
        return LinearRing([(wp.x, wp.y) for wp in self.waypoints])

    @property
    def track_geometry(self) -> Polygon:
        return self.center_ring.buffer(self.track_width / 2)

    @property
    def outer_ring(self) -> LinearRing:
        return self.track_geometry.exterior

    @property
    def inner_ring(self) -> LinearRing:
        return self.track_geometry.interiors[0]


class TrackWaypoints:
    __slots__ = 'waypoints', 'waypoints_map', 'center_ring', 'track_geometry', 'outer_ring', 'inner_ring'

    def __init__(self) -> None:
        self.waypoints: List[TrackWaypoint] = []
        self.waypoints_map: Dict[int, TrackWaypoint] = {}

    def _create_waypoints(self, waypoints: List[Tuple[float, float]], track_width: float) -> None:
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
        if last_wp:
            last_wp.set_next_waypoint(first_wp)
        first_wp.set_prev_waypoint(last_wp)
        self.waypoints = tuple(self.waypoints)
        self.center_ring: LinearRing = LinearRing(waypoints)
        self.track_geometry = self.center_ring.buffer(track_width / 2)
        self.outer_ring = self.track_geometry.exterior
        self.inner_ring = self.track_geometry.interiors[0]


class TrackSegments:
    __slots__ = 'segments'

    def __init__(self) -> None:
        self.segments = []

    def add_waypoint_segment(self, start: TrackPoint, end: TrackPoint) -> None:
        return

    def create_segments(self, waypoints: List[TrackWaypoint]) -> None:
        return

    def get_closest_segment(self, closest_ahead_waypoint_index: int) -> LinearWaypointSegment:
        for segment in self.segments:
            if closest_ahead_waypoint_index in segment.waypoint_indices:
                return segment
        raise Exception('No segment found for closest waypoint index: {}'.format(closest_ahead_waypoint_index))

