from pydantic import BaseModel


class Params(BaseModel):
    all_wheels_on_track: bool
    x: float
    y: float
    closest_objects: list[int]
    closest_waypoints: list[int]
    distance_from_center: float
    is_crashed: bool
    is_left_of_center: bool
    is_offtrack: bool
    is_reversed: bool
    heading: float
    objects_distance: list[float]
    objects_heading: list[float]
    objects_left_of_center: list[bool]
    objects_location: list[list[float]]
    objects_speed: list[float]
    progress: float
    speed: float
    steering_angle: float
    steps: int
    track_length: float
    track_width: float
    waypoints: list[list[float]]
