
from typing import List

from pydantic import BaseModel, model_validator

from .constants import MAX_SPEED
from .geometry import Point
from .metadata import ModelMetadata
from .models import Distance, Heading, Heading360, Index, Percentage, Speed, SteeringAngle, Steps, TrackLength, TrackWidth


class CoreParams(BaseModel):
    is_crashed: bool
    is_offtrack: bool
    is_reversed: bool
    all_wheels_on_track: bool


class Params(CoreParams):
    x: float
    y: float
    closest_objects: List[Index]
    closest_waypoints: List[Index]
    distance_from_center: Distance
    is_left_of_center: bool
    heading: Heading
    objects_distance: List[float]
    objects_heading: List[float]
    objects_left_of_center: List[bool]
    objects_location: List[List[float]]
    objects_speed: List[float]
    progress: float
    speed: Speed
    steering_angle: SteeringAngle
    steps: Steps
    track_length: TrackLength
    track_width: TrackWidth
    waypoints: List[List[float]]
    closest_ahead_waypoint_index: Index
    heading360: Heading360
    location: Point
    progress_percentage: float
    speed_ratio: Percentage

    metadata: ModelMetadata
    sim_time: float

    # noinspection PyNestedDecorators,PyUnusedFunction
    @model_validator(mode='before')
    @classmethod
    def populate_fields(cls, data):
        heading = data['heading']
        data['closest_ahead_waypoint_index'] = data['closest_waypoints'][1]
        data['heading360'] = heading if heading >= 0.0 else 360.0 + heading
        data['location'] = Point(x=data['x'], y=data['y'])
        data['progress_percentage'] = max(0, data['progress'] * 0.01)
        data['progress'] = max(0, data['progress'])
        data['speed_ratio'] = data['speed'] / MAX_SPEED
        return data






