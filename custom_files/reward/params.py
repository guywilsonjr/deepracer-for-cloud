from typing import List

from pydantic import BaseModel, model_validator

from .geometry import Point
from .metadata import ModelMetadata
from .models import Distance, Heading, Heading360, Index, Percentage, Progress, Speed, SteeringAngle, Steps, TrackLength, TrackWidth


class Params(BaseModel):
    all_wheels_on_track: bool
    x: float
    y: float
    closest_objects: List[Index]
    closest_waypoints: List[Index]
    distance_from_center: Distance
    is_crashed: bool
    is_left_of_center: bool
    is_offtrack: bool
    is_reversed: bool
    heading: Heading
    objects_distance: List[float]
    objects_heading: List[float]
    objects_left_of_center: List[bool]
    objects_location: List[List[float]]
    objects_speed: List[float]
    progress: Progress
    speed: Speed
    steering_angle: SteeringAngle
    steps: Steps
    track_length: TrackLength
    track_width: TrackWidth
    waypoints: List[List[float]]

    closest_ahead_waypoint_index: Index
    heading360: Heading360
    location: Point
    progress_percentage: Percentage

    model_metadata: ModelMetadata
    sim_time: float

    # noinspection PyNestedDecorators,PyUnusedFunction
    @model_validator(mode='before')
    @classmethod
    def populate_fields(cls, data):
        heading = data['heading']
        data['closest_ahead_waypoint_index'] = data['closest_waypoints'][1]
        data['heading360'] = heading if heading >= 0.0 else 360.0 + heading
        data['location'] = Point(x=data['x'], y=data['y'])
        data['progress_percentage'] = data['progress'] * 0.01
        return data






