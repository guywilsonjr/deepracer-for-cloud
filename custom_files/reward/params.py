import os
from typing import List

from pydantic import BaseModel, Field
from typing_extensions import Annotated

from .constants import MAX_SPEED
from .geometry import TrackPoint
from .metadata import ModelMetadata
from .models import Distance, Heading, Angle360, Index, Percentage, Speed, SteeringAngle, Steps
from .util import to360


class CoreParams(BaseModel):
    is_crashed: bool
    is_offtrack: bool
    is_reversed: bool
    all_wheels_on_track: bool


# noinspection PyUnusedName
class Params(CoreParams):
    date_time: str
    x: float = Field(exclude=True)
    y: float = Field(exclude=True)
    closest_objects: List[Index] = Field(exclude=True)
    closest_waypoints: List[Index] = Field(exclude=True)
    distance_from_center: Distance = Field(exclude=True)
    is_left_of_center: bool
    heading: Heading
    objects_distance: List[float] = Field(exclude=True)
    objects_heading: List[float] = Field(exclude=True)
    objects_left_of_center: List[bool] = Field(exclude=True)
    objects_location: List[List[float]] = Field(exclude=True)
    objects_speed: List[float] = Field(exclude=True)
    progress: Annotated[float, Field(ge=0, le=100)] = Field(exclude=True)
    speed: Speed
    steering_angle: SteeringAngle
    steps: Steps
    track_length: Distance
    track_width: Distance
    waypoints: List[List[float]] = Field(exclude=True)
    closest_ahead_waypoint_index: Index
    closest_behind_waypoint_index: Index
    heading360: Angle360
    location: TrackPoint
    progress_percentage: Annotated[float, Field(ge=0, le=1)]
    metadata: ModelMetadata = Field(exclude=True)
    sim_time: Annotated[float, Field(ge=0)]
    rollout_idx: Annotated[int, Field(ge=0)]
    training_uuid: Annotated[str, Field(min_length=36, max_length=36)]
    sim_run_id: Annotated[str, Field(min_length=36, max_length=36)]

    @classmethod
    def get_params(cls, data) -> 'Params':
        data['closest_ahead_waypoint_index'] = data['closest_waypoints'][1]
        data['closest_behind_waypoint_index'] = data['closest_waypoints'][0]
        data['heading360'] = to360(data['heading'])
        data['location'] = TrackPoint(x=data['x'], y=data['y'])
        data['progress_percentage'] = max(0, data['progress'] * 0.01)
        data['progress'] = max(0, data['progress'])
        return cls(**data)






