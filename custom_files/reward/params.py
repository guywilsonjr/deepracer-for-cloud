import os
from typing import List, Tuple

from pydantic import BaseModel, Field
from typing_extensions import Annotated

from .geometry import TrackPoint
from .metadata import ModelMetadata
from .models import Distance, Heading, Angle360, Index, Speed, SteeringAngle, Steps
from .util import to360


class CoreParams(BaseModel):
    is_crashed: bool
    is_offtrack: bool
    is_reversed: bool
    all_wheels_on_track: bool


class Params(CoreParams):
    metadata: ModelMetadata = Field(exclude=True)
    sim_time: Annotated[float, Field(ge=0)]
    episode_id: Annotated[str, Field(min_length=36, max_length=36)]

    x: float = Field(exclude=True)
    y: float = Field(exclude=True)

    closest_objects: List[Index] = Field(exclude=True)
    closest_waypoints: List[Index] = Field(exclude=True)
    objects_distance: List[float] = Field(exclude=True)
    objects_heading: List[float] = Field(exclude=True)
    objects_left_of_center: List[bool] = Field(exclude=True)
    objects_location: List[List[float]] = Field(exclude=True)
    objects_speed: List[float] = Field(exclude=True)

    date_time: str
    episode_status: str
    pause: bool

    distance_from_center: Distance
    progress: Annotated[float, Field(ge=0, le=100)] = Field(exclude=True)
    track_length: Distance = Field(exclude=True)
    track_width: Distance = Field(exclude=True)
    waypoints: List[Tuple[float, float]] = Field(exclude=True)

    closest_ahead_waypoint_index: Index
    closest_behind_waypoint_index: Index
    heading360: Angle360
    is_left_of_center: bool
    heading: Heading
    loc: TrackPoint
    progress_percentage: Annotated[float, Field(ge=0, le=1)]
    speed: Speed
    steering_angle: SteeringAngle
    steps: Steps
    rollout_idx: int = int(os.environ['ROLLOUT_IDX'])
    sim_id: int = int(os.environ['SIMULATION_ID'])

    @classmethod
    def get_params(cls, data) -> 'Params':
        data['closest_ahead_waypoint_index'] = data['closest_waypoints'][1]
        data['closest_behind_waypoint_index'] = data['closest_waypoints'][0]
        data['heading360'] = to360(data['heading'])
        data['loc'] = TrackPoint(x=data['x'], y=data['y'])
        data['progress_percentage'] = max(0, data['progress'] * 0.01)
        data['progress'] = max(0, data['progress'])
        return cls(**data)






