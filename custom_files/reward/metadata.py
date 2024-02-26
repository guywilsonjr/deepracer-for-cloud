from typing import List, Union

from pydantic import BaseModel, confloat, conint, Discriminator, model_validator, Tag
from typing_extensions import Annotated, Literal

from .settings import SpeedSetting, SteeringAngleSetting


class ContinuousActionSpaceConfig(BaseModel):
    high: Union[SpeedSetting, SteeringAngleSetting]
    low: Union[SpeedSetting, SteeringAngleSetting]

    @model_validator(mode='after')
    def validate_high_low(self):
        if self.high < self.low:
            raise ValueError('high must be greater than low')
        return self


class ContinuousActionSpaceSpeedConfig(ContinuousActionSpaceConfig):
    high: SpeedSetting
    low: SpeedSetting


class ContinuousActionSpaceSteeringAngleConfig(ContinuousActionSpaceConfig):
    pass


class ContinuousActionSpaceConfig(BaseModel):
    speed:  ContinuousActionSpaceSpeedConfig
    steering_angle: ContinuousActionSpaceSteeringAngleConfig


class DiscreteActionSpaceConfigItem(BaseModel):
    speed: SpeedSetting
    steering_angle: SteeringAngleSetting

DiscreteActionSpaceConfig = List[DiscreteActionSpaceConfigItem]

class LidarConfig(BaseModel):
    num_sectors: conint(ge=0)
    num_values_per_sector: conint(ge=0)
    clipping_dist: confloat(ge=0)


def action_space_discriminator(action_space):
    if isinstance(action_space, dict):
        return 'continuous'
    elif isinstance(action_space, list):
        return 'discrete'
    else:
        raise ValueError(f'Unknown action space type: {type(action_space)}')

class ModelMetadata(BaseModel):
    sensor: List[str]
    neural_network: str
    version: confloat(ge=0)
    training_algorithm: Literal['clipped_ppo', 'sac']
    action_space: Annotated[
        Union[
            Annotated[DiscreteActionSpaceConfig, Tag('discrete')],
            Annotated[ContinuousActionSpaceConfig, Tag('continuous')]
        ],
        Discriminator(action_space_discriminator)
    ]
    lidar_config: LidarConfig

    @property
    def max_speed(self):
        if isinstance(self.action_space, list):
            return max(item.speed for item in self.action_space)
        elif isinstance(self.action_space, ContinuousActionSpaceConfig):
            return self.action_space.speed.high
        else:
            raise ValueError(f'Unknown action space type: {type(self.action_space)}')

    @property
    def min_speed(self):
        if isinstance(self.action_space, list):
            return min(item.speed for item in self.action_space)
        elif isinstance(self.action_space, ContinuousActionSpaceConfig):
            return self.action_space.speed.low
        else:
            raise ValueError(f'Unknown action space type: {type(self.action_space)}')

    @property
    def abs_max_steering_angle(self):
        if isinstance(self.action_space, list):
            return max(abs(item.steering_angle) for item in self.action_space)
        elif isinstance(self.action_space, ContinuousActionSpaceConfig):
            return max(abs(self.action_space.steering_angle.high), self.action_space.steering_angle.high)
        else:
            raise ValueError(f'Unknown action space type: {type(self.action_space)}')

