from enum import auto, Enum
from typing import Optional
from typing_extensions import Annotated

from pydantic import AfterValidator, BaseModel, Field

from .constants import MAX_CONFIG_SPEED, MAX_CONFIG_STEERING, MAX_SPEED, MIN_CONFIG_SPEED, MIN_CONFIG_STEERING, MIN_REWARD


Distance = Annotated[float, Field(ge=0)]

Index = Annotated[int, Field(ge=0)]
Speed = Annotated[float, Field(ge=0, le=4)]
SteeringAngle = Annotated[float, Field(ge=-30, le=30)]
Steps = Annotated[int, Field(ge=0)]
Heading = Annotated[float, Field(ge=-180, le=180)]
Angle360 = Annotated[float, Field(ge=0, le=360)]
Percentage = Annotated[float, Field(ge=0, le=1)]
InputSpeed = Annotated[float, Field(ge=0, le=MAX_SPEED)]
Reward = Annotated[float, Field(ge=0, le=1), AfterValidator(lambda x: max(x, MIN_REWARD))]
SpeedSetting = Annotated[float, Field(ge=MIN_CONFIG_SPEED, le=MAX_CONFIG_SPEED)]
SteeringAngleSetting = Annotated[float, Field(ge=MIN_CONFIG_STEERING, le=MAX_CONFIG_STEERING)]
'''
class Reward(ConstrainedFloat):
    @model_validator(mode='after')
    def validator(self):
        self.root = max(self.root, MIN_REWARD)
        return self

    @property
    def asfloat(self):
        return self.root
'''

class CurveInfo(BaseModel):
    straight: Optional[Angle360]
    curve: Optional[Angle360]
    curve_enter: Optional[Angle360]
    curve_exit: Optional[Angle360]
    straight_enter: Optional[Angle360]
    straight_exit: Optional[Angle360]


class SubReward(BaseModel, frozen=True):
    reward: Reward


class CenterlineReward(SubReward, frozen=True):
    pass


class HeadingReward(SubReward, frozen=True):
    heading360: Angle360


class SpeedReward(SubReward, frozen=True):
    target_speed: float
    speed: float
    curve_info: CurveInfo
    curve_param: float
    max_speed: float
    min_speed: float
    prev_speed: Optional[float]


class SteeringReward(SubReward, frozen=True):
    curve_param: float
    steering_angle: float
    heading360: float
    curve_info: CurveInfo


class HistoricData(BaseModel):
    dx: Optional[float]
    dy: Optional[float]
    distance: Optional[float]
    dt: Optional[float]
    velocity: Optional[float]
    prev_speed: Optional[float]
