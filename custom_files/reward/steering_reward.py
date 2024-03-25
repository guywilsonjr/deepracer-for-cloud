import math

from pydantic import BaseModel, Field

from .constants import ABS_MAX_STEERING_ANGLE
from .models import Angle360, CurveInfo, Reward, SteeringAngle, SteeringReward
from .track import TrackSegments


class SteeringRewardProcessor(BaseModel):
    steering_angle: SteeringAngle
    heading360: Angle360
    closest_ahead_waypoint_index: int
    curve_info: CurveInfo = Field(exclude=True)
    track_segments: TrackSegments

    class Config:
        arbitrary_types_allowed = True

    @property
    def abs_steering_reward(self) -> SteeringReward:
        '''
        Straights can be a 0 degree angle therefore check if it's not none
        '''
        curve_param = 1.0
        if self.curve_info.curve:
            curve_param = 0.0
        elif self.curve_info.curve_enter:
            curve_param = 0.25
        if self.curve_info.curve_exit or self.curve_info.straight_exit is not None:
            curve_param *= 0.25
        if self.curve_info.straight is not None:
            curve_param = curve_param ** 0.5
        if self.curve_info.straight_enter is not None:
            curve_param = curve_param ** 0.5

        min_steering_reward = math.cos(math.radians(ABS_MAX_STEERING_ANGLE))
        steering_reward = math.cos(math.radians(self.steering_angle) * curve_param)
        reward = (steering_reward - min_steering_reward) / (1 - min_steering_reward)
        return SteeringReward(reward=Reward(reward), heading360=self.heading360, steering_angle=self.steering_angle, curve_info=self.curve_info, curve_param=curve_param)

    @property
    def target_steering_reward(self) -> float:
        abs_steering_diff = min(abs(self.target_steering_angle - self.steering_angle), 90)
        return math.cos(math.radians(abs_steering_diff))

    @property
    def target_steering_angle(self) -> float:
        segment = self.track_segments.get_closest_segment(self.closest_ahead_waypoint_index)
        pre_target_steering = self.heading360 - segment.angle
        return min(ABS_MAX_STEERING_ANGLE, pre_target_steering) if pre_target_steering > 0 else max(-ABS_MAX_STEERING_ANGLE, pre_target_steering)

    def get_reward(self) -> SteeringReward:
        return self.abs_steering_reward
