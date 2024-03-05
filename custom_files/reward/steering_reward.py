import math

from pydantic import BaseModel

from .constants import ABS_MAX_STEERING_ANGLE
from .reward_utils import SubReward
from .track import TrackSegments


class SteeringRewardProcessor(BaseModel):
    steering_angle: float
    heading360: float
    closest_ahead_waypoint_index: int
    curve_factor: float
    track_segments: TrackSegments
    class Config:
        arbitrary_types_allowed = True

    @property
    def abs_steering_reward(self):
        return math.cos(math.radians(self.steering_angle) * self.curve_factor)

    @property
    def target_steering_reward(self):
        abs_steering_diff = min(abs(self.target_steering_angle - self.steering_angle), 90)
        return math.cos(math.radians(abs_steering_diff))

    @property
    def target_steering_angle(self):
        segment = self.track_segments.get_closest_segment(self.closest_ahead_waypoint_index)
        pre_target_steering = self.heading360 - segment.angle
        return min(ABS_MAX_STEERING_ANGLE, pre_target_steering) if pre_target_steering > 0 else max(-ABS_MAX_STEERING_ANGLE, pre_target_steering)

    @property
    def reward(self):
        # We want to reward for both being on target and for requiring a small steering angle
        steering_reward = self.abs_steering_reward
        return SubReward(reward=1)