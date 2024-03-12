import json
import uuid

import rospy

from .reward.centerline_reward import CenterlineRewardProcessor
from .reward.constants import MIN_REWARD
from .reward.curveprocessor import CurveProcessor
from .reward.emitter import Emitter
from .reward.heading_reward import HeadingRewardProcessor
from .reward.params import Params
from .reward.speed_reward import SpeedRewardProcessor
from .reward.steering_reward import SteeringRewardProcessor
from .reward.target_direction import TargetProcessor
from .reward.timer import Timer
from .reward.track import TrackSegments, TrackWaypoints


track_segments = TrackSegments()
track_waypoints = TrackWaypoints()


class RunState:

    def __init__(self, params, prev_run_states):
        self.failure = False
        prev_run_state = prev_run_states[-1]
        self.prev_run_state = prev_run_state
        self.params = Params(**params)
        self.target_processor = TargetProcessor(
            track_waypoints=track_waypoints,
            location=self.params.location,
            closest_ahead_waypoint_index=self.params.closest_ahead_waypoint_index
        )
        self.curve_processor = CurveProcessor(
            x=self.params.x,
            y=self.params.y,
            track_waypoints=track_waypoints,
            closest_behind_waypoint_index=self.params.closest_behind_waypoint_index,
            closest_ahead_waypoint_index=self.params.closest_ahead_waypoint_index,
            track_width=self.params.track_width
        )
        self.centerline_rew = CenterlineRewardProcessor(distance_from_center=self.params.distance_from_center, track_width=self.params.track_width)
        self.steering_rew = SteeringRewardProcessor(
            steering_angle=self.params.steering_angle,
            heading360=self.params.heading360,
            closest_ahead_waypoint_index=self.params.closest_ahead_waypoint_index,
            curve_factors=self.curve_processor.curve_factors,
            track_segments=track_segments
        )
        self.heading_rew = HeadingRewardProcessor(
            track_waypoints=track_waypoints,
            location=self.params.location,
            closest_ahead_waypoint_index=self.params.closest_ahead_waypoint_index,
            heading360=self.params.heading360,
            target_data=self.target_processor.target_data
        )
        self.speed_rew = SpeedRewardProcessor(
            speed=self.params.speed,
            curve_factors=self.curve_processor.curve_factors,
            steering_reward=self.steering_rew.reward.reward,
            max_speed=self.params.metadata.max_speed,
            min_speed=self.params.metadata.min_speed,
            prev_speed=self.prev_run_state.params.speed if self.prev_run_state else None
        )

    @property
    def reward(self):
        if self.failure:
            return MIN_REWARD
        if self.params.all_wheels_on_track and not self.params.is_reversed:
            return self.centerline_rew.reward.reward
            reward = self.speed_rew.reward * self.params.progress_percentage * (self.centerline_rew.reward.reward + self.heading_rew.reward + self.steering_rew.reward.reward) / 3

            return max(reward, MIN_REWARD)
        else:
            return MIN_REWARD

    @property
    def reward_data(self):
        return {
            'reward': self.reward,
            'steps': self.params.steps,
            'progress': self.params.progress_percentage,
            'curve_factor': self.curve_processor.curve_factor,
            'waypoint_heading_reward': self.heading_rew.reward,
            'steering_reward': self.steering_rew.reward,
            'speed_reward': self.speed_rew.reward,
            'progress_reward': self.params.progress_percentage,
            'center_line_reward': self.centerline_rew.reward,
        }


class Simulation:
    __slots__ = 'sim_state_initialized', 'run_states', 'timer', 'emitter', 'run_id'

    def __init__(self):
        self.sim_state_initialized = False
        self.run_states = [None]
        self.timer = Timer()
        self.emitter = Emitter()
        self.run_id = uuid.uuid4().hex

    def initialize(self, params):
        track_waypoints.create_waypoints(params['waypoints'])
        track_segments.create_segments(track_waypoints.waypoints)
        self.publish_initialization(params)
        self.sim_state_initialized = True

    def publish_initialization(self, params):
        pub_data = {
            'run_id': self.run_id,
            'waypoints': params['waypoints'],
            'track_width': params['track_width'],
            'track_length': params['track_length']
        }
        self.emitter.emit(json.dumps(pub_data))

    def publish_data(self):
        rs = self.run_states[-1].params.model_dump()
        del rs['waypoints']
        rs['run_id'] = self.run_id
        msg = json.dumps(rs)
        self.emitter.emit(msg)


    def add_run_state(self, params, sim_time):
        if not self.sim_state_initialized:
            self.initialize(params)

        steps = params['steps']
        params['sim_time'] = sim_time
        params['metadata'] = params['model_metadata']
        del params['model_metadata']
        run_state = RunState(params, self.run_states)
        self.run_states.append(run_state)
        self.publish_data()
        self.timer.record_time(steps)
        print(run_state.reward_data)
        return run_state



sim = Simulation()

# noinspection PyUnusedFunction
def reward_function(params):
    '''
    Example of penalize steering, which helps mitigate zig-zag behaviors
    '''
    tn = rospy.Time().now()
    sim_time = tn.secs + tn.nsecs * 1e-9
    params['sim_time'] = sim_time
    run_state = sim.add_run_state(params, sim_time)
    return run_state.reward
