import json
import os
import uuid
from datetime import datetime
from pprint import pprint
from typing import Any, Dict

import rospy

from .reward.centerline_reward import CenterlineRewardProcessor
from .reward.constants import MIN_REWARD
from .reward.curveprocessor import CurveProcessor
from .reward.emitter import Emitter
from .reward.heading_reward import HeadingRewardProcessor
from .reward.history_processor import HistoryProcessor
from .reward.params import Params
from .reward.speed_reward import SpeedRewardProcessor
from .reward.steering_reward import SteeringRewardProcessor
from .reward.target_direction import TargetProcessor
from .reward.timer import Timer
from .reward.track import TrackSegments, TrackWaypoints


track_segments = TrackSegments()
track_waypoints = TrackWaypoints()


class RunState:
    __slots__ = 'params', 'center_reward', 'heading_reward', 'steering_reward', 'speed_reward', 'historic_data', 'target_data', 'curve_data'

    def __init__(self, params: Params, prev_params: Params) -> None:
        self.params = params
        target_processor = TargetProcessor(
            track_waypoints=track_waypoints,
            location=self.params.location,
            closest_ahead_waypoint_index=self.params.closest_ahead_waypoint_index
        )
        curve_processor = CurveProcessor(
            x=self.params.x,
            y=self.params.y,
            prev_wp=track_waypoints.waypoints[self.params.closest_behind_waypoint_index],
            next_wp=track_waypoints.waypoints[self.params.closest_ahead_waypoint_index],
            track_width=self.params.track_width
        )
        history_processor = HistoryProcessor(params=params, prev_params=prev_params)
        self.historic_data = history_processor.get_historic_data()
        self.target_data = target_processor.target_data
        self.curve_data = curve_processor.curve_info
        center_processor = CenterlineRewardProcessor(distance_from_center=self.params.distance_from_center, track_width=self.params.track_width)
        steering_processor = SteeringRewardProcessor(
            steering_angle=self.params.steering_angle,
            heading360=self.params.heading360,
            closest_ahead_waypoint_index=self.params.closest_ahead_waypoint_index,
            curve_info=self.curve_data,
            track_segments=track_segments
        )



        heading_processor = HeadingRewardProcessor(
            track_waypoints=track_waypoints,
            location=self.params.location,
            closest_ahead_waypoint_index=self.params.closest_ahead_waypoint_index,
            heading360=self.params.heading360,
            target_data=self.target_data
        )
        speed_processor = SpeedRewardProcessor(
            speed=self.params.speed,
            curve_info=self.curve_data,
            max_speed=self.params.metadata.max_speed,
            min_speed=self.params.metadata.min_speed,
            prev_speed=self.historic_data.prev_speed
        )
        self.center_reward = center_processor.get_reward()
        self.heading_reward = heading_processor.get_reward()
        self.steering_reward = steering_processor.get_reward()
        self.speed_reward = speed_processor.get_reward()




    @property
    def reward(self) -> float:
        if not self.params.is_crashed and not self.params.is_offtrack and not self.params.is_reversed:
            reward = (self.center_reward.reward + self.speed_reward.reward + self.heading_reward.reward + self.steering_reward.reward) / 4
            return max(reward, MIN_REWARD)
        else:
            return MIN_REWARD

    @property
    def publishing_data(self) -> Dict[str, Any]:
        return {
            'version': 0,
            'version_type': 'phr',
            'params': self.params.model_dump(),
            'sim_run_id': self.params.sim_run_id,
            'date_time': self.params.date_time,
            **self.print_data
        }

    @property
    def print_data(self):
        return {
            'steps': self.params.steps,
            'sim_time': self.params.sim_time,
            'progress': self.params.progress,
            'target': self.target_data.model_dump(),
            'curve': self.curve_data.model_dump(),
            'history': self.historic_data.model_dump(),
            **self.reward_data
        }

    @property
    def reward_data(self):
        return {
            'reward': self.reward,
            'heading': self.heading_reward.model_dump(),
            'speed': self.speed_reward.model_dump(),
            'steering': self.steering_reward.model_dump(),
            'centerline': self.center_reward.model_dump()
        }



class Simulation:
    __slots__ = 'sim_state_initialized', 'run_states', 'timer', 'emitter', 'sim_run_id', 'curve_metrics', 'prev_params', 'run_state'

    def __init__(self):
        self.sim_state_initialized = False
        self.timer = Timer()
        self.emitter = Emitter()
        self.sim_run_id = str(uuid.uuid1())
        self.prev_params = None
        self.run_state = None


    def initialize(self, params: Params):
        track_waypoints._create_waypoints(params.waypoints, params.track_width)
        track_segments.create_segments(track_waypoints.waypoints)
        self.publish_initialization(params)
        self.sim_state_initialized = True

    def publish_initialization(self, params: Params):

        pub_data = {
            'date_time': params.date_time,
            'sagemaker_prefix': params.sagemaker_prefix,
            'world_name': params.world_name,
            'training_uuid': params.training_uuid,
            'rollout_idx': params.rollout_idx,
            'sim_run_id': self.sim_run_id,
            'steps': params.steps,
            'track_width': params.track_width,
            'track_length': params.track_length,
            'hyperparameters': params.hyperparameters.model_dump(),
            'model_metadata': params.metadata.model_dump(),
            'waypoints': params.waypoints
        }
        self.emitter.emit(json.dumps(pub_data))

    def publish_data(self):
        rs = self.run_state.publishing_data
        msg = json.dumps(rs)
        self.emitter.emit(msg)


    def add_run_state(self, param_dict, sim_time):
        param_dict['sim_time'] = sim_time
        param_dict['sim_run_id'] = self.sim_run_id
        params = Params.get_params(param_dict)
        if not self.sim_state_initialized:
            self.initialize(params)


        run_state = RunState(params, self.prev_params)
        self.prev_params = run_state.params
        self.run_state = run_state
        self.publish_data()
        self.timer.record_time(params.steps)
        return run_state



sim = Simulation()


# noinspection PyUnusedFunction
def reward_function(param_dict: Dict[str, Any]) -> float:
    '''
    Example of penalize steering, which helps mitigate zig-zag behaviors
    '''
    tn = rospy.Time().now()
    dt = datetime.now().isoformat()
    sim_time = tn.secs + tn.nsecs * 1e-9
    param_dict['sim_time'] = sim_time
    param_dict['date_time'] = dt
    run_state = sim.add_run_state(param_dict, sim_time)
    pprint(run_state.print_data)

    return run_state.reward
