import math
import sys

from .reward import constants
from .reward.centerline_reward import CenterlineReward
from .reward.geometry import LinearFunction, LineSegment, Point
from .reward.speed_reward import SpeedReward
from .reward.steering_reward import SteeringReward
from .reward.timer import Timer
from .reward.track import TrackSegments, TrackWaypoints


track_segments = TrackSegments()
track_waypoints = TrackWaypoints()


class RunState:

    def __init__(self, params, prev_run_state, fps):
        self.centerline_rew = CenterlineReward(distance_from_center=params['distance_from_center'], track_width=params['track_width'])
        self._set_raw_inputs(params)
        self._set_derived_inputs(fps)
        self._set_future_inputs()
        self.prev_run_state = prev_run_state
        self._set_prev_inputs()

    def set_next_state(self, next_state):
        self.next_state = next_state

    def _set_prev_inputs(self):
        if self.prev_run_state:
            self.prev_speed = self.prev_run_state.speed
            self.prev_steering_angle = self.prev_run_state.steering_angle
            self.prev_heading360 = self.prev_run_state.heading360
        else:
            self.prev_speed = None

    def _set_derived_inputs(self, fps):
        self.progress_val = (self.progress / 100)
        self.speed_ratio = self.speed / constants.MAX_SPEED
        self.heading360 = self.heading if self.heading >= 0 else 360 + self.heading
        self.abs_steering_angle = abs(self.steering_angle)
        self.x_velocity = self.speed * math.cos(math.radians(self.heading360))
        self.y_velocity = self.speed * math.sin(math.radians(self.heading360))
        self.next_x = self.x + self.x_velocity / constants.FPS
        self.next_y = self.y + self.y_velocity / constants.FPS
        self.closest_behind_waypoint_index = self.closest_waypoints[0]
        self.closest_ahead_waypoint_index = self.closest_waypoints[1]
        self.half_track_width = self.track_width / 2
        self.quarter_track_width = self.half_track_width / 2
        self.max_distance_traveled = self.steps * constants.MAX_SPEED / constants.FPS
        self.max_progress_percentage = self.max_distance_traveled / self.track_length
        self.progress_percentage = self.progress / 100
        self.location = Point(self.x, self.y)

    def _set_future_inputs(self):
        self.x_velocity = self.speed * math.cos(math.radians(self.heading360))
        self.y_velocity = self.speed * math.sin(math.radians(self.heading360))
        self.next_x = self.x + self.x_velocity / constants.FPS
        self.next_y = self.y + self.y_velocity / constants.FPS

    def validate_field(self, field, value):
        if value is None:
            return f'{field}: {value} is None'
        elif value > 1:
            return f'{field}: {value} is greater than 1'
        elif value < 0:
            return f'{field}: {value} is less than 0. Found '
        elif not math.isfinite(value):
            return f'{field}: {value} is not finite'
        else:
            return None

    def validate(self):
        validation_dict = {
            'speed_ratio': self.speed_ratio,
            'progress_reward': self.progress_reward,
            'curve_factor': self.curve_factor,
            'waypoint_heading_reward': self.waypoint_heading_reward,
            'steering_reward': self.steering_reward,
            'center_line_reward': self.center_line_reward,
            'reward': self.reward
        }
        messages = [self.validate_field(k, v) for k, v in validation_dict.items()]
        exc_messages = [m for m in messages if m]
        if exc_messages:
            raise Exception('\n'.join(exc_messages))

    @property
    def reward(self):
        reward = self.speed_ratio * self.progress_reward * (self.center_line_reward + self.waypoint_heading_reward + self.steering_reward) / 3
        if self.all_wheels_on_track and not self.is_crashed and not self.is_offtrack and not self.is_reversed:
            return reward
        else:
            return 0.0001

    @property
    def reward_data(self):
        return {
            'reward': self.reward,
            'steps': self.steps,
            'progress': self.progress_percentage,
            'curve_factor': self.curve_factor,
            'waypoint_heading_reward': self.waypoint_heading_reward,
            'steering_reward': self.steering_reward,
            'speed_reward': self.speed_reward,
            'progress_reward': self.progress_reward,
            'center_line_reward': self.center_line_reward,
        }

    @property
    def progress_reward(self):
        return self.progress_percentage

    @property
    def steering_reward(self):
        # We want to reward for both being on target and for requiring a small steering angle
        steering_reward = SteeringReward(
            steering_angle=self.steering_angle,
            heading360=self.heading360,
            closest_ahead_waypoint_index=self.closest_ahead_waypoint_index,
            curve_factor=self.curve_factor,
            track_segments=track_segments
        )
        return steering_reward.reward



    @property
    def center_line_reward(self):
        '''
        Reward for being close to the center line
        Reward is exponentially based on distance from center line with max reward at quarter track width
        and minimum reward at half track width
        '''
        reward_data = CenterlineReward(distance_from_center=self.distance_from_center, track_width=self.track_width)
        return reward_data.reward

    @property
    def waypoint_heading_reward(self):
        '''
        Reward for heading towards the next waypoint
        Reward is based on the heading error between the car and the current waypoint segment
        '''
        next_wp = track_waypoints.waypoints_map[self.closest_ahead_waypoint_index]
        start_x, start_y = self.x, self.y
        end_x, end_y = next_wp.x, next_wp.y
        for i in range(constants.WAYPOINT_LOOKAHEAD_DISTANCE):
            start_x, start_y = end_x, end_y
            next_wp = next_wp.next_waypoint
            end_x, end_y = next_wp.x, next_wp.y

        segment = LinearFunction.from_points(start_x, start_y, end_x, end_y)
        perp_waypoint_func = LinearFunction.get_perp_func(end_x, end_y, segment.slope)
        target_point = perp_waypoint_func.get_closest_point_on_line(self.x, self.y)
        end_point = Point(target_point.x, target_point.y)
        target_line = LineSegment(self.location, end_point)

        heading_error = min(abs(target_line.angle - self.heading360), constants.MAX_HEADING_ERROR)
        heading_factor = math.cos(math.radians(heading_error))
        return heading_factor

    @property
    def speed_reward(self):
        reward_data = SpeedReward(
            speed=self.speed,
            curve_factor=self.curve_factor,
            steering_reward=self.steering_reward,
            max_speed=constants.MAX_SPEED,
            min_speed=constants.MIN_SPEED,
            prev_speed=self.prev_speed
        )
        return reward_data.reward


    def _set_raw_inputs(self, params):
        self.all_wheels_on_track = params['all_wheels_on_track']
        self.x = params['x']
        self.y = params['y']
        self.closest_objects = params['closest_objects']
        self.closest_waypoints = params['closest_waypoints']
        self.distance_from_center = params['distance_from_center']
        self.is_crashed = params['is_crashed']
        self.is_left_of_center = params['is_left_of_center']
        self.is_offtrack = params['is_offtrack']
        self.is_reversed = params['is_reversed']
        self.heading = params['heading']
        self.objects_distance = params['objects_distance']
        self.objects_heading = params['objects_heading']
        self.objects_left_of_center = params['objects_left_of_center']
        self.objects_location = params['objects_location']
        self.objects_speed = params['objects_speed']
        self.progress = params['progress']
        self.speed = params['speed']
        self.steering_angle = params['steering_angle']
        self.steps = params['steps']
        self.track_length = params['track_length']
        self.track_width = params['track_width']
        self.waypoints = params['waypoints']

    @property
    def curve_factor(self):
        segment = track_segments.get_closest_segment(self.closest_ahead_waypoint_index)
        next_segment = segment.next_segment

        lookahead_segment = next_segment
        lookahead_length = next_segment.length
        while lookahead_length < self.track_width * constants.LOOKAHEAD_TRACK_WIDTH_FACTOR:
            lookahead_segment = lookahead_segment.next_segment
            lookahead_length += lookahead_segment.length
        lookahead_start = lookahead_segment.start
        lookahead_distance = math.sqrt((lookahead_start.x - self.x) ** 2 + (lookahead_start.y - self.y) ** 2)
        curve_distance_ratio = lookahead_distance / self.track_width

        max_angle_diff = 90
        angle_diff = min(abs(lookahead_segment.angle - self.heading360), max_angle_diff)
        angle_diff_radians = math.radians(angle_diff)
        curve_factor = math.cos(angle_diff_radians)
        if curve_distance_ratio < constants.CURVE_DISTANCE_RATIO_THRESHOLD and angle_diff > constants.CURVE_ANGLE_THRESHOLD:
            return curve_factor
        else:
            return 1


class Simulation:
    __slots__ = 'sim_state_initialized', 'run_state', 'timer'

    def __init__(self):
        self.sim_state_initialized = False
        self.run_state = None
        self.timer = Timer()

    def add_run_state(self, params):
        if not self.sim_state_initialized:
            track_waypoints.create_waypoints(params['waypoints'])
            track_segments.create_segments(track_waypoints.waypoints)
            self.sim_state_initialized = True

        steps = params['steps']
        self.timer.record_time(steps)
        run_state = RunState(params, self.run_state, 15)
        run_state.validate()
        self.run_state = run_state
        print(self.run_state.reward_data)
        size_data = {
            'sim': sys.getsizeof(self),
            'run_state': sys.getsizeof(run_state),
            'params': sys.getsizeof(params),
            'track_waypoints': sys.getsizeof(track_waypoints),
            'track_segments': sys.getsizeof(track_segments),
            'timer': sys.getsizeof(self.timer),
        }
        print(size_data)


sim = Simulation()


# noinspection PyUnusedFunction
def reward_function(params):
    '''
    Example of penalize steering, which helps mitigate zig-zag behaviors
    '''
    sim.add_run_state(params)
    return sim.run_state.reward
