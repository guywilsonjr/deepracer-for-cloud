# TODO SIMPLE CHECKS TO RETURN MIN_REWARD. CAN ALSO DO PREV CHECKS TO RETURN MIN_REWARD
import math
import os
import time

import numpy
import requests
import rospy
from shapely.geometry import LineString, Point  # Shapely v1.6.4


ENDPOINT_URL = 'http://192.168.1.106'
HYPERPARAMETERS = {
    "batch_size": 512,
    "beta_entropy": 0.01,
    "discount_factor": 0.99,
    "e_greedy_value": 0.05,
    "epsilon_steps": 10000,
    "exploration_type": "categorical",
    "loss_type": "huber",
    "lr": 0.0001,
    "num_episodes_between_training": 16,
    "num_epochs": 10,
    "sac_alpha": 0.2,
    "stack_size": 1,
    "term_cond_avg_score": 350.0,
    "term_cond_max_episodes": 10000
}

MODEL_METADATA = {
    "action_space": {
        "speed": {
            "high": 4,
            "low": 1.75
        },
        "steering_angle": {
            "high": 30,
            "low": -30
        }
    },
    "sensor": [
        "FRONT_FACING_CAMERA"
    ],
    "neural_network": "DEEP_CONVOLUTIONAL_NETWORK_SHALLOW",
    "version": "5",
    "training_algorithm": "clipped_ppo",
    "action_space_type": "continuous"
}


def get_nan(numerator):
    return INF if numerator >= 0 else NINF


class TrackPoint(Point):
    __slots__ = 'x', 'y'

    def __init__(self, x, y):
        super().__init__([x, y])
        self.x = x
        self.y = y


class Waypoint(TrackPoint):
    __slots__ = 'x', 'y', 'index', 'next_waypoint', 'prev_waypoint'

    def __init__(self, x, y, index, prev_waypoint):
        super().__init__(x, y)
        self.index = index
        self.next_waypoint = None
        self.prev_waypoint = prev_waypoint

    def set_prev_waypoint(self, waypoint):
        self.prev_waypoint = waypoint

    def set_next_waypoint(self, waypoint):
        self.next_waypoint = waypoint


class LineSegment(LineString):
    __slots__ = 'start', 'end', 'slope', 'angle', 'length'

    def __init__(self, coords):
        super().__init__(coords)
        self.start = TrackPoint(*coords[0])
        self.end = TrackPoint(*coords[1])
        numerator = (self.end.y - self.start.y)
        self.slope = numerator / (self.end.x - self.start.x) if self.end.x != self.start.x else get_nan(numerator)
        radians = math.atan2(self.end.y - self.start.y, self.end.x - self.start.x)
        self.angle = math.degrees(radians)
        self.length = self.start.distance(self.end)


class LinearWaypointSegment(LineSegment):
    __slots__ = 'start', 'end', 'waypoints', 'waypoint_indices', 'prev_segment', 'next_segment'

    def __init__(self, start, end, prev_segment):
        coords = ((start.x, start.y), (end.x, end.y))
        super().__init__(coords)
        self.waypoint_indices = {start.index, end.index}
        self.prev_segment = prev_segment
        self.next_segment = None

    def add_waypoint(self, waypoint):
        self.end = waypoint
        self.waypoint_indices.add(waypoint.index)

    def set_next_segment(self, segment):
        self.next_segment = segment

    def set_prev_segment(self, segment):
        self.prev_segment = segment


class TrackWaypoints:
    __slots__ = 'waypoints', 'waypoints_map'

    def __init__(self, waypoints):
        self.waypoints = []
        self.waypoints_map = {}

        for i, wp in enumerate(waypoints):
            prev_waypoint = self.waypoints[-1] if self.waypoints else None
            waypoint = Waypoint(wp[0], wp[1], i, prev_waypoint)

            if prev_waypoint:
                prev_waypoint.set_next_waypoint(waypoint)

            self.waypoints.append(waypoint)
            self.waypoints_map[i] = waypoint
        first_waypoint = self.waypoints[0]
        last_waypoint = self.waypoints[-1]
        last_waypoint.set_next_waypoint(first_waypoint)
        first_waypoint.set_prev_waypoint(last_waypoint)


class Timer:
    __slots__ = 'track_time', 'time', 'total_frames', 'fps', 'rtf'

    def __init__(self):
        self.track_time = True
        TIME_WINDOW = 10
        self.time = numpy.zeros([TIME_WINDOW, 2])
        self.total_frames = 0
        self.fps = 15

    def get_time(self):
        wall_time_incr = numpy.max(self.time[:, 0]) - numpy.min(self.time[:, 0])
        sim_time_incr = numpy.max(self.time[:, 1]) - numpy.min(self.time[:, 1])

        rtf = sim_time_incr / wall_time_incr
        frames = (self.time.shape[0] - 1)
        fps = frames / sim_time_incr

        return rtf, fps, frames

    def record_time(self, steps):
        index = int(steps) % self.time.shape[0]
        self.time[index, 0] = time.time()
        self.time[index, 1] = rospy.get_time()
        self.rtf, self.fps, frames = self.get_time()
        self.total_frames += frames
        print("TIME: s: {}, rtf: {}, fps:{}, frames: {}".format(int(steps), round(self.rtf, 2), round(self.fps, 2), frames))


class Simulation:
    __slots__ = 'timer', 'run_uuid'

    def __init__(self):
        self.timer = Timer()
        self.run_uuid = None

    @staticmethod
    def initialize_online_learning():
        data = {
            'SAGEMAKER_SHARED_S3_PREFIX': os.environ['SAGEMAKER_SHARED_S3_PREFIX'],
            'ROLLOUT_IDX': os.environ['ROLLOUT_IDX'],
            'WORLD_NAME': os.environ['WORLD_NAME'],
            'RTF_OVERRIDE': os.environ['RTF_OVERRIDE'],
            'ROS_HOSTNAME': os.environ['ROS_HOSTNAME'],
            'HOSTNAME': os.environ['HOSTNAME'],
            'hyperparameters': HYPERPARAMETERS,
            'model_metadata': MODEL_METADATA,
            'sim_time': rospy.get_time(),
            'wall_time': time.time()
        }
        url = f'{ENDPOINT_URL}/initialize'
        resp = requests.post(url, json=data)
        if resp.status_code == 200:
            return resp.text
        else:
            raise Exception('Failed to initialize online learning. Code: {}. Message: {}'.format(resp.status_code, resp.text))

    def fetch_run_state(self, params):
        url = f'{ENDPOINT_URL}/reward'
        data = {
            'sim_run_id': self.run_uuid,
            'sim_time': rospy.get_time(),
            'wall_time': time.time(),
            'rtf': self.timer.rtf,
            'fps': self.timer.fps,
            **params
        }
        resp = requests.post(url, json=data)
        if resp.status_code == 200:
            return resp.json()
        else:
            raise Exception('Failed to fetch run state. Code: {}. Message: {}'.format(resp.status_code, resp.text))

    def initialize_sim_state(self):
        self.run_uuid = self.initialize_online_learning()

    def get_reward(self, params):
        if not self.run_uuid:
            self.initialize_sim_state()

        print('Run UUID: {}'.format(self.run_uuid))
        steps = params['steps']
        self.timer.record_time(steps)
        reward_data = self.fetch_run_state(params)
        print(reward_data)
        return reward_data['reward']


sim = Simulation()


# noinspection PyUnusedFunction
def reward_function(params):
    '''
    Example of penalize steering, which helps mitigate zig-zag behaviors
    '''
    return 1
