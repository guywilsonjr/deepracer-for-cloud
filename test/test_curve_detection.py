import json
import math

import matplotlib.pyplot as plt
import numpy as np

from sklearn.cluster import KMeans

from custom_files.reward.curveprocessor import CurveProcessor
from custom_files.reward.track import TrackSegments, TrackWaypoints


class TestPoint:
    def __init__(self, x, y, cbwpi, cawpi, attached_angle):
        self.x = x
        self.y = y
        self.closest_behind_waypoint_index = cbwpi
        self.closest_ahead_waypoint_index = cawpi
        self.attached_angle = attached_angle


with open('test/init_msg.json') as f:
    init_json = json.load(f)

init_json_msg = init_json['Message']

init_json_msg_json = json.loads(init_json_msg)
track_width = init_json_msg_json['track_width']
track_length = init_json_msg_json['track_length']
wps = init_json_msg_json['waypoints']
track_waypoints = TrackWaypoints()
track_segments = TrackSegments()
track_waypoints._create_waypoints(wps, 0.225 * 4)
track_segments.create_segments(track_waypoints.waypoints)
NUM_GEN_POINTS = 4
print('Found waypoints:', len(wps))
wp_set = set([(wp[0], wp[1]) for wp in wps])
print('Unique waypoints:', len(wp_set))

num_wps = len(wps) - 1
#num_wps = 50
lines = [(wps[i], wps[i+1]) for i in range(0, num_wps)]
lines.append((lines[-1][1], wps[0]))
print('Lines:', lines)
def get_slope(line):
    return (line[1][1] - line[0][1]) / (line[1][0] - line[0][0])

def generate_points_along_line(line, cwbi, cawpi):
    x_points = np.linspace(line[0][0], line[1][0], NUM_GEN_POINTS)
    if (line[1][0] - line[0][0]) != 0:
        slope = get_slope(line)
        y_points = slope * (x_points - line[0][0]) + line[0][1]
    else:
        y_points = np.linspace(line[1][1], line[0][1], NUM_GEN_POINTS)
    xy_data = list(zip(x_points, y_points))
    attached_angle = math.degrees(math.atan2(y_points[1] - y_points[0], x_points[1] - x_points[0]))
    attached_angle = (attached_angle + 360) % 360
    return [TestPoint(x, y, cwbi, cawpi, attached_angle) for x, y in xy_data]


def generate_curve_processor(tp: TestPoint):
    cp = CurveProcessor(
        x=tp.x,
        y=tp.y,
        prev_wp=track_waypoints.waypoints[tp.closest_ahead_waypoint_index],
        next_wp=track_waypoints.waypoints[tp.closest_behind_waypoint_index],
        track_width=track_width
    )
    return cp.curve_factors

test_points = [tp for i, line in enumerate(lines) for tp in generate_points_along_line(line, i, i + 1)]
for tp in test_points[-NUM_GEN_POINTS:]:
    tp.closest_ahead_waypoint_index = 0
print([tp.closest_ahead_waypoint_index for tp in test_points])

curvings = [generate_curve_processor(tp) for tp in test_points]
curve_enters = [i for i, c in enumerate(curvings) if 'CURVE_ENTER' in c]
straight_enters = [i for i, c in enumerate(curvings) if 'STRAIGHT_ENTER' in c]
in_curves = [i for i, c in enumerate(curvings) if 'TOTAL_CURVE' in c]
straights = [i for i, c in enumerate(curvings) if 'TOTAL_STRAIGHT' in c]
xwps = np.array([wp[0] for wp in wps])
ywps = np.array([wp[1] for wp in wps])
x_curve_tps = [tp.x for i, tp in enumerate(test_points) if i in curve_enters]
y_curve_tps = [tp.y for i, tp in enumerate(test_points) if i in curve_enters]
x_straight_tps = [tp.x for i, tp in enumerate(test_points) if i in straight_enters]
y_straight_tps = [tp.y for i, tp in enumerate(test_points) if i in straight_enters]
x_in_curve_tps = [tp.x for i, tp in enumerate(test_points) if i in in_curves]
y_in_curve_tps = [tp.y for i, tp in enumerate(test_points) if i in in_curves]
#x_straight_tps = [tp.x for i, tp in enumerate(test_points) if i in straights]
#y_straight_tps = [tp.y for i, tp in enumerate(test_points) if i in straights]
plt.plot(xwps, ywps)
plt.scatter(x_curve_tps, y_curve_tps, c='red', s=10)
plt.plot(x_straight_tps, y_straight_tps, 'go')
plt.scatter(x_in_curve_tps, y_in_curve_tps, c='blue', s=2)
plt.show()