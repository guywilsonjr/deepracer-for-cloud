import time
from typing import Tuple

import numpy
import rospy


class Timer:
    __slots__ = 'track_time', 'time', 'total_frames', 'fps', 'rtf', 'time_window', 'prefix'

    def __init__(self, prefix: str, time_window: int) -> None:
        self.track_time = True
        self.time_window = time_window
        self.prefix = prefix
        self.time = numpy.zeros([self.time_window, 2])
        self.total_frames = 0
        self.fps = 15

    def get_time(self) -> Tuple[float, float, int]:
        wall_time_incr = numpy.max(self.time[:, 0]) - numpy.min(self.time[:, 0])
        sim_time_incr = numpy.max(self.time[:, 1]) - numpy.min(self.time[:, 1])

        rtf = sim_time_incr / wall_time_incr
        frames = (self.time.shape[0] - 1)
        fps = frames / sim_time_incr if sim_time_incr > 0 else float('inf')

        return rtf, fps, frames

    def record_time(self, steps: int) -> None:
        index = int(steps) % self.time.shape[0]
        self.time[index, 0] = time.time()
        self.time[index, 1] = rospy.get_time()
        self.rtf, self.fps, frames = self.get_time()
        self.total_frames += frames
        if int(steps) % 25 == 0:
            print("{}: TIME: s: {}, rtf: {}, fps:{}, frames: {}".format(self.prefix, int(steps), round(self.rtf, 2), round(self.fps, 2), frames))
