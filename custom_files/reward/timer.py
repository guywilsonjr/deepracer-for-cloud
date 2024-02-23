import time

import numpy
import rospy


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
