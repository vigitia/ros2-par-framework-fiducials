# Written by Claude Code
"""Notices when colour frames arrive but depth frames do not."""


class DepthArrivalMonitor:
    """
    Tells whether colour frames arrive but depth frames don't, e.g. without a depth camera.

    Frames are only processed in colour + depth pairs, so missing depth means nothing is
    published at all. Times are in seconds from any monotonic clock.
    """

    def __init__(self, timeout_sec):
        self.timeout_sec = timeout_sec
        self.first_frame_time_sec = None
        self.last_frame_time_sec = None
        self.last_depth_time_sec = None

    def on_frame(self, now_sec):
        self.last_frame_time_sec = now_sec
        if self.first_frame_time_sec is None:
            self.first_frame_time_sec = now_sec

    def on_depth(self, now_sec):
        self.last_depth_time_sec = now_sec

    def is_depth_missing(self, now_sec):
        """Return True while colour frames arrive and depth frames have stopped or never came."""
        if self.last_frame_time_sec is None:
            return False
        frames_arrive = now_sec - self.last_frame_time_sec < self.timeout_sec
        # Not before timeout_sec of colour frames: a driver may only start computing registered
        # depth once it sees a subscriber
        frames_arrived_long_enough = now_sec - self.first_frame_time_sec > self.timeout_sec
        if not (frames_arrive and frames_arrived_long_enough):
            return False
        if self.last_depth_time_sec is None:
            return True
        return now_sec - self.last_depth_time_sec > self.timeout_sec
