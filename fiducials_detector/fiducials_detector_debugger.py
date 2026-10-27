# Class and variable names changed with help of Claude Code, so they say what they count, with
# units. Claude Code also moved the counters into __init__, so each debugger counts on its own,
# from when it is created.

import time


class DetectionAccuracyDebugger:

    # Co-authored by Claude Code: counters per instance
    def __init__(self, marker_id_to_debug=-1):
        self.marker_id_to_debug = marker_id_to_debug
        self.frame_count = 0
        self.marker_detection_count = 0
        self.interval_start_sec = time.time()
        self.seen_marker_ids = []

    # Co-authored by Claude Code: names
    def on_new_frame(self):
        now_sec = time.time()

        self.frame_count += 1
        print(f'[DetectionAccuracyDebugger]: Detection accuracy: {round(self.marker_detection_count/self.frame_count, 2)*100} % ({self.marker_detection_count}/{self.frame_count})')

        # Reset
        RESET_INTERVAL_SEC = 1
        if now_sec - self.interval_start_sec > RESET_INTERVAL_SEC:
            self.interval_start_sec = now_sec
            self.frame_count = 0
            self.marker_detection_count = 0
            self.seen_marker_ids = []

    # Co-authored by Claude Code: names
    def on_new_marker(self, marker_id):
        if marker_id == self.marker_id_to_debug or self.marker_id_to_debug == -1:
            self.marker_detection_count += 1
            if marker_id not in self.seen_marker_ids:
                self.seen_marker_ids.append(marker_id)
            if len(self.seen_marker_ids) > 1:
                print('[DetectionAccuracyDebugger]: WARNING: multiple markers detected. This script currently cannot debug accuracy for multiple markers.')
