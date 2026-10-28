# Class and variable names changed with help of Claude Code, so they say what they count, with
# units. Claude Code also moved the counters into __init__, so each debugger counts on its own,
# from when it is created, and made it report the detection rate of every marker once per
# interval.

import time


class DetectionAccuracyDebugger:

    # Co-authored by Claude Code: counters per instance, one per marker id
    def __init__(self, marker_id_to_debug=-1):
        self.marker_id_to_debug = marker_id_to_debug
        self.frame_count = 0
        # marker id -> number of frames of the interval in which it was detected
        self.detection_counts_by_marker_id = {}
        self.interval_start_sec = time.time()

    # Co-authored by Claude Code: names, one line per interval with the rate of every marker
    def on_new_frame(self):
        now_sec = time.time()

        # Print the rates of the interval that just ended, then reset, before this frame is
        # counted, so the frame and its markers (on_new_marker) land in the same interval. Once
        # per interval, so the output stays readable at any frame rate
        RESET_INTERVAL_SEC = 1
        if now_sec - self.interval_start_sec > RESET_INTERVAL_SEC and self.frame_count > 0:
            print(f'[DetectionAccuracyDebugger]: Detection rate over {self.frame_count} frames: '
                  f'{self._create_detection_rates_text()}')
            self.interval_start_sec = now_sec
            self.frame_count = 0
            self.detection_counts_by_marker_id = {}

        self.frame_count += 1

    # Co-authored by Claude Code: names, counted per marker id
    def on_new_marker(self, marker_id):
        if marker_id == self.marker_id_to_debug or self.marker_id_to_debug == -1:
            marker_id = int(marker_id)
            previous_count = self.detection_counts_by_marker_id.get(marker_id, 0)
            self.detection_counts_by_marker_id[marker_id] = previous_count + 1

    # Written by Claude Code
    def _create_detection_rates_text(self):
        """Return e.g. 'id 96: 100 %, id 97: 52 %', sorted by id, for the markers seen."""
        if not self.detection_counts_by_marker_id:
            return 'no markers'
        detection_rate_texts = []
        for marker_id, detection_count in sorted(self.detection_counts_by_marker_id.items()):
            detection_rate_percent = round(100 * detection_count / self.frame_count)
            detection_rate_texts.append(f'id {marker_id}: {detection_rate_percent} %')
        return ', '.join(detection_rate_texts)
