import time


class FiducialsDetectorDebugger:

    num_frames = 0
    num_marker_detected = 0
    start_timestamp = time.time()
    active_markers = []

    def __init__(self, marker_id_to_debug=-1):
        self.marker_id_to_debug = marker_id_to_debug

    def on_new_frame(self):
        now = time.time()

        self.num_frames += 1
        print(f'Detection accuracy: {round(self.num_marker_detected/self.num_frames, 2)*100} % ({self.num_marker_detected}/{self.num_frames})')

        # Reset
        RESET_INTERVAL_SEC = 1
        if now - self.start_timestamp > RESET_INTERVAL_SEC:
            self.start_timestamp = now
            self.num_frames = 0
            self.num_marker_detected = 0
            self.active_markers = []


    def on_new_marker(self, marker_id):
        if marker_id == self.marker_id_to_debug or self.marker_id_to_debug == -1:
            self.num_marker_detected += 1
            if marker_id not in self.active_markers:
                self.active_markers.append(marker_id)
            if len(self.active_markers) > 1:
                print('WARNING: multiple markers detected. This script currently cannot debug accuracy for multiple markers.')