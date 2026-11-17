# Written with the help of Claude Code
"""Used in Tracking mode: searches whole frames for new or lost markers in a background thread."""

import threading
import time


class BackgroundFullScanner:
    """
    Searches whole frames for markers in its own thread, at most scan_rate_hz times a second.

    It finds markers that tracking can't: a page just put down, or a marker that was covered and
    lost. submit_frame() hands over the newest frame (only a reference, no copy), and
    get_new_result() returns the markers of the last finished scan once. Uses its own detector,
    because one OpenCV detector must not run in two threads at once; OpenCV releases the GIL
    during detection, so the scan really runs alongside the tracking.
    """

    def __init__(self, fiducials_detector, scan_rate_hz):
        self.fiducials_detector = fiducials_detector
        self.scan_period_sec = 1.0 / scan_rate_hz
        self.lock = threading.Lock()

        # variable is named _mono to clarify that at this point, greyscale frames are required because the detector
        # expects them like this.
        self.newest_frame_mono = None
        self.newest_frame_time_sec = None
        self.new_result = None
        self.stop_event = threading.Event()
        self.scan_thread = threading.Thread(target=self._run, daemon=True)
        self.scan_thread.start()

    def set_newest_frame(self, frame_mono, time_sec):
        with self.lock:
            self.newest_frame_mono = frame_mono
            self.newest_frame_time_sec = time_sec

    def get_new_result(self):
        """Return (fiducials, frame time) of the last scan not returned yet, or None."""
        with self.lock:
            result = self.new_result
            self.new_result = None
        return result

    def stop(self):
        self.stop_event.set()
        self.scan_thread.join()

    def _run(self):
        while not self.stop_event.is_set():
            scan_start_sec = time.monotonic()
            with self.lock:
                frame_mono = self.newest_frame_mono
                frame_time_sec = self.newest_frame_time_sec
                self.newest_frame_mono = None
            if frame_mono is not None:
                fiducials = self.fiducials_detector.detect_fiducials(frame_mono)
                with self.lock:
                    self.new_result = (fiducials, frame_time_sec)
            # A scan slower than the period starts the next one right away
            remaining_time_sec = self.scan_period_sec - (time.monotonic() - scan_start_sec)
            self.stop_event.wait(max(remaining_time_sec, 0.0))
