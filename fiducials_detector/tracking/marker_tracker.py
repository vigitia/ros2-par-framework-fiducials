# Written by Claude Code
"""Tracking mode: remembers markers and where to search for them in the next frame."""

import numpy as np


class MarkerTracker:
    """
    Remembers where markers were and where to search for them in the next frame.

    Each tracked marker keeps its last centroid, its velocity from the last two sightings, and
    when it was last seen. compute_search_windows_px() returns a square window around each
    marker's predicted position, so a detector only has to look there instead of in the whole
    frame. Markers not tracked yet are added; markers not seen for max_unseen_time_sec are
    dropped.
    """

    def __init__(self, search_window_size_px, max_unseen_time_sec):
        self._search_window_size_px = search_window_size_px
        self._max_unseen_time_sec = max_unseen_time_sec
        # One dict per tracked marker: id, centroid_px, velocity_px_per_sec, last_seen_time_sec
        self._tracked_markers = []

    def compute_search_windows_px(self, time_sec, frame_width_px, frame_height_px):
        """Return a (left, top, right, bottom) window per tracked marker, clipped to the frame."""
        half_window_size_px = self._search_window_size_px // 2
        search_windows_px = []
        for tracked_marker in self._tracked_markers:
            elapsed_time_sec = time_sec - tracked_marker['last_seen_time_sec']
            predicted_centroid_px = (tracked_marker['centroid_px']
                                     + tracked_marker['velocity_px_per_sec'] * elapsed_time_sec)
            center_x_px = int(round(predicted_centroid_px[0]))
            center_y_px = int(round(predicted_centroid_px[1]))
            left_px = max(center_x_px - half_window_size_px, 0)
            top_px = max(center_y_px - half_window_size_px, 0)
            right_px = min(center_x_px + half_window_size_px, frame_width_px)
            bottom_px = min(center_y_px + half_window_size_px, frame_height_px)
            if right_px > left_px and bottom_px > top_px:
                search_windows_px.append((left_px, top_px, right_px, bottom_px))
        return search_windows_px

    def update_from_window_detections(self, detected_fiducials, time_sec):
        """Update the tracks with the markers found in this frame's windows, drop stale ones."""
        for detected_fiducial in detected_fiducials:
            self._update_or_add_tracked_marker(detected_fiducial, time_sec)
        self._drop_unseen_markers(time_sec)

    def add_untracked_fiducials(self, detected_fiducials, detection_time_sec):
        """
        Add the detected markers that are not tracked yet, and leave tracked ones unchanged.

        For detections that may be older than the tracks, e.g. from a search of a whole frame
        that finished while later frames were already tracked. A tracked marker with the same id
        that was close to the detected position at detection_time_sec (backdated with its
        velocity) is the same marker and is left alone: its track is more recent.

        Simplification: if a marker changed its speed after the detected frame, backdating misses
        it and a second track is added at the old position. Its window finds nothing, and it is
        dropped after max_unseen_time_sec; until then it costs one extra window per frame.
        Upgrade: also match by id alone when the id is tracked only once.
        """
        for detected_fiducial in detected_fiducials:
            matching_marker = self._find_matching_tracked_marker(detected_fiducial,
                                                                 detection_time_sec)
            if matching_marker is None:
                self._add_tracked_marker(detected_fiducial, detection_time_sec)

    def _update_or_add_tracked_marker(self, detected_fiducial, time_sec):
        tracked_marker = self._find_matching_tracked_marker(detected_fiducial, time_sec)
        if tracked_marker is None:
            self._add_tracked_marker(detected_fiducial, time_sec)
            return
        elapsed_time_sec = time_sec - tracked_marker['last_seen_time_sec']
        # A frame older than the last sighting (frames can arrive out of order) would move the
        # track back in time
        if elapsed_time_sec < 0.0:
            return
        centroid_px = np.asarray(detected_fiducial['centroid'], np.float64)
        if elapsed_time_sec > 0.0:
            displacement_px = centroid_px - tracked_marker['centroid_px']
            tracked_marker['velocity_px_per_sec'] = displacement_px / elapsed_time_sec
        tracked_marker['centroid_px'] = centroid_px
        tracked_marker['last_seen_time_sec'] = time_sec

    def _find_matching_tracked_marker(self, detected_fiducial, time_sec):
        """Return the tracked marker with the same id within half a window at time_sec, or None."""
        centroid_px = np.asarray(detected_fiducial['centroid'], np.float64)
        max_distance_px = self._search_window_size_px / 2
        for tracked_marker in self._tracked_markers:
            if tracked_marker['id'] != detected_fiducial['id']:
                continue
            elapsed_time_sec = time_sec - tracked_marker['last_seen_time_sec']
            tracked_centroid_px = (tracked_marker['centroid_px']
                                   + tracked_marker['velocity_px_per_sec'] * elapsed_time_sec)
            distance_px = np.linalg.norm(centroid_px - tracked_centroid_px)
            if distance_px < max_distance_px:
                return tracked_marker
        return None

    def _add_tracked_marker(self, detected_fiducial, time_sec):
        self._tracked_markers.append({
            'id': detected_fiducial['id'],
            'centroid_px': np.asarray(detected_fiducial['centroid'], np.float64),
            'velocity_px_per_sec': np.zeros(2),
            'last_seen_time_sec': time_sec,
        })

    def _drop_unseen_markers(self, time_sec):
        still_tracked_markers = []
        for tracked_marker in self._tracked_markers:
            unseen_time_sec = time_sec - tracked_marker['last_seen_time_sec']
            if unseen_time_sec <= self._max_unseen_time_sec:
                still_tracked_markers.append(tracked_marker)
        self._tracked_markers = still_tracked_markers
