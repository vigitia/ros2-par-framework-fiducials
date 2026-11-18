# Written with the help of Claude Code
"""Debug preview: an OpenCV window showing each frame with the detected markers."""

import cv2
import cv2.aruco as aruco
import numpy as np

# The frame scaled to this width for every camera (up or down). The window can be resized by
# dragging
PREVIEW_WIDTH_PX = 800
PREVIEW_WINDOW_NAME = 'fiducials_detector'


class DetectionPreviewWindow:
    """Shows greyscale frames with the detected markers outlined and labelled with their ids."""

    def __init__(self):
        self._window_sized = False

    def show_preview_frame_with_markers(self, camera_frame_mono, marker_corners_px, marker_ids):

        # Scale down the preview frame
        frame_height_px, frame_width_px = camera_frame_mono.shape[:2]
        preview_scale = PREVIEW_WIDTH_PX / frame_width_px
        preview_height_px = round(frame_height_px * preview_scale)
        preview_image_mono = cv2.resize(camera_frame_mono, (PREVIEW_WIDTH_PX, preview_height_px))

        # Convert greyscale frame back to colour, so hat the marker outlines stand out
        preview_image = cv2.cvtColor(preview_image_mono, cv2.COLOR_GRAY2BGR)
        if marker_ids is not None:
            preview_corners_px = []
            for corners_px in marker_corners_px:
                scaled_corners_px = corners_px * preview_scale
                preview_corners_px.append(scaled_corners_px.astype(np.float32))
            aruco.drawDetectedMarkers(preview_image, preview_corners_px, marker_ids)

        # Window is resizable by dragging, but keeps the aspect ratio;
        # no Qt toolbar and status bar, which would take part of the window size below.
        cv2.namedWindow(PREVIEW_WINDOW_NAME,
                        cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_NORMAL)
        # Qt opens such a window small, whatever the image size: set it to the image's, once, so
        # later frames don't undo a size dragged by hand
        if not self._window_sized:
            cv2.resizeWindow(PREVIEW_WINDOW_NAME, PREVIEW_WIDTH_PX, preview_height_px)
            self._window_sized = True

        cv2.imshow(PREVIEW_WINDOW_NAME, preview_image)
        cv2.waitKey(1)
