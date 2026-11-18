# Written by Claude Code
"""Scaling of 16-bit greyscale frames (e.g. infrared) to 8 bit for marker detection."""

import cv2


def scale_mono16_to_mono8(frame_mono16):
    """
    Return a 16-bit greyscale frame as 8 bit, scaled so its brightest pixel becomes 255.

    16-bit cameras rarely use their full range: infrared images often reach only a few
    thousand. A fixed division by 256 (cv_bridge's mono16 to mono8) then leaves an almost black
    image. Scaling each frame by its own maximum keeps the markers' contrast at any exposure,
    and the detector's adaptive threshold only needs local contrast, not absolute values.

    Simplification: a few very bright pixels (reflections, hot pixels) lower the contrast of
    everything else. Upgrade: scale by a high percentile instead, at the cost of a sort per
    frame.
    """
    _, max_value, _, _ = cv2.minMaxLoc(frame_mono16)
    if max_value == 0:
        return cv2.convertScaleAbs(frame_mono16)
    scale_factor = 255.0 / max_value
    return cv2.convertScaleAbs(frame_mono16, alpha=scale_factor)
