# Written by Claude Code
from fiducials_detector.fiducials_detector import corners_to_camera_space
import numpy as np

K = np.array([[1000.0, 0.0, 320.0], [0.0, 1000.0, 240.0], [0.0, 0.0, 1.0]])
NO_DISTORTION = np.zeros(5)
# A 100 px marker around the image centre, clockwise from the top left like OpenCV's corners
CORNERS = [(270, 190), (370, 190), (370, 290), (270, 290)]


def rays(pixels):
    pixels = np.asarray(pixels, np.float64)
    return np.column_stack([(pixels[:, 0] - K[0, 2]) / K[0, 0], (pixels[:, 1] - K[1, 2]) / K[1, 1],
                            np.ones(len(pixels))])


def plane_depth(normal, offset):
    """Depth image of the plane normal . p = offset (z where each pixel's ray meets it)."""
    v, u = np.mgrid[0:480, 0:640]
    r = rays(np.column_stack([u.ravel(), v.ravel()]))
    return (offset / (r @ normal)).reshape(480, 640).astype(np.float32)


def project(points):
    """Pixels of 3D points in the optical frame, without distortion."""
    return (points[:, :2] / points[:, 2:]) * (K[0, 0], K[1, 1]) + (K[0, 2], K[1, 2])


def tilted_square(degrees, size=0.057, distance=0.7):
    """A real square marker on the axis, its plane tilted about x, and that plane's normal."""
    a = np.radians(degrees)
    normal = np.array([0.0, np.sin(a), np.cos(a)])
    # Two in-plane directions: x, and x cross normal
    ex, ey = np.array([1.0, 0.0, 0.0]), np.cross(normal, [1.0, 0.0, 0.0])
    half = size / 2
    square = np.array([distance * np.array([0.0, 0.0, 1.0]) + sx * half * ex + sy * half * ey
                       for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
    return square, normal


def test_tilted_marker_is_its_real_square():
    square, normal = tilted_square(30)
    depth = plane_depth(normal, normal @ square[0])
    corners = corners_to_camera_space(project(square), depth, K, NO_DISTORTION)
    np.testing.assert_allclose(corners, square, atol=5e-4)


def test_marker_facing_the_camera():
    # 1 degree: a square facing the camera exactly, without any noise, is IPPE's degenerate case,
    # which a real camera does not produce
    square, normal = tilted_square(1)
    depth = plane_depth(normal, normal @ square[0])
    corners = corners_to_camera_space(project(square), depth, K, NO_DISTORTION)
    np.testing.assert_allclose(corners, square, atol=5e-4)


def test_corners_that_are_no_square_give_none():
    # A kite: no view of a square looks like this, e.g. a wrong detection
    kite = [(320, 170), (370, 240), (320, 330), (270, 240)]
    assert corners_to_camera_space(kite, np.full((480, 640), 0.8, np.float32), K,
                                   NO_DISTORTION) is None


def test_wrong_depth_tilt_keeps_the_orientation_of_the_image():
    # Seen live on a cube's side face: the depth plane 10 degrees steeper than the face
    # (multipath). The orientation must come from the square in the image, only the distance
    # from the depth.
    square, _ = tilted_square(30)
    wrong = np.array([0.0, np.sin(np.radians(40)), np.cos(np.radians(40))])
    depth = plane_depth(wrong, wrong @ square.mean(axis=0))
    corners = corners_to_camera_space(project(square), depth, K, NO_DISTORTION)
    np.testing.assert_allclose(corners, square, atol=1e-3)


def test_mostly_missing_depth_gives_none():
    # Like a side face seen almost edge-on: most of the marker has no depth, the rest is the table
    depth = np.full((480, 640), 0.93, np.float32)
    depth[190:290, 270:370] = 0
    depth[200:280, 300:310] = 0.93
    assert corners_to_camera_space(CORNERS, depth, K, NO_DISTORTION) is None


def test_finger_on_the_marker_is_ignored():
    square, normal = tilted_square(1, distance=0.8)
    depth = plane_depth(normal, normal @ square[0])
    # A finger 4 cm in front of the marker, over a strip of its inner part
    depth[200:280, 315:330] -= 0.04
    corners = corners_to_camera_space(project(square), depth, K, NO_DISTORTION)
    np.testing.assert_allclose(corners, square, atol=1e-3)


def test_too_little_depth_gives_none():
    depth = np.full((480, 640), 0.8, np.float32)
    depth[180:300, 260:380] = 0
    assert corners_to_camera_space(CORNERS, depth, K, NO_DISTORTION) is None
