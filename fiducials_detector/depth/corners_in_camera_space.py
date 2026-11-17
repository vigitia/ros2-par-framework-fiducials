# Written by Claude Code
"""3D marker corners from a depth image registered to the colour image."""

import cv2
import numpy as np

# The depth of a marker is taken from its inner part only: the polygon shrunk towards its centre to
# this fraction. Pixels at the edge may already see what lies around the marker.
DEPTH_SAMPLE_INNER_FRACTION = 0.7
# Fewer valid depth pixels than this in the inner part: no 3D position, e.g. a marker far away
MIN_DEPTH_PIXEL_COUNT = 20
# Less of the inner part with depth than this: no 3D position. Seen live on a cube's side face that
# the depth camera saw almost edge-on: 13 % valid pixels, and those were the table behind it.
MIN_DEPTH_COVERAGE_FRACTION = 0.5
# A square of edge 1 in solvePnP's IPPE_SQUARE order, which is also OpenCV's marker corner order
# (clockwise from the top left). The real size comes from the depth, so no marker size is needed.
UNIT_SQUARE_CORNERS = np.array([[-0.5, 0.5, 0.0], [0.5, 0.5, 0.0], [0.5, -0.5, 0.0],
                                [-0.5, -0.5, 0.0]])
# A pose that misses the corners by more than this is discarded: they are not the image of a square
# (e.g. a wrong detection). Also IPPE's one degenerate case, a noise-free square facing the camera
# exactly, which only synthetic data produces.
MAX_REPROJECTION_ERROR_PX = 3.0


def compute_corners_in_camera_space(corners_px, depth_image_m, camera_matrix,
                                    distortion_coefficients):
    """
    Return the four corners of a marker in the camera's optical frame, in metres, or None.

    corners_px: 4x2 pixel positions in the raw colour image. depth_image_m: depth image
    registered to that image (the same pixel grid), in metres, 0 where there is no depth.
    camera_matrix, distortion_coefficients: intrinsics of the colour image.

    The orientation comes from the marker's square in the colour image (IPPE), the distance from
    the depth inside it. The depth alone gave steep faces a wrong tilt: on a cube's side face 10
    degrees too steep, a 60 x 77 mm "square" for a 57 mm marker, probably because the
    time-of-flight light also reaches the face via the table in the corner between them
    (multipath). The result is a square that lies on the corners' rays. Returns a 4x3 array; None
    when too little of the marker has depth, or the corners are no view of a square.
    """
    corners_px = np.asarray(corners_px, np.float64).reshape(4, 2)
    center_px = corners_px.mean(axis=0)
    inner_corners_px = center_px + DEPTH_SAMPLE_INNER_FRACTION * (corners_px - center_px)
    # Mask only the bounding box, not the whole image, for every marker in every frame
    depth_height_px, depth_width_px = depth_image_m.shape[:2]
    box_left_px, box_top_px = np.maximum(np.floor(inner_corners_px.min(axis=0)).astype(int), 0)
    box_right_px, box_bottom_px = np.minimum(np.ceil(inner_corners_px.max(axis=0)).astype(int) + 1,
                                             (depth_width_px, depth_height_px))
    if box_left_px >= box_right_px or box_top_px >= box_bottom_px:
        return None
    depth_box_m = depth_image_m[box_top_px:box_bottom_px, box_left_px:box_right_px]
    inner_mask = np.zeros(depth_box_m.shape, np.uint8)
    inner_corners_in_box_px = np.round(inner_corners_px - (box_left_px, box_top_px))
    cv2.fillConvexPoly(inner_mask, inner_corners_in_box_px.astype(np.int32), 1)
    valid_rows_px, valid_columns_px = np.nonzero((inner_mask > 0) & (depth_box_m > 0))
    valid_pixel_count = valid_columns_px.size
    if (valid_pixel_count < MIN_DEPTH_PIXEL_COUNT
            or valid_pixel_count < MIN_DEPTH_COVERAGE_FRACTION * np.count_nonzero(inner_mask)):
        return None

    # A square seen at an angle is ambiguous: IPPE returns both tilts that fit the image
    pose_found, rotation_vectors, translation_vectors, reprojection_errors_px = (
        cv2.solvePnPGeneric(UNIT_SQUARE_CORNERS, corners_px, camera_matrix,
                            distortion_coefficients, flags=cv2.SOLVEPNP_IPPE_SQUARE))
    candidate_poses = []
    for rotation_vector, translation_vector, reprojection_error_px in zip(
            rotation_vectors, translation_vectors, reprojection_errors_px):
        if reprojection_error_px.ravel()[0] <= MAX_REPROJECTION_ERROR_PX:
            rotation_matrix = cv2.Rodrigues(rotation_vector)[0]
            candidate_poses.append((rotation_matrix, translation_vector.ravel()))
    if not pose_found or not candidate_poses:
        return None
    # The depth's tilt is not exact, but good enough to tell the two apart: its plane's normal is
    # the direction of least spread of the depth points
    depths_m = depth_box_m[valid_rows_px, valid_columns_px].astype(np.float64)
    valid_pixels_px = np.column_stack([valid_columns_px + box_left_px,
                                       valid_rows_px + box_top_px]).astype(np.float64)
    normalized_points = cv2.undistortPoints(valid_pixels_px.reshape(-1, 1, 2), camera_matrix,
                                            distortion_coefficients).reshape(-1, 2)
    depth_points_m = np.column_stack([normalized_points * depths_m[:, None], depths_m])
    # Only Vt (3x3) is needed. The full SVD would also build U, N x N for the N depth pixels: for a
    # marker that is large in the image, gigabytes and seconds per marker
    centred_depth_points_m = depth_points_m - depth_points_m.mean(axis=0)
    depth_plane_normal = np.linalg.svd(centred_depth_points_m, full_matrices=False)[2][2]
    # Keep the pose whose square normal is closest to the depth plane's normal. The square lies
    # in its own xy plane, so the rotation's third column is its normal.
    square_rotation = None
    square_translation = None
    best_normal_agreement = -1.0
    for rotation_matrix, translation_vector in candidate_poses:
        square_normal = rotation_matrix[:, 2]
        normal_agreement = abs(square_normal @ depth_plane_normal)
        if normal_agreement > best_normal_agreement:
            best_normal_agreement = normal_agreement
            square_rotation = rotation_matrix
            square_translation = translation_vector
    unit_square_in_camera = UNIT_SQUARE_CORNERS @ square_rotation.T + square_translation
    # Scaled about the camera, every corner stays on its ray. The median depth of the inner part
    # is the depth of the marker's centre; a finger on part of the marker does not move it.
    metric_scale = np.median(depths_m) / unit_square_in_camera.mean(axis=0)[2]
    if not (np.isfinite(metric_scale) and metric_scale > 0):
        return None
    return unit_square_in_camera * metric_scale
