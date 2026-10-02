"""Pinhole back-projection in the calibrated camera optical frame, in metres."""

from math import isfinite


def project_pixel(u, v, depth_m, fx, fy, cx, cy):
    values = (u, v, depth_m, fx, fy, cx, cy)
    if not all(isfinite(value) for value in values):
        raise ValueError("pixel, depth and intrinsics must be finite")
    if depth_m <= 0.0 or fx <= 0.0 or fy <= 0.0:
        raise ValueError("depth and focal lengths must be positive")
    return (u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m
