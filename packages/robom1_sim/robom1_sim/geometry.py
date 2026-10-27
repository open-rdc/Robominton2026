"""Pure geometry; all world lengths in metres, angles in radians."""
import math
import numpy as np


def rotation_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rotation_rpy(rpy):
    """URDF fixed-axis roll/pitch/yaw: Rz(yaw) Ry(pitch) Rx(roll)."""
    roll, pitch, yaw = rpy
    c, s = math.cos(roll), math.sin(roll)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    return rotation_z(yaw) @ rotation_up(-pitch) @ rx


def rotation_up(pitch):
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])


def project(point, fx, fy, cx, cy):
    """Camera link: +x forward, +y left, +z up. Image: +u right, +v down."""
    x, y, z = point
    if x <= 0.05:
        return None
    return (cx - fx * y / x, cy - fy * z / x)


def sphere_ranges(origin, directions, center, radius):
    """Nearest positive ray/sphere intersection; misses are +inf."""
    offset = np.asarray(center) - origin
    along = directions @ offset
    discriminant = along**2 - (offset @ offset - radius**2)
    root = np.sqrt(np.maximum(discriminant, 0.0))
    near, far = along - root, along + root
    distance = np.where(near > 0, near, far)
    return np.where((discriminant >= 0) & (distance > 0), distance, np.inf)


def scan_scene(origin, directions, shuttle, radius, maximum, minimum=0.05):
    """Shuttle sphere, ground and four enclosing walls (not the court net)."""
    ranges = sphere_ranges(origin, directions, shuttle, radius)
    # Enclosure is x=[-1,14.4], y=[-4,4], height=4.5m.
    with np.errstate(divide='ignore', invalid='ignore'):
        ground = -origin[2] / directions[:, 2]
        ranges = np.minimum(ranges, np.where(ground > 0, ground, np.inf))
        for axis, bound in [(0, -1.0), (0, 14.4), (1, -4.0), (1, 4.0)]:
            distance = (bound - origin[axis]) / directions[:, axis]
            hit = origin + directions * distance[:, None]
            other = 1 - axis
            lower, upper = (-4.0, 4.0) if other == 1 else (-1.0, 14.4)
            valid = ((distance > 0) & (hit[:, 2] >= 0) & (hit[:, 2] <= 4.5)
                     & (hit[:, other] >= lower) & (hit[:, other] <= upper))
            ranges = np.minimum(ranges, np.where(valid, distance, np.inf))
    return np.where((ranges >= minimum) & (ranges <= maximum), ranges, np.inf)
