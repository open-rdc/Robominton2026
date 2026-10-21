import math
import numpy as np
from robom1_sim.geometry import project, rotation_up, rotation_z, scan_scene, sphere_ranges


def test_camera_axes_and_behind_camera():
    assert project([2, 0, 0], 220, 220, 160, 120) == (160, 120)
    assert project([2, 1, 1], 220, 220, 160, 120) == (50, 10)
    assert project([-1, 0, 0], 220, 220, 160, 120) is None


def test_shuttle_requires_scan_plane_intersection():
    origin = np.array([0.0, 0.0, 1.0])
    ray = np.array([[1.0, 0.0, 0.0]])
    shuttle = np.array([2.0, 0.0, 2.0])
    assert np.isinf(sphere_ranges(origin, ray, shuttle, 0.035)[0])
    pitched = ray @ rotation_up(math.atan2(1, 2)).T
    assert np.isclose(sphere_ranges(origin, pitched, shuttle, 0.035)[0], math.sqrt(5)-0.035)
    assert np.isinf(sphere_ranges(origin, -pitched, shuttle, 0.035)[0])


def test_walls_ground_and_world_rotation():
    rays = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0], [0, 0, 1.0]])
    ranges = scan_scene(np.array([1.0, 0, 1]), rays, [5, 2, 3], 0.035, 20)
    np.testing.assert_allclose(ranges[:3], [13.4, 4, 1])
    assert np.isinf(ranges[3])
    np.testing.assert_allclose(rotation_z(math.pi/2) @ rays[0], [0, 1, 0], atol=1e-10)


def test_nearest_surface_occludes_shuttle():
    rays = np.array([[1.0, 0, 0]])
    assert np.isclose(scan_scene(np.array([1., 0, 1]), rays, [16, 0, 1], 0.035, 20)[0], 13.4)
    assert np.isinf(scan_scene(np.array([1., 0, 1]), rays, [16, 0, 1], 0.035, 10)[0])
