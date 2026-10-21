import math
import numpy as np
import pytest
from robom1_sim.physics import (RacketGeometry, collide_racket, flight_step,
                               make_impact_feed, predict_landing, racket_pose,
                               sphere_cylinder_contact)
from robom1_sim.swing import PitchSwing


@pytest.mark.parametrize('side', [-1.0, 1.0])
def test_stationary_face_reflects_normal_and_preserves_tangent(side):
    geometry = RacketGeometry()
    base = np.zeros(3)
    _, center, _ = racket_pose(base, 0, 0, geometry)
    position = center + np.array([side*0.02, 0, 0])
    incoming = np.array([-side*2, 0.7, -0.4])
    corrected, outgoing, point = collide_racket(
        position, incoming, 0.035, base, 0, 0, np.zeros(3), 0, 0, geometry, 0.8)
    assert point is not None
    np.testing.assert_allclose(outgoing, [side*1.6, 0.7, -0.4])
    assert side*(corrected[0]-center[0]) > 0.035+geometry.thickness/2


def test_finite_face_miss_rim_and_separating_contact():
    geometry = RacketGeometry()
    base = np.zeros(3)
    _, center, orientation = racket_pose(base, 0, 0, geometry)
    assert sphere_cylinder_contact(center+[0.02, 0.3, 0], .035, center, orientation, geometry) is None
    rim = sphere_cylinder_contact(center+[0.02, 0.16, 0], .035, center, orientation, geometry)
    assert rim is not None and rim[1][0] > 0 and rim[1][1] > 0
    incoming = np.array([2.0, 0, 0])
    _, outgoing, point = collide_racket(center+[0.02, 0, 0], incoming, .035,
        base, 0, 0, np.zeros(3), 0, 0, geometry, .8)
    assert point is None
    np.testing.assert_array_equal(outgoing, incoming)


def test_translating_surface_and_rotating_surface_add_strike_velocity():
    geometry = RacketGeometry()
    base = np.zeros(3)
    _, center, _ = racket_pose(base, 0, 0, geometry)
    _, outgoing, _ = collide_racket(center+[0.02, 0, 0], np.array([-2., 0, 0]), .035,
        base, 0, 0, np.array([1., 0, 0]), 0, 0, geometry, .8)
    assert outgoing[0] == pytest.approx(3.4)
    _, outgoing, _ = collide_racket(center+[0.02, 0, 0], np.zeros(3), .035,
        base, 0, 0, np.zeros(3), 0, 5., geometry, 1.)
    assert outgoing[0] == pytest.approx(2*5*geometry.reach)


def test_rotated_base_has_the_same_physics_in_world_coordinates():
    geometry = RacketGeometry()
    base = np.array([2., 3, 0])
    _, center, orientation = racket_pose(base, math.pi/2, -0.8, geometry)
    normal = orientation[:, 0]
    _, outgoing, point = collide_racket(center+0.02*normal, -2*normal, .035,
        base, math.pi/2, -0.8, np.zeros(3), 0, 0, geometry, .5)
    assert point is not None
    np.testing.assert_allclose(outgoing, normal, atol=1e-10)


def test_no_drag_prediction_matches_analytic_ballistic_landing():
    position, velocity = np.array([0., 0, 1]), np.array([3., 0, 4])
    points, times, landing = predict_landing(position, velocity, .035, 0.)
    expected_time = (4+math.sqrt(16+2*9.81*(1-.035)))/9.81
    assert times[-1] == pytest.approx(expected_time, abs=1e-5)
    np.testing.assert_allclose(landing, [3*expected_time, 0, .035], atol=3e-5)
    assert points[:, 2].min() >= .035
    _, _, short_landing = predict_landing(position, velocity, .035, 0., horizon=.1)
    assert short_landing is None


@pytest.mark.parametrize('h', [.001, .002])
def test_real_feed_hits_once_and_actual_landing_matches_prediction(h):
    geometry = RacketGeometry()
    base = np.array([1., 0, 0])
    position, velocity = make_impact_feed(base, 0, geometry, .035, .12)
    swing = PitchSwing()
    swing.trigger()
    hits = 0
    landing = None
    for _ in range(math.ceil(4/h)):
        previous_angle = swing.position
        angle = swing.step(h)
        previous_position = position.copy()
        position, velocity = flight_step(position, velocity, h, .12)
        position, velocity, point = collide_racket(position, velocity, .035, base, 0,
            angle, np.zeros(3), 0, (angle-previous_angle)/h, geometry, .8)
        if point is not None:
            hits += 1
            assert velocity[0] > 0 and velocity[2] > 0
            _, _, landing = predict_landing(position, velocity, .035, .12)
        if position[2] <= .035:
            fraction = (previous_position[2]-.035)/(previous_position[2]-position[2])
            actual = previous_position + fraction*(position-previous_position)
            break
    assert hits == 1 and landing is not None
    assert np.linalg.norm(actual-landing) < .01
    assert landing[0] > 6.7  # This feed returns toward the far court.


def test_fast_ball_does_not_tunnel_when_relative_step_is_bounded():
    geometry = RacketGeometry()
    base = np.zeros(3)
    _, center, _ = racket_pose(base, 0, 0, geometry)
    position, velocity = center+np.array([.2, 0, 0]), np.array([-100., 0, 0])
    h = .4*.035/100
    hits = 0
    for _ in range(math.ceil(.004/h)):
        position, velocity = flight_step(position, velocity, h, 0.)
        position, velocity, point = collide_racket(position, velocity, .035, base, 0, 0,
            np.zeros(3), 0, 0, geometry, 1.)
        hits += point is not None
    assert hits == 1 and velocity[0] > 0
