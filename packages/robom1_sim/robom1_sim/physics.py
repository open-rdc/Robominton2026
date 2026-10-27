"""Lightweight shuttle flight and sphere/finite-racket collision, SI units.

The racket is a prescribed moving cylinder (infinite effective mass). Its
normal is local +x and the face spans local y-z, matching the URDF cylinder.
No string deformation, feather orientation, net or chassis contacts are modeled.
"""
from dataclasses import dataclass
import math
import numpy as np
from robom1_sim.geometry import rotation_up, rotation_z, rotation_rpy


@dataclass
class RacketGeometry:
    pivot_x: float = 0.05
    pivot_y: float = -0.36
    pivot_z: float = 1.05  # base_footprint, not base_link
    reach: float = 0.62
    radius: float = 0.14
    thickness: float = 0.015

    mount_roll: float = 0.0
    mount_pitch: float = 0.0
    mount_yaw: float = 0.0

    @property
    def mount(self):
        return rotation_rpy([self.mount_roll, self.mount_pitch, self.mount_yaw])

    @property
    def pivot(self):
        return np.array([self.pivot_x, self.pivot_y, self.pivot_z])


def flight_step(position, velocity, dt, drag):
    """Midpoint integration: dv/dt = gravity - drag*|v|*v.

    Negative dt is allowed for constructing a reproducible incoming test feed.
    Ground contact is handled by the caller, never by the free-flight predictor.
    """
    gravity = np.array([0.0, 0.0, -9.81])
    acceleration = gravity - drag * np.linalg.norm(velocity) * velocity
    midpoint = velocity + 0.5 * dt * acceleration
    next_position = position + dt * midpoint
    next_velocity = velocity + dt * (gravity - drag * np.linalg.norm(midpoint) * midpoint)
    return next_position, next_velocity


def racket_pose(base, yaw, angle, geometry):
    yaw_rotation = rotation_z(yaw)
    orientation = yaw_rotation @ geometry.mount @ rotation_up(-angle)
    pivot = np.asarray(base) + yaw_rotation @ geometry.pivot
    center = pivot + orientation @ np.array([0.0, 0.0, geometry.reach])
    return pivot, center, orientation


def sphere_cylinder_contact(position, sphere_radius, center, orientation, geometry):
    """Return surface point, outward normal and penetration, or None.

    Closest point on a solid cylinder detects face, rim and side contacts.
    An overlapping sphere center inside the cylinder is projected to its nearest exit.
    """
    local = orientation.T @ (position - center)
    radial = float(np.linalg.norm(local[1:]))
    half = geometry.thickness / 2
    closest = local.copy()
    closest[0] = np.clip(local[0], -half, half)
    if radial > geometry.radius:
        closest[1:] *= geometry.radius / radial
    delta = local - closest
    distance = float(np.linalg.norm(delta))
    if distance > sphere_radius:
        return None
    if distance > 1e-10:
        normal = delta / distance
        penetration = sphere_radius - distance
    else:
        face_gap = half - abs(local[0])
        rim_gap = geometry.radius - radial
        if face_gap <= rim_gap:
            normal = np.array([1.0 if local[0] >= 0 else -1.0, 0.0, 0.0])
            closest[0] = normal[0] * half
            penetration = sphere_radius + face_gap
        else:
            normal = np.array([0.0, 1.0, 0.0])
            if radial > 1e-10:
                normal[1:] = local[1:] / radial
            closest[1:] = normal[1:] * geometry.radius
            penetration = sphere_radius + rim_gap
    return center + orientation @ closest, orientation @ normal, penetration


def collide_racket(position, velocity, sphere_radius, base, yaw, angle,
                   base_velocity, yaw_rate, pitch_rate, geometry, restitution):
    """Reflect only approaching relative normal velocity against a moving racket.

    v_out = v_in - (1+e)*dot(v_in-v_surface,n)*n.
    Tangential relative velocity is unchanged (frictionless contact).
    """
    pivot, center, orientation = racket_pose(base, yaw, angle, geometry)
    contact = sphere_cylinder_contact(position, sphere_radius, center, orientation, geometry)
    if contact is None:
        return position, velocity, None
    point, normal, penetration = contact
    pitch_axis = rotation_z(yaw) @ geometry.mount @ np.array([0.0, 1.0, 0.0])
    surface_velocity = (np.asarray(base_velocity)
                        + np.cross([0.0, 0.0, yaw_rate], point - base)
                        + np.cross(pitch_axis * pitch_rate, point - pivot))
    corrected = position + normal * (penetration + 1e-6)
    approaching = float(np.dot(velocity - surface_velocity, normal))
    if approaching >= -1e-6:
        return corrected, velocity, None
    outgoing = velocity - (1.0 + restitution) * approaching * normal
    return corrected, outgoing, point


def predict_landing(position, velocity, sphere_radius, drag, dt=0.002, horizon=8.0):
    """Free-flight samples, elapsed times, landing point or None on horizon expiry.

    Ground touch is at center height sphere_radius. The final step is interpolated
    to this plane, so a horizon endpoint is never mislabeled as a landing.
    """
    position, velocity = np.asarray(position, dtype=float).copy(), np.asarray(velocity, dtype=float).copy()
    points, times = [position.copy()], [0.0]
    if position[2] <= sphere_radius:
        position[2] = sphere_radius
        return np.array([position]), np.array([0.0]), position.copy()
    elapsed = 0.0
    for _ in range(math.ceil(horizon / dt)):
        step = min(dt, horizon - elapsed)
        new_position, new_velocity = flight_step(position, velocity, step, drag)
        if new_position[2] <= sphere_radius:
            fraction = (position[2] - sphere_radius) / (position[2] - new_position[2])
            landing = position + fraction * (new_position - position)
            landing[2] = sphere_radius
            points.append(landing.copy())
            times.append(elapsed + fraction * step)
            return np.array(points), np.array(times), landing
        elapsed += step
        position, velocity = new_position, new_velocity
        points.append(position.copy())
        times.append(elapsed)
    return np.array(points), np.array(times), None


def make_impact_feed(base, yaw, geometry, sphere_radius, drag, swing=None):
    """Aim a test serve at the rising part of a READY->backswing->stroke sequence.

    This is a deterministic feed for a STATIONARY base, not an interception planner.
    It does not force a hit: runtime collision detection still decides contact.
    """
    from robom1_sim.swing import PitchSwing
    swing = PitchSwing() if swing is None else swing
    backswing = swing.backswing
    follow = swing.follow_through
    contact_angle = backswing + (follow-backswing) * (0.95/3.65)
    target_blend = (contact_angle - backswing) / (follow - backswing)
    low, high = 0.0, 1.0
    for _ in range(50):
        u = (low + high) / 2
        blend = u**3 * (10 - 15*u + 6*u*u)
        if blend < target_blend:
            low = u
        else:
            high = u
    contact_time = swing.duration(swing.position, backswing, swing.preparation_time) + swing.duration(backswing, follow, swing.stroke_time) * (low + high) / 2
    _, center, orientation = racket_pose(base, yaw, contact_angle, geometry)
    position = center + orientation[:, 0] * (sphere_radius + geometry.thickness/2)
    velocity = rotation_z(yaw) @ np.array([-2.0, 0.0, -6.0])
    count = math.ceil(contact_time / 0.001)
    for _ in range(count):
        position, velocity = flight_step(position, velocity, -contact_time/count, drag)
    if not np.isfinite(position).all() or not np.isfinite(velocity).all() or position[2] <= sphere_radius:
        raise ValueError('Feed is not feasible for this geometry/drag')
    return position, velocity
