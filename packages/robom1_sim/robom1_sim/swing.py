"""Pitch stroke animation, independent of ROS. Angles are in radians."""
import math

LOWER = -1.8
UPPER = 2.0
READY = -0.35
BACKSWING = -1.75
FOLLOW_THROUGH = 1.9
MAX_SPEED = 8.0


class PitchSwing:
    """Quintic strokes have zero speed/acceleration at each phase boundary."""

    def __init__(self, lower=LOWER, upper=UPPER, ready=READY, backswing=BACKSWING,
                 follow_through=FOLLOW_THROUGH, velocity=MAX_SPEED, manual_velocity=4.0,
                 preparation_time=0.6, stroke_time=0.9, hold_time=0.25, return_time=0.95):
        values = [lower, upper, ready, backswing, follow_through, velocity, manual_velocity,
                  preparation_time, stroke_time, hold_time, return_time]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Swing parameters must be finite')
        if not lower <= backswing < ready < follow_through <= upper:
            raise ValueError('Swing angles must lie inside ordered limits')
        if min(velocity, manual_velocity, preparation_time, stroke_time, return_time) <= 0 or hold_time < 0:
            raise ValueError('Swing speeds and moving-phase times must be positive')
        if manual_velocity > velocity:
            raise ValueError('Manual speed exceeds swing joint limit')
        self.lower, self.upper = lower, upper
        self.ready, self.backswing, self.follow_through = ready, backswing, follow_through
        self.max_speed, self.manual_velocity = velocity, manual_velocity
        self.preparation_time, self.stroke_time = preparation_time, stroke_time
        self.hold_time, self.return_time = hold_time, return_time
        self.position = ready
        self.elapsed = 0.0
        self.segments = []

    @property
    def active(self):
        return bool(self.segments)

    def duration(self, start, end, requested):
        # Peak derivative of quintic blend is 1.875. Extend motion time when needed.
        return max(requested, 1.875 * abs(end-start) / self.max_speed)

    def trigger(self):
        if self.active:
            return False
        self.elapsed = 0.0
        self.segments = [
            (self.duration(self.position, self.backswing, self.preparation_time), self.position, self.backswing),
            (self.duration(self.backswing, self.follow_through, self.stroke_time), self.backswing, self.follow_through),
            (self.hold_time, self.follow_through, self.follow_through),
            (self.duration(self.follow_through, self.ready, self.return_time), self.follow_through, self.ready),
        ]
        return True

    def cancel(self, position):
        self.segments = []
        self.elapsed = 0.0
        self.position = min(self.upper, max(self.lower, position))

    def step(self, dt):
        if not math.isfinite(dt) or dt < 0:
            raise ValueError('dt must be finite and nonnegative')
        if not self.active:
            return self.position
        self.elapsed += dt
        while self.segments and self.elapsed >= self.segments[0][0]:
            duration, _, end = self.segments.pop(0)
            self.elapsed -= duration
            self.position = end
        if self.segments:
            duration, start, end = self.segments[0]
            u = self.elapsed / duration
            blend = u*u*u * (10.0 - 15.0*u + 6.0*u*u)
            self.position = start + (end - start) * blend
        return self.position
