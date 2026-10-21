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

    def __init__(self):
        self.position = READY
        self.elapsed = 0.0
        self.segments = []

    @property
    def active(self):
        return bool(self.segments)

    def trigger(self):
        if self.active:
            return False
        self.elapsed = 0.0
        # From any allowed manual position, keep peak quintic speed <= MAX_SPEED.
        preparation = max(0.6, 1.875 * abs(BACKSWING - self.position) / MAX_SPEED)
        self.segments = [
            (preparation, self.position, BACKSWING),
            (0.9, BACKSWING, FOLLOW_THROUGH),
            (0.25, FOLLOW_THROUGH, FOLLOW_THROUGH),
            (0.95, FOLLOW_THROUGH, READY),
        ]
        return True

    def cancel(self, position):
        self.segments = []
        self.elapsed = 0.0
        self.position = min(UPPER, max(LOWER, position))

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
