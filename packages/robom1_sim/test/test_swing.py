import math
import pytest
from robom1_sim.swing import BACKSWING, FOLLOW_THROUGH, LOWER, MAX_SPEED, READY, UPPER, PitchSwing


def test_stroke_reaches_takeback_and_followthrough_then_returns_smoothly():
    swing = PitchSwing()
    assert swing.trigger()
    assert not swing.trigger()
    values = [swing.position]
    for _ in range(3000):
        values.append(swing.step(0.001))
    assert min(values) == pytest.approx(BACKSWING, abs=0.001)
    assert max(values) == pytest.approx(FOLLOW_THROUGH, abs=0.001)
    assert values[-1] == READY
    assert not swing.active
    assert max(abs(b-a)/0.001 for a, b in zip(values, values[1:])) <= MAX_SPEED
    # Head goes behind and ahead of the fixed grip in the x-z plane.
    assert 0.62 * math.sin(min(values)) < -0.6
    assert 0.62 * math.sin(max(values)) > 0.58


@pytest.mark.parametrize('start', [LOWER, UPPER])
def test_manual_takeover_and_retrigger_respect_limits_and_speed(start):
    swing = PitchSwing()
    swing.trigger()
    swing.step(0.3)
    swing.cancel(start)
    assert not swing.active and swing.position == start
    swing.trigger()
    previous = start
    for _ in range(4000):
        current = swing.step(0.001)
        assert LOWER <= current <= UPPER
        assert abs(current-previous)/0.001 <= MAX_SPEED + 1e-8
        previous = current
    assert current == READY


def test_large_step_finishes_stroke_and_invalid_time_is_rejected():
    swing = PitchSwing()
    swing.trigger()
    assert swing.step(10.0) == READY
    assert not swing.active
    with pytest.raises(ValueError):
        swing.step(-0.1)
