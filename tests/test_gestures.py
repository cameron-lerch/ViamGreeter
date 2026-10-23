'''
run all of the tests in this directory with:
from the main ViamGreeter directory, run:
pytest tests/*

--
Tests for gestures.py: trajectory smoothing, joint limits, and the gestures
themselves (stay within limits, stay under the arm's speed, idle repeats).
'''

import pytest

import gestures as g

SIM_SPEED_LIMIT = 60.0   # deg/s: the simulated arm's measured speed cap
DT = 0.01                # s: step for sampling the gestures

GESTURES = {
    "idle":   (lambda t: g.idle_pose(t), g.IDLE_CYCLE_S),
    "point":  (lambda t: g.point_pose(0.0), 5.0),
    "beckon": (lambda t: g.beckon_pose(t, 0.0), 2 * g.BECKON_PERIOD_S),
    "wave":   (lambda t: g.wave_pose(t, 0.0), 2 * g.WAVE_PERIOD_S),
}


def samples(pose_fn, duration):
    return [pose_fn(i * DT) for i in range(int(duration / DT) + 1)]


# --- smoothstep / smooth_transition -------------------------------------------

def test_smoothstep_endpoints_and_midpoint():
    assert g.smoothstep(0.0) == 0.0
    assert g.smoothstep(1.0) == 1.0
    assert g.smoothstep(0.5) == pytest.approx(0.5)


def test_smoothstep_clamps_outside_zero_to_one():
    assert g.smoothstep(-2.0) == 0.0
    assert g.smoothstep(3.0) == 1.0


def test_smoothstep_always_increases():
    values = [g.smoothstep(i / 100) for i in range(101)]
    assert all(b >= a for a, b in zip(values, values[1:]))


def test_smoothstep_starts_and_ends_with_zero_slope():
    h = 1e-4
    assert (g.smoothstep(h) - g.smoothstep(0.0)) / h == pytest.approx(0.0, abs=1e-3)
    assert (g.smoothstep(1.0) - g.smoothstep(1.0 - h)) / h == pytest.approx(0.0, abs=1e-3)


def test_smooth_transition_goes_from_a_to_b():
    a, b = [0.0, 10.0, -20.0, 5.0, 90.0], [30.0, -10.0, 40.0, -5.0, 0.0]
    assert g.smooth_transition(a, b, 0.0) == pytest.approx(a)
    assert g.smooth_transition(a, b, 1.0) == pytest.approx(b)
    assert g.smooth_transition(a, b, 0.5) == pytest.approx([(x + y) / 2 for x, y in zip(a, b)])


# --- clamp_pose ------------------------------------------------------------------

def test_clamp_pose_keeps_extreme_requests_inside_the_limits():
    extreme_high = g.clamp_pose([1000.0] * 5)
    extreme_low = g.clamp_pose([-1000.0] * 5)
    for value, (lo, hi) in zip(extreme_high, g.JOINT_LIMITS):
        assert value == pytest.approx(hi - g.SAFETY_MARGIN_DEG)
    for value, (lo, hi) in zip(extreme_low, g.JOINT_LIMITS):
        assert value == pytest.approx(lo + g.SAFETY_MARGIN_DEG)


def test_clamp_pose_caps_the_shoulder_above_the_table():
    shoulder = g.clamp_pose([0.0, 90.0, 0.0, 0.0, 0.0])[1]
    assert shoulder == pytest.approx(g.JOINT_LIMITS[1][1] - g.SAFETY_MARGIN_DEG)
    assert shoulder < 20.0


def test_clamp_pose_leaves_reachable_poses_unchanged():
    pose = [10.0, -20.0, 30.0, -40.0, 90.0]
    assert g.clamp_pose(pose) == pytest.approx(pose)


# --- the gestures --------------------------------------------------------------

@pytest.mark.parametrize("name", GESTURES)
def test_gesture_stays_within_joint_limits(name):
    pose_fn, duration = GESTURES[name]
    for pose in samples(pose_fn, duration):
        assert g.clamp_pose(pose) == pytest.approx(pose), f"{name} asks for a pose outside the limits"


@pytest.mark.parametrize("name", GESTURES)
def test_gesture_stays_under_the_simulated_arm_speed(name):
    pose_fn, duration = GESTURES[name]
    poses = samples(pose_fn, duration)
    peak = max(abs(b - a) / DT for p, q in zip(poses, poses[1:]) for a, b in zip(p, q))
    assert peak <= SIM_SPEED_LIMIT, f"{name} peaks at {peak:.1f} deg/s"


def test_gestures_aim_at_the_given_base_angle():
    assert g.point_pose(25.0)[0] == pytest.approx(25.0)
    assert g.beckon_pose(1.0, 25.0)[0] == pytest.approx(25.0)
    wave_bases = [g.wave_pose(t, 25.0)[0] for t in (0.0, g.WAVE_PERIOD_S / 2, g.WAVE_PERIOD_S)]
    assert wave_bases == pytest.approx([25.0, 25.0, 25.0], abs=1e-9)   # sway is centered on the target


def test_idle_repeats_every_cycle():
    for t in (0.0, 3.3, 47.0):
        assert g.idle_pose(t) == pytest.approx(g.idle_pose(t + g.IDLE_CYCLE_S))


@pytest.mark.parametrize("t", [0.0, 12.3, 60.0, 101.7])
def test_nearest_idle_time_finds_the_point_on_the_idle_path(t):
    found = g.nearest_idle_time(g.idle_pose(t))
    assert g.idle_pose(found) == pytest.approx(g.idle_pose(t), abs=0.5)
