'''
Greeter's motions, written as functions of time that return SO-101 joint angles.

Joint order: [base, shoulder, elbow, wrist_pitch, wrist_roll], in degrees.
Measured on the simulated SO-101:
  - all zeros points the arm straight out along world +x
  - base +   turns the arm clockwise (to its right, toward world -y)
  - shoulder, elbow, wrist_pitch: negative lifts, positive lowers
  - wrist_roll twists the hand about its own axis
  - every joint moves at ~60 deg/s no matter what speed is requested, so each
    motion keeps amplitude * 2*pi / period under that to avoid lagging behind

Because every motion is just pose = f(time), the controller can sample whichever
one is active, switch between them at any instant, and blend across the switch.
'''

import math

# Joint limits from the arm's kinematics, minus a safety margin. Shoulder is
# further capped so the arm never reaches down into the table it sits on.
SAFETY_MARGIN_DEG = 5.0
JOINT_LIMITS = [
    (-110.0, 110.0),   # base
    (-100.0, 20.0),    # shoulder (kinematic max is +100, which points below the base)
    (-96.8, 96.8),     # elbow
    (-95.0, 95.0),     # wrist pitch
    (-157.2, 162.8),   # wrist roll
]

IDLE_CENTER = [0.0, -10.0, 5.0, 10.0, 0.0]     # out in front, hand tipped slightly down
POINT = [0.0, -15.0, 10.0, 5.0, 0.0]            # arm outstretched, hand level: pointing at the person
RAISED = [0.0, -25.0, -55.0, -10.0, 0.0]        # forearm and hand pointing up, like a raised hand
REACH = [0.0, 5.0, 20.0, 10.0, 0.0]             # hand low and out toward the person

WAVE_PERIOD_S = 2.0
BECKON_PERIOD_S = 2.0


def _sin(t: float, period: float, phase: float = 0.0) -> float:
    return math.sin(2 * math.pi * t / period + phase)


def clamp_pose(pose: list[float]) -> list[float]:
    '''Keep every joint inside its limits, so no command can ask for an unreachable pose.'''
    return [max(lo + SAFETY_MARGIN_DEG, min(hi - SAFETY_MARGIN_DEG, v))
            for v, (lo, hi) in zip(pose, JOINT_LIMITS)]


def smoothstep(s: float) -> float:
    '''0 -> 1 with zero slope at both ends, so blends start and finish gently.'''
    s = max(0.0, min(1.0, s))
    return s * s * (3 - 2 * s)


def blend(a: list[float], b: list[float], s: float) -> list[float]:
    k = smoothstep(s)
    return [x + (y - x) * k for x, y in zip(a, b)]


def idle_pose(t: float) -> list[float]:
    '''Slow scanning sweep: side to side with a gentle up-and-down bob.

    The periods don't divide evenly, so the path drifts instead of repeating
    exactly and looks less mechanical. The wrist swings from pointing well
    above the horizon to well below it, so the hand dips as it scans.
    Peak speed is ~22 deg/s.
    '''
    base, shoulder, elbow, wrist, roll = IDLE_CENTER
    return [
        base + 30.0 * _sin(t, 14.0),
        shoulder + 10.0 * _sin(t, 9.0),
        elbow + 12.0 * _sin(t, 9.0, 1.0),
        wrist + 25.0 * _sin(t, 7.0),
        roll,
    ]


def point_pose(base: float) -> list[float]:
    '''Arm outstretched toward the person: makes it obvious who is being greeted.'''
    return [base] + POINT[1:]


def wave_pose(t: float, base: float) -> list[float]:
    '''Hand up, swaying side to side while the wrist rocks, like a wave.

    The SO-101 has no wrist yaw, so the side-to-side comes from the base joint,
    with wrist roll adding the hand's rocking motion.
    '''
    pose = [base] + RAISED[1:]
    pose[0] += 10.0 * _sin(t, WAVE_PERIOD_S)
    pose[4] += 15.0 * _sin(t, WAVE_PERIOD_S)
    return pose


def beckon_pose(t: float, base: float) -> list[float]:
    '''"Come over here": the hand reaches out and curls back in a loop.

    Elbow and wrist pitch oscillate a quarter cycle apart, so the hand traces an
    ellipse instead of a line. The reach is low and forward, pointing down at
    the person, so the curl reads as "you, come here" rather than a wave.
    '''
    _, shoulder, elbow, wrist, roll = REACH
    return [
        base,
        shoulder,
        elbow + 10.0 * _sin(t, BECKON_PERIOD_S),
        wrist - 15.0 * (1 - math.cos(2 * math.pi * t / BECKON_PERIOD_S)),  # curls up to 30 deg
        roll,
    ]
