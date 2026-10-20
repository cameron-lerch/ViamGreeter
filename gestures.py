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

HAND_ROLL = 90.0  # wrist roll that turns the gripper so it reads as a flat hand

IDLE_CENTER = [0.0, -10.0, 5.0, 10.0, HAND_ROLL]   # out in front, hand tipped slightly down
POINT = [0.0, -15.0, 10.0, 5.0, HAND_ROLL]         # arm outstretched, hand level: pointing at the person
RAISED = [0.0, -25.0, -55.0, -10.0, HAND_ROLL]     # forearm and hand pointing up, like a raised hand
REACH = [0.0, 5.0, 20.0, 10.0, HAND_ROLL]          # hand low and out toward the person

WAVE_PERIOD_S = 2.0
BECKON_PERIOD_S = 3.7    # the 70 deg wrist curl needs this long to stay under the ~60 deg/s cap
BECKON_CURL_DEG = 70.0   # wrist pitch travel: from +10 (pointing down) to -60 (pulled back up)
IDLE_CYCLE_S = 126.0     # idle periods 14, 9 and 7 s line up every 126 s


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
    exactly and looks less mechanical. 
    Peak speed is ~22 deg/s.
    '''
    base, shoulder, elbow, wrist, roll = IDLE_CENTER
    return [
        base + 30.0 * _sin(t, 14.0),
        shoulder + 10 * _sin(t, 9.0),
        elbow + 12.0 * _sin(t, 9.0, 1.0),
        wrist + 25.0 * _sin(t, 7.0),
        roll,
    ]


def nearest_idle_time(pose: list[float], step_s: float = 0.1) -> float:
    '''The point in the idle cycle closest to `pose`.

    Resuming idle from here, instead of wherever the clock happens to be, means
    the arm picks up its scanning motion from where it already is rather than
    swinging across to a far-off point on the path.
    '''
    # The base is weighted most: a big base swing is the jarring part, and the
    # raised wave pose can't be matched by idle's other joints anyway.
    weights = (10.0, 1.0, 1.0, 1.0, 0.0)
    times = (i * step_s for i in range(int(IDLE_CYCLE_S / step_s)))
    return min(times, key=lambda t: sum(w * (a - b) ** 2
                                        for w, a, b in zip(weights, idle_pose(t), pose)))


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
        wrist - BECKON_CURL_DEG / 2 * (1 - math.cos(2 * math.pi * t / BECKON_PERIOD_S)),
        roll,
    ]
