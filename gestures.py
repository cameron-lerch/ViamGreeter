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
one is active, switch between them at any instant, and smooth the trajectory across the switch.
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
POINT  = [0.0, -15.0, 10.0, 5.0, HAND_ROLL]        # arm outstretched, hand level: pointing at the person
RAISED = [0.0, -25.0, -55.0, -10.0, HAND_ROLL]     # forearm and hand pointing up, like a raised hand
REACH  = [0.0, 5.0, 20.0, 10.0, HAND_ROLL]         # hand low and out toward the person

# Idle: each joint sways either side of IDLE_CENTER on its own period.
IDLE_SWAY_DEG = (30.0, 10.0, 12.0, 25.0)   # base, shoulder, elbow, wrist pitch: how far each sways either way
IDLE_PERIOD_S = (14, 9, 9, 7)              # seconds per sway, same joint order; uneven periods so the path drifts
IDLE_CYCLE_S  = float(math.lcm(*IDLE_PERIOD_S))   # 126s: when all the periods line up and the idle path repeats

# Wave: the base sways side to side while the wrist rolls back and forth.
WAVE_PERIOD_S = 2.0      # one side-to-side sway
WAVE_SWAY_DEG = 10.0     # base swing either side of the person
WAVE_ROCK_DEG = 15.0     # wrist roll either side of HAND_ROLL

# Beckon: the hand reaches out low, then curls up and back toward the arm.
BECKON_PERIOD_S = 3.7    # the 70 deg wrist curl needs this long to stay under the ~60 deg/s speed cap
BECKON_CURL_DEG = 70.0   # wrist pitch travel: from +10 (pointing down) to -60 (pulled back up)
BECKON_ELBOW_DEG = 10.0  # elbow swing either side of REACH, a quarter cycle off the wrist curl


def _sin(t: float, period: float, phase: float = 0.0) -> float:
    ''' A sine wave with the given period in seconds (and optional phase offset in radians). '''
    return math.sin(2 * math.pi * t / period + phase)

def clamp_pose(pose: list[float]) -> list[float]:
    ''' Keep every joint inside its limits, so no command can ask for an unreachable pose. '''
    return [max(lo + SAFETY_MARGIN_DEG, min(hi - SAFETY_MARGIN_DEG, v))
            for v, (lo, hi) in zip(pose, JOINT_LIMITS)]


def smoothstep(s: float) -> float:
    ''' Smoothstep easing: turns elapsed time (s, 0 -> 1) (normalized to 1s) into how far the arm
        should have moved (0 -> 1), so transitions between gestures start and
        finish gently instead of jerking like a simple linear interpolation.

        f(s) = 3s^2 - 2s^3 is the simplest polynomial (a cubic) with
          f(0) = 0, f(1) = 1     -> starts at the old pose, ends at the new one
          f'(0) = 0, f'(1) = 0   -> zero speed at both ends, so no jolt or abrupt stop
        Solving those four conditions for a s^3 + b s^2 + c s + d gives a = -2, b = 3, c = d = 0.
        A plain linear blend (f(s) = s) can't have zero speed at the ends, so it jerks.
        Also known as "smoothstep" in computer graphics: https://en.wikipedia.org/wiki/Smoothstep
    '''
    s = max(0.0, min(1.0, s))     # clamp: before the transition stay at 0, after it stay at 1
    return s * s * (3 - 2 * s)    # = 3s^2 - 2s^3


def smooth_transition(a: list[float], b: list[float], s: float) -> list[float]:
    ''' Trajectory smoothing between gestures: move every joint from pose a toward
        pose b by the eased fraction smoothstep(s), where s is the fraction of the
        transition's time that has passed (0 -> 1).
    '''
    k = smoothstep(s)
    return [x + (y - x) * k for x, y in zip(a, b)]   # for each joint: start + (end - start) * k


def idle_pose(t: float) -> list[float]:
    ''' Slow scanning sweep: side to side with a gentle up-and-down bob.

        The periods don't divide evenly, so the path drifts instead of repeating
        exactly and looks less mechanical. 
        Peak speed is ~22 deg/s.
    '''
    base, shoulder, elbow, wrist, roll = IDLE_CENTER
    sway_base, sway_shoulder, sway_elbow, sway_wrist = IDLE_SWAY_DEG
    period_base, period_shoulder, period_elbow, period_wrist = IDLE_PERIOD_S
    return [
        base + sway_base * _sin(t, period_base),
        shoulder + sway_shoulder * _sin(t, period_shoulder),
        elbow + sway_elbow * _sin(t, period_elbow, 1.0),   # lags the shoulder by 1 rad so they don't move in lockstep
        wrist + sway_wrist * _sin(t, period_wrist),
        roll,
    ]


def nearest_idle_time(pose: list[float], step_s: float = 0.1) -> float:
    ''' This function finds the time along the idle trajectory that is closest to the 
        arm's current pose, so the transition back to idle is smooth and doesn't jerk the arm around. 

        The weights ensure that the base is prioritized over the other joints, so the arm 
        doesn't swing around to match the idle pose's base angle since the base accounts for the largest motion.
    '''

    weights = (10.0, 1.0, 1.0, 1.0, 0.0)   # base, shoulder, elbow, wrist pitch, wrist roll (roll is constant in idle)
    times = (i * step_s for i in range(int(IDLE_CYCLE_S / step_s)))   # every step_s seconds over one full idle cycle
    return min(times, key=lambda t: sum(w * (a - b) ** 2
                                        for w, a, b in zip(weights, idle_pose(t), pose)))


def point_pose(base: float) -> list[float]:
    ''' Arm outstretched toward the person: makes it obvious who is being greeted. '''
    return [base] + POINT[1:]


def wave_pose(t: float, base: float) -> list[float]:
    ''' Hand up, swaying side to side while the wrist rocks, like a wave.

        The SO-101 has no wrist yaw, so the side-to-side comes from the base joint,
        with wrist roll adding the hand's rocking motion.
    '''
    pose = [base] + RAISED[1:]
    pose[0] += WAVE_SWAY_DEG * _sin(t, WAVE_PERIOD_S)   # base
    pose[4] += WAVE_ROCK_DEG * _sin(t, WAVE_PERIOD_S)   # wrist roll
    return pose


def beckon_pose(t: float, base: float) -> list[float]:
    ''' The hand reaches out and curls back in a loop, beckoning the person 
        to come closer. 
        
        The elbow and wrist pitch move a quarter cycle apart, so the hand 
        traces a loop; the wrist curls from +10 deg (pointing down) to -60 deg 
        (pulled back up)
    
    '''
    _, shoulder, elbow, wrist, roll = REACH
    return [
        base,
        shoulder,
        elbow + BECKON_ELBOW_DEG * _sin(t, BECKON_PERIOD_S),
        # (1 - cos) runs 0 -> 2 -> 0 over one period, so the wrist curls BECKON_CURL_DEG up and back
        wrist - BECKON_CURL_DEG / 2 * (1 - math.cos(2 * math.pi * t / BECKON_PERIOD_S)),
        roll,
    ]
