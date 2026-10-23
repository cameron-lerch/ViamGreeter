'''
Greeter: notices people walking by the booth, turns to them, beckons
them over, waves, then goes back to idling until the next new person shows up.

Two concurrent loops share state:
  - perception (~6 Hz): camera -> people detector -> PeopleTracker. The camera
    is fixed at the arm's base, so it keeps watching while the arm moves.
  - motion (10 Hz): a small state machine picks the active motion, samples it
    at the current time, smooths the trajectory across switches, clamps to joint 
    limits, and sends the pose to the arm.

Greeter follows the following sequence: 
  idle -> (new confirmed person)-> point -> beckon -> wave -> idle (or next person)
  (if a person leaves early: cut the greeting short and go back to idle)

Run:  python greeter.py   (watch the arm in the app's 3D scene; Ctrl-C stops)
'''

import asyncio
import os
import time
from datetime import timedelta
from enum import Enum

from viam.components.arm import Arm
from viam.proto.component.arm import JointPositions
from viam.robot.client import RobotClient

import gestures
from connect import connect
from detect_person import PersonDetector
from tracking import PeopleTracker

ARM_NAME = os.environ.get("GREETER_ARM", "arm-1")               # set in .env; "arm-1" is the simulated arm
SPEED    = float(os.environ.get("GREETER_SPEED", "1.0"))        # scales how quickly the motions are carried out, used to slow down when first testing on the real arm.
HAND_ROLL_BY_ARM = {"arm-SO101-real": 0.0}                      # wrist roll changes from simulated arm to real arm, simulated = 90deg, real = 0deg
ROLL_OFFSET = HAND_ROLL_BY_ARM.get(ARM_NAME, gestures.HAND_ROLL) - gestures.HAND_ROLL   # offset needed for the real arm wrist joint
WAVE_RAISED_BY_ARM = {"arm-SO101-real": [-15.0, -50.0, -25.0]}  # [shoulder, elbow, wrist pitch], tuned so that the real arm extends less than the simulated arm to avoid shaking
WAVE_OFFSET = [want - base for want, base in
               zip(WAVE_RAISED_BY_ARM.get(ARM_NAME, gestures.RAISED[1:4]), gestures.RAISED[1:4])]   # offset to keep the real arm more stable
MOTION_BY_ARM = {"arm-SO101-real": "streamed"}                  # the real arm requires different motion commands than the simulated arm to avoid jerky behavior
MOTION = MOTION_BY_ARM.get(ARM_NAME, "steps")                    # any other arm (e.g. the simulated arm) uses steps

# All times below are plain float seconds (what time.monotonic() and asyncio.sleep() use).
TICK_S = 0.1              # motion loop period (10 Hz): one move per tick on the simulated arm, one trajectory point per tick on the real arm
STREAM_LEAD_S = 0.3       # each streamed point is timestamped this far ahead, so the real arm always has ~3 points queued
TRANSITION_S  = 1.5       # trajectory smoothing time whenever the active gesture changes
WAVE_SMOOTH_S = 1.2       # smoother (shorter) transition between the beckon and the wave
POINT_S  = 2.5            # duration for pointing at a person
BECKON_S = TRANSITION_S + 2 * gestures.BECKON_PERIOD_S    # total duration for the beckon, with time for smoothing
WAVE_S   = WAVE_SMOOTH_S + 2 * gestures.WAVE_PERIOD_S     # total duration for the wave, with time for smoothing
MIN_IDLE_S  = 2.0         # idle at least this long between greetings to avoid rapidly jumping from person to person
LOST_S      = 2.0         # end a greeting early once the target is gone this long
BASE_FOLLOW = 0.2         # per-tick smoothing of the base toward the person's bearing

# Error handling (see recover() and run())
ARM_CALL_TIMEOUT_S = 2.0  # give up on any single arm call after this long
RETRY_DELAY_S      = 0.5  # pause before retrying a failed camera frame or start-pose read, so failures don't spin in a tight loop
BACKOFF_STEP_S     = 0.25 # after a failed arm command, wait this long per failure in a row...
BACKOFF_MAX_S      = 2.0  # ...but never longer than this
START_POSE_ATTEMPTS = 5   # tries at reading the arm's starting pose before giving up


class Phase(Enum):
    ''' The state machine's phases. Using an enum instead of plain strings means a
        misspelled phase fails immediately instead of silently skipping a gesture.
    '''
    IDLE = "idle"
    POINT = "point"
    BECKON = "beckon"
    WAVE = "wave"


class Greeter:
    def __init__(self, arm: Arm, detector: PersonDetector):
        self.arm      = arm
        self.detector = detector
        self.tracker  = PeopleTracker()

        self.phase = Phase.IDLE
        self.motion_clock = 0.0  # seconds of motion played; advances at SPEED x real time
        self.phase_start  = 0.0  # motion_clock when the current phase began
        self.smooth_from: list[float] | None = None
        self.last_cmd: list[float] | None = None
        self.target_id: int | None = None
        self.target_base = 0.0
        self.idle_epoch  = 0.0           # idle motion time = motion_clock - idle_epoch
        self.smooth_s    = TRANSITION_S  # trajectory smoothing time for the current phase

    # --- perception ---------------------------------------------------------

    async def perceive(self):
        while True:
            try:
                people = await self.detector.detect()
            except Exception as e:  # a dropped frame or RPC blip must not stop the robot
                print(f"  detection failed: {type(e).__name__}: {e}")
                people = []
                await asyncio.sleep(RETRY_DELAY_S)
            self.tracker.update(people, time.monotonic())

    # --- behavior -----------------------------------------------------------

    def set_phase(self, phase: Phase, now: float, smooth_s: float = TRANSITION_S):
        waiting = self.tracker.waiting(now, exclude=self.target_id)
        print(f"{phase.value:>6}  (others waiting: {waiting})")
        self.phase, self.phase_start = phase, self.motion_clock
        self.smooth_from = self.last_cmd  # every switch smooths from wherever the arm is
        self.smooth_s = smooth_s

    def start_idle(self, now: float):
        ''' Resume idle at the point on its path nearest the arm's current pose. '''
        self.idle_epoch = self.motion_clock - gestures.nearest_idle_time(self.last_cmd)
        self.set_phase(Phase.IDLE, now)

    def idle_pose(self) -> list[float]:
        return gestures.idle_pose(self.motion_clock - self.idle_epoch)

    def finish_greeting(self, now: float):
        target = self.tracker.get(self.target_id)
        if target:
            target.greeted = True
        self.target_id = None
        self.start_idle(now)

    def desired_pose(self, now: float) -> list[float]:
        ''' Advance the state machine and return the pose for this instant.

            `now` is real time, for perception and tracking; motion timing uses
            motion_clock so GREETER_SPEED can slow the gestures down for testing on a real arm
        '''
        t = self.motion_clock - self.phase_start

        if self.phase == Phase.IDLE:
            target = self.tracker.next_target(now)
            if target and t >= MIN_IDLE_S:
                self.target_id = target.id
                self.target_base = self.last_cmd[0] if self.last_cmd else 0.0
                self.set_phase(Phase.POINT, now)
                t = 0.0
            else:
                return self.idle_pose()

        target = self.tracker.get(self.target_id)
        if target is None or now - target.last_seen > LOST_S:
            print("        target left")
            self.finish_greeting(now)
            return self.idle_pose()

        # The base keeps following the person through every greeting phase.
        self.target_base += BASE_FOLLOW * (-target.bearing_deg - self.target_base)
        gesture_t = max(0.0, t - TRANSITION_S)  # gesture clock starts once the transition is smoothed

        if self.phase == Phase.POINT:
            if t < POINT_S:
                return gestures.point_pose(self.target_base)
            self.set_phase(Phase.BECKON, now)
            t, gesture_t = 0.0, 0.0
        if self.phase == Phase.BECKON:
            if t < BECKON_S:
                return gestures.beckon_pose(gesture_t, self.target_base)
            self.set_phase(Phase.WAVE, now, smooth_s=WAVE_SMOOTH_S)
            t = 0.0
        if t < WAVE_S:
            pose = gestures.wave_pose(t, self.target_base)  # sways during the transition too
            pose[1:4] = [v + o for v, o in zip(pose[1:4], WAVE_OFFSET)]
            return pose
        self.finish_greeting(now)
        return self.idle_pose()

    async def move(self):
        last = time.monotonic()
        failures = 0
        while True:
            tick = time.monotonic()
            self.motion_clock += (tick - last) * SPEED
            last = tick
            pose = self.desired_pose(tick)
            pose[4] += ROLL_OFFSET  # wrist roll; before smoothing, because smooth_from is in the arm's own roll
            if self.smooth_from is not None:
                pose = gestures.smooth_transition(self.smooth_from, pose,
                                                  (self.motion_clock - self.phase_start) / self.smooth_s)
            pose = gestures.clamp_pose(pose)
            try:
                await self.arm.move_to_joint_positions(JointPositions(values=pose), timeout=ARM_CALL_TIMEOUT_S)
                self.last_cmd = pose
                failures = 0
            except Exception as e:  # report, back off, and keep going
                failures += 1
                print(f"  move failed ({failures} in a row): {type(e).__name__}: {e}")
                await self.recover(failures)
            # sleep for whatever is left of this tick, so ticks stay TICK_S apart even though each command takes time
            await asyncio.sleep(max(0.0, TICK_S - (time.monotonic() - tick)))

    def next_pose(self, tick: float, dt: float) -> list[float]:
        ''' Advance the motion clock by dt and return the clamped pose to command. '''
        self.motion_clock += dt * SPEED
        pose = self.desired_pose(tick)
        pose[4] += ROLL_OFFSET  # wrist roll; before smoothing, because smooth_from is in the arm's own roll
        if self.smooth_from is not None:
            pose = gestures.smooth_transition(self.smooth_from, pose,
                                              (self.motion_clock - self.phase_start) / self.smooth_s)
        return gestures.clamp_pose(pose)

    async def move_streamed(self):
        ''' Real-arm motion: the same poses as move(), sent as one continuous,
            time-stamped trajectory so the arm flows between them without stopping.

            Each tick's pose is stamped STREAM_LEAD_S ahead of now, keeping a small
            buffer in the arm. If the stream faults, recover and start a new one
            from wherever the arm is (a stream must begin at rest, at time zero).
        '''
        failures = 0
        while True:
            start = last = time.monotonic()
            origin = list(self.last_cmd)

            async def batches():    # generate the next batch of trajectory points, one per tick, until the stream ends 
                nonlocal last
                yield [Arm.TrajectoryPoint(time=timedelta(seconds=0.0), positions=origin)]
                while True:
                    await asyncio.sleep(max(0.0, TICK_S - (time.monotonic() - last)))   # wait out the rest of this tick
                    tick = time.monotonic()
                    pose = self.next_pose(tick, tick - last)
                    last = tick
                    yield [Arm.TrajectoryPoint(
                        time=timedelta(seconds=tick - start + STREAM_LEAD_S), positions=pose)]
                    self.last_cmd = pose

            try:
                async for _ in self.arm.move_through_joint_positions_streamed(batches()):
                    failures = 0
                print("  trajectory stream ended; starting a new one")
            except Exception as e:  # report, back off, and stream again
                failures += 1
                print(f"  stream failed ({failures} in a row): {type(e).__name__}: {e}")
                await self.recover(failures)

    async def recover(self, failures: int):
        ''' Best-effort recovery after a failed move. Never raises.

            On real hardware the recovery calls can fail too (e.g. a servo bus
            timeout), so each step is guarded; the loop just tries again next tick.
            Repeated failures back off (BACKOFF_STEP_S per failure in a row, up to
            BACKOFF_MAX_S) to give the bus time to clear.
        '''
        try:
            await self.arm.stop(timeout=ARM_CALL_TIMEOUT_S)
        except Exception as e:
            print(f"    stop failed too: {type(e).__name__}")
        try:  # smooth on from where the arm really is; keep the old pose if unreadable
            self.last_cmd = list((await self.arm.get_joint_positions(timeout=ARM_CALL_TIMEOUT_S)).values)
        except Exception as e:
            print(f"    could not read joints: {type(e).__name__}")
        await asyncio.sleep(min(BACKOFF_MAX_S, BACKOFF_STEP_S * failures))

    async def run(self):
        for attempt in range(1, START_POSE_ATTEMPTS + 1):  # the starting pose is needed to ease into idle
            try:
                self.last_cmd = list((await self.arm.get_joint_positions(timeout=ARM_CALL_TIMEOUT_S)).values)
                break
            except Exception as e:
                print(f"  reading start pose failed (attempt {attempt}/{START_POSE_ATTEMPTS}): {type(e).__name__}")
                if attempt == START_POSE_ATTEMPTS:
                    raise
                await asyncio.sleep(RETRY_DELAY_S)
        print(f"Greeter running on '{ARM_NAME}' ({MOTION} motion) at {SPEED:g}x speed (Ctrl-C to stop)...")
        self.start_idle(time.monotonic())  # ease from wherever the arm starts into idle
        try:
            # If either loop crashes, gather raises and the finally stops the arm.
            move = self.move_streamed() if MOTION == "streamed" else self.move()
            await asyncio.gather(self.perceive(), move)
        finally:
            try:
                await self.arm.stop(timeout=ARM_CALL_TIMEOUT_S)
            except Exception as e:
                print(f"  final stop failed: {type(e).__name__}: {e}")


async def main():
    async with await connect() as machine:
        greeter = Greeter(Arm.from_robot(machine, ARM_NAME), PersonDetector(machine))
        await greeter.run()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
