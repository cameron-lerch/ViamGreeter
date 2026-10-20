'''
Greeter: notices people walking by the booth, turns to them, waves, beckons
them over, then goes back to idling until the next new person shows up.

Two concurrent loops share state:
  - perception (~6 Hz): camera -> people detector -> PeopleTracker. The camera
    is fixed at the arm's base, so it keeps watching while the arm moves.
  - motion (10 Hz): a small state machine picks the active motion, samples it
    at the current time, blends across switches, clamps to joint limits, and
    sends the pose to the arm.

Greeter follows the following sequence: 
  idle -> (new confirmed person)-> point -> beckon -> wave --
  -> move to next person in the queue or idle
  (person leaves early: cut the greeting short and go back to idle)

Run:  python greeter.py   (watch the arm in the app's 3D scene; Ctrl-C stops)
'''

import asyncio
import os
import time

from viam.components.arm import Arm
from viam.proto.component.arm import JointPositions
from viam.robot.client import RobotClient

import gestures
from connect import connect
from detect_person import PersonDetector
from tracking import PeopleTracker

ARM_NAME = os.environ.get("GREETER_ARM", "arm-1")  # set in .env; "arm-1" is the simulated arm
# Scales how fast the motions play (0.5 = half speed), e.g. for a first run on
# real hardware. Perception and tracking timings stay in real time.
SPEED = float(os.environ.get("GREETER_SPEED", "1.0"))
TICK_S = 0.1              # motion loop period
TRANSITION_S = 1.5        # blend time whenever the active motion changes
POINT_S = 2.5             # point at the person so it's obvious who is being greeted
# Gestures hold still while blending in, then run their full cycles, so the
# blend never swallows the first beckon or wave.
BECKON_S = TRANSITION_S + 2 * gestures.BECKON_PERIOD_S
WAVE_S = TRANSITION_S + 2 * gestures.WAVE_PERIOD_S
MIN_IDLE_S = 2.0          # idle at least this long between greetings
LOST_S = 2.0              # end a greeting early once the target is gone this long
BASE_FOLLOW = 0.2         # per-tick smoothing of the base toward the person's bearing


class Greeter:
    def __init__(self, arm: Arm, detector: PersonDetector):
        self.arm = arm
        self.detector = detector
        self.tracker = PeopleTracker()

        self.phase = "idle"
        self.motion_clock = 0.0  # seconds of motion played; advances at SPEED x real time
        self.phase_start = 0.0   # motion_clock when the current phase began
        self.blend_from: list[float] | None = None
        self.last_cmd: list[float] | None = None
        self.target_id: int | None = None
        self.target_base = 0.0
        self.idle_epoch = 0.0  # idle motion time = motion_clock - idle_epoch

    # --- perception ---------------------------------------------------------

    async def perceive(self):
        while True:
            try:
                people = await self.detector.detect()
            except Exception as e:  # a dropped frame or RPC blip must not stop the robot
                print(f"  detection failed: {type(e).__name__}: {e}")
                people = []
                await asyncio.sleep(0.5)
            self.tracker.update(people, time.monotonic())

    # --- behavior -----------------------------------------------------------

    def set_phase(self, phase: str, now: float):
        waiting = self.tracker.waiting(now, exclude=self.target_id)
        print(f"{phase:>6}  (others waiting: {waiting})")
        self.phase, self.phase_start = phase, self.motion_clock
        self.blend_from = self.last_cmd  # every switch blends from wherever the arm is

    def start_idle(self, now: float):
        '''Resume idle at the point on its path nearest the arm's current pose.'''
        self.idle_epoch = self.motion_clock - gestures.nearest_idle_time(self.last_cmd)
        self.set_phase("idle", now)

    def idle_pose(self) -> list[float]:
        return gestures.idle_pose(self.motion_clock - self.idle_epoch)

    def finish_greeting(self, now: float):
        target = self.tracker.get(self.target_id)
        if target:
            target.greeted = True
        self.target_id = None
        self.start_idle(now)

    def desired_pose(self, now: float) -> list[float]:
        '''Advance the state machine and return the pose for this instant.

        `now` is real time, for perception and tracking; motion timing uses
        motion_clock so GREETER_SPEED can slow the gestures down.
        '''
        t = self.motion_clock - self.phase_start

        if self.phase == "idle":
            target = self.tracker.next_target(now)
            if target and t >= MIN_IDLE_S:
                self.target_id = target.id
                self.target_base = self.last_cmd[0] if self.last_cmd else 0.0
                self.set_phase("point", now)
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
        gesture_t = max(0.0, t - TRANSITION_S)  # gesture clock starts once the blend is done

        if self.phase == "point":
            if t < POINT_S:
                return gestures.point_pose(self.target_base)
            self.set_phase("beckon", now)
            t, gesture_t = 0.0, 0.0
        if self.phase == "beckon":
            if t < BECKON_S:
                return gestures.beckon_pose(gesture_t, self.target_base)
            self.set_phase("wave", now)
            t, gesture_t = 0.0, 0.0
        if t < WAVE_S:
            return gestures.wave_pose(gesture_t, self.target_base)
        self.finish_greeting(now)
        return self.idle_pose()

    async def move(self):
        last = time.monotonic()
        while True:
            tick = time.monotonic()
            self.motion_clock += (tick - last) * SPEED
            last = tick
            pose = self.desired_pose(tick)
            if self.blend_from is not None:
                pose = gestures.blend(self.blend_from, pose,
                                      (self.motion_clock - self.phase_start) / TRANSITION_S)
            pose = gestures.clamp_pose(pose)
            try:
                await self.arm.move_to_joint_positions(JointPositions(values=pose), timeout=2)
                self.last_cmd = pose
            except Exception as e:  # stop, report, and keep going on the next tick
                print(f"  move failed: {type(e).__name__}: {e}")
                await self.arm.stop()
                self.last_cmd = list((await self.arm.get_joint_positions()).values)
            await asyncio.sleep(max(0.0, TICK_S - (time.monotonic() - tick)))

    async def run(self):
        self.last_cmd = list((await self.arm.get_joint_positions()).values)
        print(f"Greeter running on '{ARM_NAME}' at {SPEED:g}x speed (Ctrl-C to stop)...")
        self.start_idle(time.monotonic())  # ease from wherever the arm starts into idle
        try:
            # If either loop crashes, gather raises and the finally stops the arm.
            await asyncio.gather(self.perceive(), self.move())
        finally:
            await self.arm.stop()


async def main():
    async with await connect() as machine:
        greeter = Greeter(Arm.from_robot(machine, ARM_NAME), PersonDetector(machine))
        await greeter.run()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
