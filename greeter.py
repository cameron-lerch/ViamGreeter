'''
Greeter: notices people walking by the booth, turns to them, waves, beckons
them over, then goes back to idling until the next new person shows up.

Two concurrent loops share state:
  - perception (~6 Hz): camera -> people detector -> PeopleTracker. The camera
    is fixed at the arm's base, so it keeps watching while the arm moves.
  - motion (10 Hz): a small state machine picks the active motion, samples it
    at the current time, blends across switches, clamps to joint limits, and
    sends the pose to the arm.

  idle --(new confirmed person)--> point -> wave -> beckon --> idle
          (person leaves early: cut the greeting short and go back to idle)

Run:  python greeter.py   (watch the arm in the app's 3D scene; Ctrl-C stops)
'''

import asyncio
import time

from viam.components.arm import Arm
from viam.proto.component.arm import JointPositions
from viam.robot.client import RobotClient

import gestures
from connect import connect
from detect_person import PersonDetector
from tracking import PeopleTracker

ARM_NAME = "arm-1"
TICK_S = 0.1              # motion loop period
TRANSITION_S = 1.5        # blend time whenever the active motion changes
POINT_S = 2.5             # point at the person so it's obvious who is being greeted
WAVE_S = 2 * gestures.WAVE_PERIOD_S
BECKON_S = 2 * gestures.BECKON_PERIOD_S
MIN_IDLE_S = 2.0          # idle at least this long between greetings
LOST_S = 2.0              # end a greeting early once the target is gone this long
BASE_FOLLOW = 0.2         # per-tick smoothing of the base toward the person's bearing


class Greeter:
    def __init__(self, arm: Arm, detector: PersonDetector):
        self.arm = arm
        self.detector = detector
        self.tracker = PeopleTracker()

        self.phase = "idle"
        self.phase_start = time.monotonic()
        self.blend_from: list[float] | None = None
        self.last_cmd: list[float] | None = None
        self.target_id: int | None = None
        self.target_base = 0.0

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
        self.phase, self.phase_start = phase, now
        self.blend_from = self.last_cmd  # every switch blends from wherever the arm is

    def finish_greeting(self, now: float):
        target = self.tracker.get(self.target_id)
        if target:
            target.greeted = True
        self.target_id = None
        self.set_phase("idle", now)

    def desired_pose(self, now: float) -> list[float]:
        '''Advance the state machine and return the pose for this instant.'''
        t = now - self.phase_start

        if self.phase == "idle":
            target = self.tracker.next_target(now)
            if target and t >= MIN_IDLE_S:
                self.target_id = target.id
                self.target_base = self.last_cmd[0] if self.last_cmd else 0.0
                self.set_phase("point", now)
                t = 0.0
            else:
                return gestures.idle_pose(now)

        target = self.tracker.get(self.target_id)
        if target is None or now - target.last_seen > LOST_S:
            print("        target left")
            self.finish_greeting(now)
            return gestures.idle_pose(now)

        # The base keeps following the person through every greeting phase.
        self.target_base += BASE_FOLLOW * (-target.bearing_deg - self.target_base)

        if self.phase == "point":
            if t < POINT_S:
                return gestures.point_pose(self.target_base)
            self.set_phase("wave", now)
            t = 0.0
        if self.phase == "wave":
            if t < WAVE_S:
                return gestures.wave_pose(t, self.target_base)
            self.set_phase("beckon", now)
            t = 0.0
        if t < BECKON_S:
            return gestures.beckon_pose(t, self.target_base)
        self.finish_greeting(now)
        return gestures.idle_pose(now)

    async def move(self):
        while True:
            tick = time.monotonic()
            pose = self.desired_pose(tick)
            if self.blend_from is not None:
                pose = gestures.blend(self.blend_from, pose, (tick - self.phase_start) / TRANSITION_S)
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
        self.blend_from = self.last_cmd  # ease from wherever the arm starts into idle
        self.phase_start = time.monotonic()
        print("Greeter running (Ctrl-C to stop)...")
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
