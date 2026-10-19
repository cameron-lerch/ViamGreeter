'''
Tracking test: turn the arm's base toward the largest (closest) person in view.

Checks the whole camera -> detection -> bearing -> base joint chain before the
greeting gestures are built on top. Only the base joint moves; the other joints
hold whatever pose the arm starts in.

Mapping: the SO-101 points along world +x at base joint 0, matching the camera
axis, but a positive base angle turns it clockwise (toward -y) while bearings
are positive counter-clockwise, so base_joint = -bearing.

Run:  python track_person.py   (watch the arm in the app's 3D scene; Ctrl-C stops)
'''

import asyncio
import time

from viam.components.arm import Arm
from viam.proto.component.arm import JointPositions

from detect_person import PersonDetector
from connect import connect

ARM_NAME = "arm-1"
BASE_LIMIT_DEG = 110.0   # SO-101 base joint limit, from the arm's kinematics
SMOOTHING = 0.5          # [0.0-1.0],fraction of the way to the new target each frame
DEADBAND_DEG = 3.0       # [deg], ignore changes smaller than this so the arm doesn't jitter
LOST_TIMEOUT_S = 2.0     # hold on the last target this long through dropped detections
POLL_S = 0.2


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


async def main():
    async with await connect() as machine:
        arm = Arm.from_robot(machine, ARM_NAME)
        detector = PersonDetector(machine)

        joints = list((await arm.get_joint_positions()).values)
        commanded = joints[0]
        smoothed = commanded
        last_seen = 0.0
        print("Tracking (Ctrl-C to stop)...")

        try:
            while True:
                try:
                    people = await detector.detect()
                except Exception as e:  # keep tracking through a dropped frame or RPC blip
                    print(f"  detection failed: {type(e).__name__}: {e}")
                    people = []

                now = time.monotonic()
                if people:
                    last_seen = now
                    target = clamp(-people[0].bearing_deg, -BASE_LIMIT_DEG, BASE_LIMIT_DEG)
                    smoothed += SMOOTHING * (target - smoothed)
                elif now - last_seen > LOST_TIMEOUT_S:
                    smoothed += SMOOTHING * (0.0 - smoothed)  # nobody around: drift back to center

                if abs(smoothed - commanded) >= DEADBAND_DEG:
                    joints[0] = smoothed
                    try:
                        await arm.move_to_joint_positions(JointPositions(values=joints), timeout=10)
                        commanded = smoothed
                        who = f"person at bearing {people[0].bearing_deg:+.1f}°" if people else "no one"
                        print(f"base -> {commanded:+6.1f}°  ({who})")
                    except Exception as e:  # never let one bad move end the loop
                        print(f"  move failed: {type(e).__name__}: {e}")
                        await arm.stop()

                await asyncio.sleep(POLL_S)
        finally:
            await arm.stop()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
