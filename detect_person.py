'''
Person detection for the ViamGreeter robot.

Pulls frames from the webcam through a Viam vision service running a pre-trained
people detector (the EfficientDet-COCO model by viam-labs on the TFLite CPU ML
model service "people_model", wrapped by the mlmodel vision service "people_detector"),
keeps only the
"person" detections, and converts each bounding box into a direction in the
robot's world frame.

Camera assumption: the camera sits at the arm's base, fixed to the pedestal (it
does not rotate with the arm), with its optical axis along world +x and the image
"up" aligned with world +z. With the world frame's x forward, y left, z up:
  - bearing   = rotation about +z from +x, positive counter-clockwise (to the left)
  - elevation = rotation up from the horizontal plane, positive up

Run directly to print live detections and per-frame latency:
    python detect_person.py
'''

import asyncio
import math
import time
from dataclasses import dataclass

from viam.robot.client import RobotClient
from viam.services.vision import VisionClient

VISION_NAME = "people_detector"
CAMERA_NAME = "cam"
CAMERA_HFOV_DEG = 70.0   # horizontal field of view of the camera (70 deg is good for most webcams)
MIRRORED = False         # set True if the webcam feed is horizontally flipped
MIN_CONFIDENCE = 0.5     # ignore detections the model is less than 50% sure of
MIN_SIZE = 0.25          # ignore people whose box is under 25% of the image height: too far from the booth to greet

PERSON_LABEL = "person"  # compared case-insensitively (EfficientDet-COCO reports "Person")


@dataclass
class Person:
    confidence: float
    cx: float              # box center in pixels
    cy: float
    width_frac: float      # box size as a fraction of the image
    height_frac: float
    bearing_deg: float     # world frame, + is to the left of +x
    elevation_deg: float   # + is above the camera's horizon

    @property
    def size(self) -> float:
        ''' Box height as a fraction of the image: a rough proximity proxy (bigger is closer). '''
        return self.height_frac


def pixel_to_direction(u: float, v: float, width: int, height: int,
                       hfov_deg: float = CAMERA_HFOV_DEG,
                       mirrored: bool = MIRRORED) -> tuple[float, float]:
    ''' Map a pixel to (bearing_deg, elevation_deg) with a pinhole camera model.

        The focal length in pixels comes from the horizontal field of view; square
        pixels are assumed, so the same focal length applies vertically. Image +u
        (right) is world -y and image +v (down) is world -z, hence the negations.
    '''
    f_px = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
    bearing = -math.degrees(math.atan2(u - width / 2, f_px))
    elevation = -math.degrees(math.atan2(v - height / 2, f_px))
    if mirrored:
        bearing = -bearing
    return bearing, elevation


class PersonDetector:
    def __init__(self, robot: RobotClient, min_confidence: float = MIN_CONFIDENCE, min_size: float = MIN_SIZE):
        self.vision = VisionClient.from_robot(robot, VISION_NAME)
        self.camera_name = CAMERA_NAME
        self.min_confidence = min_confidence
        self.min_size = min_size

    async def detect(self) -> list[Person]:
        ''' One frame's people, largest (likely closest) first.

            Image and detections come back from the same capture, so the box pixel
            coordinates always match the image size used to convert them.
        '''
        result = await self.vision.capture_all_from_camera(
            self.camera_name, return_image=True, return_detections=True,
            timeout=5,   # seconds; a frame normally takes ~160 ms, so this only trips if the camera or service hangs
        )
        if result.image is None or not result.image.width:
            raise RuntimeError("vision service returned no image; cannot size detections")
        width, height = result.image.width, result.image.height

        people = []
        for d in result.detections or []:
            if d.class_name.lower() != PERSON_LABEL or d.confidence < self.min_confidence:
                continue
            if (d.y_max - d.y_min) / height < self.min_size:   # filters out people who are in the background, so that only people within a certain (tunable) range are detected.
                continue
            cx, cy = (d.x_min + d.x_max) / 2, (d.y_min + d.y_max) / 2
            bearing, elevation = pixel_to_direction(cx, cy, width, height)
            people.append(Person(
                confidence=d.confidence, cx=cx, cy=cy,
                width_frac=(d.x_max - d.x_min) / width,
                height_frac=(d.y_max - d.y_min) / height,
                bearing_deg=bearing, elevation_deg=elevation,
            ))
        return sorted(people, key=lambda p: p.size, reverse=True)


async def main():
    from connect import connect

    async with await connect() as machine:
        names = {rn.name for rn in machine.resource_names}
        missing = [n for n in (VISION_NAME, CAMERA_NAME) if n not in names]
        if missing:
            print(f"Not configured on the machine yet: {', '.join(missing)}")
            return

        detector = PersonDetector(machine)
        print("Watching for people (Ctrl-C to stop)...")
        while True:
            start = time.perf_counter()
            try:
                people = await detector.detect()
            except Exception as e:  # keep watching through a dropped frame or RPC blip
                print(f"  detection failed: {type(e).__name__}: {e}")
                await asyncio.sleep(1)   # viewer only: pause a second before retrying a failed frame
                continue
            latency_ms = (time.perf_counter() - start) * 1000
            summary = " | ".join(
                f"person {p.confidence:.2f} bearing {p.bearing_deg:+6.1f}° "
                f"elev {p.elevation_deg:+5.1f}° size {p.size:.2f}"
                for p in people) or "no one"
            print(f"[{latency_ms:5.0f} ms] {summary}")
            
            await asyncio.sleep(0.2)    # # Viewer only: slows the printout to ~5 lines a second so it's readable. Does not impact Greeter's perception loop


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
