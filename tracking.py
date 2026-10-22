'''
Lightweight people tracker: turns per-frame detections into persistent tracks.

Detections carry no identity, so each frame's people are matched to existing
tracks by nearest bearing. That gives Greeter what it needs to:
  - ignore flicker: a track must be seen in several frames before it counts, and
    survives a short dropout before it is forgotten
  - remember who it has already greeted, so it moves on instead of greeting the
    same person over and over while they stand there
  - keep a queue: the next target is the closest confirmed person not yet greeted

Matching on bearing alone is deliberately simple; two people crossing paths can
swap tracks. Good enough for a booth, and called out in the README.
'''

import itertools
from dataclasses import dataclass, field

from detect_person import Person

MATCH_GATE_DEG = 12.0   # a detection this close to a track's bearing is the same person
CONFIRM_HITS = 3        # frames a new track needs before it can be greeted
FORGET_S = 3.0          # drop a track not seen for this long
FRESH_S = 0.6           # only pick targets seen this recently

_ids = itertools.count(1)


@dataclass
class Track:
    bearing_deg: float
    size: float
    last_seen: float
    id: int = field(default_factory=lambda: next(_ids))
    hits: int = 1
    greeted: bool = False

    @property
    def confirmed(self) -> bool:
        return self.hits >= CONFIRM_HITS


class PeopleTracker:
    def __init__(self):
        self.tracks: list[Track] = []

    def update(self, people: list[Person], now: float) -> None:
        unmatched = list(self.tracks)
        for p in people:  # track the largest (closest) bounding box first
            best = min(unmatched, key=lambda t: abs(t.bearing_deg - p.bearing_deg), default=None)
            if best is not None and abs(best.bearing_deg - p.bearing_deg) <= MATCH_GATE_DEG:
                best.bearing_deg, best.size, best.last_seen = p.bearing_deg, p.size, now
                best.hits += 1
                unmatched.remove(best)
            else:
                self.tracks.append(Track(p.bearing_deg, p.size, now))
        self.tracks = [t for t in self.tracks if now - t.last_seen <= FORGET_S]

    def get(self, track_id: int) -> Track | None:
        return next((t for t in self.tracks if t.id == track_id), None)

    def next_target(self, now: float) -> Track | None:
        ''' Closest confirmed, not-yet-greeted person currently in view. '''
        queue = [t for t in self.tracks
                 if t.confirmed and not t.greeted and now - t.last_seen <= FRESH_S]
        return max(queue, key=lambda t: t.size, default=None)

    def waiting(self, now: float, exclude: int | None = None) -> int:
        return sum(1 for t in self.tracks if t.confirmed and not t.greeted
                   and now - t.last_seen <= FRESH_S and t.id != exclude)
