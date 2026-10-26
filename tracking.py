'''
People tracker: follows each person from frame to frame and decides who to greet next.

The detector reports "there is a person at this bearing" in each frame; it
can't tell whether that's the same person as last frame. 
The tracker gives people an identity by matching each frame's detections to 
existing tracks (one track per person) by nearest bearing. 
This lets us tackle several of the challenges with Greeter:
  - ignore flicker: a new person must be seen in several frames before they can
    be greeted, and a short dropout doesn't make Greeter forget them
  - remember who has been greeted, so it moves on instead of greeting someone
    over and over while they stand there
  - keep a queue: the next person to greet is the closest one who has been
    confirmed and not greeted yet

People too far from the booth never reach the tracker: detect_person.py filters
them out by box size (MIN_SIZE). This helps with random locking onto people in the background.

Known limitation: matching on bearing alone is simple, so two people
crossing paths can swap tracks, several options moving forward are discussed in the README.
'''

import itertools
from dataclasses import dataclass, field

from detect_person import Person

MATCH_GATE_DEG = 12.0   # a detection within this many degrees of a track's bearing is the same person
CONFIRM_HITS = 3        # frames a new person must be seen in before they can be greeted (~0.5s at ~6 Hz)
FORGET_S = 3.0          # forget a person not seen for this long; if they come back, they're new
FRESH_S = 0.6           # only choose someone to greet if they were seen this recently (still in view)

_ids = itertools.count(1)   # unique, increasing track IDs: 1, 2, 3, ...


@dataclass
class Track:
    ''' One person being followed across frames. '''
    bearing_deg: float      # latest direction, degrees (+ = camera's left)
    size: float             # latest box height as a fraction of the image (bigger = closer)
    last_seen: float        # time of the latest matching detection (seconds, time.monotonic)
    id: int = field(default_factory=lambda: next(_ids))
    hits: int = 1           # number of frames this person has been matched in (not necessarily in a row)
    greeted: bool = False   # set once Greeter has greeted (or given up on) this person

    @property
    def confirmed(self) -> bool:
        ''' Seen in enough frames to be a real person rather than a one-off false detection. '''
        return self.hits >= CONFIRM_HITS


class PeopleTracker:
    ''' Follows people across frames and picks who to greet next. '''

    def __init__(self):
        self.tracks: list[Track] = []

    def update(self, people: list[Person], now: float) -> None:
        ''' Match one frame's detections to existing tracks, start tracks for new
            people, and forget people who haven't been seen for FORGET_S.
        '''
        unmatched = list(self.tracks)   # each track can be matched by at most one detection per frame
        for p in people:  # people arrive largest (closest) first, so the closest claim tracks first
            best = min(unmatched, key=lambda t: abs(t.bearing_deg - p.bearing_deg), default=None)
            if best is not None and abs(best.bearing_deg - p.bearing_deg) <= MATCH_GATE_DEG:
                # Same person as an existing track: update where they are and count the sighting.
                best.bearing_deg, best.size, best.last_seen = p.bearing_deg, p.size, now
                best.hits += 1
                unmatched.remove(best)
            else:
                # Nobody close enough in bearing: a new person (not confirmed yet).
                self.tracks.append(Track(p.bearing_deg, p.size, now))
        self.tracks = [t for t in self.tracks if now - t.last_seen <= FORGET_S]   # drop people gone too long

    def get(self, track_id: int) -> Track | None:
        ''' The track with this ID, or None if that person has been forgotten. '''
        return next((t for t in self.tracks if t.id == track_id), None)

    def next_target(self, now: float) -> Track | None:
        ''' The next person to greet: the closest (largest box) who is confirmed,
            not yet greeted, and currently in view. None if nobody qualifies.
        '''
        queue = [t for t in self.tracks
                 if t.confirmed and not t.greeted and now - t.last_seen <= FRESH_S]
        return max(queue, key=lambda t: t.size, default=None)

    def waiting(self, now: float, exclude: int | None = None) -> int:
        ''' How many people are waiting to be greeted, not counting `exclude`
            (the person currently being greeted). Used for log messages.
        '''
        return sum(1 for t in self.tracks if t.confirmed and not t.greeted
                   and now - t.last_seen <= FRESH_S and t.id != exclude)
