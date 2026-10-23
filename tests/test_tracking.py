'''
run all of the tests in this directory with:
from the main ViamGreeter directory, run:
pytest tests/*

--
Tests for tracking.PeopleTracker: confirming new people, matching them frame to
frame, forgetting people who leave, and choosing who to greet next.
See README section 5.4.
'''

from detect_person import Person
from tracking import CONFIRM_HITS, FORGET_S, FRESH_S, MATCH_GATE_DEG, PeopleTracker

FRAME_S = 0.16   # about one perception frame


def person(bearing_deg, size=0.4):
    return Person(confidence=0.9, cx=0.0, cy=0.0, width_frac=size / 2, height_frac=size,
                  bearing_deg=bearing_deg, elevation_deg=0.0)


def run_frames(tracker, people_per_frame, start=0.0):
    '''Feed the same people for several frames; return the time after the last one.'''
    now = start
    for people in people_per_frame:
        tracker.update(people, now)
        now += FRAME_S
    return now - FRAME_S


def test_new_person_needs_several_frames_before_being_greeted():
    tracker = PeopleTracker()
    now = run_frames(tracker, [[person(10.0)]] * (CONFIRM_HITS - 1))
    assert tracker.next_target(now) is None          # not confirmed yet: could be a flicker
    now = run_frames(tracker, [[person(10.0)]], start=now + FRAME_S)
    assert tracker.next_target(now) is not None      # confirmed


def test_single_frame_flicker_is_never_greeted():
    # The flicker requirement itself, independent of the CONFIRM_HITS setting.
    tracker = PeopleTracker()
    tracker.update([person(10.0)], 0.0)
    assert tracker.next_target(0.0) is None


def test_small_movement_keeps_the_same_identity():
    tracker = PeopleTracker()
    now = run_frames(tracker, [[person(10.0)], [person(14.0)], [person(18.0)]])
    assert len(tracker.tracks) == 1
    assert tracker.tracks[0].bearing_deg == 18.0


def test_jump_beyond_the_gate_is_a_new_person():
    tracker = PeopleTracker()
    run_frames(tracker, [[person(0.0)], [person(MATCH_GATE_DEG + 5.0)]])
    assert len(tracker.tracks) == 2


def test_person_is_forgotten_after_leaving_for_too_long():
    tracker = PeopleTracker()
    now = run_frames(tracker, [[person(10.0)]] * CONFIRM_HITS)
    tracker.update([], now + FORGET_S / 2)
    assert len(tracker.tracks) == 1                  # short dropout: still remembered
    tracker.update([], now + FORGET_S + 0.1)
    assert len(tracker.tracks) == 0                  # gone too long: forgotten


def test_greeted_person_is_not_greeted_again():
    tracker = PeopleTracker()
    now = run_frames(tracker, [[person(10.0)]] * CONFIRM_HITS)
    tracker.next_target(now).greeted = True
    now = run_frames(tracker, [[person(10.0)]] * 5, start=now + FRAME_S)
    assert tracker.next_target(now) is None


def test_person_not_seen_recently_is_not_chosen():
    tracker = PeopleTracker()
    now = run_frames(tracker, [[person(10.0)]] * CONFIRM_HITS)
    assert tracker.next_target(now + FRESH_S + 0.1) is None


def test_closest_person_is_greeted_first():
    tracker = PeopleTracker()
    far, near = person(-30.0, size=0.2), person(30.0, size=0.6)
    now = run_frames(tracker, [[near, far]] * CONFIRM_HITS)
    assert tracker.next_target(now).bearing_deg == 30.0


def test_waiting_count_excludes_the_current_target():
    tracker = PeopleTracker()
    now = run_frames(tracker, [[person(30.0, size=0.6), person(-30.0, size=0.2)]] * CONFIRM_HITS)
    target = tracker.next_target(now)
    assert tracker.waiting(now) == 2
    assert tracker.waiting(now, exclude=target.id) == 1
