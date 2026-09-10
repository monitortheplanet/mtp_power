#!/usr/bin/env python3
# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: test the standstill guard, including every way it must REFUSE.

"""
Test the standstill guard, including every way it must REFUSE.

The interesting cases here are the negative ones. A guard that says "still"
when it does not know is worse than no guard: it converts a missing motor
bridge, a frozen odometry publisher or a one-second gap in the stream into
permission to power off a machine that may be rolling. Each of those has a
test that asserts a refusal and names it.
"""

from mtp_power.standstill import (
    REASON_MISSING,
    REASON_MOVING,
    REASON_STALE,
    StandstillWatch,
)
import pytest

#: The measured odometry rate on this robot, 2026-09-10.
ODOM_PERIOD_S = 0.1

#: The shipped thresholds.
STILL_LINEAR = 0.02
STILL_ANGULAR = 0.02
STILL_FOR_S = 1.0
TIMEOUT_S = 1.0


def watch():
    """Build a guard with the shipped configuration."""
    return StandstillWatch(linear_mps=STILL_LINEAR, angular_rps=STILL_ANGULAR,
                           still_for_s=STILL_FOR_S, timeout_s=TIMEOUT_S)


def feed(guard, seconds, linear=0.0, angular=0.0, start=100.0):
    """Feed `seconds` of odometry at the measured rate; return the end time."""
    steps = int(round(seconds / ODOM_PERIOD_S))
    now = start
    for step in range(1, steps + 1):
        now = start + step * ODOM_PERIOD_S
        guard.sample(now, linear, angular)
    return now


def test_a_guard_that_has_heard_nothing_refuses():
    """No odometry at all is not "stopped", it is "unknown"."""
    assert watch().reason(100.0) == REASON_MISSING


def test_a_robot_still_for_long_enough_is_accepted():
    """The ordinary case: a parked robot, ten samples of it."""
    guard = watch()
    now = feed(guard, 1.5)
    assert guard.reason(now) is None


def test_a_robot_not_yet_still_for_long_enough_is_refused():
    """still_for_s is a window, not an instant: 0.5 s is not 1.0 s."""
    guard = watch()
    now = feed(guard, 0.5)
    assert guard.reason(now) == REASON_MOVING


def test_a_moving_robot_is_refused():
    """The headline case: the tracks are turning."""
    guard = watch()
    now = feed(guard, 2.0, linear=0.3)
    assert guard.reason(now) == REASON_MOVING


def test_a_turning_robot_is_refused():
    """A tracked robot spinning on the spot has no linear speed at all."""
    guard = watch()
    now = feed(guard, 2.0, angular=0.5)
    assert guard.reason(now) == REASON_MOVING


def test_the_noise_of_a_parked_robot_is_not_movement():
    """Wheel odometry never reads exactly zero; the thresholds exist for that."""
    guard = watch()
    now = feed(guard, 2.0, linear=0.01, angular=0.015)
    assert guard.reason(now) is None


def test_motion_restarts_the_window():
    """One moving sample costs the whole still window, not just its own slot."""
    guard = watch()
    now = feed(guard, 2.0)
    assert guard.reason(now) is None
    guard.sample(now + ODOM_PERIOD_S, 0.4, 0.0)
    assert guard.reason(now + ODOM_PERIOD_S) == REASON_MOVING


def test_stale_odometry_is_refused():
    """A publisher that stopped is not a robot that stopped."""
    guard = watch()
    now = feed(guard, 2.0)
    assert guard.reason(now + TIMEOUT_S + 0.1) == REASON_STALE


def test_a_gap_in_the_stream_restarts_the_window():
    """
    Samples that never arrived cannot prove anything about that interval.

    The robot may have driven across the room during the gap; the guard has to
    re-earn its window from the far side of it rather than stitch the two
    halves together.
    """
    guard = watch()
    now = feed(guard, 2.0)
    assert guard.reason(now) is None
    resumed = now + TIMEOUT_S + 0.5
    guard.sample(resumed, 0.0, 0.0)
    assert guard.reason(resumed) == REASON_MOVING
    later = feed(guard, 1.5, start=resumed)
    assert guard.reason(later) is None


def test_a_clock_step_backwards_refuses_rather_than_accepts():
    """A guard whose clock jumped has no window it can trust."""
    guard = watch()
    now = feed(guard, 2.0)
    assert guard.reason(now) is None
    assert guard.reason(now - 60.0) == REASON_STALE


def test_a_sample_from_the_past_restarts_the_window():
    """The same rule on the input side: a backwards sample is a gap."""
    guard = watch()
    now = feed(guard, 2.0)
    guard.sample(now - 60.0, 0.0, 0.0)
    assert guard.reason(now - 60.0) == REASON_MOVING


def test_negative_speeds_count_as_speeds():
    """Reversing is moving. The guard compares magnitudes."""
    guard = watch()
    now = feed(guard, 2.0, linear=-0.3)
    assert guard.reason(now) == REASON_MOVING


def test_the_last_speed_is_reported_for_the_log():
    """The refusal in the log is worth more with the numbers next to it."""
    guard = watch()
    feed(guard, 0.5, linear=-0.25, angular=0.1)
    assert guard.last_speed == (pytest.approx(0.25), pytest.approx(0.1))


def test_a_zero_still_window_still_needs_a_fresh_sample():
    """still_for_s = 0 turns the window off; it does not turn the guard off."""
    guard = StandstillWatch(still_for_s=0.0, timeout_s=TIMEOUT_S)
    assert guard.reason(100.0) == REASON_MISSING
    guard.sample(100.0, 0.0, 0.0)
    assert guard.reason(100.0) is None
    assert guard.reason(100.0 + TIMEOUT_S + 0.1) == REASON_STALE
