#!/usr/bin/env python3
# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: exhaustively test the hold gesture against a fake clock.

"""
Exhaustively test the hold gesture against a fake clock.

No rclpy, no sleeping, no real time: the machine is driven message by message
through a `Radio` helper that reproduces what the robot actually publishes --
a LEVEL at 20 Hz, pressed or not (measured 2026-09-10) -- so a test can hold
the button for exactly 2.95 s and assert that nothing fired.

Every rule the feature brief names has a test of its own here: a hold shorter
than `arm_hold_s`, a release during `confirming`, silence counting as a
release, a refusal while moving that cannot be re-armed by keeping the button
down, a second press after an abort, and a clock that steps backwards
re-anchoring instead of firing.
"""

from mtp_power.hold_gesture import (
    ABORTED,
    ARMING,
    CONFIRMING,
    GestureConfig,
    HoldGesture,
    IDLE,
    REASON_MOVING,
    REASON_RELEASED,
    SHUTTING_DOWN,
)
import pytest

#: The shipped gesture: 1.0 s to arm, 2.0 s more to halt, 3.0 s in total.
CONFIG = GestureConfig(arm_hold_s=1.0, confirm_hold_s=2.0, arming_ack_s=0.5,
                       button_timeout_s=1.0, aborted_hold_s=2.0)

#: The measured radio rate. The machine is driven at exactly this.
BUTTON_PERIOD_S = 0.05


class Radio:
    """Drive a HoldGesture the way the real radio topic and timer would."""

    def __init__(self, config=CONFIG, start=1000.0):
        self.gesture = HoldGesture(config)
        self.start = start
        # Counted in ticks and multiplied out, never accumulated: adding 0.05
        # sixty times lands a few ulps short of 3.0 s, and a test for "fires
        # at exactly the total hold" would then fail on float dust rather
        # than on the machine.
        self.ticks = 0
        self.last = None

    @property
    def now(self):
        """Return the fake clock, in seconds."""
        return self.start + self.ticks * BUTTON_PERIOD_S

    def run(self, seconds, level=None, standstill_reason=None, dry_run=False,
            publish_button=True):
        """
        Advance `seconds`, delivering `level` at 20 Hz unless told not to.

        `publish_button=False` is the radio going silent while the previous
        level stands -- the case `button_timeout_s` exists for.
        """
        steps = max(1, int(round(seconds / BUTTON_PERIOD_S)))
        for _ in range(steps):
            self.ticks += 1
            if publish_button and level is not None:
                self.gesture.button(level, self.now)
            self.last = self.gesture.update(self.now, standstill_reason, dry_run)
        return self.last

    def states(self, seconds, level, standstill_reason=None, dry_run=False):
        """Advance and return every distinct state seen, in order."""
        seen = []
        steps = max(1, int(round(seconds / BUTTON_PERIOD_S)))
        for _ in range(steps):
            self.ticks += 1
            self.gesture.button(level, self.now)
            result = self.gesture.update(self.now, standstill_reason, dry_run)
            if not seen or seen[-1] != result.state:
                seen.append(result.state)
        self.last = result
        return seen


@pytest.fixture
def radio():
    """Build a machine at rest, with the shipped gesture and a robot still."""
    return Radio()


# ── The gesture, stage by stage ─────────────────────────────────────────

def test_a_machine_nobody_has_touched_is_idle(radio):
    """Before any button message the state is idle with nothing pending."""
    result = radio.run(0.5, level=None)
    assert result.state == IDLE
    assert result.seconds_left == 0.0
    assert result.reason == ''


def test_a_hold_shorter_than_arm_hold_does_nothing(radio):
    """The whole point of arm_hold_s: a brush of the button is not a request."""
    assert radio.run(0.9, level=True).state == IDLE
    assert radio.run(0.5, level=False).state == IDLE


def test_the_request_arms_at_arm_hold_s(radio):
    """One tick past 1.0 s of continuous hold, the request is acknowledged."""
    radio.run(0.9, level=True)
    assert radio.gesture.state == IDLE
    assert radio.run(0.2, level=True).state == ARMING


def test_the_countdown_starts_after_the_acknowledgement(radio):
    """`arming` lasts arming_ack_s, then the state becomes `confirming`."""
    radio.run(1.4, level=True)
    assert radio.gesture.state == ARMING
    assert radio.run(0.2, level=True).state == CONFIRMING


def test_the_halt_is_reached_at_the_total_hold_and_not_before(radio):
    """2.95 s is not 3.0 s: the machine must not fire early."""
    assert radio.run(2.95, level=True).state == CONFIRMING
    assert radio.run(0.1, level=True).state == SHUTTING_DOWN


def test_the_whole_sequence_is_seen_in_order(radio):
    """Walk idle -> arming -> confirming -> shutting_down, and nothing else."""
    assert radio.states(3.2, level=True) == [IDLE, ARMING, CONFIRMING, SHUTTING_DOWN]


def test_the_countdown_falls_towards_zero(radio):
    """seconds_left is the time left before the halt, to one decimal."""
    assert radio.run(1.1, level=True).seconds_left == pytest.approx(1.95, abs=0.1)
    assert radio.run(1.0, level=True).seconds_left == pytest.approx(0.95, abs=0.1)


def test_seconds_left_is_zero_outside_arming_and_confirming(radio):
    """The contract says 0.0 outside the two pending states."""
    assert radio.run(0.5, level=True).seconds_left == 0.0
    assert radio.run(3.0, level=True).state == SHUTTING_DOWN
    assert radio.last.seconds_left == 0.0


# ── Releasing ───────────────────────────────────────────────────────────

def test_release_during_arming_aborts(radio):
    """Letting go before the countdown cancels the request."""
    radio.run(1.2, level=True)
    assert radio.gesture.state == ARMING
    result = radio.run(0.1, level=False)
    assert result.state == ABORTED
    assert result.reason == REASON_RELEASED


def test_release_during_confirming_aborts(radio):
    """The rule the operator relies on: let go and it stops."""
    radio.run(2.5, level=True)
    assert radio.gesture.state == CONFIRMING
    result = radio.run(0.1, level=False)
    assert result.state == ABORTED
    assert result.reason == REASON_RELEASED


def test_an_abort_returns_to_idle_after_aborted_hold_s(radio):
    """`aborted` is terminal for the request, then the machine is idle again."""
    radio.run(1.2, level=True)
    radio.run(0.1, level=False)
    assert radio.gesture.state == ABORTED
    assert radio.run(1.0, level=False).state == ABORTED
    assert radio.run(1.2, level=False).state == IDLE


def test_a_second_press_after_an_abort_works(radio):
    """An aborted request must not poison the button."""
    radio.run(1.5, level=True)
    radio.run(2.5, level=False)
    assert radio.gesture.state == IDLE
    assert radio.states(3.2, level=True)[-1] == SHUTTING_DOWN


def test_a_release_and_a_new_press_restart_the_hold_from_zero(radio):
    """Two 2 s presses are not one 4 s press."""
    radio.run(2.0, level=True)
    radio.run(2.5, level=False)
    assert radio.run(2.0, level=True).state == CONFIRMING


# ── Silence ─────────────────────────────────────────────────────────────

def test_silence_counts_as_a_release(radio):
    """The radio going away must not carry a shutdown to completion."""
    radio.run(1.5, level=True)
    assert radio.gesture.state == ARMING
    result = radio.run(1.1, level=True, publish_button=False)
    assert result.state == ABORTED
    assert result.reason == REASON_RELEASED


def test_silence_shorter_than_the_timeout_does_not_abort(radio):
    """
    A dropped message is a stutter, not a release.

    The hold goes on running through it -- the level is latched, and the
    latch has not expired -- so the state advances exactly as it would have
    with the messages present.
    """
    radio.run(1.2, level=True)
    assert radio.gesture.state == ARMING
    assert radio.run(0.5, level=True, publish_button=False).state != ABORTED


def test_a_steady_hold_is_never_read_as_a_release(radio):
    """
    The level publisher, asserted as a whole gesture.

    The measured topic repeats the SAME level at 20 Hz rather than publishing
    only on change; a machine that treated a repeat as "nothing happened"
    would time the hold out at button_timeout_s and never fire.
    """
    assert radio.run(3.1, level=True).state == SHUTTING_DOWN


def test_a_press_after_the_silence_starts_a_fresh_gesture(radio):
    """The press that expired is over; the next message begins a new one."""
    radio.run(2.0, level=True)
    radio.run(1.2, level=True, publish_button=False)
    assert radio.gesture.state == ABORTED
    radio.run(2.0, level=False)
    assert radio.run(2.0, level=True).state == CONFIRMING


# ── The standstill guard ────────────────────────────────────────────────

def test_a_request_while_the_robot_moves_is_refused(radio):
    """Powering off a moving machine is not something a button should do."""
    result = radio.run(1.5, level=True, standstill_reason=REASON_MOVING)
    assert result.state == ABORTED
    assert result.reason == REASON_MOVING


def test_a_refusal_cannot_be_re_armed_by_keeping_the_button_held(radio):
    """The rule that stops a shutdown firing the instant the tracks settle."""
    radio.run(1.5, level=True, standstill_reason=REASON_MOVING)
    assert radio.gesture.state == ABORTED
    # The robot stops, and the operator never let go.
    result = radio.run(5.0, level=True)
    assert result.state == IDLE
    assert result.reason == ''


def test_a_refusal_is_cleared_by_releasing_and_pressing_again(radio):
    """The only way back: let go, press again."""
    radio.run(1.5, level=True, standstill_reason=REASON_MOVING)
    radio.run(5.0, level=True)
    radio.run(2.5, level=False)
    assert radio.states(3.2, level=True)[-1] == SHUTTING_DOWN


def test_the_guard_is_read_only_at_the_arming_instant(radio):
    """
    A robot that starts moving after arming does not cancel the countdown.

    It cannot: the shutdown is what stops it, the Roboteq watchdog brakes on
    the cmd_vel timeout, and a machine that refused halfway would leave the
    operator holding a button that does nothing.
    """
    radio.run(1.5, level=True)
    assert radio.gesture.state == ARMING
    assert radio.run(1.7, level=True, standstill_reason=REASON_MOVING).state \
        == SHUTTING_DOWN


# ── Terminal states ─────────────────────────────────────────────────────

def test_shutting_down_is_terminal(radio):
    """Once the halt has been issued, nothing a button does matters."""
    radio.run(3.1, level=True)
    assert radio.gesture.state == SHUTTING_DOWN
    assert radio.run(5.0, level=False).state == SHUTTING_DOWN


def test_in_dry_run_a_release_returns_to_idle(radio):
    """What makes a live rehearsal possible: let go and the node is idle."""
    radio.run(3.1, level=True, dry_run=True)
    assert radio.gesture.state == SHUTTING_DOWN
    assert radio.run(0.2, level=False, dry_run=True).state == IDLE


def test_a_forced_abort_reports_the_reason_and_latches(radio):
    """A halt that did not run must not leave the robot claiming otherwise."""
    radio.run(3.1, level=True)
    radio.gesture.force_abort(radio.now, 'shutdown command failed')
    result = radio.run(5.0, level=True)
    assert radio.gesture.state != SHUTTING_DOWN
    assert result.state == IDLE
    radio.run(0.2, level=True)
    assert radio.gesture.state == IDLE


# ── Time going backwards ────────────────────────────────────────────────

def test_time_going_backwards_re_anchors_instead_of_firing():
    """A clock step must never be read as "held long enough"."""
    machine = HoldGesture(CONFIG)
    for step in range(40):                       # 2.0 s of hold at 20 Hz
        machine.button(True, 1000.0 + step * BUTTON_PERIOD_S)
        machine.update(1000.0 + step * BUTTON_PERIOD_S)
    assert machine.state == CONFIRMING
    # The clock steps back an hour, then the hold continues.
    machine.button(True, 1000.0 - 3600.0)
    result = machine.update(1000.0 - 3600.0)
    assert result.state != SHUTTING_DOWN


def test_after_a_backwards_step_the_gesture_still_completes():
    """Re-anchoring restarts the hold; it does not break the machine."""
    machine = HoldGesture(CONFIG)
    machine.button(True, 500.0)
    machine.update(500.0)
    machine.button(True, 100.0)
    machine.update(100.0)
    for step in range(1, 80):
        now = 100.0 + step * BUTTON_PERIOD_S
        machine.button(True, now)
        result = machine.update(now)
    assert result.state == SHUTTING_DOWN


def test_a_backwards_step_does_not_abort_a_pending_request():
    """The abort must come from the operator, not from the clock."""
    machine = HoldGesture(CONFIG)
    for step in range(30):
        now = 1000.0 + step * BUTTON_PERIOD_S
        machine.button(True, now)
        machine.update(now)
    assert machine.state == ARMING
    machine.button(True, 900.0)
    assert machine.update(900.0).state == ARMING


# ── The configuration ───────────────────────────────────────────────────

def test_the_config_reports_the_total_gesture():
    """Check arm + confirm is the number the operator was given: 3.0 s."""
    assert CONFIG.total_hold_s == pytest.approx(3.0)
    assert CONFIG.confirming_after_s == pytest.approx(1.5)


def test_an_acknowledgement_longer_than_the_window_never_swallows_the_halt():
    """
    Even mis-configured, the total gesture is what fires the halt.

    params.py refuses such a pair and falls back, but the machine itself must
    not depend on that: a confirming stage that never starts still has to end
    in a halt at arm + confirm.
    """
    machine = Radio(GestureConfig(arm_hold_s=1.0, confirm_hold_s=2.0,
                                  arming_ack_s=99.0))
    assert machine.states(3.2, level=True) == [IDLE, ARMING, SHUTTING_DOWN]
