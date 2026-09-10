#!/usr/bin/env python3
# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: the ROS-free state machine that turns a held radio button into a
# shutdown request.

"""
The ROS-free state machine that turns a held radio button into a shutdown request.

Nothing in this module imports rclpy, opens a subscription or runs a command.
It is fed three things -- the button LEVEL, the current time, and whether the
robot is provably standing still -- and answers with a state, a countdown and
a refusal reason. That is deliberate: powering a 350 kg machine off is the one
decision in this package that has to be exhaustively testable against a fake
clock, and it cannot be if it is entangled with an executor.

What the button actually is
---------------------------
Measured on the robot 2026-09-10. `mtp_autec_radio/scripts/joy_to_cmd_vel_node.py`
publishes every mapped button on EVERY `/radio/joy` message
(`_handle_buttons` loops over all publishers unconditionally), so the topic
carries a LEVEL at the joy rate -- 20.0 Hz measured, steady `false` with
nobody touching the radio -- and not an edge on change. A held button is
therefore a stream of `true` at 20 Hz, which is what this machine is built
for.

Because it is a level and not an edge, the machine may not treat "no message"
as "still pressed": if the radio link drops mid-gesture the last thing heard
would be `true` for ever. So the last level is latched and expires:
`button_timeout_s` (1.0 s = 20 missed messages at the measured rate) of
silence counts as a RELEASE, which aborts a pending request. The consequence
is stated plainly because it is a real trade-off: a radio that stutters for
more than a second during the gesture costs the operator the gesture, and he
has to press again. Losing a shutdown is cheap; continuing a shutdown for a
radio that is no longer talking to us is not.

Should the button topic ever be re-mapped to a publisher that only sends on
change, this latch would hold a steady press correctly but the timeout would
have to be turned off (`button_timeout_s` covering the whole gesture), and the
release edge would become the only thing that aborts. That is not the topic
this node ships against; `README.md` says so under "The button".

The gesture
-----------
Continuous hold, one uninterrupted press:

    press ──arm_hold_s──> ARMING ──arming_ack_s──> CONFIRMING ──> SHUTTING_DOWN
                                                   (rest of confirm_hold_s)

`ARMING` and `CONFIRMING` together are exactly `confirm_hold_s` long, so the
whole gesture is `arm_hold_s + confirm_hold_s` from the moment the button goes
down. Releasing anywhere before the end aborts with reason `released`.

Two rules that are not obvious and are tested one by one:

* A request made while the robot is moving (or while the odometry is missing
  or stale) is REFUSED at the arming instant, and the refusal latches: the
  operator cannot re-arm by simply keeping the button down until the robot
  happens to stop. He has to let go and press again. A machine that decides
  to power off the instant its wheels settle is not something a button should
  do.
* Time going backwards -- a clock step, a bag replay, a node whose clock
  source changed -- re-anchors every timer instead of firing them. A negative
  elapsed time must never be read as "held long enough".
"""

from dataclasses import dataclass
from typing import Optional

#: The five states of the `/power/state` contract. Frozen: `mtp_carrier_io`
#: switches its LED and buzzer pattern on these exact strings, and the HMI
#: shows them.
IDLE = 'idle'
ARMING = 'arming'
CONFIRMING = 'confirming'
SHUTTING_DOWN = 'shutting_down'
ABORTED = 'aborted'

#: States in which the countdown is running and letting go still stops it.
PENDING_STATES = (ARMING, CONFIRMING)

#: Refusal reasons. Prose, because they are shown to a human: the HMI prints
#: them and they go in the log next to the state. Frozen for the same reason
#: the states are.
REASON_RELEASED = 'released'
REASON_MOVING = 'robot is moving'
REASON_ODOM_STALE = 'odometry is stale'
REASON_ODOM_MISSING = 'no odometry'

#: `seconds_left` is reported to one decimal. The HMI and the log both show it
#: as a countdown for a human; more digits would only flicker.
COUNTDOWN_DECIMALS = 1


@dataclass(frozen=True)
class GestureConfig:
    """How long each stage of the hold lasts, in seconds."""

    arm_hold_s: float = 1.0
    confirm_hold_s: float = 2.0
    arming_ack_s: float = 0.5
    button_timeout_s: float = 1.0
    aborted_hold_s: float = 2.0

    @property
    def total_hold_s(self) -> float:
        """Seconds of continuous hold between the press and the halt."""
        return self.arm_hold_s + self.confirm_hold_s

    @property
    def confirming_after_s(self) -> float:
        """Seconds of continuous hold after which the countdown starts."""
        return self.arm_hold_s + min(self.arming_ack_s, self.confirm_hold_s)


@dataclass(frozen=True)
class Gesture:
    """What the machine says about the request right now."""

    state: str
    seconds_left: float
    reason: str
    changed: bool

    @property
    def is_pending(self) -> bool:
        """Whether a request is running and a release would still stop it."""
        return self.state in PENDING_STATES


class HoldGesture:
    """
    Track one button level over time and decide when the robot may halt.

    The owner feeds `button()` from the subscription and calls `update()` on a
    timer; `update()` is what advances time, so a machine nobody ticks never
    fires. `update()` takes the standstill verdict as a plain reason string
    (None meaning "provably standing still"), which keeps this module free of
    both ROS and odometry arithmetic.
    """

    def __init__(self, config: Optional[GestureConfig] = None):
        self._config = config or GestureConfig()
        self._state = IDLE
        self._reason = ''
        self._seconds_left = 0.0
        # Last level heard, and when. `None` for the time means "nothing has
        # ever been heard", which is not the same as a stale `false`.
        self._level = False
        self._heard_at: Optional[float] = None
        # When the current uninterrupted press started, or None while released.
        self._pressed_since: Optional[float] = None
        # When the machine entered ABORTED, so it can go back to idle.
        self._aborted_at = 0.0
        # Set on every refusal and on every abort: blocks re-arming until the
        # button has actually been released. Without it, an operator holding
        # the button through a "robot is moving" refusal would arm the moment
        # the tracks settled, which is precisely the surprise this forbids.
        self._blocked = False

    @property
    def config(self) -> GestureConfig:
        """Return the timings this machine was built with."""
        return self._config

    @property
    def state(self) -> str:
        """Return the current state, without advancing time."""
        return self._state

    # ── Inputs ──────────────────────────────────────────────────────────

    def button(self, level: bool, now: float) -> None:
        """
        Record one button message: its level and the moment it arrived.

        Every message counts, not only the ones that change the level: the
        stream itself is the evidence that the radio is still talking, and
        `update()` uses its absence to declare a release.
        """
        self._rewind_if_time_went_backwards(now)
        level = bool(level)
        if level and not self._pressed_is_live(now):
            # A press starts here: either the level just went true, or the
            # previous press already expired into a release.
            self._pressed_since = now
        if not level:
            self._pressed_since = None
        self._level = level
        self._heard_at = now

    # ── Advance ─────────────────────────────────────────────────────────

    def update(self, now: float, standstill_reason: Optional[str] = None,
               dry_run: bool = False) -> Gesture:
        """
        Advance the machine to `now` and return what it says.

        `standstill_reason` is None when the robot is provably standing still,
        and otherwise the sentence explaining why it is not -- moving, stale
        odometry, no odometry at all. It is read only at the arming instant:
        the guard decides whether a request may START, and a robot that cannot
        move once its stack is going down does not need it re-checked.

        `dry_run` changes exactly one thing: the halt is not real, so
        `shutting_down` is not terminal and a release returns the machine to
        idle. That is what makes a live rehearsal possible -- the operator
        holds the button, sees and hears the whole sequence, lets go, and the
        robot is still on.
        """
        self._rewind_if_time_went_backwards(now)
        previous = (self._state, self._reason)
        held_for = self._held_for(now)
        if held_for is None:
            self._released(now, dry_run)
        else:
            self._held(now, held_for, standstill_reason, dry_run)
        self._seconds_left = self._countdown(held_for)
        return Gesture(
            state=self._state,
            seconds_left=self._seconds_left,
            reason=self._reason,
            changed=(self._state, self._reason) != previous,
        )

    # ── The two halves of update() ──────────────────────────────────────

    def _released(self, now: float, dry_run: bool) -> None:
        """Advance the machine while the button is up (or has gone silent)."""
        self._pressed_since = None
        # The release is what clears a refusal: this is the only way back to a
        # state from which a new request can be armed.
        self._blocked = False
        if self._state in PENDING_STATES:
            self._abort(now, REASON_RELEASED)
        elif self._state == SHUTTING_DOWN and dry_run:
            self._to_idle()
        elif self._state == ABORTED and now - self._aborted_at >= self._config.aborted_hold_s:
            self._to_idle()

    def _held(self, now: float, held_for: float,
              standstill_reason: Optional[str], dry_run: bool) -> None:
        """Advance the machine while the button is down."""
        if self._state == SHUTTING_DOWN:
            # Terminal for this request: the halt has been issued and there is
            # nothing a button can do about it any more.
            return
        if self._state == ABORTED:
            # The abort still shows for its full `aborted_hold_s`, then falls
            # back to idle -- but `_blocked` keeps it there until the release.
            if now - self._aborted_at >= self._config.aborted_hold_s:
                self._to_idle()
            return
        if self._state == IDLE:
            if self._blocked or held_for < self._config.arm_hold_s:
                return
            if standstill_reason is not None:
                self._abort(now, standstill_reason)
                return
            self._state, self._reason = ARMING, ''
        if held_for >= self._config.total_hold_s:
            self._state, self._reason = SHUTTING_DOWN, ''
        elif held_for >= self._config.confirming_after_s:
            self._state, self._reason = CONFIRMING, ''

    def force_abort(self, now: float, reason: str) -> None:
        """
        Abort the request from outside, with a reason the machine cannot know.

        There is exactly one caller: the node, when the halt it just issued
        came back refused (no sudoers rule) or failed. `shutting_down` is
        terminal for a request that WORKED; a request that did not work must
        not leave the robot claiming it is going down, so the owner pushes it
        into `aborted` with the reason the operator has to read. The refusal
        latches like every other abort: he has to let go and press again.
        """
        self._rewind_if_time_went_backwards(now)
        self._abort(now, reason)

    # ── Small helpers ───────────────────────────────────────────────────

    def _abort(self, now: float, reason: str) -> None:
        """Refuse or cancel the request, and latch it until the release."""
        self._state, self._reason = ABORTED, reason
        self._aborted_at = now
        self._blocked = True

    def _to_idle(self) -> None:
        """Return to idle, with nothing pending and nothing to explain."""
        self._state, self._reason = IDLE, ''

    def _pressed_is_live(self, now: float) -> bool:
        """Whether a press is currently running and has not gone silent."""
        return self._pressed_since is not None and not self._is_silent(now)

    def _is_silent(self, now: float) -> bool:
        """Whether the button topic has gone quiet for longer than allowed."""
        if self._heard_at is None:
            return True
        return now - self._heard_at > self._config.button_timeout_s

    def _held_for(self, now: float) -> Optional[float]:
        """Seconds the button has been continuously down, or None if it is up."""
        if not self._level or not self._pressed_is_live(now):
            return None
        return now - self._pressed_since

    def _countdown(self, held_for: Optional[float]) -> float:
        """Seconds left before the halt; 0.0 outside `arming`/`confirming`."""
        if self._state not in PENDING_STATES or held_for is None:
            return 0.0
        return round(max(0.0, self._config.total_hold_s - held_for), COUNTDOWN_DECIMALS)

    def _rewind_if_time_went_backwards(self, now: float) -> None:
        """
        Re-anchor every timer when the clock steps back instead of firing them.

        A negative elapsed time would compare as "not held long enough" here
        and as "held long enough" nowhere -- but the abort and the silence
        tests would both misfire, and a request would be cancelled by a clock
        step rather than by the operator. Re-anchoring turns a backwards jump
        into "the gesture starts again from here", which is the only reading
        that cannot power the robot off by accident.
        """
        anchors = [t for t in (self._heard_at, self._pressed_since, self._aborted_at)
                   if t is not None]
        if not anchors or now >= max(anchors):
            return
        if self._heard_at is not None:
            self._heard_at = now
        if self._pressed_since is not None:
            self._pressed_since = now
        self._aborted_at = min(self._aborted_at, now)
