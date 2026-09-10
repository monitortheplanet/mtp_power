#!/usr/bin/env python3
# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: the ROS-free guard that answers whether the robot is provably
# standing still.

"""
The ROS-free guard that answers whether the robot is provably standing still.

The witness is the WHEEL odometry, `/motors/odom` (measured 10.0 Hz on the
robot, 2026-09-10), and not the EKF output: the tracks are what actually move
the machine, and `/odometry/filtered` fuses in an IMU and a GNSS that can each
hold a fused velocity near zero while a track is turning, or drift a parked
robot to a few cm/s. When the question is "may this button power the robot
off", the honest witness is the one wired to the motors.

The guard is deliberately not a live-velocity test. It answers "still" only
when three things hold at once:

* a sample has arrived within `odom_timeout_s` -- a robot whose motion is
  UNKNOWN is not a robot that is stopped, and this refuses rather than
  assumes;
* every sample since `still_for_s` ago was under both thresholds;
* there was no gap in the stream during that window. A gap longer than
  `odom_timeout_s` hides whatever happened inside it, so the still window
  restarts from the far side of it.

The thresholds are what the parked machine actually reads, not zero: wheel
odometry of a stationary tracked robot never reports exactly 0.0.
"""

from typing import Optional

#: Reasons this guard hands back, matching `hold_gesture` so a refusal reads
#: the same wherever it is shown.
REASON_MOVING = 'robot is moving'
REASON_STALE = 'odometry is stale'
REASON_MISSING = 'no odometry'


class StandstillWatch:
    """
    Decide, from a stream of odometry samples, whether the robot is stopped.

    Fed by the odometry callback through `sample()` and asked by the gesture
    through `reason()`, which returns None for "provably standing still" and
    otherwise the sentence explaining the refusal.
    """

    def __init__(self, linear_mps: float = 0.02, angular_rps: float = 0.02,
                 still_for_s: float = 1.0, timeout_s: float = 1.0):
        self._linear_mps = abs(linear_mps)
        self._angular_rps = abs(angular_rps)
        self._still_for_s = max(0.0, still_for_s)
        self._timeout_s = max(0.0, timeout_s)
        self._last_at: Optional[float] = None
        self._still_since: Optional[float] = None
        self._last_speed = (0.0, 0.0)

    @property
    def last_speed(self):
        """Return the last (linear, angular) pair heard, for the log."""
        return self._last_speed

    def sample(self, now: float, linear: float, angular: float) -> None:
        """
        Record one odometry sample, taken at `now`.

        `now` is the arrival time on the node's own clock, not the message
        stamp: this guard is asking "is somebody still telling me the robot is
        stopped", and a publisher with a wrong or frozen stamp would otherwise
        look fresh for ever.
        """
        linear, angular = abs(float(linear)), abs(float(angular))
        self._last_speed = (linear, angular)
        moving = linear > self._linear_mps or angular > self._angular_rps
        gap = self._last_at is None or (now - self._last_at) > self._timeout_s
        backwards = self._last_at is not None and now < self._last_at
        if moving or gap or backwards:
            # Moving restarts the window for the obvious reason; a gap or a
            # clock step restarts it because the samples that would have
            # filled it are missing, and a window built on missing samples is
            # exactly the kind of proof this guard must not accept.
            self._still_since = None if moving else now
        elif self._still_since is None:
            self._still_since = now
        self._last_at = now

    def reason(self, now: float) -> Optional[str]:
        """Return None when the robot is provably still, else why it is not."""
        if self._last_at is None:
            return REASON_MISSING
        if now - self._last_at > self._timeout_s or now < self._last_at:
            return REASON_STALE
        if self._still_since is None:
            return REASON_MOVING
        if now - self._still_since < self._still_for_s:
            return REASON_MOVING
        return None
