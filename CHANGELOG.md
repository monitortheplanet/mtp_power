<!--
  Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
  Proprietary and confidential. Unauthorized copying, distribution or use
  of this file, via any medium, is strictly prohibited.
  Purpose: human-written history of this repository.
-->

# Changelog

All notable changes to this repository are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and this repository
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Power the robot off from the radio.** The operator asked for it on
  2026-09-10: cut the machine without a laptop. A continuous 3.0 s hold on one
  radio button (1.0 s to arm, 2.0 s more to confirm, letting go cancels) runs
  `sudo -n /sbin/shutdown -h now`. The gesture is the operator's own choice.

- `mtp_power/hold_gesture.py`, ROS-free: the state machine over (button level,
  now, standstill) -> state, countdown, reason. It is built around what the
  button MEASURABLY is rather than what the source suggests: every radio
  button topic carries a LEVEL at 20 Hz whether pressed or not (2402 messages
  in 120 s on each of the ten, measured 2026-09-10), because the teleop node
  republishes all of them on every `/radio/joy` message. So a hold is a stream
  of `true`, a release a stream of `false`, and SILENCE means the radio is
  gone -- which `button_timeout_s` (1.0 s = 20 missed messages) treats as a
  release, aborting a pending request. Continuing a shutdown on behalf of a
  radio that has stopped talking to us is the failure mode worth designing
  against; the cost is that a link stuttering for over a second mid-gesture
  costs the operator the gesture.

- `mtp_power/standstill.py`, ROS-free: the guard that refuses to arm unless
  the WHEEL odometry (`/motors/odom`, 10.0 Hz measured) proves the machine has
  been still. Not `/odometry/filtered`: the EKF fuses an IMU and a GNSS that
  can hold a fused velocity near zero while a track is turning. Missing or
  stale odometry REFUSES rather than assumes -- a robot whose motion is
  unknown is not a robot that is stopped -- and a gap in the stream restarts
  the still window instead of being stitched over.

- The refusal LATCHES. A request refused while the robot moves cannot be
  re-armed by keeping the button down until the tracks settle; the operator
  has to let go and press again. A machine that powers off the instant its
  wheels stop is the surprise this rule exists to prevent.

- `/power/state` (`std_msgs/String` JSON, RELIABLE + TRANSIENT_LOCAL depth 1,
  on every change and at least every 1.0 s): the shared contract with
  `mtp_carrier_io`, which turns it into the buzzer and the LED, and with the
  HMI bridge. `shutting_down` is PUBLISHED BEFORE the halt is issued, because
  a human standing next to 350 kg has to hear it while the ROS graph is still
  alive to deliver it.

- `mtp_power/halt.py`, ROS-free: the privileged call, the same argv, timeout
  and failure classification as `mtp_hmi_bridge/mtp_hmi_bridge/power.py`
  (ROB-31), the reference implementation. No new privilege: the sudoers rule
  `/etc/sudoers.d/91-hmi-power` is already installed. It never publishes a
  cmd_vel and never touches the mux -- the Roboteq watchdog brakes on its own
  when the stack goes away. A halt that does NOT run pushes the state back to
  `aborted` with the reason, rather than leaving the robot claiming it is
  going down.

- `dry_run` (default false): the whole gesture, every state, the buzzer and
  the LED -- and a log line where the halt would be. In dry run
  `shutting_down` is not terminal and a release returns the node to idle, so
  the gesture can be rehearsed on the real robot as often as one likes with
  the machine still on.

- `launch/power_manager.launch.py` with argument `power_manager_config` (not
  `config_file`: `IncludeLaunchDescription` does not scope launch
  configurations, and `mecgeos_bringup` declares that name at top level),
  `config/power_manager.yaml` with a WHY comment on every value, and the
  bringup include behind `features.power.enabled`.

- 140 pytest cases across ten files, four gates green (`ament_flake8`,
  `ament_pep257`, the proprietary header check, and the parameter/launch
  binding). `test/conftest.py` forces `ROS_DOMAIN_ID=77`, and no test in this
  package ever reaches the real halt: every one passes a fake runner and
  asserts on argv.

### Notes

- `button_topic` is a plain parameter and nothing in the package -- code,
  tests or docs -- assumes which button it is. The shipped default is
  `/radio/button/robot/fn1`, the operator's physical fn1, which has no other
  function on his panel. The robot button topic names were measured shifted by
  one against the transmitter's labels (physical fn1 is joy index 8, which was
  mapped to no topic at all); `mtp_autec_radio` is being corrected separately
  and this package neither touches it nor depends on that fix landing.

- `arming_ack_s` (0.5 s) splits the confirm window so that `arming` and
  `confirming` are both long enough to be recognised as the distinct
  LED/buzzer patterns `mtp_carrier_io` gives them. It is taken out of the
  confirm window, not added to it: the total gesture stays 3.0 s.
