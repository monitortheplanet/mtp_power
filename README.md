<!--
  Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
  Proprietary and confidential. Unauthorized copying, distribution or use
  of this file, via any medium, is strictly prohibited.
  Purpose: what this package is, how it is wired, and how it is verified.
-->

# mtp_power

Power the MECGEOS robot off from the radio, without a laptop, and tell the
rest of the machine it is happening.

A continuous hold on one radio button arms, confirms and then runs
`sudo -n /sbin/shutdown -h now`. Letting go at any point before the end
cancels. A request made while the robot is moving — or while nobody can prove
it is not — is refused with a reason. Every step is published on
`/power/state`, which `mtp_carrier_io` turns into the buzzer and the LED and
the HMI bridge forwards to the operator's screen.

## Purpose

The operator asked for two things on 2026-09-10. The first is this package:
power the robot off with a button on the Autec radio. The second — a beep and
the LED going dark at the very END of the shutdown, when the main switch can
safely be cut — is `mtp_ops`, because at that point the ROS stack is long
gone.

Powering a 350 kg tracked machine off is the one command in this stack that
cannot be undone remotely: once the Jetson is down, the only way back is
somebody walking to the robot. So it is deliberately not a single press. The
design mirrors the arm/confirm dance the HMI power path already uses
(`mtp_hmi_bridge/mtp_hmi_bridge/power.py`, ROB-31), translated from two
messages into one gesture a thumb can perform on a transmitter.

### The button

Measured on the robot 2026-09-10. `mtp_autec_radio`'s teleop node republishes
**every** mapped button on **every** `/radio/joy` message, so each button
topic carries a **level at 20 Hz** — a steady stream of `false` with nobody
touching the radio, a steady stream of `true` while one is held. It is not an
edge and not a change-only topic; 2402 messages arrived on each of the ten
topics in a 120 s window with nothing pressed.

Two consequences the node is built around:

* A steady hold is a repeated `true`, never a silence. A state machine that
  treated "no new message" as "nothing happened" would work; one that treated
  a repeat as "nothing happened" would time the gesture out and never fire.
* Silence therefore means the radio driver, or the radio, is **gone**.
  `button_timeout_s` (1.0 s = 20 missed messages) counts that as a RELEASE,
  which aborts a pending request. Continuing a shutdown on behalf of a radio
  that has stopped talking to us is the failure mode worth designing against.
  The cost is that a link stuttering for more than a second mid-gesture costs
  the operator the gesture; he presses again.

Which button is a **parameter and only a parameter** — `button_topic`,
`std_msgs/Bool`, nothing else in this package assumes anything about it.
Shipped default `/radio/button/robot/fn1`, the operator's physical fn1, which
has no other function on his panel (fn2 drives lights that are not installed,
fn3a/fn3b are the mux mode select, fn1 free). Note the topic names were
measured shifted by one against the transmitter's labels: physical fn1 is joy
index 8, which `BUTTON_TOPICS` did not map at all, and `mtp_autec_radio` is
being corrected separately so that index 8 becomes `/radio/button/robot/fn1`.
This package does not touch `mtp_autec_radio` and does not depend on that fix
landing — until it does, the topic simply carries the neighbouring button.

### The gesture

The operator's own choice: **3.0 s of continuous hold**, split 1.0 + 2.0.

```
press ──1.0 s──> arming ──0.5 s──> confirming ──1.5 s──> shutting_down → halt
       arm_hold_s        arming_ack_s     rest of confirm_hold_s
   └──────────────── release anywhere in here aborts ─────────────────┘
```

`arm_hold_s` 1.0 s is what makes a brush of the button — climbing on or off
the machine, a hand resting on the transmitter — not a shutdown: 20
consecutive `true` messages is not something a contact bounce produces.

`arming` and `confirming` are two states rather than one because
`mtp_carrier_io` gives them deliberately different patterns: a 5 Hz blink and
beep for "I heard you", 10 Hz for "let go and it stops". At 5 Hz one blink is
200 ms, so the acknowledgement has to last a couple of periods to read as a
pattern rather than a glitch. `arming_ack_s` (0.5 s) is taken **out of** the
confirm window, not added to it, so the whole gesture stays exactly 3.0 s.

### The standstill guard

The node refuses to arm unless `/motors/odom` — the **wheel** odometry, not
`/odometry/filtered` — has read under `still_linear_mps` and
`still_angular_rps` for `still_for_s`, and is fresher than `odom_timeout_s`.
The tracks are what move the machine; the EKF fuses in an IMU and a GNSS that
can hold a fused velocity near zero while a track is turning, or drift a
parked robot to a few cm/s.

Missing or stale odometry **refuses**, it does not assume. A robot whose
motion is unknown is not a robot that is stopped, and powering one off is not
something a button should do.

The refusal **latches**. The operator cannot re-arm by keeping the button
down until the tracks happen to settle; he has to let go and press again. A
machine that decides to power off the instant its wheels stop is exactly the
surprise this rule exists to prevent.

### What this package never does

It never publishes on a cmd_vel topic and never touches the mux. The motors
are not commanded to stop, because they stop on their own: the Roboteq
watchdog brakes on a cmd_vel timeout as soon as the ROS stack goes away, which
`power.py` documents. A zero Twist from here would be commanding motion, which
this stack forbids outright — `test_halt.py` asserts the code does not exist,
not merely that the branch is not taken.

## Nodes

### `power_manager`

Executable `power_manager`. Two subscriptions, one publisher, one timer.

| | |
| --- | --- |
| Subscribes | `button_topic` (`std_msgs/Bool`, RELIABLE, depth 10), `odom_topic` (`nav_msgs/Odometry`, RELIABLE, depth 5) |
| Publishes | `state_topic` (`std_msgs/String`, RELIABLE + TRANSIENT_LOCAL, depth 1) |
| Runs | `sudo -n /sbin/shutdown -h now`, once, at the end of a completed gesture |
| Never runs | anything else, and nothing at all when `dry_run` is true |

The decision lives in four ROS-free modules, each tested against a fake clock:
`hold_gesture.py` (the state machine), `standstill.py` (the guard),
`halt.py` (the privileged call) and `state_message.py` (the wire payload).
`power_manager_node.py` is wiring only.

**Order of operations at the end.** `shutting_down` is PUBLISHED FIRST and the
halt is issued afterwards. `mtp_carrier_io` turns the LED steady and the
buzzer continuous on that message, and a human standing next to the machine
has to hear it *before* the stack starts going down — once `shutdown -h now`
has run, the ROS graph has a few seconds to live and nothing is guaranteed to
be delivered. `test_power_manager_node.py` asserts that ordering directly.

**A halt that does not run** (no sudoers rule, `shutdown` failing) does not
leave the robot claiming it is going down: the state is pushed back to
`aborted` with the reason, latched like any other refusal. This refines the
contract's "`shutting_down` is terminal", which holds for a request that
worked.

## Topics

### `/power/state`

`std_msgs/String` carrying JSON, RELIABLE + TRANSIENT_LOCAL depth 1 (latched,
so a late subscriber — the HMI bridge reconnecting, `carrier_io` restarting
mid-gesture — sees the current state at once). Published on every change and
at least every `publish_period_s` (1.0 s).

```json
{"state": "idle|arming|confirming|shutting_down|aborted",
 "action": "shutdown",
 "source": "radio",
 "seconds_left": 2.4,
 "reason": "robot is moving",
 "seq": 17}
```

| Key | Meaning |
| --- | --- |
| `state` | `idle` nothing pending; `arming` the hold passed `arm_hold_s` and the request is acknowledged; `confirming` the countdown is running and releasing aborts; `shutting_down` the halt has been issued; `aborted` released early or refused |
| `action` | always `shutdown`. `reboot` is reserved by the contract and is deliberately not reachable from the radio |
| `source` | always `radio`. `hmi` is reserved for the bridge's own power path |
| `seconds_left` | time left before the halt, one decimal. `0.0` outside `arming`/`confirming` |
| `reason` | only with `aborted`, else `""` |
| `seq` | counts PUBLISHED MESSAGES, so a subscriber that sees it jump by more than one knows it missed something |

`aborted` returns to `idle` after `aborted_hold_s` — which does **not** re-open
the gesture: the refusal latch is cleared by the release, not by the timer.

Refusal reasons, frozen (they are shown to a human):

| Reason | When |
| --- | --- |
| `released` | the button went up, or the topic went silent for `button_timeout_s` |
| `robot is moving` | speed over a threshold, or not yet still for `still_for_s` |
| `odometry is stale` | no sample within `odom_timeout_s` |
| `no odometry` | nothing has ever arrived on `odom_topic` |
| `shutdown not permitted: the sudoers rule is missing (…)` | `sudo -n` refused |
| `shutdown command failed (…)` | anything else |

## Parameters

All read-only: the node turns them into subscriptions, a timer and one state
machine once, in its constructor. `ros2 param set` is refused rather than
accepted and ignored — an `arm_hold_s` changed at runtime would alter what the
node *reports* while the gesture in progress kept the old timing. To change
one, edit `config/power_manager.yaml` and restart.

Nothing is fatal: a value rclpy refuses against its descriptor, an empty topic
name, or an acknowledgement longer than the confirm window it lives in, is
logged and replaced by the default. A mistyped parameter should cost the
operator a default, not a bringup with no way to power the robot off.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `button_topic` | `/radio/button/robot/fn1` | The `std_msgs/Bool` the gesture listens to. Any button topic; nothing assumes which. |
| `odom_topic` | `/motors/odom` | The wheel odometry the standstill guard reads. |
| `state_topic` | `/power/state` | Where the JSON above is published. |
| `arm_hold_s` | `1.0` | Continuous hold before the request is acknowledged. |
| `confirm_hold_s` | `2.0` | Further hold before the halt. Total gesture 3.0 s. |
| `arming_ack_s` | `0.5` | How much of the confirm window `arming` takes before `confirming` starts. Inside the window. |
| `button_timeout_s` | `1.0` | Silence on the button topic that counts as a release (20 missed messages at 20 Hz). |
| `aborted_hold_s` | `2.0` | How long `aborted` is shown before returning to `idle`. |
| `still_linear_mps` / `still_angular_rps` | `0.02` / `0.02` | At or below these the robot counts as parked. |
| `still_for_s` | `1.0` | How long it must have been parked before a request may arm. |
| `odom_timeout_s` | `1.0` | Age past which the odometry is stale and the request is refused. |
| `tick_period_s` | `0.05` | How often the state machine is advanced (the button's own rate). |
| `publish_period_s` | `1.0` | Heartbeat republication; every change is published immediately regardless. |
| `dry_run` | `false` | Run the whole gesture and publish every state, but log instead of halting. |

WHAT each one means is in `mtp_power/params.py` and comes back out of
`ros2 param describe /power_manager <name>`. WHY this robot runs these
particular values is in `config/power_manager.yaml`, next to the values.

### `dry_run`

`true` runs the entire gesture, publishes every state, drives the buzzer and
the LED through the whole sequence — and logs a line where the halt would be.
In dry run `shutting_down` is **not** terminal: releasing the button returns
the node to `idle`, so the gesture can be rehearsed as often as one likes with
the robot still on. That is deliberate and is the only behavioural difference.

## Launch

### `power_manager.launch.py`

```bash
ros2 launch mtp_power power_manager.launch.py
ros2 launch mtp_power power_manager.launch.py power_manager_config:=/path/to/other.yaml
```

Starts one node named `power_manager` — which is also the root key of
`config/power_manager.yaml`, and ROS 2 drops a `ros__parameters` block whose
key matches no node without a word in the log. For this node the resulting
default set includes the button topic and `dry_run: false`, so a mismatch is
not a cosmetic failure. The configuration argument is `power_manager_config`,
**not** `config_file`: `mecgeos_bringup` declares a top-level `config_file` and
`IncludeLaunchDescription` does not scope launch configurations, so a shared
name is inherited in one direction and leaks in the other.
`test/test_param_binding.py` exists to catch both.

From the robot bringup it is behind the `features.power.enabled` flag in
`mecgeos_bringup/config/robot_config.yaml` (`true`). It needs `mtp_autec_radio`
(for the button) and the motor stack (for `/motors/odom`); without odometry the
node runs and refuses every request, saying so.

## Tests

```bash
python3 -m pytest test/ -q
cd /home/nvidia/ros2_ws
colcon build --symlink-install --packages-select mtp_power mecgeos_bringup
colcon test --packages-select mtp_power --event-handlers console_direct+
colcon test-result --verbose
```

| Test | Covers |
| --- | --- |
| `test/test_hold_gesture.py` | the whole state machine on a fake clock: a hold shorter than `arm_hold_s`, the stage boundaries, 2.95 s not firing, release during `arming` and during `confirming`, an abort returning to idle, a second press after an abort, two short presses not adding up, silence counting as a release, a short stutter not aborting, a steady 20 Hz hold never read as silence, a refusal while moving, that refusal not re-arming while held, the guard read only at the arming instant, `shutting_down` terminal, dry run returning to idle, a forced abort, and a clock stepping backwards re-anchoring instead of firing |
| `test/test_standstill.py` | the guard, mostly its refusals: nothing heard, not still long enough, moving, turning, parked-robot noise not counting as motion, one moving sample costing the whole window, stale odometry, a gap in the stream restarting the window, a backwards clock, reversing, and `still_for_s: 0` still needing a fresh sample |
| `test/test_halt.py` | the privileged call as a CONTRACT with `mtp_hmi_bridge/power.py`: exact argv, `-n` never prompting, the 10 s timeout, no reboot reachable, exactly one command, every `sudo` denial fragment classified as "not permitted", other failures not blamed on sudoers, a raising runner not escaping — and that the module contains no cmd_vel or Twist at all. Every test passes a fake runner; nothing here reaches the real command |
| `test/test_state_message.py` | the `/power/state` payload as the wire contract with `mtp_carrier_io` and the HMI: all six keys always present, the five state strings spelled out literally, `action`/`source` pinned, `seconds_left` zeroed outside the pending states and `reason` outside `aborted`, JSON parseable, no padding |
| `test/test_power_manager_node.py` | the wiring: the latched initial `idle`, the state QoS, the two subscriptions from parameters, that the node publishes NOTHING but the state topic, the 1 Hz heartbeat, a growing `seq`, the full gesture reaching the halt, **`shutting_down` published before the halt runs**, the halt running exactly once, release/moving/stale/missing/silence all refusing without halting, a brush doing nothing, dry run through every state and repeatable, a refused halt becoming an `aborted` that cannot be retried while held, a malformed message not killing the node — plus two tests that go through DDS for real (a late subscriber receiving the latched state, and the QoS matching a RELIABLE publisher) |
| `test/test_params.py` | the parameter contract: every parameter declared once, a field of the snapshot, read-only, described, bounded, its default inside its own range; the shipped YAML setting exactly the parameters that exist and surviving its own validation; `dry_run` shipped false; and every bad override falling back instead of killing the node |
| `test/test_param_binding.py` | the node name is the parameter-file root key, the launch file declares `power_manager_config` and not `config_file`, it starts that node and nothing else with no remaps and no inline parameters, the argument actually reaches the node, and the launch file still builds under `ros2 launch --show-args` |
| `test/test_header.py` | every source file carries the proprietary header, and the checker rejects a file without it |
| `test/test_flake8.py` | `ament_flake8`, 99 columns |
| `test/test_pep257.py` | `ament_pep257`, docstrings (D213 house convention) |

140 pytest cases as of 2026-09-10, all green.

`test/conftest.py` forces `ROS_DOMAIN_ID=77` for the whole package. The robot
exports `ROS_DOMAIN_ID=0` from `~/.bashrc`, and `/power/state` is the topic
that drives the real buzzer and the real LED of a machine somebody may be
standing next to. This is the one package in the stack where getting that
wrong is not merely noisy.

No test in this package ever calls the real halt: every one passes a fake
runner and asserts on argv.

**Not covered here:** the gesture performed by a human on the real
transmitter, the interaction with `mtp_carrier_io`'s buzzer and LED as
processes, and an actual power-off. Those are verified on the robot — see
below — and written down in the pull request.

## Verifying it live

### The rehearsal (`dry_run`, no power-off)

Copy the shipped config, flip one value, and run a **second** node on the live
domain with a name that does not clash with the bringup's:

```bash
sed 's/^    dry_run : false/    dry_run : true/' \
  /home/nvidia/ros2_ws/install/mtp_power/share/mtp_power/config/power_manager.yaml \
  > /tmp/power_manager_dry_run.yaml
grep dry_run /tmp/power_manager_dry_run.yaml       # must read: dry_run : true
```

Then, on domain 0, with the operator holding the real button:

```bash
source /home/nvidia/ros2_ws/install/setup.bash
ros2 launch mtp_power power_manager.launch.py \
  power_manager_config:=/tmp/power_manager_dry_run.yaml
```

and in a second terminal:

```bash
ros2 topic echo /power/state --field data
```

Expected: `idle` once a second; on the hold, `arming` at 1.0 s, `confirming`
at 1.5 s with `seconds_left` counting down, `shutting_down` at 3.0 s, then a
log line `Power: DRY RUN, not running the halt`; on release, back to `idle`.
Releasing early gives `aborted` with `"reason":"released"`. The robot stays
on throughout.

### Read-only checks after the bringup restart

```bash
ros2 node list | grep power_manager
ros2 param get /power_manager button_topic
ros2 param get /power_manager dry_run                     # must be False
ros2 topic echo /power/state --once --field data          # latched: answers at once
ros2 topic hz /power/state                                # ~1 Hz idle heartbeat
ros2 topic info /power/state --verbose | grep -i durability   # TRANSIENT_LOCAL
ros2 topic hz $(ros2 param get /power_manager button_topic | awk '{print $NF}')
sudo -n /sbin/shutdown --help >/dev/null && echo 'sudoers rule present'
```

## Troubleshooting

| Symptom | Cause to check first |
| --- | --- |
| Nothing happens when the button is held | Wrong topic. `ros2 param get /power_manager button_topic`, then `ros2 topic echo` that topic while pressing. Remember the fn-name shift. |
| Every request is refused with `no odometry` | `/motors/odom` is not publishing: the motor bridge is down. `ros2 topic hz /motors/odom`. |
| Every request is refused with `odometry is stale` | The publisher stopped mid-session, or the node's clock jumped. Same check. |
| Refused with `robot is moving` on a parked robot | The wheel odometry reads above 0.02. `ros2 topic echo /motors/odom --field twist.twist`. Raise `still_linear_mps`/`still_angular_rps` only if the numbers justify it. |
| It refuses, the robot stops, and holding on does nothing | By design: the refusal latches. Let go and press again. |
| `aborted` with `released` although the button was held | The button topic went silent for over `button_timeout_s` — the radio link, not the thumb. `ros2 topic hz` that topic during the hold. |
| `aborted` with `sudoers rule is missing` | `/etc/sudoers.d/91-hmi-power` is gone or was edited. Re-render it from `mtp_ops/security/sudoers-hmi-power.in`. |
| The buzzer and the LED do nothing | That is `mtp_carrier_io`, not this node. Check `/power/state` is publishing first. |
| The state topic is silent | The node is not running: `ros2 node list`, and `features.power.enabled` in `robot_config.yaml`. |

## License

Proprietary. Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
See [LICENSE](LICENSE).
