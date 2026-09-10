#!/usr/bin/env python3
# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: every ROS parameter of the power manager node, declared and
# validated in one place.

"""
Every ROS parameter of the power manager node, declared and validated in one place.

`PowerManagerParams` is both the declaration list and the validated snapshot:
a key that is not a field here does not exist. The node reads the frozen
instance instead of calling `get_parameter`, so no callback can invent a
parameter of its own, and the whole path is testable against a stub node.

Every parameter is declared read-only. The node turns this snapshot into two
subscriptions, one publisher, one timer and one state machine once, in its
constructor, and never reads a parameter again. A `ros2 param set arm_hold_s`
at runtime would change what the node *reports* while the gesture already in
progress kept the old timing -- read-only turns that silent lie into a
refusal the operator can see. What this robot runs, and why, is in
`config/power_manager.yaml`.

Nothing here is ever fatal. A bad value is logged and replaced by the default:
an operator who mistyped a threshold should get the shipped gesture and an
error in the log, not a bringup with the shutdown button missing. That covers
what rclpy rejects against the descriptors (wrong type, out of range) and what
is checked in Python: an empty topic name, and an acknowledgement stage longer
than the confirm window it has to fit inside.

`button_topic` is a PLAIN parameter and nothing in this package assumes which
button it is. The shipped default is the operator's chosen button, and
changing it is a one-line edit to `config/power_manager.yaml` and a restart --
no code, no test and no document is written around a particular one.
"""

from dataclasses import dataclass
from typing import Any, Iterator, Optional, Tuple

from rcl_interfaces.msg import (
    FloatingPointRange,
    ParameterDescriptor,
    ParameterType,
)

from rclpy.exceptions import InvalidParameterTypeException, InvalidParameterValueException

#: The button the gesture listens to: the operator's physical fn1, which has
#: no other function on this machine. Every one of the radio button topics
#: behaves identically (std_msgs/Bool, level at 20 Hz, measured 2026-09-10),
#: so this is a parameter and only a parameter -- see config/power_manager.yaml
#: for the topic-name shift this default depends on.
DEFAULT_BUTTON_TOPIC = '/radio/button/robot/fn1'

#: The wheel odometry, the honest witness of the tracks. Measured 10.0 Hz.
DEFAULT_ODOM_TOPIC = '/motors/odom'

#: The shared contract with mtp_carrier_io and the HMI bridge.
DEFAULT_STATE_TOPIC = '/power/state'

#: The operator's gesture: 3.0 s of continuous hold in total, 1.0 s to have
#: the request acknowledged and 2.0 s more to see it through.
DEFAULT_ARM_HOLD_S = 1.0
DEFAULT_CONFIRM_HOLD_S = 2.0
#: How much of the confirm window the `arming` acknowledgement takes before
#: the countdown starts. Inside the window, so the total gesture stays 3.0 s.
DEFAULT_ARMING_ACK_S = 0.5
#: Silence on the button topic that counts as a release: 1.0 s is 20 missed
#: messages at the measured 20 Hz.
DEFAULT_BUTTON_TIMEOUT_S = 1.0
#: How long an `aborted` is shown before the machine returns to idle.
DEFAULT_ABORTED_HOLD_S = 2.0

DEFAULT_STILL_LINEAR_MPS = 0.02
DEFAULT_STILL_ANGULAR_RPS = 0.02
DEFAULT_STILL_FOR_S = 1.0
DEFAULT_ODOM_TIMEOUT_S = 1.0

#: How often the machine is advanced, and how often the state is republished
#: even when nothing changed (the contract says at least every 1.0 s).
DEFAULT_TICK_PERIOD_S = 0.05
DEFAULT_PUBLISH_PERIOD_S = 1.0

#: Rehearsal mode: the whole gesture, the whole `/power/state` sequence, the
#: buzzer and the LED -- and a log line where the halt would be.
DEFAULT_DRY_RUN = False

#: Bounds rclpy enforces and `ros2 param describe` reports. Deliberately wide:
#: their job is to refuse a value that would break the node outright, not to
#: encode the tuning, which lives in the YAML next to its reasoning.
#: A stage of the hold. 0.1 s is a tap, 60 s outlasts anybody's patience.
HOLD_S = (0.1, 60.0)
#: The acknowledgement stage may be zero (straight into the countdown).
ACK_S = (0.0, 60.0)
#: How long an abort is shown. Below ~0.2 s nobody sees or hears it.
ABORTED_HOLD_S = (0.2, 60.0)
#: A speed at or below which the robot counts as parked. Zero is allowed and
#: means "never still", because wheel odometry never reads exactly zero: it is
#: the honest way to switch the button off without removing it.
STILL_LINEAR_MPS = (0.0, 1.0)
STILL_ANGULAR_RPS = (0.0, 5.0)
#: How long the robot must have been still, and how old a sample may be.
STILL_FOR_S = (0.0, 60.0)
TIMEOUT_S = (0.05, 60.0)
#: The two periods. The tick has to be well inside the shortest stage; 1 s is
#: the contract's own republication interval.
TICK_PERIOD_S = (0.01, 1.0)
PUBLISH_PERIOD_S = (0.05, 10.0)

_READ_ONCE = 'Read once at start-up: change it in the parameter file and restart.'
_TOPIC_CONSTRAINT = _READ_ONCE + ' A valid ROS topic name.'

#: (name, description, bounds) in declaration order. The default, and so the
#: type, comes from DEFAULTS below: one list, read by `declare`, by the tests
#: and by anyone asking what this node takes.
_SPECIFICATION: Tuple[Tuple[str, str, Optional[Tuple[float, float]]], ...] = (
    ('button_topic',
     'std_msgs/Bool the hold gesture listens to. Any button topic; the node '
     'makes no assumption about which one.', None),
    ('odom_topic',
     'nav_msgs/Odometry the standstill guard reads: the wheel odometry.', None),
    ('state_topic', 'std_msgs/String JSON this node publishes its state on.', None),
    ('arm_hold_s',
     'Continuous hold after which the request is acknowledged (state '
     '"arming").', HOLD_S),
    ('confirm_hold_s',
     'Further continuous hold, after arming, before the halt is issued.',
     HOLD_S),
    ('arming_ack_s',
     'How much of the confirm window the "arming" acknowledgement takes '
     'before the "confirming" countdown starts.', ACK_S),
    ('button_timeout_s',
     'Silence on the button topic that counts as a release.', TIMEOUT_S),
    ('aborted_hold_s',
     'How long the state stays "aborted" before returning to "idle".',
     ABORTED_HOLD_S),
    ('still_linear_mps',
     'Linear speed at or below which the robot counts as standing still.',
     STILL_LINEAR_MPS),
    ('still_angular_rps',
     'Angular speed at or below which the robot counts as standing still.',
     STILL_ANGULAR_RPS),
    ('still_for_s',
     'How long the robot must have been still before a request may arm.',
     STILL_FOR_S),
    ('odom_timeout_s',
     'Age past which the odometry is stale and a request is refused.',
     TIMEOUT_S),
    ('tick_period_s', 'Interval at which the gesture state machine is advanced.',
     TICK_PERIOD_S),
    ('publish_period_s',
     'Interval at which the state is republished even when unchanged.',
     PUBLISH_PERIOD_S),
    ('dry_run',
     'Run the whole gesture and publish every state, but log instead of '
     'halting. The live rehearsal.', None),
)

#: Name to default. Frozen contract with config/power_manager.yaml: a name in
#: one and not the other is a value the operator sets and nobody reads, or one
#: the node reads and nobody can set, and ROS 2 reports neither.
DEFAULTS = {
    'button_topic': DEFAULT_BUTTON_TOPIC,
    'odom_topic': DEFAULT_ODOM_TOPIC,
    'state_topic': DEFAULT_STATE_TOPIC,
    'arm_hold_s': DEFAULT_ARM_HOLD_S,
    'confirm_hold_s': DEFAULT_CONFIRM_HOLD_S,
    'arming_ack_s': DEFAULT_ARMING_ACK_S,
    'button_timeout_s': DEFAULT_BUTTON_TIMEOUT_S,
    'aborted_hold_s': DEFAULT_ABORTED_HOLD_S,
    'still_linear_mps': DEFAULT_STILL_LINEAR_MPS,
    'still_angular_rps': DEFAULT_STILL_ANGULAR_RPS,
    'still_for_s': DEFAULT_STILL_FOR_S,
    'odom_timeout_s': DEFAULT_ODOM_TIMEOUT_S,
    'tick_period_s': DEFAULT_TICK_PERIOD_S,
    'publish_period_s': DEFAULT_PUBLISH_PERIOD_S,
    'dry_run': DEFAULT_DRY_RUN,
}

_PARAMETER_TYPES = {
    str: ParameterType.PARAMETER_STRING,
    bool: ParameterType.PARAMETER_BOOL,
    float: ParameterType.PARAMETER_DOUBLE,
}

__all__ = ['DEFAULTS', 'PowerManagerParams', 'specification']


def _describe(description: str, default: Any,
              bounds: Optional[Tuple[float, float]]) -> ParameterDescriptor:
    """Build the read-only descriptor `ros2 param describe` reports."""
    # bool before anything else: bool IS an int in Python and would otherwise
    # be declared as PARAMETER_INTEGER, which rclpy then refuses a `true` for.
    kind = _PARAMETER_TYPES[bool if isinstance(default, bool) else type(default)]
    constraint = _TOPIC_CONSTRAINT if kind == ParameterType.PARAMETER_STRING else _READ_ONCE
    descriptor = ParameterDescriptor(
        description=description,
        type=kind,
        additional_constraints=constraint,
        read_only=True,
    )
    if bounds is not None:
        low, high = bounds
        descriptor.floating_point_range = [
            FloatingPointRange(from_value=float(low), to_value=float(high), step=0.0)]
    return descriptor


def specification() -> Iterator[Tuple[str, Any, ParameterDescriptor]]:
    """
    Yield every parameter as (name, default, descriptor), in declaration order.

    A parameter that is not here is declared nowhere and read nowhere.
    """
    for name, description, bounds in _SPECIFICATION:
        yield name, DEFAULTS[name], _describe(description, DEFAULTS[name], bounds)


@dataclass(frozen=True)
class PowerManagerParams:
    """The validated parameter snapshot the node runs on."""

    button_topic: str = DEFAULT_BUTTON_TOPIC
    odom_topic: str = DEFAULT_ODOM_TOPIC
    state_topic: str = DEFAULT_STATE_TOPIC
    arm_hold_s: float = DEFAULT_ARM_HOLD_S
    confirm_hold_s: float = DEFAULT_CONFIRM_HOLD_S
    arming_ack_s: float = DEFAULT_ARMING_ACK_S
    button_timeout_s: float = DEFAULT_BUTTON_TIMEOUT_S
    aborted_hold_s: float = DEFAULT_ABORTED_HOLD_S
    still_linear_mps: float = DEFAULT_STILL_LINEAR_MPS
    still_angular_rps: float = DEFAULT_STILL_ANGULAR_RPS
    still_for_s: float = DEFAULT_STILL_FOR_S
    odom_timeout_s: float = DEFAULT_ODOM_TIMEOUT_S
    tick_period_s: float = DEFAULT_TICK_PERIOD_S
    publish_period_s: float = DEFAULT_PUBLISH_PERIOD_S
    dry_run: bool = DEFAULT_DRY_RUN

    @property
    def total_hold_s(self) -> float:
        """Seconds of continuous hold between the press and the halt."""
        return self.arm_hold_s + self.confirm_hold_s

    @classmethod
    def declare(cls, node) -> 'PowerManagerParams':
        """
        Declare every parameter on `node` and return the read-back values.

        A bad value is corrected and logged rather than fatal: an operator who
        mistyped a threshold should get the shipped gesture and an error in
        the log, not a bringup with no way to power the robot off from the
        radio.
        """
        for name, default, descriptor in specification():
            _declare(node, name, default, descriptor)

        log = node.get_logger()
        values = {name: node.get_parameter(name).value for name in DEFAULTS}
        values = {name: _checked(log, name, value) for name, value in values.items()}
        values.update(_gesture_window(log, values))
        return cls(**values)


def _declare(node, name: str, default, descriptor: ParameterDescriptor) -> None:
    """
    Declare one parameter, keeping a rejected override off the fatal path.

    rclpy raises when an override has the wrong type or falls outside the
    descriptor range. The retry ignores the override that was just refused and
    takes the default, which is inside its own range by construction, so it
    cannot raise again.
    """
    try:
        node.declare_parameter(name, default, descriptor)
    except (InvalidParameterTypeException, InvalidParameterValueException) as error:
        node.get_logger().error(
            f'Rejected value for {name} ({error}), falling back to {default!r}')
        node.declare_parameter(name, default, descriptor, ignore_override=True)


def _checked(log, name: str, value):
    """
    Coerce one read-back value to the type of its default, or fall back.

    Strings are stripped, and an empty topic name falls back: an empty name
    reaches rclpy as an invalid topic and kills the node in its constructor,
    which is exactly the outcome this module exists to avoid.
    """
    default = DEFAULTS[name]
    if isinstance(default, str):
        text = (value or '').strip()
        if not text:
            log.error(f'Empty {name}, falling back to "{default}"')
            return default
        return text
    if isinstance(default, bool):
        return bool(value)
    try:
        return float(value)
    except (TypeError, ValueError):
        log.error(f'Invalid {name}={value!r}, falling back to {default!r}')
        return default


def _gesture_window(log, values: dict) -> dict:
    """
    Keep the acknowledgement stage inside the confirm window it lives in.

    Each value is inside its own descriptor range and rclpy is happy, and the
    pair can still describe a gesture nobody can perform as documented: an
    `arming_ack_s` at or above `confirm_hold_s` means the state never reaches
    `confirming`, so the operator never gets the fast "let go and it stops"
    pattern that tells him the halt is imminent. The two fall back together,
    because mixing an operator's confirm window with the default
    acknowledgement would produce a third gesture nobody asked for.
    """
    ack, confirm = values['arming_ack_s'], values['confirm_hold_s']
    if ack < confirm:
        return {}
    log.error(
        f'arming_ack_s ({ack}) must be shorter than confirm_hold_s ({confirm}), '
        f'or the state never reaches "confirming". Falling back to '
        f'{DEFAULT_ARMING_ACK_S} / {DEFAULT_CONFIRM_HOLD_S}.')
    return {'arming_ack_s': DEFAULT_ARMING_ACK_S,
            'confirm_hold_s': DEFAULT_CONFIRM_HOLD_S}
