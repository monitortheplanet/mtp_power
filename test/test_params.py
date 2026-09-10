#!/usr/bin/env python3
# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: test the parameter contract: declared, described, bounded,
# read-only, and never fatal.

"""
Test the parameter contract: declared, described, bounded, read-only, never fatal.

The node builds itself out of these values once, in its constructor, so a bad
one has exactly two acceptable outcomes: rclpy refuses it against the
descriptor, or `params.py` corrects it. A third outcome -- an exception out of
the constructor -- would mean the operator's only laptop-free way of powering
the robot off is missing from the bringup, and the only sign of it is a line
somewhere in a launch log.

The shipped YAML is parsed and checked against the declaration list, because a
key in one and not the other is silently ignored by ROS 2.
"""

import pathlib

from mtp_power.params import (
    DEFAULTS,
    PowerManagerParams,
    specification,
)
import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import yaml

CONFIG = pathlib.Path(__file__).resolve().parents[1] / 'config' / 'power_manager.yaml'
NODE_NAME = 'power_manager'

#: Parameters whose value is a topic name, and so must never be empty.
TOPIC_PARAMETERS = ('button_topic', 'odom_topic', 'state_topic')


@pytest.fixture(scope='module', autouse=True)
def ros():
    """One rclpy context for the module; conftest.py has pinned the domain."""
    rclpy.init()
    yield
    rclpy.shutdown()


def build(**overrides):
    """Declare the parameters on a throwaway node and read them back."""
    node = Node(NODE_NAME, parameter_overrides=[
        Parameter(name, value=value) for name, value in overrides.items()])
    try:
        return PowerManagerParams.declare(node), node
    finally:
        pass


def snapshot(**overrides):
    """Return the validated snapshot only, with the node destroyed."""
    params, node = build(**overrides)
    node.destroy_node()
    return params


# ── The declaration list ────────────────────────────────────────────────

def test_every_parameter_is_declared_exactly_once():
    """The specification is the single list; a duplicate would shadow itself."""
    names = [name for name, _, _ in specification()]
    assert len(names) == len(set(names))
    assert set(names) == set(DEFAULTS)


def test_every_parameter_is_a_field_of_the_snapshot():
    """A key that is not a field here does not exist, by construction."""
    assert set(DEFAULTS) == set(PowerManagerParams.__dataclass_fields__)


def test_every_parameter_is_read_only():
    """`ros2 param set` must be refused, not accepted and ignored."""
    for name, _, descriptor in specification():
        assert descriptor.read_only, name


def test_every_parameter_has_a_description():
    """`ros2 param describe` is the documentation an operator reads at 2 a.m."""
    for name, _, descriptor in specification():
        assert descriptor.description.strip(), name


def test_every_numeric_parameter_is_bounded():
    """A range is what makes rclpy refuse a value instead of the node."""
    for name, default, descriptor in specification():
        if isinstance(default, float):
            assert descriptor.floating_point_range, name


def test_the_boolean_is_declared_as_a_boolean():
    """Declare it as a bool: bool IS an int, and rclpy would refuse `true`."""
    kinds = {name: descriptor.type for name, _, descriptor in specification()}
    assert kinds['dry_run'] == 1                              # PARAMETER_BOOL


def test_every_default_is_inside_its_own_range():
    """The fallback path re-declares with the default: it must not raise."""
    for name, default, descriptor in specification():
        if not descriptor.floating_point_range:
            continue
        bounds = descriptor.floating_point_range[0]
        assert bounds.from_value <= default <= bounds.to_value, name


def test_the_shipped_defaults_are_the_operators_gesture():
    """1.0 + 2.0 = the 3.0 s hold he asked for on 2026-09-10."""
    assert DEFAULTS['arm_hold_s'] == 1.0
    assert DEFAULTS['confirm_hold_s'] == 2.0
    assert PowerManagerParams().total_hold_s == pytest.approx(3.0)


def test_the_halt_is_off_by_default_in_the_sense_that_matters():
    """dry_run defaults to false: the shipped node really does power off."""
    assert DEFAULTS['dry_run'] is False


# ── The shipped YAML ────────────────────────────────────────────────────

def shipped():
    """Return the parameter block of config/power_manager.yaml."""
    document = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    return document[NODE_NAME]['ros__parameters']


def test_the_yaml_sets_only_parameters_that_exist():
    """A key ROS 2 does not know is accepted in silence and read by nobody."""
    unknown = sorted(set(shipped()) - set(DEFAULTS))
    assert not unknown, unknown


def test_the_yaml_sets_every_parameter_there_is():
    """The other direction: a value nobody can set from the config file."""
    missing = sorted(set(DEFAULTS) - set(shipped()))
    assert not missing, missing


def test_the_yaml_root_key_is_the_node_name():
    """ROS 2 drops a block whose key matches no node, without a word."""
    assert set(yaml.safe_load(CONFIG.read_text(encoding='utf-8'))) == {NODE_NAME}


def test_the_yaml_ships_with_the_halt_armed():
    """A robot config left in dry run would be a shutdown button that lies."""
    assert shipped()['dry_run'] is False


def test_the_yaml_values_are_all_accepted():
    """The shipped file must survive its own validation untouched."""
    params = snapshot(**shipped())
    for name, value in shipped().items():
        assert getattr(params, name) == value, name


# ── Nothing is fatal ────────────────────────────────────────────────────

def test_an_out_of_range_value_falls_back_instead_of_killing_the_node():
    """Refuse it against the descriptor, and start the node anyway."""
    assert snapshot(arm_hold_s=9999.0).arm_hold_s == DEFAULTS['arm_hold_s']


def test_a_wrong_type_falls_back_instead_of_killing_the_node():
    """A string where a float belongs is a typo, not a reason to lose it."""
    assert snapshot(confirm_hold_s='soon').confirm_hold_s == DEFAULTS['confirm_hold_s']


@pytest.mark.parametrize('name', TOPIC_PARAMETERS)
def test_an_empty_topic_name_falls_back(name):
    """An empty name reaches rclpy as an invalid topic and kills the node."""
    assert getattr(snapshot(**{name: '   '}), name) == DEFAULTS[name]


def test_an_acknowledgement_longer_than_the_confirm_window_falls_back():
    """
    Both halves fall back together, because the pair is the gesture.

    Each value is inside its own range and rclpy is happy; the pair still
    describes a gesture that never reaches `confirming`, so the operator never
    gets the fast "let go and it stops" pattern.
    """
    params = snapshot(arming_ack_s=5.0, confirm_hold_s=2.0)
    assert params.arming_ack_s == DEFAULTS['arming_ack_s']
    assert params.confirm_hold_s == DEFAULTS['confirm_hold_s']


def test_a_consistent_gesture_is_left_alone():
    """The fallback must not fire on a perfectly good override."""
    params = snapshot(arm_hold_s=2.0, confirm_hold_s=3.0, arming_ack_s=1.0)
    assert (params.arm_hold_s, params.confirm_hold_s, params.arming_ack_s) == \
        (2.0, 3.0, 1.0)


def test_the_button_topic_is_a_plain_parameter():
    """
    Any Bool topic; nothing in the package assumes which button it is.

    The operator has not chosen his button yet, so this must be a one-line
    change in the YAML and nothing else.
    """
    params = snapshot(button_topic='/radio/button/tool/fn3')
    assert params.button_topic == '/radio/button/tool/fn3'


def test_declaring_twice_on_one_node_does_not_raise():
    """The fallback path re-declares; it must be able to."""
    params, node = build(odom_timeout_s=-5.0)
    try:
        assert params.odom_timeout_s == DEFAULTS['odom_timeout_s']
    finally:
        node.destroy_node()
