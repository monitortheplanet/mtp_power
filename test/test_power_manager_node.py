#!/usr/bin/env python3
# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: test the node wiring: the subscriptions, the latched state topic,
# the order of publish-then-halt, and dry run.

"""
Test the node wiring: subscriptions, the latched state topic, publish-then-halt, dry run.

The decision itself is tested in `test_hold_gesture.py` against a fake clock,
so what is left here is the wiring, and it is tested two ways.

Most tests drive the node's own callbacks on a fake clock, which makes a 3 s
gesture instant and deterministic. `halt` is replaced by a recorder in EVERY
one of them: no test in this package may reach the real command, and the
recorder also notes the ORDER of the halt against the publications, which is
the one ordering in this node with real consequences.

Two tests go through DDS for real, because what they check -- that the state
topic is latched so a late subscriber sees the current state, and that the
node's QoS actually matches the RELIABLE publishers measured on the robot --
does not exist above the middleware. `conftest.py` has forced
ROS_DOMAIN_ID=77, so nothing here can reach the robot's own graph.
"""

import json
import time

from mtp_power import power_manager_node as node_module
from mtp_power.power_manager_node import PowerManagerNode, STATE_QOS
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String

BUTTON_TOPIC = '/test/button'
ODOM_TOPIC = '/test/odom'
STATE_TOPIC = '/test/power/state'

#: The shipped gesture, driven by a fake clock so it costs no wall time.
GESTURE = {
    'arm_hold_s': 1.0,
    'confirm_hold_s': 2.0,
    'arming_ack_s': 0.5,
    'button_timeout_s': 1.0,
    'aborted_hold_s': 2.0,
    'still_for_s': 1.0,
    'odom_timeout_s': 1.0,
    'tick_period_s': 0.05,
    'publish_period_s': 1.0,
}

TICK_S = GESTURE['tick_period_s']

#: How long a DDS test waits for discovery before giving up. Generous, because
#: it runs on a Jetson that may be carrying the whole robot stack at the time.
DISCOVERY_TIMEOUT_S = 5.0


@pytest.fixture(scope='module', autouse=True)
def ros():
    """One rclpy context for the module; conftest.py has pinned the domain."""
    rclpy.init()
    yield
    rclpy.shutdown()


class Recorder:
    """The fake clock, the halt recorder and the publication log, in one place."""

    def __init__(self):
        self.now = 5000.0
        self.halt_result = None
        #: Every publication and every halt, in the order they happened.
        self.events = []

    @property
    def published(self):
        """Every `/power/state` payload published, in order."""
        return [event for kind, event in self.events if kind == 'publish']

    @property
    def halts(self):
        """Return the times at which the halt was reached."""
        return [event for kind, event in self.events if kind == 'halt']

    @property
    def states(self):
        """Every distinct state published, in order."""
        seen = []
        for payload in self.published:
            if not seen or seen[-1] != payload['state']:
                seen.append(payload['state'])
        return seen

    def clear(self):
        """Forget what has been published so far, keeping the clock."""
        self.events.clear()

    def halt(self):
        """Stand in for the privileged call. Never runs anything."""
        self.events.append(('halt', self.now))
        return self.halt_result


def make(monkeypatch, **overrides):
    """
    Build a node on a fake clock, with the halt and every publication recorded.

    The clock is patched on the class before the constructor runs, because the
    constructor publishes the first latched `idle` and that message is part of
    the contract.
    """
    recorder = Recorder()
    monkeypatch.setattr(node_module, 'halt', recorder.halt)
    monkeypatch.setattr(PowerManagerNode, '_now', lambda _self: recorder.now)

    original_publish = PowerManagerNode._publish

    def recording_publish(self, gesture):
        publisher = self._state_pub
        sent = []
        original_send = publisher.publish
        publisher.publish = lambda msg: sent.append(msg) or original_send(msg)
        try:
            original_publish(self, gesture)
        finally:
            publisher.publish = original_send
        for msg in sent:
            recorder.events.append(('publish', json.loads(msg.data)))

    monkeypatch.setattr(PowerManagerNode, '_publish', recording_publish)

    settings = dict(GESTURE, button_topic=BUTTON_TOPIC, odom_topic=ODOM_TOPIC,
                    state_topic=STATE_TOPIC)
    settings.update(overrides)
    node = PowerManagerNode(parameter_overrides=[
        Parameter(name, value=value) for name, value in settings.items()])
    return node, recorder


def advance(node, recorder, seconds, level=None, linear=0.0, angular=0.0,
            odom=True, button=True):
    """Run the node's callbacks for `seconds` at the real message rates."""
    ticks = max(1, int(round(seconds / TICK_S)))
    for tick in range(ticks):
        recorder.now = round(recorder.now + TICK_S, 6)
        if button and level is not None:
            node._on_button(Bool(data=level))
        if odom and tick % 2 == 0:                  # 10 Hz odometry, 20 Hz ticks
            message = Odometry()
            message.twist.twist.linear.x = linear
            message.twist.twist.angular.z = angular
            node._on_odom(message)
        node._on_tick()


@pytest.fixture
def robot(monkeypatch):
    """Build a node standing still, ready for a button."""
    node, recorder = make(monkeypatch)
    yield node, recorder
    node.destroy_node()


# ── The wiring ──────────────────────────────────────────────────────────

def test_the_node_publishes_idle_before_anything_happens(robot):
    """The topic is latched: a late subscriber must see the current state."""
    _, recorder = robot
    assert recorder.published[0]['state'] == 'idle'
    assert recorder.published[0]['seq'] == 1


def test_the_state_topic_is_latched_and_reliable():
    """The contract: RELIABLE, TRANSIENT_LOCAL, depth 1."""
    assert STATE_QOS.depth == 1
    assert STATE_QOS.durability.name == 'TRANSIENT_LOCAL'
    assert STATE_QOS.reliability.name == 'RELIABLE'


def test_the_node_subscribes_the_two_topics_it_was_given(robot):
    """The topics are parameters, and the node must actually use them."""
    node, _ = robot
    subscribed = {sub.topic_name for sub in node.subscriptions}
    assert subscribed == {BUTTON_TOPIC, ODOM_TOPIC}


def test_the_node_publishes_nothing_but_the_state_topic(robot):
    """
    It must never publish a cmd_vel, and never touch the mux.

    The Roboteq watchdog brakes on its own when the stack goes away; a stop
    command from here would be commanding motion, which this stack forbids.
    """
    node, _ = robot
    topics = {pub.topic_name for pub in node.publishers
              if 'parameter_events' not in pub.topic_name
              and 'rosout' not in pub.topic_name}
    assert topics == {STATE_TOPIC}


def test_the_heartbeat_republishes_at_least_once_a_second(robot):
    """The contract says at least every 1.0 s, changed or not."""
    node, recorder = robot
    recorder.clear()
    advance(node, recorder, 3.0, level=False)
    assert 3 <= len(recorder.published) <= 4
    assert {payload['state'] for payload in recorder.published} == {'idle'}


def test_the_sequence_number_only_ever_grows(robot):
    """A subscriber uses it to notice it missed something."""
    node, recorder = robot
    advance(node, recorder, 2.0, level=False)
    numbers = [payload['seq'] for payload in recorder.published]
    assert numbers == sorted(numbers)
    assert len(set(numbers)) == len(numbers)


# ── The gesture, through the node ───────────────────────────────────────

def test_the_full_gesture_reaches_the_halt(robot):
    """The headline path, end to end through the callbacks."""
    node, recorder = robot
    advance(node, recorder, 1.5, level=False)       # earn the standstill window
    recorder.clear()
    advance(node, recorder, 3.2, level=True)
    # The leading 'idle' is the once-a-second heartbeat before the hold has
    # been long enough to arm; what matters is the three that follow it.
    assert recorder.states[-3:] == ['arming', 'confirming', 'shutting_down']
    assert len(recorder.halts) == 1


def test_shutting_down_is_published_before_the_halt_runs(robot):
    """
    The one ordering in this node with real consequences.

    carrier_io turns the LED steady and the buzzer continuous on that message,
    and a human next to the machine has to hear it BEFORE the stack starts
    going down -- afterwards, nothing is guaranteed to be delivered.
    """
    node, recorder = robot
    advance(node, recorder, 1.5, level=False)
    recorder.clear()
    advance(node, recorder, 3.2, level=True)
    kinds = [kind for kind, _ in recorder.events]
    halt_index = kinds.index('halt')
    before = [payload['state'] for kind, payload in recorder.events[:halt_index]
              if kind == 'publish']
    assert before[-1] == 'shutting_down'


def test_the_halt_runs_exactly_once(robot):
    """The timer keeps ticking for the seconds the stack has left."""
    node, recorder = robot
    advance(node, recorder, 1.5, level=False)
    advance(node, recorder, 6.0, level=True)
    assert len(recorder.halts) == 1


def test_a_release_before_the_end_aborts_and_never_halts(robot):
    """Let go and it stops: the rule the operator relies on."""
    node, recorder = robot
    advance(node, recorder, 1.5, level=False)
    recorder.clear()
    advance(node, recorder, 2.5, level=True)
    advance(node, recorder, 0.2, level=False)
    assert recorder.states[-1] == 'aborted'
    assert recorder.published[-1]['reason'] == 'released'
    assert recorder.halts == []


def test_a_moving_robot_is_refused_with_its_reason(robot):
    """Powering off a machine that is rolling is not a button's decision."""
    node, recorder = robot
    advance(node, recorder, 1.5, level=False, linear=0.4)
    recorder.clear()
    advance(node, recorder, 2.0, level=True, linear=0.4)
    assert recorder.states[-1] == 'aborted'
    assert recorder.published[-1]['reason'] == 'robot is moving'
    assert recorder.halts == []


def test_missing_odometry_is_refused(robot):
    """A robot whose motion is unknown is not a robot that is stopped."""
    node, recorder = robot
    advance(node, recorder, 2.0, level=True, odom=False)
    assert recorder.states[-1] == 'aborted'
    assert recorder.published[-1]['reason'] == 'no odometry'
    assert recorder.halts == []


def test_stale_odometry_is_refused(robot):
    """The motor bridge dying must close the button, not open it."""
    node, recorder = robot
    advance(node, recorder, 1.5, level=False)
    advance(node, recorder, 1.5, level=False, odom=False)
    recorder.clear()
    advance(node, recorder, 2.0, level=True, odom=False)
    assert recorder.published[-1]['reason'] == 'odometry is stale'
    assert recorder.halts == []


def test_the_radio_going_silent_aborts_a_pending_request(robot):
    """A shutdown must not outlive the radio that asked for it."""
    node, recorder = robot
    advance(node, recorder, 1.5, level=False)
    advance(node, recorder, 1.5, level=True)
    recorder.clear()
    advance(node, recorder, 1.2, level=True, button=False)
    assert recorder.states[-1] == 'aborted'
    assert recorder.halts == []


def test_a_brush_of_the_button_does_nothing_at_all(robot):
    """arm_hold_s exists so that a moment's contact is not a shutdown."""
    node, recorder = robot
    advance(node, recorder, 1.5, level=False)
    recorder.clear()
    advance(node, recorder, 0.8, level=True)
    advance(node, recorder, 0.5, level=False)
    assert recorder.states == ['idle']
    assert recorder.halts == []


# ── dry_run ─────────────────────────────────────────────────────────────

def test_dry_run_goes_through_every_state_and_never_halts(monkeypatch):
    """The whole point: the rehearsal the orchestrator runs on the robot."""
    node, recorder = make(monkeypatch, dry_run=True)
    try:
        advance(node, recorder, 1.5, level=False)
        recorder.clear()
        advance(node, recorder, 3.2, level=True)
        assert recorder.states[-3:] == ['arming', 'confirming', 'shutting_down']
        assert recorder.halts == []
    finally:
        node.destroy_node()


def test_dry_run_returns_to_idle_on_release(monkeypatch):
    """So the operator can rehearse as often as he likes, robot still on."""
    node, recorder = make(monkeypatch, dry_run=True)
    try:
        advance(node, recorder, 1.5, level=False)
        advance(node, recorder, 3.2, level=True)
        advance(node, recorder, 0.3, level=False)
        assert recorder.published[-1]['state'] == 'idle'
    finally:
        node.destroy_node()


def test_dry_run_can_be_repeated(monkeypatch):
    """A rehearsal that only works once is not a rehearsal."""
    node, recorder = make(monkeypatch, dry_run=True)
    try:
        advance(node, recorder, 1.5, level=False)
        advance(node, recorder, 3.2, level=True)
        advance(node, recorder, 1.0, level=False)
        recorder.clear()
        advance(node, recorder, 3.2, level=True)
        assert recorder.states[-1] == 'shutting_down'
        assert recorder.halts == []
    finally:
        node.destroy_node()


# ── A halt that does not run ────────────────────────────────────────────

def test_a_refused_halt_becomes_an_abort_with_the_reason(robot):
    """A robot that is still up must not claim it is going down."""
    node, recorder = robot
    recorder.halt_result = 'shutdown not permitted: the sudoers rule is missing'
    advance(node, recorder, 1.5, level=False)
    advance(node, recorder, 3.2, level=True)
    payload = recorder.published[-1]
    assert payload['state'] == 'aborted'
    assert 'sudoers' in payload['reason']


def test_a_refused_halt_cannot_be_retried_by_keeping_the_button_held(robot):
    """The refusal latches like every other abort: let go and press again."""
    node, recorder = robot
    recorder.halt_result = 'shutdown command failed'
    advance(node, recorder, 1.5, level=False)
    advance(node, recorder, 3.2, level=True)
    advance(node, recorder, 5.0, level=True)
    assert len(recorder.halts) == 1


# ── A callback that raises ──────────────────────────────────────────────

def test_a_malformed_message_does_not_kill_the_node(robot):
    """`guarded` exists so a bad message costs a log line, not the feature."""
    node, recorder = robot
    node._on_odom(object())
    node._on_button(object())
    advance(node, recorder, 0.2, level=False)
    assert recorder.published[-1]['state'] == 'idle'


# ── Through DDS, for real ───────────────────────────────────────────────

def test_a_late_subscriber_sees_the_current_state(monkeypatch):
    """
    The latched QoS, exercised through the middleware rather than asserted.

    A subscriber that joins after the node has settled -- the HMI bridge
    reconnecting, or carrier_io restarting -- must get the state at once.
    """
    node, _ = make(monkeypatch)
    listener = Node('late_subscriber')
    received = []
    try:
        listener.create_subscription(
            String, STATE_TOPIC, lambda msg: received.append(json.loads(msg.data)),
            STATE_QOS)
        deadline = time.time() + DISCOVERY_TIMEOUT_S
        while time.time() < deadline and not received:
            rclpy.spin_once(listener, timeout_sec=0.05)
        assert received, 'the latched state never reached a late subscriber'
        assert received[-1]['state'] == 'idle'
        assert received[-1]['action'] == 'shutdown'
        assert received[-1]['source'] == 'radio'
    finally:
        listener.destroy_node()
        node.destroy_node()


def test_the_node_matches_a_reliable_button_publisher(monkeypatch):
    """The measured publishers are RELIABLE/VOLATILE; the QoS must match."""
    node, _ = make(monkeypatch)
    talker = Node('button_talker')
    publisher = talker.create_publisher(Bool, BUTTON_TOPIC, 10)
    try:
        deadline = time.time() + DISCOVERY_TIMEOUT_S
        while time.time() < deadline and publisher.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert publisher.get_subscription_count() == 1
    finally:
        talker.destroy_node()
        node.destroy_node()
