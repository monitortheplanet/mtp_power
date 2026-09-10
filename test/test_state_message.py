#!/usr/bin/env python3
# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: pin the /power/state payload, the contract with mtp_carrier_io and
# the HMI.

"""
Pin the /power/state payload, the contract with mtp_carrier_io and the HMI.

Two other repositories parse this JSON. `mtp_carrier_io` switches the buzzer
and the LED on `state`; the HMI shows `state`, `seconds_left` and `reason`.
Neither is in this workspace's test scope, so the contract is pinned here, on
literal strings and literal keys rather than on the constants -- a test that
imports the constant it is asserting cannot notice the constant changing.
"""

import json

from mtp_power.hold_gesture import ABORTED, ARMING, CONFIRMING, IDLE, SHUTTING_DOWN
from mtp_power.state_message import PAYLOAD_KEYS, build_payload, encode
import pytest


def decode(*args, **kwargs):
    """Encode and parse back, the way a subscriber sees it."""
    return json.loads(encode(*args, **kwargs))


def test_every_key_of_the_contract_is_present():
    """A subscriber may rely on all six keys being in every message."""
    assert set(decode(IDLE, 0.0, '', 1)) == set(PAYLOAD_KEYS)
    assert PAYLOAD_KEYS == ('state', 'action', 'source', 'seconds_left', 'reason', 'seq')


def test_the_five_states_are_the_five_of_the_contract():
    """Spelled out, because carrier_io matches these exact strings."""
    assert (IDLE, ARMING, CONFIRMING, SHUTTING_DOWN, ABORTED) == (
        'idle', 'arming', 'confirming', 'shutting_down', 'aborted')


def test_the_action_is_always_shutdown():
    """Check 'reboot' is reserved by the contract and never issued here."""
    assert decode(SHUTTING_DOWN, 0.0, '', 7)['action'] == 'shutdown'


def test_the_source_is_always_radio():
    """Check 'hmi' is reserved for the bridge's own power path."""
    assert decode(ARMING, 1.0, '', 7)['source'] == 'radio'


@pytest.mark.parametrize('state', [ARMING, CONFIRMING])
def test_the_countdown_survives_the_round_trip(state):
    """seconds_left is carried in the two states that have one."""
    assert decode(state, 2.4, '', 3)['seconds_left'] == pytest.approx(2.4)


@pytest.mark.parametrize('state', [IDLE, SHUTTING_DOWN, ABORTED])
def test_the_countdown_is_zero_outside_the_pending_states(state):
    """Enforced here, not merely expected of every caller."""
    assert decode(state, 9.9, 'anything', 3)['seconds_left'] == 0.0


def test_the_reason_is_carried_only_with_aborted():
    """The contract: a reason accompanies a refusal and nothing else."""
    assert decode(ABORTED, 0.0, 'robot is moving', 4)['reason'] == 'robot is moving'
    assert decode(CONFIRMING, 1.0, 'robot is moving', 4)['reason'] == ''
    assert decode(IDLE, 0.0, 'robot is moving', 4)['reason'] == ''


def test_the_sequence_number_is_an_integer():
    """It counts published messages, so a subscriber can see a gap."""
    assert decode(IDLE, 0.0, '', 17)['seq'] == 17
    assert isinstance(build_payload(IDLE, 0.0, '', 17)['seq'], int)


def test_the_payload_is_json_a_subscriber_can_parse():
    """It travels as a std_msgs/String; malformed JSON would be invisible."""
    text = encode(ARMING, 2.4, '', 1)
    assert isinstance(text, str)
    assert json.loads(text)['state'] == 'arming'


def test_the_payload_carries_no_padding():
    """It goes over a 2-3 Mbit/s uplink once a second for a whole session."""
    assert ', ' not in encode(IDLE, 0.0, '', 1)


def test_the_countdown_is_reported_to_one_decimal():
    """More digits would only flicker on the operator's screen."""
    assert decode(CONFIRMING, 2.44999, '', 1)['seconds_left'] == pytest.approx(2.4)
