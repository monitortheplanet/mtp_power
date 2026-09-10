#!/usr/bin/env python3
# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: build the /power/state JSON payload, the wire contract with
# mtp_carrier_io and the HMI.

"""
Build the /power/state JSON payload, the wire contract with mtp_carrier_io and the HMI.

The payload is small and the temptation is to build it inline in the node.
It lives here instead because it is a CONTRACT with code in two other
repositories -- `mtp_carrier_io` switches the buzzer and the LED on `state`,
the HMI shows `state`, `seconds_left` and `reason` -- and a contract that is
only ever exercised through an rclpy node is a contract nothing tests. Here it
is one pure function with its own test file.

The shape, frozen:

    {"state": "idle|arming|confirming|shutting_down|aborted",
     "action": "shutdown",
     "source": "radio",
     "seconds_left": 2.4,
     "reason": "robot is moving",
     "seq": 17}

`action` is always "shutdown"; "reboot" is reserved by the contract and this
node never issues it. `source` is always "radio"; "hmi" is reserved for
`mtp_hmi_bridge`, which has its own power path (ROB-31) and does not publish
here today. Both keys are present anyway, so a subscriber written now against
a robot with only the radio path does not have to change shape when a second
source appears.

`seq` counts PUBLISHED MESSAGES, not state changes: the topic is published on
every change and at least once a second, so a subscriber that sees seq jump by
more than one knows it missed something rather than merely that the state sat
still.
"""

import json

from mtp_power.hold_gesture import ABORTED, PENDING_STATES

#: The only action this node issues. "reboot" is reserved by the contract.
ACTION_SHUTDOWN = 'shutdown'
#: Who asked. "hmi" is reserved for the bridge's own power path.
SOURCE_RADIO = 'radio'

#: Keys every payload carries, in this order. A subscriber may rely on all of
#: them being present in every message, including the empty ones.
PAYLOAD_KEYS = ('state', 'action', 'source', 'seconds_left', 'reason', 'seq')

__all__ = ['ACTION_SHUTDOWN', 'PAYLOAD_KEYS', 'SOURCE_RADIO', 'build_payload',
           'encode']


def build_payload(state: str, seconds_left: float, reason: str, seq: int) -> dict:
    """
    Build one `/power/state` payload as a plain dictionary.

    `seconds_left` is 0.0 outside `arming` and `confirming`, and `reason` is
    the empty string outside `aborted`: the contract says so, and it is
    enforced here rather than trusted of every caller.
    """
    return {
        'state': str(state),
        'action': ACTION_SHUTDOWN,
        'source': SOURCE_RADIO,
        'seconds_left': round(float(seconds_left), 1) if state in PENDING_STATES else 0.0,
        'reason': str(reason) if state == ABORTED else '',
        'seq': int(seq),
    }


def encode(state: str, seconds_left: float, reason: str, seq: int) -> str:
    """Build the payload and render it as the JSON string the topic carries."""
    # separators without spaces: the message goes over a 5G uplink to the HMI
    # once a second for the whole session, and the padding buys nothing.
    return json.dumps(build_payload(state, seconds_left, reason, seq),
                      separators=(',', ':'))
