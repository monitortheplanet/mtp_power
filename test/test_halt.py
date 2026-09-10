#!/usr/bin/env python3
# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: pin the privileged halt to the same argv the HMI power path uses.

"""
Pin the privileged halt to the same argv the HMI power path uses.

NOTHING in this file runs the real command. Every test passes a fake runner
that records its argv and returns a canned result; the real `subprocess.run`
is never reached, and it must stay that way -- a test that actually called
`sudo -n /sbin/shutdown -h now` would power the robot off from a `colcon test`.

What is asserted is the CONTRACT with
`mtp_hmi_bridge/mtp_hmi_bridge/power.py`, the reference implementation of this
call: the same binary, the same `-n`, the same `-h now`, the same 10 s
timeout, and the same classification of a missing sudoers rule as something
the operator can fix rather than as a generic failure.
"""

from mtp_power import halt as halt_module
from mtp_power.halt import (
    HALT_COMMAND,
    REASON_FAILED,
    REASON_NOT_PERMITTED,
    RUN_TIMEOUT_S,
    SUDO_DENIED_MARKERS,
    classify_failure,
    halt,
)
import pytest


class Runner:
    """A fake runner: records every argv, returns what the test dictates."""

    def __init__(self, returncode=0, stderr=''):
        self.calls = []
        self.returncode = returncode
        self.stderr = stderr

    def __call__(self, command):
        """Record the command and answer with the canned result."""
        self.calls.append(tuple(command))
        return self.returncode, self.stderr


# ── The command itself ──────────────────────────────────────────────────

def test_the_command_is_the_one_the_sudoers_rule_permits():
    """The argv is the contract with /etc/sudoers.d/91-hmi-power."""
    assert HALT_COMMAND == ('sudo', '-n', '/sbin/shutdown', '-h', 'now')


def test_sudo_never_prompts():
    """`-n` is what turns a missing rule into a failure instead of a hang."""
    assert '-n' in HALT_COMMAND
    assert HALT_COMMAND.index('-n') == 1


def test_the_timeout_matches_the_reference_implementation():
    """`shutdown` only schedules the halt; slower than this is a broken sudo."""
    assert RUN_TIMEOUT_S == 10.0


def test_no_reboot_is_reachable_from_this_module():
    """The radio powers the robot OFF. `-r` is reserved and not wired."""
    assert '-r' not in HALT_COMMAND
    assert not any(name.startswith('REBOOT') for name in dir(halt_module))


def test_the_halt_runs_exactly_one_command():
    """A halt issued twice is a bug nobody gets a second chance to see."""
    runner = Runner()
    assert halt(runner) is None
    assert runner.calls == [HALT_COMMAND]


# ── Failures ────────────────────────────────────────────────────────────

@pytest.mark.parametrize('marker', SUDO_DENIED_MARKERS)
def test_a_missing_sudoers_rule_is_named_as_such(marker):
    """Every fragment sudo prints for "not permitted" is recognised."""
    reason = halt(Runner(returncode=1, stderr=f'sudo: {marker.upper()}'))
    assert reason is not None
    assert reason.startswith(REASON_NOT_PERMITTED)


def test_any_other_failure_is_a_plain_failure():
    """An unknown stderr is not silently blamed on the sudoers rule."""
    reason = halt(Runner(returncode=3, stderr='shutdown: cannot contact systemd'))
    assert reason.startswith(REASON_FAILED)
    assert 'rc=3' in reason


def test_the_failure_reason_carries_the_first_line_of_stderr():
    """The operator gets the sentence, not just "it failed"."""
    reason = halt(Runner(returncode=1, stderr='boom\nsecond line\n'))
    assert 'boom' in reason
    assert 'second line' not in reason


def test_a_failure_with_no_stderr_still_reports_its_code():
    """A silent non-zero must not produce an empty explanation."""
    reason = halt(Runner(returncode=5, stderr=''))
    assert 'rc=5' in reason


def test_a_runner_that_raises_does_not_escape():
    """The node must survive whatever subprocess does; it has a robot to warn."""
    def explode(_command):
        raise OSError('no such thing')

    reason = halt(explode)
    assert reason.startswith(REASON_FAILED)
    assert 'no such thing' in reason


def test_classification_is_case_insensitive():
    """Match the markers case-insensitively: sudo does not promise its case."""
    assert classify_failure('SUDO: A PASSWORD IS REQUIRED') == REASON_NOT_PERMITTED
    assert classify_failure('') == REASON_FAILED
    assert classify_failure(None) == REASON_FAILED


# ── What this module must never do ──────────────────────────────────────

def test_the_module_never_mentions_a_velocity_topic():
    """
    The halt must not command motion, not even a zero.

    The Roboteq watchdog brakes on the cmd_vel timeout as soon as the stack
    goes away; publishing a stop here would be commanding motion, which this
    stack forbids outright. Asserted on the source because the point is that
    the code does not exist, not that a branch is not taken.
    """
    from pathlib import Path
    source = Path(halt_module.__file__).read_text(encoding='utf-8')
    body = source.split('"""', 2)[-1]
    assert 'cmd_vel' not in body
    assert 'Twist' not in body
