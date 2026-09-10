#!/usr/bin/env python3
# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: run the privileged halt, exactly as the HMI power path does.

"""
Run the privileged halt, exactly as the HMI power path does.

This is the same command, the same argv shape, the same timeout and the same
failure classification as `mtp_hmi_bridge/mtp_hmi_bridge/power.py` (ROB-31),
which has been the reference implementation of this call since the HMI got a
power button. It is duplicated rather than imported because the two packages
are separate repositories with no dependency between them; the tests here pin
the argv so the two cannot drift apart silently.

`sudo -n` never prompts. The rule that makes it work is
`/etc/sudoers.d/91-hmi-power` (rendered from
`mtp_ops/security/sudoers-hmi-power.in`), and it is already installed for user
`nvidia` on this robot -- this package needs no new privilege. Without the
rule `sudo` fails immediately instead of hanging on a password prompt, and the
operator is told exactly that rather than "failed".

What this module does NOT do, on purpose: it never publishes a cmd_vel, and it
never touches the mux. The motors are not commanded to stop, because they stop
on their own -- the Roboteq watchdog brakes on a cmd_vel timeout as soon as
the ROS stack goes away, which is the behaviour `power.py` documents and the
reason a shutdown does not need a stop command in front of it. A node that
published a zero Twist here would be commanding motion, which this stack
forbids outright.
"""

import subprocess

#: The privileged command, as it must appear in /etc/sudoers.d/91-hmi-power.
#: `-n` never prompts: a missing sudoers rule fails immediately.
SUDO = ('sudo', '-n', '/sbin/shutdown')
#: Only the halt is wired. `-r now` (reboot) is reserved by the `/power/state`
#: contract and is deliberately not reachable from the radio: the operator
#: asked for a way to cut the power, and a button that sometimes brings the
#: robot back up is a different, more surprising thing.
HALT_COMMAND = SUDO + ('-h', 'now')

#: Seconds to wait for `shutdown` to return. It only schedules the halt and
#: exits, so anything slower than this is a broken sudo, not a slow halt.
RUN_TIMEOUT_S = 10.0

#: Outcomes, in the wording the `/power/state` contract shows to a human.
REASON_NOT_PERMITTED = 'shutdown not permitted: the sudoers rule is missing'
REASON_FAILED = 'shutdown command failed'

#: Fragments sudo prints when the rule is missing or needs a password. Any of
#: them means "not permitted", which is actionable; anything else is a genuine
#: failure. Copied verbatim from power.py.
SUDO_DENIED_MARKERS = (
    'a password is required',
    'no password was provided',
    'a terminal is required',
    'not allowed to execute',
    'may not run',
    'is not in the sudoers file',
)


def default_runner(command):
    """
    Run a command, returning (returncode, stderr).

    The only impure part of this module, replaced by a fake in the tests.
    """
    try:
        completed = subprocess.run(command, capture_output=True, text=True,
                                   timeout=RUN_TIMEOUT_S, check=False)
    except FileNotFoundError as exc:
        return 127, str(exc)
    except subprocess.TimeoutExpired:
        return 124, f'{command[0]} did not return within {RUN_TIMEOUT_S:.0f} s'
    return completed.returncode, completed.stderr or ''


def classify_failure(stderr):
    """Turn a failed `sudo` run into a refusal reason a human can act on."""
    text = (stderr or '').lower()
    if any(marker in text for marker in SUDO_DENIED_MARKERS):
        return REASON_NOT_PERMITTED
    return REASON_FAILED


def halt(runner=default_runner):
    """
    Issue the halt; return None on success, or the reason it did not run.

    A non-zero return is not fatal to the node: the robot is still up, the
    operator has to be told why, and the machine goes back to `aborted` with
    this reason rather than pretending it is going down.
    """
    try:
        returncode, stderr = runner(HALT_COMMAND)
    except Exception as exc:                                    # noqa: BLE001
        return f'{REASON_FAILED} ({exc})'
    if returncode != 0:
        reason = classify_failure(stderr)
        detail = (stderr or '').strip().splitlines()
        return f'{reason} (rc={returncode}{": " + detail[0] if detail else ""})'
    return None
