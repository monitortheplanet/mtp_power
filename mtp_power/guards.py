#!/usr/bin/env python3
# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: the decorator that keeps an exception inside a ROS callback from
# ending the process.

"""
The decorator that keeps an exception inside a ROS callback from ending the node.

An exception thrown inside any rclpy callback unwinds the executor and ends
the process. For this node that would be worse than a lost feature: the
operator's only way of powering the robot off without a laptop would stop
answering, silently, and the LED and the buzzer would go on showing whatever
the last `/power/state` message said.

It lives in a module of its own, and not inside the node, so it stays a leaf
with no ROS import of its own and can be exercised directly in the tests.
"""

import functools
import traceback

#: How often a repeating fault may reach the log. A fault in a callback
#: usually repeats on every message, and a line per message at 20 Hz would
#: bury everything else in the bringup log within a minute.
ERROR_LOG_PERIOD_S = 5.0

__all__ = ['ERROR_LOG_PERIOD_S', 'guarded']


def guarded(callback):
    """
    Log whatever a ROS callback raises instead of letting it escape.

    The wrapped method has to be able to reach a logger: every user of this
    decorator is either an rclpy Node or an object holding one as `_node`.
    """
    @functools.wraps(callback)
    def wrapper(self, *args, **kwargs):
        try:
            return callback(self, *args, **kwargs)
        except Exception:
            _logger(self).error(
                f'{callback.__name__} failed: {traceback.format_exc()}',
                throttle_duration_sec=ERROR_LOG_PERIOD_S)
            return None
    return wrapper


def _logger(owner):
    """Find the logger of a node, or of an object that holds one."""
    if hasattr(owner, 'get_logger'):
        return owner.get_logger()
    return owner._node.get_logger()
