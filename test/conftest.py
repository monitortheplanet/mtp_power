#!/usr/bin/env python3
# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: keep every test of this package off the live robot's DDS domain.

"""
Keep every test of this package off the live robot's DDS domain.

The robot runs on `ROS_DOMAIN_ID=0` and exports it from `~/.bashrc`, so a test
that builds a node inherits it. Several tests here do build one, and the topic
this node publishes is `/power/state`: on domain 0 a test would drive the real
buzzer and the real LED of a machine somebody may be standing next to, and
tell the HMI the robot is about to power off. This is the one package in the
stack where getting that wrong is not merely noisy.

pytest imports this file before it collects any test module, and rclpy reads
the variable when a context is initialised, so setting it here covers every
test in the package -- including the ones a future commit adds without
thinking about it. It is set unconditionally and not with `setdefault`,
because the value that has to be overridden is precisely the one the shell
already exports.

Note what this does NOT protect against, because nothing here can: the halt
itself. No test in this package is allowed to call `mtp_power.halt.halt()`
with the real runner -- every one of them passes a fake. `test_halt.py` says
so at the top and asserts on argv rather than on effects.
"""

import os

#: Domain used by every test of this package. 77 is outside the range this
#: site uses for robots and matches the isolated domain the feature brief
#: prescribes for functional checks.
TEST_DOMAIN_ID = '77'

os.environ['ROS_DOMAIN_ID'] = TEST_DOMAIN_ID
