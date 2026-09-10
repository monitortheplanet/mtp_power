#!/usr/bin/env python3
# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: start the power manager node on the parameter file it is named
# after.

"""
Start the power manager node on the parameter file its node name comes from.

Everything the node does -- which button it listens to, how long the gesture
is, what counts as standing still, and whether the halt is real -- is a
parameter, so this file only has to get two things right, and both of them
fail silently.

The node must be called ``power_manager``: that is the root key of
``config/power_manager.yaml``, and ROS 2 drops a ``ros__parameters`` block
whose key matches no node without a word in the log, leaving every value at
the code default. For this node that default set includes ``dry_run: false``
and a button topic the operator may not have chosen -- a mismatch here is not
a cosmetic failure.

The configuration argument is deliberately not called ``config_file``.
``mecgeos_bringup/launch/parametric_bringup.launch.py`` declares a top-level
``config_file`` (the robot config), and ``IncludeLaunchDescription`` does not
scope launch configurations: a shared name is inherited by an include that
forgets to pass it, and leaks back out of one that does, into everything
included after it. Both directions have already cost this stack a debugging
session in ``mtp_carrier_io``, so each package names its argument after
itself -- ``power_manager_config`` here. That argument is also how the live
dry-run rehearsal is done, pointed at a copy of the YAML with ``dry_run:
true``.

No parameter is set here. The node is configured in exactly one place;
setting a value in the launch file as well would give two places to change
and one to forget -- and the value most worth not forgetting is ``dry_run``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = 'mtp_power'
EXECUTABLE_NAME = 'power_manager'

# The root key of config/power_manager.yaml. Renaming one without the other in
# the same commit leaves every parameter at its default, silently.
NODE_NAME = 'power_manager'

CONFIG_DIRECTORY = 'config'
CONFIG_FILE_NAME = 'power_manager.yaml'

# Unique across the bringup include chain on purpose: see the module docstring.
CONFIG_ARGUMENT = 'power_manager_config'


def generate_launch_description():
    """Generate the launch description that starts the power manager node."""
    default_config = PathJoinSubstitution([
        FindPackageShare(PACKAGE_NAME),
        CONFIG_DIRECTORY,
        CONFIG_FILE_NAME,
    ])

    config_argument = DeclareLaunchArgument(
        CONFIG_ARGUMENT,
        default_value=default_config,
        description=(
            'Parameter file for the power manager node. Its root key must be '
            f'"{NODE_NAME}", or every parameter silently stays at its default'
        ),
    )

    power_manager_node = Node(
        package=PACKAGE_NAME,
        executable=EXECUTABLE_NAME,
        name=NODE_NAME,
        # The node logs its wiring once and then only the state transitions and
        # the refusals, which are exactly what is read live while the gesture
        # is being commissioned: 'screen' keeps those lines in the
        # `ros2 launch` output, and emulate_tty keeps Python line-buffering
        # them instead of holding them back in an 8 KiB block -- which for a
        # node whose last act is to halt the machine would mean losing them.
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration(CONFIG_ARGUMENT)],
    )

    return LaunchDescription([
        config_argument,
        power_manager_node,
    ])
