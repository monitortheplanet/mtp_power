#!/usr/bin/env python3
# Copyright (c) 2026 Monitor The Planet S.r.l. All rights reserved.
# Proprietary and confidential. Unauthorized copying, distribution or use
# of this file, via any medium, is strictly prohibited.
# Purpose: the ROS node that powers the robot off from a held radio button.

"""
The ROS node that powers the robot off from a held radio button.

All this module does is WIRING. The decision -- when a held button becomes a
shutdown -- is in `hold_gesture.py`, the standstill proof is in
`standstill.py`, the privileged call is in `halt.py` and the wire payload is
in `state_message.py`, all four ROS-free and tested against a fake clock.
Everything below turns them into two subscriptions, one publisher and one
timer.

The order of operations at the end of the gesture is deliberate and is the
only part of this file with real consequences: `shutting_down` is PUBLISHED
FIRST, and the halt is issued afterwards. `mtp_carrier_io` turns the LED
steady and the buzzer continuous on that message, and a human standing next
to a 350 kg machine has to hear it before the stack starts going down, not
after -- once `shutdown -h now` has run, the ROS graph has a few seconds to
live and nothing is guaranteed to be delivered.

This node never publishes a cmd_vel and never touches the mux. See `halt.py`:
the Roboteq watchdog brakes on its own when the stack goes away.
"""

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from mtp_power.guards import guarded
from mtp_power.halt import halt
from mtp_power.hold_gesture import (
    ABORTED,
    GestureConfig,
    HoldGesture,
    SHUTTING_DOWN,
)
from mtp_power.params import PowerManagerParams
from mtp_power.standstill import StandstillWatch
from mtp_power.state_message import encode

#: The node name, which is also the root key of config/power_manager.yaml.
NODE_NAME = 'power_manager'

#: Inputs. Both publishers on this robot are RELIABLE / VOLATILE (measured
#: 2026-09-10), so a matching subscription is what actually connects. Depth 10
#: on the button because a burst of 20 Hz levels must not be thinned into
#: something that looks like a gap; depth 5 on the odometry because only the
#: freshest sample decides anything.
BUTTON_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST)
ODOM_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST)

#: Output. Latched, per the contract: a subscriber that starts late -- the HMI
#: bridge reconnecting, or carrier_io restarting mid-gesture -- must see the
#: CURRENT state immediately and not wait up to a second for the heartbeat.
STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


class PowerManagerNode(Node):
    """Turn a continuous hold on one radio button into a halt of the Jetson."""

    def __init__(self, **node_arguments):
        # `**node_arguments` reaches rclpy.node.Node untouched. Its only user
        # is the test suite, which passes `parameter_overrides` to build the
        # node without a YAML; the executable never passes anything.
        super().__init__(NODE_NAME, **node_arguments)
        self._params = PowerManagerParams.declare(self)
        params = self._params

        self._gesture = HoldGesture(GestureConfig(
            arm_hold_s=params.arm_hold_s,
            confirm_hold_s=params.confirm_hold_s,
            arming_ack_s=params.arming_ack_s,
            button_timeout_s=params.button_timeout_s,
            aborted_hold_s=params.aborted_hold_s,
        ))
        self._standstill = StandstillWatch(
            linear_mps=params.still_linear_mps,
            angular_rps=params.still_angular_rps,
            still_for_s=params.still_for_s,
            timeout_s=params.odom_timeout_s,
        )
        self._seq = 0
        self._published_at = None
        self._last_state = None
        # Set once the halt has been issued, so a timer that keeps ticking for
        # the seconds the stack has left cannot issue it twice.
        self._halted = False

        self._state_pub = self.create_publisher(String, params.state_topic, STATE_QOS)
        self.create_subscription(Bool, params.button_topic, self._on_button, BUTTON_QOS)
        self.create_subscription(Odometry, params.odom_topic, self._on_odom, ODOM_QOS)
        self.create_timer(params.tick_period_s, self._on_tick)

        self.get_logger().info(
            f'power_manager: hold {params.button_topic} for '
            f'{params.arm_hold_s:.1f} s to arm and {params.total_hold_s:.1f} s in '
            f'total to halt; standstill from {params.odom_topic} '
            f'(<= {params.still_linear_mps} m/s and {params.still_angular_rps} rad/s '
            f'for {params.still_for_s:.1f} s, fresher than '
            f'{params.odom_timeout_s:.1f} s); state on {params.state_topic}'
            + ('; DRY RUN: the halt is logged, not executed' if params.dry_run else ''))
        # Publish the initial idle at once: the topic is latched, so this is
        # what a subscriber that starts before any button message ever sees.
        now = self._now()
        self._publish(self._gesture.update(
            now, self._standstill.reason(now), params.dry_run))

    # ── Inputs ──────────────────────────────────────────────────────────

    @guarded
    def _on_button(self, msg: Bool) -> None:
        """Record one button level; the timer is what advances the gesture."""
        self._gesture.button(bool(msg.data), self._now())

    @guarded
    def _on_odom(self, msg: Odometry) -> None:
        """Feed the standstill guard with the wheel odometry twist."""
        twist = msg.twist.twist
        self._standstill.sample(self._now(), twist.linear.x, twist.angular.z)

    # ── The clock ───────────────────────────────────────────────────────

    @guarded
    def _on_tick(self) -> None:
        """Advance the gesture, publish what changed, and halt when it is due."""
        now = self._now()
        gesture = self._gesture.update(now, self._standstill.reason(now),
                                       self._params.dry_run)
        self._log_transition(gesture)
        if gesture.changed or self._is_due(now):
            self._publish(gesture)
        if gesture.state == SHUTTING_DOWN and not self._halted:
            self._halt(now)

    def _is_due(self, now: float) -> bool:
        """Whether the heartbeat republication is due, changed or not."""
        return (self._published_at is None
                or now - self._published_at >= self._params.publish_period_s
                or now < self._published_at)

    def _now(self) -> float:
        """Read the node clock in seconds: the one time source this node uses."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ── Outputs ─────────────────────────────────────────────────────────

    def _publish(self, gesture) -> None:
        """Publish one `/power/state` message and count it."""
        self._seq += 1
        self._state_pub.publish(String(data=encode(
            gesture.state, gesture.seconds_left, gesture.reason, self._seq)))
        self._published_at = self._now()

    def _log_transition(self, gesture) -> None:
        """Log every state change once, at the level the change deserves."""
        if gesture.state == self._last_state:
            return
        self._last_state = gesture.state
        if gesture.state == ABORTED:
            self.get_logger().warn(f'Power: shutdown aborted ({gesture.reason})')
        elif gesture.state == SHUTTING_DOWN:
            self.get_logger().warn('Power: shutdown confirmed from the radio')
        else:
            self.get_logger().info(f'Power: {gesture.state}')

    # ── The halt ────────────────────────────────────────────────────────

    def _halt(self, now: float) -> None:
        """
        Issue the halt, or explain why it did not run.

        `shutting_down` has already been published by the caller, so the
        buzzer and the LED are already saying it. On failure the machine is
        pushed back to `aborted` with the reason: the robot is still up, and a
        state that claimed otherwise would be a lie the operator acts on.
        """
        self._halted = True
        if self._params.dry_run:
            self.get_logger().warn(
                'Power: DRY RUN, not running the halt. Release the button to '
                'return to idle.')
            return
        reason = halt()
        if reason is None:
            self.get_logger().warn('Power: the robot is going down')
            return
        self.get_logger().error(f'Power: the halt did not run ({reason})')
        self._gesture.force_abort(now, reason)
        self._halted = False
        self._publish(self._gesture.update(now, None, self._params.dry_run))


def main(args=None):
    """Run the power manager node until the process is asked to stop."""
    rclpy.init(args=args)
    node = None
    try:
        node = PowerManagerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # `ros2 launch` sending SIGINT, and the shutdown this node itself just
        # asked for, both arrive here. Neither is a fault.
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
