#!/usr/bin/env python3
"""Shared node lifecycle helper.

Every node in this package needs the same shutdown handling, and getting it right is
fiddly enough to be worth doing once. rclpy's default SIGINT handler tears the ROS
context down *before* spin() returns, so a final action -- publishing a zero velocity,
logging a summary -- then fails with "publisher's context is invalid". Taking the signal
ourselves (SignalHandlerOptions.NO) and looping on spin_once keeps the context valid until
we choose to close it.

run_node() is that pattern in one place, so the individual nodes do not each repeat it.
"""

import signal

import rclpy
from rclpy.signals import SignalHandlerOptions


def run_node(make_node, *, on_shutdown=None, always=None, args=None):
    """Initialise ROS, run a node until Ctrl-C / SIGTERM, then clean up.

    make_node   : callable returning the constructed Node (created after rclpy.init).
    on_shutdown : callable(node) run once on exit ONLY while the ROS context is still
                  valid -- for final publishes or logs that need the middleware alive.
    always      : callable(node) run once on exit unconditionally -- for local cleanup
                  such as stopping a thread or closing a window, which must happen even
                  if the context is already down.
    args        : passed through to rclpy.init.
    """
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = make_node()

    stop = {'requested': False}

    def _request_stop(_signum, _frame):
        stop['requested'] = True

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    try:
        while rclpy.ok() and not stop['requested']:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if always is not None:
            always(node)
        if on_shutdown is not None and rclpy.ok():
            on_shutdown(node)
        node.destroy_node()
        rclpy.try_shutdown()
