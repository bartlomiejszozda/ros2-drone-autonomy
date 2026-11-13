"""Send a takeoff goal, print feedback, and optionally cancel it."""

import argparse
import math
import time

import rclpy
from collider_interfaces.action import Takeoff
from rclpy.action import ActionClient
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--altitude', type=float, default=2.0)
    parser.add_argument('--timeout', type=float, default=0.0)
    parser.add_argument('--cancel-after', type=float)
    options = parser.parse_args()
    if not math.isfinite(options.altitude) or options.altitude <= 0:
        parser.error('altitude must be finite and positive')
    for value in (options.timeout, options.cancel_after):
        if value is not None and (not math.isfinite(value) or value < 0):
            parser.error('timeouts must be finite and nonnegative')

    rclpy.init(args=[])
    node = Node('takeoff_client')
    client = ActionClient(node, Takeoff, '/starter/takeoff')
    try:
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('Start starter_node before running this client')
        future = client.send_goal_async(
            Takeoff.Goal(altitude_m=options.altitude, timeout_sec=options.timeout),
            feedback_callback=lambda msg: print(
                f'{msg.feedback.stage} ({msg.feedback.altitude_m:.2f} m): '
                f'{msg.feedback.message}', flush=True))
        rclpy.spin_until_future_complete(node, future)
        goal = future.result()
        if not goal.accepted:
            print('Goal rejected')
            return
        result_future = goal.get_result_async()
        started = time.monotonic()
        cancel_sent = False
        while rclpy.ok() and not result_future.done():
            if (options.cancel_after is not None and not cancel_sent
                    and time.monotonic() - started >= options.cancel_after):
                goal.cancel_goal_async()
                cancel_sent = True
                print('Cancellation requested', flush=True)
            rclpy.spin_once(node, timeout_sec=0.1)
        if result_future.done():
            response = result_future.result()
            print(f'Result: status={response.status}, success={response.result.success}, '
                  f'cleanup_completed={response.result.cleanup_completed}')
            print(response.result.message)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
