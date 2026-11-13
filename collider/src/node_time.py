"""
Time for nodes that may run on simulation time.

With the ROS parameter use_sim_time:=true, a node's clock follows the /clock topic
(Gazebo's simulated time); otherwise it is the computer's clock. Timers created with
node.create_timer() follow the same clock automatically; time.monotonic() would not.
"""

import math


def now_sec(node) -> float:
    """Return the node's current time in seconds: simulated or wall time, as configured."""
    return node.get_clock().now().nanoseconds * 1e-9


def age(now: float, then: float) -> float:
    """
    Seconds since `then`.

    A negative age means the clock jumped backwards, e.g. the simulator restarted and
    its time began again at zero. Anything recorded before that is from another
    simulation run, so it counts as infinitely old.
    """
    elapsed = now - then
    return math.inf if elapsed < 0 else elapsed
