"""Tests for the clock helpers shared by all nodes."""

import math

from collider.src.node_time import age


def test_age_is_elapsed_time():
    assert age(10.0, 7.5) == 2.5


def test_time_going_backwards_makes_old_observations_infinitely_old():
    # Simulator restarted: its clock starts again near zero.
    assert age(3.0, 5000.0) == math.inf


def test_never_observed_is_infinitely_old():
    assert age(3.0, -math.inf) == math.inf
