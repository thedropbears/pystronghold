
import pytest

from unittest.mock import MagicMock

from components.range_finder import RangeFinder

def test_range_finder_syntax():
    """ Super basic tests that do nothing but check for syntactical errors"""
    rf = RangeFinder(0)
    rf.range_finder_counter = MagicMock()
    pid_value = rf.pidGet()
    dist = rf._getDistance()
