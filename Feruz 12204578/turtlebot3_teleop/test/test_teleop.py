# Author: Feruz 12204578

import pytest
from turtlebot3_teleop_key import checkLinearLimitVelocity, checkAngularLimitVelocity, makeSimpleProfile, constrain

# Test checkLinearLimitVelocity function
def test_checkLinearLimitVelocity():
    assert checkLinearLimitVelocity(0.5) == 0.22

# Test checkAngularLimitVelocity function
def test_checkAngularLimitVelocity():
    assert checkAngularLimitVelocity(1.5) == 2.84

# Test makeSimpleProfile function
def test_makeSimpleProfile():
    assert makeSimpleProfile(0.1, 0.3, 0.05) == 0.15

# Test constrain function
def test_constrain():
    assert constrain(5, 0, 10) == 5
    assert constrain(-5, 0, 10) == 0
