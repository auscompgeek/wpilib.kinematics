import math

import pytest

from wpilib.geometry import Rotation2d, Translation2d


def test_sum():
    one = Translation2d(1, 3)
    two = Translation2d(2, 5)

    sum_ = one + two

    assert sum_.x == 3
    assert sum_.y == 8


def test_difference():
    one = Translation2d(1, 3)
    two = Translation2d(2, 5)

    difference = one - two

    assert difference.x == -1
    assert difference.y == -2


def test_rotate_by():
    another = Translation2d(3, 0)
    rotated = another.rotateBy(Rotation2d.fromDegrees(90))

    assert rotated.x == pytest.approx(0)
    assert math.isclose(rotated.y, 3)


def test_multiplication():
    original = Translation2d(3, 5)
    product = original * 3

    assert product.x == 9
    assert product.y == 15


def test_multiplication_reverse():
    original = Translation2d(3, 5)
    product = 3 * original

    assert product.x == 9
    assert product.y == 15


def test_division():
    original = Translation2d(3, 5)
    quotient = original / 2

    assert math.isclose(quotient.x, 1.5)
    assert math.isclose(quotient.y, 2.5)


def test_norm():
    one = Translation2d(3, 5)
    assert math.isclose(abs(one), math.hypot(3, 5))

    two = Translation2d(1, 1)
    assert math.isclose(abs(two), math.sqrt(2))


def test_distance():
    one = Translation2d(1, 1)
    two = Translation2d(6, 6)
    assert math.isclose(one.getDistance(two), 5 * math.sqrt(2))


def test_unary_minus():
    original = Translation2d(-4.5, 7)
    inverted = -original

    assert inverted.x == 4.5
    assert inverted.y == -7


def test_equality():
    one = Translation2d(9, 5.5)
    two = Translation2d(9.0, 5.5)
    assert one == two


def test_inequality():
    one = Translation2d(9.0, 5.5)
    two = Translation2d(9, 5.7)
    assert one != two
