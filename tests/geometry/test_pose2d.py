import math

import pytest

from wpilib.geometry import Pose2d, Rotation2d, Transform2d, Translation2d


def test_transform_by():
    initial = Pose2d(1, 2, Rotation2d.fromDegrees(45))
    transform = Transform2d(Translation2d(5, 0), Rotation2d.fromDegrees(5))

    transformed = initial + transform

    assert math.isclose(transformed.translation.x, 1 + 5 / math.sqrt(2))
    assert math.isclose(transformed.translation.y, 2 + 5 / math.sqrt(2))
    assert math.isclose(transformed.rotation.getDegrees(), 50)


def test_relative_to():
    initial = Pose2d(0, 0, Rotation2d.fromDegrees(45))
    final = Pose2d(5, 5, Rotation2d.fromDegrees(45))

    final_relative_to_initial = final.relativeTo(initial)

    assert math.isclose(final_relative_to_initial.translation.x, 5 * math.sqrt(2))
    assert final_relative_to_initial.translation.y == pytest.approx(0)
    assert math.isclose(final_relative_to_initial.rotation.getDegrees(), 0)


def test_equality():
    a = Pose2d(0, 5.0, Rotation2d.fromDegrees(43))
    b = Pose2d(0.0, 5, Rotation2d.fromDegrees(43))
    assert a == b


def test_inequality():
    a = Pose2d(0, 5, Rotation2d.fromDegrees(43))
    b = Pose2d(0, 1.524, Rotation2d.fromDegrees(43))
    assert a != b


def test_minus():
    initial = Pose2d(0, 0, Rotation2d.fromDegrees(45))
    final = Pose2d(5, 5, Rotation2d.fromDegrees(45))

    transform = final - initial

    assert math.isclose(transform.translation.x, 5 * math.sqrt(2))
    assert transform.translation.y == pytest.approx(0)
    assert math.isclose(transform.rotation.getDegrees(), 0)
