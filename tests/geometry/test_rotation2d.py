import math

from wpilib.geometry import Rotation2d


def test_radians_to_degrees():
    assert math.isclose(Rotation2d(math.pi / 3).getDegrees(), 60)
    assert math.isclose(Rotation2d(math.pi / 4).getDegrees(), 45)


def test_degrees_to_radians():
    assert math.isclose(Rotation2d.fromDegrees(45).getRadians(), math.pi / 4)
    assert math.isclose(Rotation2d.fromDegrees(30).getRadians(), math.pi / 6)


def test_rotate_by_from_zero():
    zero = Rotation2d()
    sum_ = zero + Rotation2d.fromDegrees(90)

    assert math.isclose(sum_.getRadians(), math.pi / 2)
    assert math.isclose(sum_.getDegrees(), 90)


def test_rotate_by_non_zero():
    rot = Rotation2d.fromDegrees(90)
    rot += Rotation2d.fromDegrees(30)

    assert math.isclose(rot.getDegrees(), 120)


def test_minus():
    one = Rotation2d.fromDegrees(70)
    two = Rotation2d.fromDegrees(30)

    assert math.isclose((one - two).getDegrees(), 40)


def test_equality():
    one = Rotation2d.fromDegrees(43.0)
    two = Rotation2d.fromDegrees(43)

    assert one == two


def test_inequality():
    one = Rotation2d.fromDegrees(43)
    two = Rotation2d.fromDegrees(43.5)

    assert one != two
