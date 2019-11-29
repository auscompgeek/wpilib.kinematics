import math

from wpilib.geometry import Pose2d, Rotation2d, Twist2d


def test_straight():
    straight = Twist2d(5, 0, 0)
    straight_pose = Pose2d().exp(straight)

    assert straight_pose.translation.x == 5
    assert straight_pose.translation.y == 0
    assert straight_pose.rotation.getRadians() == 0


def test_quarter_circle():
    quarter_circle = Twist2d(5 / 2 * math.pi, 0, math.pi / 2)
    quarter_circle_pose = Pose2d().exp(quarter_circle)

    assert math.isclose(quarter_circle_pose.translation.x, 5)
    assert math.isclose(quarter_circle_pose.translation.y, 5)
    assert math.isclose(quarter_circle_pose.rotation.getDegrees(), 90)


def test_diagonal_no_dtheta():
    diagonal = Twist2d(2, 2, 0)
    diagonal_pose = Pose2d().exp(diagonal)

    assert math.isclose(diagonal_pose.translation.x, 2)
    assert math.isclose(diagonal_pose.translation.y, 2)
    assert math.isclose(diagonal_pose.rotation.getDegrees(), 0)


def test_equality():
    one = Twist2d(5, 1, 3.0)
    two = Twist2d(5.0, 1.0, 3)

    assert one == two


def test_inequality():
    one = Twist2d(5, 1, 3)
    two = Twist2d(5.0, 1.2, 3.0)

    assert one != two


def test_pose2d_log():
    end = Pose2d(5, 5, Rotation2d.fromDegrees(90))
    start = Pose2d()

    twist = start.log(end)

    assert math.isclose(twist.dx, 5 / 2 * math.pi)
    assert math.isclose(twist.dy, 0)
    assert math.isclose(twist.dtheta, math.pi / 2)
