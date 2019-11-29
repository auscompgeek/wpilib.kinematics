import math

import pytest

from wpilib.geometry import Pose2d, Rotation2d, Translation2d
from wpilib.kinematics.swerve import (
    SwerveDriveKinematics,
    SwerveDriveOdometry,
    SwerveModuleState,
)

FL = Translation2d(+12, +12)
FR = Translation2d(+12, -12)
BL = Translation2d(-12, +12)
BR = Translation2d(-12, -12)

kinematics = SwerveDriveKinematics(FL, FR, BL, BR)


@pytest.fixture
def odometry():
    return SwerveDriveOdometry(kinematics, Rotation2d())


def test_two_iterations(odometry: SwerveDriveOdometry):
    state = SwerveModuleState(5, Rotation2d())

    odometry.updateWithTime(
        0,
        Rotation2d(),
        SwerveModuleState(),
        SwerveModuleState(),
        SwerveModuleState(),
        SwerveModuleState(),
    )
    pose = odometry.updateWithTime(0.1, Rotation2d(), state, state, state, state)

    assert math.isclose(0.5, pose.translation.x)
    assert math.isclose(0, pose.translation.y)
    assert math.isclose(0, pose.rotation.getRadians())


def test_90deg_turn(odometry: SwerveDriveOdometry):
    wheel_speeds = (
        SwerveModuleState(18.85, Rotation2d.fromDegrees(90.0)),
        SwerveModuleState(42.15, Rotation2d.fromDegrees(26.565)),
        SwerveModuleState(18.85, Rotation2d.fromDegrees(-90)),
        SwerveModuleState(42.15, Rotation2d.fromDegrees(-26.565)),
    )
    zero = SwerveModuleState()

    odometry.updateWithTime(0, Rotation2d(), zero, zero, zero, zero)
    pose = odometry.updateWithTime(1, Rotation2d.fromDegrees(90.0), *wheel_speeds)

    assert math.isclose(pose.translation.x, 12.0, abs_tol=0.01)
    assert math.isclose(pose.translation.y, 12.0, abs_tol=0.01)
    assert math.isclose(pose.rotation.getDegrees(), 90.0)


def test_gyro_angle_reset(odometry: SwerveDriveOdometry):
    gyro = Rotation2d.fromDegrees(90)
    odometry.resetPosition(Pose2d(), gyro)

    state = SwerveModuleState()
    odometry.updateWithTime(0, gyro, state, state, state, state)

    state = SwerveModuleState(5, Rotation2d())
    pose = odometry.updateWithTime(0.1, gyro, state, state, state, state)

    assert math.isclose(pose.translation.x, 0.5)
    assert math.isclose(pose.translation.y, 0)
    assert math.isclose(pose.rotation.getRadians(), 0)
