import math

import pytest

from wpilib.geometry import Translation2d, Rotation2d
from wpilib.kinematics import ChassisSpeeds
from wpilib.kinematics.swerve import SwerveDriveKinematics, SwerveModuleState

FL = Translation2d(+12, +12)
FR = Translation2d(+12, -12)
BL = Translation2d(-12, +12)
BR = Translation2d(-12, -12)

kinematics = SwerveDriveKinematics(FL, FR, BL, BR)


def test_straight_line_inverse():
    speeds = ChassisSpeeds(5, 0, 0)
    fl, fr, bl, br = kinematics.toSwerveModuleStates(speeds)

    assert math.isclose(fl.speed, 5)
    assert math.isclose(fr.speed, 5)
    assert math.isclose(bl.speed, 5)
    assert math.isclose(br.speed, 5)

    assert math.isclose(fl.angle.getRadians(), 0)
    assert math.isclose(fr.angle.getRadians(), 0)
    assert math.isclose(bl.angle.getRadians(), 0)
    assert math.isclose(br.angle.getRadians(), 0)


def test_straight_line_forward():
    state = SwerveModuleState(5, Rotation2d())
    chassisSpeeds = kinematics.toChassisSpeeds(state, state, state, state)

    assert math.isclose(chassisSpeeds.vx, 5)
    assert math.isclose(chassisSpeeds.vy, 0)
    assert chassisSpeeds.omega == pytest.approx(0)


def test_straight_strafe_inverse():
    speeds = ChassisSpeeds(0, 5, 0)
    fl, fr, bl, br = kinematics.toSwerveModuleStates(speeds)

    assert math.isclose(fl.speed, 5)
    assert math.isclose(fr.speed, 5)
    assert math.isclose(bl.speed, 5)
    assert math.isclose(br.speed, 5)

    assert math.isclose(fl.angle.getDegrees(), 90)
    assert math.isclose(fr.angle.getDegrees(), 90)
    assert math.isclose(bl.angle.getDegrees(), 90)
    assert math.isclose(br.angle.getDegrees(), 90)


def test_straight_strafe_forward():
    state = SwerveModuleState(5, Rotation2d.fromDegrees(90))
    chassisSpeeds = kinematics.toChassisSpeeds(state, state, state, state)

    assert chassisSpeeds.vx == pytest.approx(0)
    assert math.isclose(chassisSpeeds.vy, 5)
    assert chassisSpeeds.omega == pytest.approx(0)


def test_turn_in_place_inverse_kinematics():
    speeds = ChassisSpeeds(0, 0, math.tau)
    fl, fr, bl, br = kinematics.toSwerveModuleStates(speeds)

    assert math.isclose(fl.speed, 106.63, abs_tol=0.01)
    assert math.isclose(fr.speed, 106.63, abs_tol=0.01)
    assert math.isclose(bl.speed, 106.63, abs_tol=0.01)
    assert math.isclose(br.speed, 106.63, abs_tol=0.01)

    assert math.isclose(fl.angle.getDegrees(), 135)
    assert math.isclose(fr.angle.getDegrees(), 45)
    assert math.isclose(bl.angle.getDegrees(), -135)
    assert math.isclose(br.angle.getDegrees(), -45)


def test_turn_in_place_forward_kinematics():
    fl = SwerveModuleState(106.629, Rotation2d.fromDegrees(135))
    fr = SwerveModuleState(106.629, Rotation2d.fromDegrees(45))
    bl = SwerveModuleState(106.629, Rotation2d.fromDegrees(-135))
    br = SwerveModuleState(106.629, Rotation2d.fromDegrees(-45))

    chassisSpeeds = kinematics.toChassisSpeeds(fl, fr, bl, br)

    assert chassisSpeeds.vx == pytest.approx(0)
    assert chassisSpeeds.vy == pytest.approx(0)
    assert math.isclose(chassisSpeeds.omega, math.tau, abs_tol=0.01)


def test_off_centre_cor_rotation_inverse_kinematics():
    speeds = ChassisSpeeds(0, 0, math.tau)
    fl, fr, bl, br = kinematics.toSwerveModuleStates(speeds, FL)

    assert math.isclose(fl.speed, 0)
    assert math.isclose(fr.speed, 150.796, abs_tol=0.001)
    assert math.isclose(bl.speed, 150.796, abs_tol=0.001)
    assert math.isclose(br.speed, 213.258, abs_tol=0.001)

    assert math.isclose(fl.angle.getDegrees(), 0)
    assert math.isclose(fr.angle.getDegrees(), 0)
    assert math.isclose(bl.angle.getDegrees(), -90)
    assert math.isclose(br.angle.getDegrees(), -45)


def test_off_centre_cor_rotation_forward_kinematics():
    fl = SwerveModuleState(0, Rotation2d(0))
    fr = SwerveModuleState(150.796, Rotation2d(0))
    bl = SwerveModuleState(150.796, Rotation2d.fromDegrees(-90))
    br = SwerveModuleState(213.258, Rotation2d.fromDegrees(-45))

    chassis_speeds = kinematics.toChassisSpeeds(fl, fr, bl, br)

    assert math.isclose(chassis_speeds.vx, 75.398, abs_tol=0.001)
    assert math.isclose(chassis_speeds.vy, -75.398, abs_tol=0.001)
    assert math.isclose(chassis_speeds.omega, math.tau, abs_tol=0.001)


def test_off_centre_cor_rotation_and_translation_inverse_kinematics():
    speeds = ChassisSpeeds(0, 3.0, 1.5)
    fl, fr, bl, br = kinematics.toSwerveModuleStates(speeds, Translation2d(24, 0))

    assert math.isclose(fl.speed, 23.43, abs_tol=0.01)
    assert math.isclose(fr.speed, 23.43, abs_tol=0.01)
    assert math.isclose(bl.speed, 54.08, abs_tol=0.01)
    assert math.isclose(br.speed, 54.08, abs_tol=0.01)

    assert math.isclose(fl.angle.getDegrees(), -140.19, abs_tol=0.01)
    assert math.isclose(fr.angle.getDegrees(), -39.81, abs_tol=0.01)
    assert math.isclose(bl.angle.getDegrees(), -109.44, abs_tol=0.01)
    assert math.isclose(br.angle.getDegrees(), -70.56, abs_tol=0.01)


def test_off_centre_cor_rotation_and_translation_forward_kinematics():
    fl = SwerveModuleState(23.43, Rotation2d.fromDegrees(-140.19))
    fr = SwerveModuleState(23.43, Rotation2d.fromDegrees(-39.81))
    bl = SwerveModuleState(54.08, Rotation2d.fromDegrees(-109.44))
    br = SwerveModuleState(54.08, Rotation2d.fromDegrees(-70.56))

    chassis_speeds = kinematics.toChassisSpeeds(fl, fr, bl, br)

    assert math.isclose(chassis_speeds.vx, 0.0, abs_tol=0.1)
    assert math.isclose(chassis_speeds.vy, -33.0, abs_tol=0.1)
    assert math.isclose(chassis_speeds.omega, 1.5, abs_tol=0.1)


def test_normalize():
    fl = SwerveModuleState(5, Rotation2d())
    fr = SwerveModuleState(6, Rotation2d())
    bl = SwerveModuleState(4, Rotation2d())
    br = SwerveModuleState(7, Rotation2d())

    states = [fl, fr, bl, br]
    SwerveDriveKinematics.normalizeWheelSpeeds(states, 5.5)

    factor = 5.5 / 7

    assert math.isclose(states[0].speed, 5 * factor)
    assert math.isclose(states[1].speed, 6 * factor)
    assert math.isclose(states[2].speed, 4 * factor)
    assert math.isclose(states[3].speed, 7 * factor)
