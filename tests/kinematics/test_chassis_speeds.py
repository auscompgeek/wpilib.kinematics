import math

import pytest

from wpilib.geometry import Rotation2d
from wpilib.kinematics import ChassisSpeeds


def test_field_relative_construction():
    chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        1, 0, 0.5, Rotation2d.fromDegrees(-90)
    )

    assert chassis_speeds.vx == pytest.approx(0)
    assert math.isclose(1, chassis_speeds.vy)
    assert math.isclose(0.5, chassis_speeds.omega)
