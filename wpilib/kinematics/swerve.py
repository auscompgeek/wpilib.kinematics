import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from ..geometry import (
    Pose2d,
    Rotation2d,
    Translation2d,
    Twist2d,
    _identity_translation,
    _zero_rotation,
)
from .chassisspeeds import ChassisSpeeds


__all__ = ("SwerveModuleState", "SwerveDriveKinematics", "SwerveDriveOdometry")


@dataclass
class SwerveModuleState:
    """Represents the state of one swerve module."""

    #: Speed of the wheel of the module.
    speed: float
    #: Angle of the module.
    angle: Rotation2d

    __slots__ = ("speed", "angle")

    def __init__(self, speed: float = 0, angle: Rotation2d = _zero_rotation):
        self.speed = speed
        self.angle = angle


class SwerveDriveKinematics:
    """Helper class that converts a chassis velocity (dx, dy, and dtheta components)
    into individual module states (speed and angle).

    The inverse kinematics (converting from a desired chassis velocity to
    individual module states) uses the relative locations of the modules with
    respect to the center of rotation. The center of rotation for inverse
    kinematics is also variable. This means that you can set your set your center
    of rotation in a corner of the robot to perform special evasion maneuvers.

    Forward kinematics (converting an array of module states into the overall
    chassis motion) is performs the exact opposite of what inverse kinematics
    does. Since this is an overdetermined system (more equations than variables),
    we use a least-squares approximation.

    The inverse kinematics: [moduleStates] = [moduleLocations] * [chassisSpeeds]
    We take the Moore-Penrose pseudoinverse of [moduleLocations] and then
    multiply by [moduleStates] to get our chassis speeds.

    Forward kinematics is also used for odometry -- determining the
    position of the robot on the field using encoders and a gyro.
    """

    __slots__ = (
        "modules",
        "num_modules",
        "_inverse_kinematics",
        "forward_kinematics",
        "_prev_cor",
    )

    def __init__(self, *wheels: Translation2d):
        """Constructs a swerve drive kinematics object.

        This takes in a variable number of wheel locations as Translation2ds.
        The order in which you pass in the wheel locations is the same order that
        you will receive the module states when performing inverse kinematics.
        It is also expected that you pass in the module states in the
        same order when calling the forward kinematics methods.

        :param wheels: The locations of the wheels relative to the
                       physical center of the robot.
        """
        assert len(wheels) >= 2, "A swerve drive requires at least two modules"

        inverse_kinematics = np.array([(1, 0, 0), (0, 1, 0)] * len(wheels), dtype=float)
        for i, module in enumerate(wheels):
            inverse_kinematics[2 * i, 2] = -module.y
            inverse_kinematics[2 * i + 1, 2] = module.x

        self.modules = wheels
        self.num_modules = len(wheels)
        self._inverse_kinematics = inverse_kinematics
        self.forward_kinematics = np.linalg.pinv(inverse_kinematics)
        self._prev_cor = _identity_translation

    def toSwerveModuleStates(
        self,
        chassisSpeeds: ChassisSpeeds,
        centerOfRotation: Translation2d = _identity_translation,
    ) -> List[SwerveModuleState]:
        """Performs inverse kinematics to return the module states from a desired
        chassis velocity.

        This method is often used to convert joystick values into
        module speeds and angles.

        This function also supports variable centers of rotation.
        During normal operations, the center of rotation is usually the
        same as the physical center of the robot; therefore, the argument
        is defaulted to that use case.  However, if you wish to change
        the center of rotation for evasive maneuvers, vision alignment,
        or for any other use case, you can do so.

        :param chassisSpeeds: The desired chassis speed.

        :param centerOfRotation: The center of rotation.
            For example, if you set the center of rotation at one corner
            of the robot and provide a chassis speed that only has a
            dtheta component, the robot will rotate around that corner.

        :returns: An array containing the module states.
            Use caution because these module states are not normalized.
            Sometimes, a user input may cause one of the module speeds
            to go above the attainable max velocity.
            Use the :meth:`normalizeWheelSpeeds` function to rectify this issue.
        """
        prev_cor = self._prev_cor
        inverse_kinematics = self._inverse_kinematics
        if prev_cor is not centerOfRotation and prev_cor != centerOfRotation:
            cor_y = centerOfRotation.y
            cor_x = centerOfRotation.x
            for i, module in enumerate(self.modules):
                inverse_kinematics[2 * i, 2] = -module.y + cor_y
                inverse_kinematics[2 * i + 1, 2] = module.x - cor_x
            self._prev_cor = centerOfRotation

        chassis_vel_vec = np.array(chassisSpeeds)
        module_states = inverse_kinematics @ chassis_vel_vec
        return [
            SwerveModuleState(math.hypot(x, y), Rotation2d(x, y))
            for x, y in module_states.reshape(-1, 2)
        ]

    def toChassisSpeeds(self, *wheel_states: SwerveModuleState) -> ChassisSpeeds:
        """Performs forward kinematics to return the resulting chassis state
        from the given module states.

        This method is often used for odometry -- determining the robot's
        position on the field using data from the real-world speed and
        angle of each module on the robot.

        :param wheel_states: The state of the modules (as a SwerveModuleState type)
                             as measured from respective encoders and gyros.
                             The order of the swerve module states should be
                             same as passed into the constructor of this class.

        :returns: The resulting chassis speed.
        """
        num_modules = self.num_modules
        assert (
            len(wheel_states) == num_modules
        ), "Number of modules must be consistent with number of wheel locations."
        module_states_mat = np.array(
            [
                (module.speed * module.angle.cos, module.speed * module.angle.sin)
                for module in wheel_states
            ]
        ).reshape(-1)
        chassis_vel_vec = self.forward_kinematics @ module_states_mat
        return ChassisSpeeds(*chassis_vel_vec)

    @staticmethod
    def normalizeWheelSpeeds(
        moduleStates: List[SwerveModuleState], attainableMaxSpeed: float
    ) -> None:
        """Normalizes the wheel speeds using some max attainable speed.

        Sometimes, after inverse kinematics, the requested speed from
        a/several modules may be above the max attainable speed for
        the driving motor on that module. To fix this issue, one can
        "normalize" all the wheel speeds to make sure that all requested
        module speeds are below the absolute threshold, while maintaining
        the ratio of speeds between modules.

        :param moduleStates: Reference to list of module states.
                             The list will be mutated with the normalized speeds!

        :param attainableMaxSpeed: The absolute max speed that a module can reach.
        """
        real_max_speed = max(module.speed for module in moduleStates)
        if real_max_speed > attainableMaxSpeed:
            factor = attainableMaxSpeed / real_max_speed
            for module in moduleStates:
                # module.speed = module.speed / real_max_speed * attainableMaxSpeed
                module.speed *= factor


class SwerveDriveOdometry:
    """Class for swerve drive odometry.

    Odometry allows you to track the robot's position on the field over
    a course of a match using readings from your swerve drive encoders
    and swerve azimuth encoders.

    Teams can use odometry during the autonomous period for complex
    tasks like path following. Furthermore, odometry can be used for
    latency compensation when using computer-vision systems.
    """

    __slots__ = (
        "kinematics",
        "_pose",
        "_previous_time",
        "_previous_angle",
        "_gyro_offset",
    )

    def __init__(
        self,
        kinematics: SwerveDriveKinematics,
        gyroAngle: Rotation2d,
        initialPose: Optional[Pose2d] = None,
    ):
        if initialPose is None:
            initialPose = Pose2d()

        self.kinematics = kinematics
        self._pose = initialPose
        self._previous_time: Optional[float] = None
        self._previous_angle = initialPose.rotation
        self._gyro_offset = initialPose.rotation - gyroAngle

    def resetPosition(self, pose: Pose2d, gyroAngle: Rotation2d) -> None:
        """Resets the robot's position on the field.

        The gyroscope angle does not need to be reset here on the user's robot
        code. The library automatically takes care of offsetting the gyro angle.

        :param pose: The position on the field that your robot is at.

        :param gyroAngle: The angle reported by the gyroscope.
        """
        self._pose = pose
        self._previous_angle = pose.rotation
        self._gyro_offset = pose.rotation - gyroAngle

    def getPose(self) -> Pose2d:
        """Returns the position of the robot on the field."""
        return self._pose

    def updateWithTime(
        self,
        currentTime: float,
        gyroAngle: Rotation2d,
        *module_states: SwerveModuleState,
    ) -> Pose2d:
        """Updates the robot's position on the field using forward kinematics
        and integration of the pose over time.

        This method takes in the current time as a parameter to calculate
        period (difference between two timestamps). The period is used
        to calculate the change in distance from a velocity. This also
        takes in an angle parameter which is used instead of the angular
        rate that is calculated from forward kinematics.

        :param currentTime: The current time.

        :param gyroAngle: The angle reported by the gyroscope.

        :param module_states: The current state of all swerve modules.
                    Please provide the states in the same order in which
                    you instantiated your SwerveDriveKinematics.

        :returns: The new pose of the robot.
        """
        prev_time = self._previous_time
        delta_time = currentTime - prev_time if prev_time is not None else 0
        self._previous_time = currentTime

        angle = gyroAngle + self._gyro_offset
        dx, dy, _dtheta = self.kinematics.toChassisSpeeds(*module_states)

        new_pose = self._pose.exp(
            Twist2d(
                dx * delta_time,
                dy * delta_time,
                (angle - self._previous_angle).getRadians(),
            )
        )

        self._previous_angle = angle
        self._pose = Pose2d(new_pose.translation, angle)

        return self._pose
