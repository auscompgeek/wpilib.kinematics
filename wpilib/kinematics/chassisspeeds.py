import typing

from ..geometry import Rotation2d


class ChassisSpeeds(typing.NamedTuple):
    """Represents the speed of a robot chassis.

    Although this struct contains similar members compared to a
    Twist2d, they do NOT represent the same thing.  Whereas a Twist2d
    represents a change in pose w.r.t. to the robot frame of reference,
    this ChassisSpeeds struct represents a velocity w.r.t. to the robot
    frame of reference.

    A strictly non-holonomic drivetrain, such as a differential drive, should
    never have a dy component because it can never move sideways. Holonomic
    drivetrains such as swerve and mecanum will often have all three components.
    """

    #: Represents forward velocity w.r.t the robot frame of reference. (Fwd is +)
    vx: float = 0
    #: Represents strafe velocity w.r.t the robot frame of reference. (Left is +)
    vy: float = 0
    #: Represents the angular velocity of the robot frame. (CCW is +)
    omega: float = 0

    @classmethod
    def fromFieldRelativeSpeeds(
        cls, vx: float, vy: float, omega: float, robotAngle: Rotation2d
    ) -> "ChassisSpeeds":
        """Converts a user provided field-relative set of speeds into a robot-relative
        ChassisSpeeds object.

        :param vx: The component of speed in the x direction relative to the field.
                   Positive x is away from your alliance wall.

        :param vy: The component of speed in the y direction relative to the field.
                   Positive y is to your left when standing behind the alliance wall.

        :param omega: The angular rate of the robot.

        :param robotAngle: The angle of the robot as measured by a gyroscope.
            The robot's angle is considered to be zero when it is facing
            directly away from your alliance station wall.
            Remember that this should be CCW positive.

        :returns: Object representing the speeds in the robot's frame of reference.
        """
        return cls(
            vx * robotAngle.cos + vy * robotAngle.sin,
            -vx * robotAngle.sin + vy * robotAngle.cos,
            omega,
        )
