import math
from dataclasses import dataclass
from typing import overload

from .rotation2d import Rotation2d
from .translation2d import Translation2d
from .twist2d import Twist2d

__all__ = ("Rotation2d", "Translation2d", "Twist2d", "Transform2d", "Pose2d")

_identity_translation = Translation2d()
_zero_rotation = Rotation2d()


@dataclass
class Transform2d:
    """Represents a transformation for a Pose2d."""

    translation: Translation2d
    rotation: Rotation2d

    __slots__ = ("translation", "rotation")

    def __init__(
        self,
        translation: Translation2d = _identity_translation,
        rotation: Rotation2d = _zero_rotation,
    ):
        self.translation = translation
        self.rotation = rotation

    def __mul__(self, other: float) -> "Transform2d":
        """Multiplies the transform by a scalar."""
        if not isinstance(other, (float, int)):
            return NotImplemented
        return Transform2d(self.translation * other, self.rotation * other)

    __rmul__ = __mul__


@dataclass
class Pose2d:
    """Represents a 2d pose containing translational and rotational elements."""

    translation: Translation2d
    rotation: Rotation2d

    __slots__ = ("translation", "rotation")

    @overload
    def __init__(self):
        """Constructs a pose at the origin facing toward the positive X axis."""

    @overload
    def __init__(self, translation: Translation2d, rotation: Rotation2d):
        """Constructs a pose with the specified translation and rotation."""

    @overload
    def __init__(self, x: float, y: float, rotation: Rotation2d):
        """Convenience constructor that takes x and y values directly
        instead of requiring a Translation2d.
        """

    def __init__(self, *args, rotation=None):
        if rotation is None:
            if args:
                *args, rotation = args
            else:
                rotation = _zero_rotation

        if len(args) == 1:
            translation = args[0]
            assert isinstance(translation, Translation2d)
        else:
            translation = Translation2d(*args)

        self.translation = translation
        self.rotation = rotation

    def __add__(self, other: "Transform2d") -> "Pose2d":
        """Transforms the pose by the given transformation.

        ::

            [x_new]   [cos, -sin, 0][transform.x]
            [y_new] = [sin,  cos, 0][transform.y]
            [t_new]   [0,    0,   1][transform.t]

        :param other: The transform to transform the pose by.

        :returns: The transformed pose.
        """
        if not isinstance(other, Transform2d):
            return NotImplemented
        return Pose2d(
            self.translation + other.translation.rotateBy(self.rotation),
            self.rotation + other.rotation,
        )

    def __sub__(self, other: "Pose2d") -> "Transform2d":
        """Returns the Transform2d that maps the other pose to self.

        :param other: The initial pose of the transformation.

        :returns: The transform that maps the other pose to the current pose.
        """
        if not isinstance(other, Pose2d):
            return NotImplemented
        # We are rotating the difference between the translations
        # using a clockwise rotation matrix. This transforms the global
        # delta into a local delta (relative to the initial pose).
        translation = (self.translation - other.translation).rotateBy(-other.rotation)
        rotation = self.rotation - other.rotation
        return Transform2d(translation, rotation)

    def relativeTo(self, other: "Pose2d") -> "Pose2d":
        """Returns the other pose relative to the current pose.

        This function can often be used for trajectory tracking or pose
        stabilization algorithms to get the error between the reference
        and the current pose.

        :param other: The pose that is the origin of the new coordinate
                      frame that the current pose will be converted into.

        :returns: The current pose relative to the new origin pose.
        """
        transform = self - other
        return Pose2d(transform.translation, transform.rotation)

    def exp(self, twist: Twist2d) -> "Pose2d":
        """Obtain a new Pose2d from a (constant curvature) velocity.

        See <https://file.tavsys.net/control/state-space-guide.pdf>
        section on nonlinear pose estimation for derivation.

        The twist is a change in pose in the robot's coordinate frame
        since the previous pose update. When the user runs exp() on the
        previous known field-relative pose with the argument being the
        twist, the user will receive the new field-relative pose.

        "Exp" represents the pose exponential, which is solving a
        differential equation moving the pose forward in time.

        :param twist: The change in pose in the robot's coordinate frame
             since the previous pose update. For example, if a
             non-holonomic robot moves forward 0.01 meters and changes
             angle by 0.5 degrees since the previous pose update, the
             twist would be ``Twist2d(0.01, 0.0, math.radians(0.5))``

        :returns: The new pose of the robot.
        """
        dx = twist.dx
        dy = twist.dy
        dtheta = twist.dtheta

        sin_theta = math.sin(dtheta)
        cos_theta = math.cos(dtheta)

        if abs(dtheta) < 1e-9:
            s = 1.0 - 1 / 6 * dtheta ** 2
            c = 0.5 * dtheta
        else:
            s = sin_theta / dtheta
            c = (1.0 - cos_theta) / dtheta

        transform = Transform2d(
            Translation2d(dx * s - dy * c, dx * c + dy * s),
            Rotation2d(cos_theta, sin_theta),
        )
        return self + transform

    def log(self, end: "Pose2d") -> Twist2d:
        """Returns a Twist2d that maps this pose to the end pose.

        If c = a.log(b), then a.exp(c) = b.

        :param end: The end pose for the transformation.

        :returns: The twist that maps self to end.
        """
        transform = end - self
        dtheta = transform.rotation.value
        half_dtheta = 0.5 * dtheta

        cos_minus_one = transform.rotation.cos - 1.0

        if abs(cos_minus_one) < 1e-9:
            half_theta_by_tan_half_dtheta = 1.0 - 1 / 12 * dtheta ** 2
        else:
            half_theta_by_tan_half_dtheta = (
                -(half_dtheta * transform.rotation.sin) / cos_minus_one
            )

        translation_part = transform.translation.rotateBy(
            Rotation2d(half_theta_by_tan_half_dtheta, -half_dtheta)
        ) * math.hypot(half_theta_by_tan_half_dtheta, half_dtheta)
        return Twist2d(translation_part.x, translation_part.y, dtheta)
