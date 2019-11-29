import math
from dataclasses import dataclass

from .rotation2d import Rotation2d


@dataclass(frozen=True)
class Translation2d:
    """Represents a translation in 2d space.

    This object can be used to represent a point or a vector.

    This assumes that you are using conventional mathematical axes.
    When the robot is placed on the origin, facing toward the X direction,
    moving forward increases the X, whereas moving to the left increases the Y.
    """

    #: The X component of the translation.
    x: float
    #: The Y component of the translation.
    y: float

    __slots__ = ("x", "y")

    def __init__(self, x: float = 0, y: float = 0):
        object.__setattr__(self, "x", x)
        object.__setattr__(self, "y", y)

    def getDistance(self, other: "Translation2d") -> float:
        """Calculates the distance between two translations in 2d space."""
        return math.hypot(other.x - self.x, other.y - self.y)

    def norm(self) -> float:
        """Returns the norm, or distance from the origin to the translation."""
        return math.hypot(self.x, self.y)

    def __abs__(self) -> float:
        """Returns the norm of this translation.

        This is equivalent to the norm method.
        """
        return self.norm()

    def rotateBy(self, other: Rotation2d) -> "Translation2d":
        """Applies a rotation to the translation in 2d space.

        This multiplies the translation vector by a counterclockwise
        rotation matrix of the given angle::

            [x_new]   [other.cos, -other.sin][x]
            [y_new] = [other.sin,  other.cos][y]

        For example, rotating a Translation2d(2, 0) by 90 degrees
        will return a Translation2d(0, 2).

        :param other: The rotation to rotate the translation by.

        :returns: The new rotated translation.
        """
        x = self.x
        y = self.y
        cos = other.cos
        sin = other.sin
        return Translation2d(x * cos - y * sin, x * sin + y * cos)

    def __add__(self, other: "Translation2d") -> "Translation2d":
        """Adds two translations in 2d space."""
        if not isinstance(other, Translation2d):
            return NotImplemented
        return Translation2d(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Translation2d") -> "Translation2d":
        """Subtracts the other translation from self."""
        if not isinstance(other, Translation2d):
            return NotImplemented
        return Translation2d(self.x - other.x, self.y - other.y)

    def __neg__(self) -> "Translation2d":
        """Takes the inverse of the translation."""
        return Translation2d(-self.x, -self.y)

    def __mul__(self, other: float) -> "Translation2d":
        """Multiplies the translation by a scalar."""
        if not isinstance(other, (float, int)):
            return NotImplemented
        return Translation2d(self.x * other, self.y * other)

    __rmul__ = __mul__

    def __truediv__(self, other: float) -> "Translation2d":
        """Divides the translation by a scalar."""
        if not isinstance(other, (float, int)):
            return NotImplemented
        return Translation2d(self.x / other, self.y / other)

    def __eq__(self, other: "Translation2d") -> bool:
        if not isinstance(other, Translation2d):
            return NotImplemented
        return math.isclose(self.x, other.x) and math.isclose(self.y, other.y)
