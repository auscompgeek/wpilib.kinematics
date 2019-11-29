import math
from dataclasses import dataclass
from typing import overload


@dataclass(frozen=True)
class Rotation2d:
    """A rotation in a 2d coordinate frame represented a point on the unit circle."""

    #: The value of the rotation in radians.
    value: float
    #: The cosine of the rotation.
    cos: float
    #: The sine of the rotation.
    sin: float

    __slots__ = ("value", "cos", "sin")

    @overload
    def __init__(self, __value: float = 0):
        """Constructs a Rotation2d with the given radian value."""

    @overload
    def __init__(self, __x: float, __y: float):
        """Constructs a Rotation2d with the given cosine and sine components.

        The x and y don't have to be normalised.
        """

    def __init__(self, *args: float):
        """Constructs a Rotation2d.

        This can either be called with zero or one arguments:

        :param float value: The value in radians (default 0).

        ... or with two arguments:

        :param float x: The x component or cosine of the rotation.
        :param float y: The y component or sine of the rotation.

        In this case the x and y do not need to be normalised.

        """
        value: float
        cos: float
        sin: float

        if not args:
            value = 0
            cos = 1
            sin = 0
        elif len(args) == 1:
            value = args[0]
            cos = math.cos(value)
            sin = math.sin(value)
        else:
            x, y = args
            magnitude = math.hypot(x, y)
            if magnitude > 1e-6:
                sin = y / magnitude
                cos = x / magnitude
            else:
                sin = 0
                cos = 1
            value = math.atan2(sin, cos)

        object.__setattr__(self, "value", value)
        object.__setattr__(self, "cos", cos)
        object.__setattr__(self, "sin", sin)

    @classmethod
    def fromDegrees(cls, degrees: float) -> "Rotation2d":
        """Creates a Rotation2d with the given degrees value."""
        return cls(math.radians(degrees))

    def __repr__(self) -> str:
        return f"Rotation2d({self.value})"

    def __add__(self, other: "Rotation2d") -> "Rotation2d":
        """Adds two rotations together, with the result bounded between -pi and pi."""
        if not isinstance(other, Rotation2d):
            return NotImplemented
        cos_a = self.cos
        cos_b = other.cos
        sin_a = self.sin
        sin_b = other.sin
        return Rotation2d(cos_a * cos_b - sin_a * sin_b, cos_a * sin_b + sin_a * cos_b)

    def __sub__(self, other: "Rotation2d") -> "Rotation2d":
        """Subtracts the other rotation from self."""
        if not isinstance(other, Rotation2d):
            return NotImplemented
        return self + -other

    def __neg__(self) -> "Rotation2d":
        """Takes the inverse of the current rotation.

        This is simply the negative of the current angular value.
        """
        return Rotation2d(-self.value)

    def __mul__(self, other: float) -> "Rotation2d":
        """Multiplies the current rotation by a scalar."""
        if not isinstance(other, (float, int)):
            return NotImplemented
        return Rotation2d(self.value * other)

    __rmul__ = __mul__

    def __eq__(self, other: "Rotation2d") -> bool:
        if not isinstance(other, Rotation2d):
            return NotImplemented
        return math.isclose(self.value, other.value)

    def getRadians(self) -> float:
        """Returns the value of the rotation in radians."""
        return self.value

    def getDegrees(self) -> float:
        """Returns the value of the rotation in degrees."""
        return math.degrees(self.value)

    def tan(self) -> float:
        """Returns the tangent of the rotation."""
        return self.sin / self.cos
