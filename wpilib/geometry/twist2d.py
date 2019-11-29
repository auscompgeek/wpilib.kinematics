import math
from dataclasses import dataclass


@dataclass
class Twist2d:
    """A change in distance along arc since the last pose update.

    We can use ideas from differential calculus to create new Pose2ds
    from a Twist2d and vice versa.

    A Twist can be used to represent a difference between two poses.
    """

    #: Linear "dx" component
    dx: float
    #: Linear "dy" component
    dy: float
    #: Angular "dtheta" component (radians)
    dtheta: float

    __slots__ = ("dx", "dy", "dtheta")

    def __init__(self, dx: float = 0, dy: float = 0, dtheta: float = 0):
        self.dx = dx
        self.dy = dy
        self.dtheta = dtheta

    def __eq__(self, other: "Twist2d") -> bool:
        if not isinstance(other, Twist2d):
            return NotImplemented
        return (
            math.isclose(self.dx, other.dx)
            and math.isclose(self.dy, other.dy)
            and math.isclose(self.dtheta, other.dtheta)
        )
