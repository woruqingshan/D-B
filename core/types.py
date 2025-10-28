# ================================
# file: code/core/types.py
# ================================
"""Shared data structures for pose and LiDAR scans.
Use minimal typing: Tuple/Optional/Dict/Sequence only.
"""
from __future__ import annotations
from typing import Sequence, Optional
import math


class Pose2D:
    """2D pose of the robot in world coordinates.


    Attributes
    -----------
    x, y : meters
    theta : radians
    """
    __slots__ = ("x", "y", "theta")


    def __init__(self, x: float, y: float, theta: float) -> None:
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)


    def copy(self) -> "Pose2D":
        return Pose2D(self.x, self.y, self.theta)


    def distance_to(self, other: "Pose2D") -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        return math.hypot(dx, dy)




class LaserScan:
    """LiDAR scan container.


    Parameters
    ----------
    angle_min : float
    Starting angle (radians) relative to robot +x axis.
    angle_increment : float
    Angle increment per beam (radians).
    ranges : Sequence[float]
    Range array in meters; length equals beam count.
    robot_pose : Optional[Pose2D]
    Pose when scan was taken (optional).
    t : Optional[float]
    Timestamp seconds.
    """
    __slots__ = ("angle_min", "angle_increment", "ranges", "robot_pose", "t")


    def __init__(self, angle_min: float, angle_increment: float,
        ranges: Sequence[float], robot_pose: Optional[Pose2D] = None,
        t: Optional[float] = None) -> None:
        self.angle_min = float(angle_min)
        self.angle_increment = float(angle_increment)
        self.ranges = list(ranges)
        self.robot_pose = robot_pose
        self.t = t


    def beam_count(self) -> int:
        return len(self.ranges)