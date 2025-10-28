# ================================
# file: code/sim/robot_sim.py
# ================================
from __future__ import annotations
from typing import Tuple
import math, random, time
import numpy as np
from core.types import Pose2D, LaserScan
from core.config import (
    MAP_RES, LIDAR_RANGE, LIDAR_BEAMS, LIDAR_FOV_DEG,
    V_MAX, W_MAX, ACC_MAX, WACC_MAX, ROBOT_RADIUS
)
from .maze_map import MazeMap

class RobotSim:
    """Differential-drive robot simulator with idealized odom + raycast LiDAR.
    This class is a SIMULATION INTERFACE. In real-robot mode, replace usage
    with an object that provides the same methods (get_lidar_scan, get_odometry,
    apply_control, update).
    Thread-safety: assume single-threaded calls from main loop.
    """
    def __init__(self, maze: MazeMap) -> None:
        self.maze = maze
        self.pose = Pose2D(0.0, 0.0, 0.0)
        self.v = 0.0
        self.w = 0.0
        self._last_odom_pose = self.pose.copy()
        self._last_update_t = time.time()

    def set_pose(self, pose: Pose2D) -> None:
        self.pose = pose.copy()
        self._last_odom_pose = self.pose.copy()
        self._last_update_t = time.time()

    def apply_control(self, v: float, w: float) -> None:
        """Set control commands. Saturation is applied here.
        # For REAL robot: send via BLE/TCP in appio layer with same signature.
        """
        self.v = max(-V_MAX, min(V_MAX, float(v)))
        self.w = max(-W_MAX, min(W_MAX, float(w)))

    def update(self, dt: float, linear_vel: float, angular_vel: float) -> None:
        """Update robot pose based on velocity commands.
        
        Args:
            dt: Time step in seconds
            linear_vel: Linear velocity in m/s
            angular_vel: Angular velocity in rad/s
        """
        # Store the current pose before update for odometry calculation
        old_pose = self.pose.copy()
        
        # Apply velocity limits
        linear_vel = max(-V_MAX, min(V_MAX, float(linear_vel)))
        angular_vel = max(-W_MAX, min(W_MAX, float(angular_vel)))
        
        # Calculate new pose using differential drive model
        if abs(angular_vel) < 1e-6:  # Straight line motion
            new_x = self.pose.x + linear_vel * math.cos(self.pose.theta) * dt
            new_y = self.pose.y + linear_vel * math.sin(self.pose.theta) * dt
            new_theta = self.pose.theta
        else:  # Curved motion
            # Differential drive kinematics
            new_theta = self.pose.theta + angular_vel * dt
            radius = linear_vel / angular_vel
            new_x = self.pose.x + radius * (math.sin(new_theta) - math.sin(self.pose.theta))
            new_y = self.pose.y - radius * (math.cos(new_theta) - math.cos(self.pose.theta))
        
        # Check boundary collision (from config)
        # English: Use center-based boundary check with small margin
        # This prevents robot from getting stuck at boundaries while maintaining safety
        from core.config import WORLD_SIZE, ROBOT_RADIUS
        
        BOUNDARY_MARGIN = 0.05  # 5cm safety margin for robot center
        
        # Check if robot center would be outside safe boundaries
        if (new_x < BOUNDARY_MARGIN or new_x > WORLD_SIZE - BOUNDARY_MARGIN or
            new_y < BOUNDARY_MARGIN or new_y > WORLD_SIZE - BOUNDARY_MARGIN):
            # Boundary collision - don't update position, only allow rotation
            print(f"[BOUNDARY] Collision at ({new_x:.3f}, {new_y:.3f}), " +
                  f"bounds=[{BOUNDARY_MARGIN:.2f}, {WORLD_SIZE-BOUNDARY_MARGIN:.2f}]")
            new_x = self.pose.x
            new_y = self.pose.y
        
        # Check obstacle collision - use physical radius for simulation
        # Physical radius is a hardware constant, NOT the planning radius (which is 0)
        PHYSICAL_RADIUS = 0.10  # 10cm physical robot radius (hardware specification)
        inflate = max(0, int(PHYSICAL_RADIUS / MAP_RES))  # ~10 pixels
        if self.maze.is_obstacle_world(new_x, new_y, inflate_cells=inflate):
            print(f"[DEBUG] Obstacle collision detected at ({new_x:.3f}, {new_y:.3f}), blocking position update")
            new_x = self.pose.x
            new_y = self.pose.y
        
        # Update pose
        self.pose.x = new_x
        self.pose.y = new_y
        self.pose.theta = new_theta
        
        # Normalize angle to [-pi, pi]
        self.pose.theta = math.atan2(math.sin(self.pose.theta), math.cos(self.pose.theta))
        
        # Update internal velocity state for consistency
        self.v = linear_vel
        self.w = angular_vel
        
        # Update odometry tracking
        self._last_odom_pose = old_pose
        self._last_update_t = time.time()

    def get_odometry(self) -> Tuple[float, float, float]:
        """Return (d_trans, d_theta, dt) since last call.
        Sim odom is noiseless; add small noise to emulate drift if needed.
        """
        now = time.time()
        dt = max(1e-3, now - self._last_update_t)
        dx = self.pose.x - self._last_odom_pose.x
        dy = self.pose.y - self._last_odom_pose.y
        d_trans = math.hypot(dx, dy)
        d_theta = self.pose.theta - self._last_odom_pose.theta
        self._last_odom_pose = self.pose.copy()
        self._last_update_t = now
        return (d_trans, d_theta, dt)

    def get_lidar_scan(self) -> LaserScan:
        """Raycast LiDAR in the maze grid. Simple step sampling at MAP_RES.
        Adds small Gaussian noise and 1% random dropout to mimic reality.
        """
        beams = LIDAR_BEAMS
        angle_min = -math.radians(LIDAR_FOV_DEG) / 2.0
        ang_inc = math.radians(LIDAR_FOV_DEG) / float(max(1, beams-1))
        ranges = []
        for k in range(beams):
            a = self.pose.theta + angle_min + k * ang_inc
            r = self._raycast(a)
            # noise & dropout
            r = max(0.0, min(LIDAR_RANGE, r + random.gauss(0.0, 0.02)))
            if random.random() < 0.01:
                r = LIDAR_RANGE
            ranges.append(r)
        return LaserScan(angle_min, ang_inc, ranges, robot_pose=self.pose.copy(), t=time.time())

    # robot_sim.py  修改 _raycast 函数（仅此处）
    def _raycast(self, ang: float) -> float:
        # English: use resolution-aware step and treat world border as a hit
        from core.config import SLAM_RESOLUTION, WORLD_SIZE, LIDAR_RANGE

        # step <= 0.5 cell, but not too small
        step = max(0.5 * SLAM_RESOLUTION, 0.005)  # e.g., 5 mm
        d = 0.0
        cos_a, sin_a = math.cos(ang), math.sin(ang)

        while d < LIDAR_RANGE:
            x = self.pose.x + d * cos_a
            y = self.pose.y + d * sin_a

            # treat going outside world as a wall hit
            if not (0.0 <= x <= WORLD_SIZE and 0.0 <= y <= WORLD_SIZE):
                return d

            # real obstacle in world coords
            if self.maze.is_obstacle_world(x, y, inflate_cells=0):
                return d

            d += step

        return LIDAR_RANGE
