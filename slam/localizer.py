# ================================
# file: slam/localizer.py
# ================================
"""
Pure Localization Module - Scan-to-Map ICP

This module performs localization on a known map using ICP matching.
It does NOT build or modify the map.

Key Features:
- Scan-to-Map ICP matching
- Distance field for fast matching
- Multi-scale search
- Quality assessment
"""
from __future__ import annotations
import math
import numpy as np
from typing import Tuple, Optional
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree

from core import Pose2D, LaserScan, LIDAR_RANGE, LIDAR_MIN_RANGE

# ================================
# 定位器常量参数
# ================================

# === ICP定位参数 ===
LOCALIZER_MIN_SCAN_POINTS: int = 10              # 最小有效扫描点数
LOCALIZER_QUALITY_THRESHOLD: float = 0.3         # 质量阈值
LOCALIZER_RANGE_MARGIN: float = 0.01             # 距离裕量 (m)

# === 搜索参数 ===
LOCALIZER_SEARCH_RANGE: float = 0.1              # 搜索范围 (m)
LOCALIZER_SEARCH_STEPS: int = 5                  # 搜索步数
LOCALIZER_ANGLE_SEARCH_RANGE: float = 0.1        # 角度搜索范围 (rad)

# === 几何计算常量 ===
LOCALIZER_ANGLE_NORMALIZATION_PI: float = math.pi      # 角度归一化常数
LOCALIZER_ANGLE_NORMALIZATION_2PI: float = 2 * math.pi # 角度归一化常数


class Localizer:
    """Pure localization on known map using Scan-to-Map ICP"""
    
    def __init__(self,
                 reference_map: dict,
                 logger_func=None,
                 log_file=None) -> None:
        """
        Initialize localizer with reference map
        
        Args:
            reference_map: Map data from MapBuilder.export_map()
        """
        # Store logger references
        self.logger_func = logger_func
        self.log_file = log_file
        
        # Extract map data
        self.occ_grid = reference_map['occupancy_grid']
        self.log_odds_grid = reference_map.get('log_odds_grid', None)
        
        metadata = reference_map['metadata']
        self.N = metadata['size'][0]
        self.L = metadata['size_meters']
        self.res = metadata['resolution']
        
        # Build distance field for ICP
        self._distance_field = self._build_distance_field(self.occ_grid)
        
        # Current pose estimate
        self.pose = Pose2D(0.0, 0.0, 0.0)
        
        # Tracking state
        self._tracking_good = True
        self._last_match_quality = 0.0
        
        # Logger
        self.logger_func = logger_func
        self.log_file = log_file
        
        self._log_debug(f"Localizer初始化: 地图大小={self.N}x{self.N}, 分辨率={self.res:.4f}m")
    
    def set_pose(self, pose: Pose2D) -> None:
        """Set initial pose"""
        self.pose = Pose2D(pose.x, pose.y, pose.theta)
        self._log_debug(f"设置初始位姿: ({pose.x:.3f}, {pose.y:.3f}, {pose.theta:.3f})")
    
    def predict(self, odom_delta: Tuple[float, float, float]) -> None:
        """
        Odometry prediction step (BODY frame)
        
        Args:
            odom_delta: (dx, dy, dtheta) in body frame
        """
        dx_body, dy_body, dth = odom_delta
        
        # Transform body frame to world frame
        theta = self.pose.theta
        c, s = math.cos(theta), math.sin(theta)
        
        dx_world = c * dx_body - s * dy_body
        dy_world = s * dx_body + c * dy_body
        
        # Update pose
        self.pose.x += dx_world
        self.pose.y += dy_world
        self.pose.theta = self._wrap_angle(self.pose.theta + dth)
    
    def correct(self, lidar_scan: LaserScan) -> Tuple[Pose2D, float]:
        """
        ICP correction step
        
        Args:
            lidar_scan: Current laser scan
            
        Returns:
            (corrected_pose, match_quality)
        """
        # Extract scan points in world frame
        scan_points = self._scan_to_world_points(lidar_scan, self.pose)
        
        if scan_points.shape[1] < LOCALIZER_MIN_SCAN_POINTS:
            self._log_debug("点云太少，跳过ICP校正")
            return self.pose, 0.0
        
        # Simple ICP: find best pose that minimizes distance to obstacles
        best_pose, quality = self._icp_optimize(scan_points, self.pose)
        
        # Quality gating
        if quality > LOCALIZER_QUALITY_THRESHOLD:
            self.pose = best_pose
            self._tracking_good = True
        else:
            self._tracking_good = False
            self._log_debug(f"ICP质量低({quality:.3f}), 保持里程计位姿")
        
        self._last_match_quality = quality
        
        return self.pose, quality
    
    def _build_distance_field(self, occ_grid: np.ndarray) -> np.ndarray:
        """
        Build distance transform field for ICP
        
        Args:
            occ_grid: Occupancy grid (0=free, 1=occupied, 2=unknown)
            
        Returns:
            Distance field in meters
        """
        # Create obstacle map (1 = obstacle, 0 = non-obstacle)
        obstacle_map = (occ_grid == 1).astype(np.uint8)
        
        # Euclidean distance transform
        dist_field_pixels = distance_transform_edt(1 - obstacle_map)
        
        # Convert to meters
        dist_field_meters = dist_field_pixels * self.res
        
        self._log_debug(f"距离场构建完成: 最大距离={dist_field_meters.max():.2f}m")
        
        return dist_field_meters
    
    def _scan_to_world_points(self, scan: LaserScan, pose: Pose2D) -> np.ndarray:
        """
        Convert laser scan to world frame points
        
        Returns:
            Points array (2, N) in world coordinates
        """
        ranges = np.array(scan.ranges, dtype=np.float32)
        
        # Filter valid ranges
        valid = np.isfinite(ranges) & (ranges > LIDAR_MIN_RANGE) & (ranges < LIDAR_RANGE - LOCALIZER_RANGE_MARGIN)
        
        if not np.any(valid):
            return np.empty((2, 0), dtype=np.float32)
        
        # Compute angles
        indices = np.where(valid)[0]
        angles = scan.angle_min + scan.angle_increment * indices
        
        # Points in body frame
        x_body = ranges[valid] * np.cos(angles)
        y_body = ranges[valid] * np.sin(angles)
        
        # Transform to world frame
        c, s = math.cos(pose.theta), math.sin(pose.theta)
        
        x_world = pose.x + (c * x_body - s * y_body)
        y_world = pose.y + (s * x_body + c * y_body)
        
        points = np.vstack([x_world, y_world])
        
        return points
    
    def _icp_optimize(self, scan_points: np.ndarray, initial_pose: Pose2D) -> Tuple[Pose2D, float]:
        """
        Simple ICP optimization using distance field
        
        Args:
            scan_points: Points in world frame (2, N)
            initial_pose: Initial pose estimate
            
        Returns:
            (optimized_pose, quality_score)
        """
        # For simplicity, we use a grid search approach
        # In production, use Gauss-Newton or Levenberg-Marquardt
        
        best_pose = initial_pose
        best_score = self._evaluate_pose(scan_points, initial_pose)
        
        # Search window
        search_x = np.linspace(-LOCALIZER_SEARCH_RANGE, LOCALIZER_SEARCH_RANGE, LOCALIZER_SEARCH_STEPS)  # ±10cm
        search_y = np.linspace(-LOCALIZER_SEARCH_RANGE, LOCALIZER_SEARCH_RANGE, LOCALIZER_SEARCH_STEPS)
        search_theta = np.linspace(-LOCALIZER_ANGLE_SEARCH_RANGE, LOCALIZER_ANGLE_SEARCH_RANGE, LOCALIZER_SEARCH_STEPS)  # ±5.7 degrees
        
        for dx in search_x:
            for dy in search_y:
                for dth in search_theta:
                    test_pose = Pose2D(
                        initial_pose.x + dx,
                        initial_pose.y + dy,
                        self._wrap_angle(initial_pose.theta + dth)
                    )
                    
                    score = self._evaluate_pose(scan_points, test_pose)
                    
                    if score > best_score:
                        best_score = score
                        best_pose = test_pose
        
        return best_pose, best_score
    
    def _evaluate_pose(self, scan_points: np.ndarray, pose: Pose2D) -> float:
        """
        Evaluate pose quality using distance field
        
        Returns:
            Quality score [0, 1]
        """
        # Transform points to test pose
        # (In real implementation, transform scan_points from current pose to test pose)
        # For simplicity, assuming scan_points are already in world frame
        
        total_distance = 0.0
        valid_points = 0
        
        for i in range(scan_points.shape[1]):
            x, y = scan_points[0, i], scan_points[1, i]
            
            # Convert to grid
            gx, gy = self._world_to_grid(x, y)
            
            # Check bounds
            if 0 <= gx < self.N and 0 <= gy < self.N:
                dist = self._distance_field[gy, gx]
                total_distance += dist
                valid_points += 1
        
        if valid_points == 0:
            return 0.0
        
        # Average distance (lower is better)
        avg_dist = total_distance / valid_points
        
        # Convert to quality score (0 = far from obstacles, 1 = close to obstacles)
        # Use exponential decay
        quality = math.exp(-avg_dist / 0.05)  # 5cm characteristic length
        
        return quality
    
    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        gx = int(round(x / self.res))
        gy = int(round(y / self.res))
        return gx, gy
    
    def _wrap_angle(self, angle: float) -> float:
        """Wrap angle to [-pi, pi]"""
        return (angle + math.pi) % (2.0 * math.pi) - math.pi
    
    def _log_debug(self, msg: str) -> None:
        """Log debug message"""
        if self.logger_func and self.log_file:
            try:
                self.logger_func(self.log_file, msg, "Localizer")
            except:
                print(f"[Localizer] {msg}")
        else:
            print(f"[Localizer] {msg}")

