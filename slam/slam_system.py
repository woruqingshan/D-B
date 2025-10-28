# ================================
# file: slam/slam_system.py (REFACTORED)
# ================================
"""
SLAM System with Decoupled Mapping and Localization

Architecture:
- Mapping Phase: Use ground truth pose (simulation) to build high-quality map
- Localization Phase: Use ICP on known map for pose estimation

This decoupling eliminates the "bad pose → bad map → worse pose" problem.
"""
from __future__ import annotations
from typing import Tuple, Optional
import math
import numpy as np
import time

from core import Pose2D, LaserScan, LIDAR_RANGE, LIDAR_MIN_RANGE
from core.config import (
    SLAM_MAP_SIZE_PIXELS, SLAM_MAP_SIZE_METERS,
    SLAM_RESOLUTION,
    LOG_ODDS_HIT, LOG_ODDS_MISS, LOG_ODDS_MIN, LOG_ODDS_MAX,
)

# Import new modules
from slam.map_builder import MapBuilder
from slam.localizer import Localizer


class SlamSystem:
    """
    Refactored SLAM System with Mapping/Localization Decoupling
    
    Usage:
        # Initialization
        slam = SlamSystem(mode='simulation')
        slam.set_initial_pose(start_pose)
        
        # Mapping phase (use ground truth)
        while mapping:
            slam.update(lidar_scan, odom_delta, true_pose=gt_pose)
        
        # Switch to localization
        slam.complete_mapping()
        
        # Localization phase (ICP on known map)
        while running:
            slam.update(lidar_scan, odom_delta)
    """
    
    def __init__(self,
                 map_size_pixels: int = SLAM_MAP_SIZE_PIXELS,
                 map_size_meters: float = SLAM_MAP_SIZE_METERS,
                 mode: str = 'simulation',  # 'simulation' or 'real'
                 logger_func=None,
                 log_file=None) -> None:
        """
        Initialize SLAM system
        
        Args:
            map_size_pixels: Map size in pixels
            map_size_meters: Map size in meters
            mode: 'simulation' (use ground truth) or 'real' (estimate during mapping)
            logger_func: Logger function
            log_file: Log file handle
        """
        self.mode = mode
        self.N = int(map_size_pixels)
        self.L = float(map_size_meters)
        self.res = float(SLAM_RESOLUTION)
        
        # Module 1: Map Builder (pure mapping)
        self.mapper = MapBuilder(
            map_size_pixels=self.N,
            map_size_meters=self.L,
            resolution=self.res,
            log_hit=LOG_ODDS_HIT,
            log_miss=LOG_ODDS_MISS,
            log_min=LOG_ODDS_MIN,
            log_max=LOG_ODDS_MAX,
            logger_func=logger_func,
            log_file=log_file
        )
        
        # Module 2: Localizer (pure localization, initialized after mapping)
        self.localizer: Optional[Localizer] = None
        
        # State management
        self.pose = Pose2D(0.0, 0.0, 0.0)  # Current pose estimate
        self.true_pose: Optional[Pose2D] = None  # Ground truth (simulation only)
        self.occ_grid = np.full((self.N, self.N), 2, dtype=np.uint8)
        
        # Phase management
        self.mapping_phase = True    # True = mapping, False = localization
        self.mapping_complete = False
        self._mapping_start_time = time.time()
        
        # Logger
        self.logger_func = logger_func
        self.log_file = log_file
        
        self._initialized = False
        
        self._log_debug(f"SLAM系统初始化:")
        self._log_debug(f"  模式: {mode}")
        self._log_debug(f"  地图像素: {self.N}x{self.N}")
        self._log_debug(f"  地图物理尺寸: {self.L:.2f}m x {self.L:.2f}m")
        self._log_debug(f"  分辨率: {self.res:.6f}m/pixel")
        self._log_debug(f"  验证: {self.N} * {self.res:.6f} = {self.N * self.res:.3f}m (应等于{self.L}m)")

    def set_initial_pose(self, pose: Pose2D) -> None:
        """Set initial robot pose"""
        self.pose = Pose2D(pose.x, pose.y, pose.theta)
        if self.mode == 'simulation':
            self.true_pose = Pose2D(pose.x, pose.y, pose.theta)
        self._initialized = True
        self._log_debug(f"设置初始位姿: ({pose.x:.3f}, {pose.y:.3f}, {pose.theta:.3f})")
    
    def update(self,
               lidar_scan: LaserScan,
               odom_delta: Tuple[float, float, float],
               true_pose: Optional[Pose2D] = None) -> None:
        """
        Main update loop
        
        Args:
            lidar_scan: Laser scan data
            odom_delta: Odometry delta (dx, dy, dtheta) in body frame
            true_pose: Ground truth pose (simulation only, used in mapping phase)
        """
        if not self._initialized:
            self._log_debug("SLAM系统未初始化，跳过更新")
            return
        
        if self.mapping_phase:
            # === MAPPING PHASE ===
            if self.mode == 'simulation':
                # Simulation: use ground truth pose
                if true_pose is None:
                    self._log_debug("⚠️ 仿真模式但未提供真实位姿，跳过建图")
                    return
                
                self.true_pose = true_pose
                use_pose = true_pose
                
            else:  # real mode
                # Real mode: use odometry estimate (accumulate)
                self._update_pose_odometry(odom_delta)
                use_pose = self.pose
            
            # Update map using reliable pose
            self.mapper.update(use_pose, lidar_scan)
            
            # Update outputs
            self.pose = Pose2D(use_pose.x, use_pose.y, use_pose.theta)
            self.occ_grid = self.mapper.get_occupancy_grid()
            
        else:
            # === LOCALIZATION PHASE ===
            if self.localizer is None:
                self._log_debug("⚠️ 定位器未初始化，跳过定位")
                return
            
            # Prediction: odometry
            self.localizer.predict(odom_delta)
            
            # Correction: ICP on known map
            estimated_pose, quality = self.localizer.correct(lidar_scan)
            
            # Update outputs
            self.pose = estimated_pose
            self.occ_grid = self.localizer.occ_grid  # Use reference map
            
            if quality < 0.3:
                self._log_debug(f"⚠️ ICP质量低: {quality:.3f}")
    

    def get_pose(self) -> Pose2D:
        """Get current pose estimate"""
        return self.pose

    def get_occ_grid(self) -> np.ndarray:
        """Get occupancy grid (0=free, 1=occupied, 2=unknown)"""
        return self.occ_grid
    
    def is_initialized(self) -> bool:
        """Check if SLAM is initialized"""
        return self._initialized
    
    def get_coverage_ratio(self) -> float:
        """Get map coverage ratio"""
        if self.mapping_phase:
            return self.mapper.get_coverage_ratio()
        else:
            return 1.0  # Localization phase uses complete map
    
    def get_mapping_duration(self) -> float:
        """Get mapping duration in seconds"""
        if self.mapping_phase:
            return time.time() - self._mapping_start_time
        else:
            return 0.0
    
    def export_map(self, filename: str) -> None:
        """Export map to file"""
        if self.mapping_phase:
            map_data = self.mapper.export_map()
        else:
            # Already in localization phase, export reference map
            map_data = {
                'occupancy_grid': self.occ_grid,
                'metadata': {
                    'resolution': self.res,
                    'size': (self.N, self.N),
                    'size_meters': self.L,
                }
            }
        
        np.savez(filename, **map_data)
        self._log_debug(f"地图已保存: {filename}")
    
    def _update_pose_odometry(self, odom_delta: Tuple[float, float, float]) -> None:
        """Update pose using odometry (for real mode during mapping)"""
        dx_body, dy_body, dth = odom_delta
        
        # Transform to world frame
        theta = self.pose.theta
        c, s = math.cos(theta), math.sin(theta)
        
        dx_world = c * dx_body - s * dy_body
        dy_world = s * dx_body + c * dy_body
        
        # Update pose
        self.pose.x += dx_world
        self.pose.y += dy_world
        self.pose.theta = self._wrap_angle(self.pose.theta + dth)
    
    def _wrap_angle(self, angle: float) -> float:
        """Wrap angle to [-pi, pi]"""
        return (angle + math.pi) % (2.0 * math.pi) - math.pi
    
    def _log_debug(self, msg: str) -> None:
        """Log debug message"""
        if self.logger_func and self.log_file:
            try:
                self.logger_func(self.log_file, msg, "SLAM")
            except:
                print(f"[SLAM] {msg}")
        else:
            print(f"[SLAM] {msg}")