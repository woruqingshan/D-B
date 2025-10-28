# ================================
# file: core/__init__.py
# ================================
"""
Core Package

Exports fundamental types, configurations, and utilities.
"""
from core.types import Pose2D, LaserScan
from core.coords import coord_system, world_to_map, map_to_world
from core.config import (
    # Map configuration
    MAP_RES, MAP_SIZE, WORLD_SIZE, GRID_SIZE,
    
    # SLAM configuration
    SLAM_MAP_SIZE_PIXELS, SLAM_MAP_SIZE_METERS, SLAM_RESOLUTION,
    SLAM_USE_DECOUPLED_ARCHITECTURE,
    MAPPING_TIME_THRESHOLD, MAPPING_COVERAGE_THRESHOLD,
    
    # Robot configuration
    ROBOT_RADIUS, SAFE_BUFFER_M,
    
    # Sensor configuration
    LIDAR_RANGE, LIDAR_MIN_RANGE, LIDAR_FOV_DEG, LIDAR_BEAMS,
    
    # Control configuration
    V_MAX, W_MAX, CONTROL_HZ, GOAL_TOLERANCES, DWA_MIN_V,
    
    
    # Exploration configuration
    EXPLORATION_MIN_TIME, MIN_FRONTIER_SIZE,
)
from core.robot_adapter import RobotAdapter
from core.robot_factory import RobotFactory

__all__ = [
    # Types
    'Pose2D', 'LaserScan',
    
    # Coordinates
    'coord_system', 'world_to_map', 'map_to_world',
    
    # Configuration
    'MAP_RES', 'MAP_SIZE', 'WORLD_SIZE', 'GRID_SIZE',
    'SLAM_MAP_SIZE_PIXELS', 'SLAM_MAP_SIZE_METERS', 'SLAM_RESOLUTION',
    'SLAM_USE_DECOUPLED_ARCHITECTURE',
    'MAPPING_TIME_THRESHOLD', 'MAPPING_COVERAGE_THRESHOLD',
    'ROBOT_RADIUS', 'SAFE_BUFFER_M',
    'LIDAR_RANGE', 'LIDAR_MIN_RANGE', 'LIDAR_FOV_DEG', 'LIDAR_BEAMS',
    'V_MAX', 'W_MAX', 'CONTROL_HZ', 'GOAL_TOLERANCES', 'DWA_MIN_V',
    'EXPLORATION_MIN_TIME', 'MIN_FRONTIER_SIZE',
    
    # Adapters and factories
    'RobotAdapter', 'RobotFactory',
]