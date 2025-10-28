# ================================
# file: code1/core/robot_adapter.py
# ================================
from abc import ABC, abstractmethod
from typing import Tuple, Optional, Union
import numpy as np
import time

# 使用现有的数据结构，保持兼容性
from core.types import Pose2D, LaserScan

# 为了兼容性，创建别名
RobotPose = Pose2D
LidarScan = LaserScan

class OdometryData:
    """兼容现有代码的里程计数据结构"""
    def __init__(self, x: float, y: float, theta: float):
        self.x = x
        self.y = y
        self.theta = theta

class RobotAdapter(ABC):
    """机器人适配器基类 - 统一仿真和真实机器人接口"""
    
    def __init__(self):
        self._last_abs_odom = None
        self._last_timestamp = time.time()
    
    @abstractmethod
    def get_lidar_scan(self) -> LidarScan:
        """获取LiDAR扫描数据 - 兼容现有接口"""
        pass
    
    @abstractmethod
    def get_odometry(self) -> Tuple[float, float, float]:
        """获取里程计增量 - 兼容现有接口"""
        pass
    
    @abstractmethod
    def get_latest_data(self) -> Tuple[Optional[LidarScan], Optional[OdometryData]]:
        """获取最新数据 - 兼容现有接口"""
        pass
    
    @abstractmethod
    def update(self, dt: float, linear_vel: float, angular_vel: float) -> None:
        """更新机器人状态 - 兼容现有接口"""
        pass
    
    def apply_control(self, linear_vel: float, angular_vel: float) -> None:
        """应用控制命令 - 兼容现有接口"""
        # 默认实现：调用update方法
        self.update(0.05, linear_vel, angular_vel)  # 使用默认时间步长
    
    @property
    @abstractmethod
    def pose(self) -> RobotPose:
        """获取机器人位姿 - 兼容现有接口"""
        pass
    
    @abstractmethod
    def disconnect(self) -> None:
        """断开连接 - 兼容现有接口"""
        pass
    
    @abstractmethod
    def get_true_pose(self) -> Optional[Pose2D]:
        """
        获取真实位姿（仅仿真模式可用）
        
        Returns:
            Pose2D: 真实位姿（仿真模式）
            None: 真实模式无法获取真实位姿
        """
        pass