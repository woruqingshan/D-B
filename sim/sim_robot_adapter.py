# ================================
# file: code1/sim/sim_robot_adapter.py
# ================================
from core.robot_adapter import RobotAdapter, LidarScan, RobotPose, OdometryData
from sim.robot_sim import RobotSim  # 现有的RobotSim类
from typing import Tuple, Optional
import time

class SimRobotAdapter(RobotAdapter):
    """仿真机器人适配器 - 包装现有的RobotSim"""
    
    def __init__(self, robot_sim: RobotSim):
        super().__init__()
        self.robot_sim = robot_sim
        self._last_pose = None
    
    def get_lidar_scan(self) -> LidarScan:
        """获取仿真LiDAR扫描数据"""
        # 直接调用现有的RobotSim方法，返回LaserScan对象
        return self.robot_sim.get_lidar_scan()
    
    def get_odometry(self) -> Tuple[float, float, float]:
        """获取仿真里程计增量"""
        # 直接调用现有的RobotSim方法
        return self.robot_sim.get_odometry()
    
    def get_latest_data(self) -> Tuple[Optional[LidarScan], Optional[OdometryData]]:
        """获取最新仿真数据"""
        # 获取LiDAR扫描
        scan = self.get_lidar_scan()
        
        # 获取当前位姿作为绝对里程计
        current_pose = self.pose
        
        odom_data = OdometryData(
            x=current_pose.x,
            y=current_pose.y,
            theta=current_pose.theta
        )
        
        return scan, odom_data
    
    def update(self, dt: float, linear_vel: float, angular_vel: float) -> None:
        """更新仿真机器人状态"""
        # 直接调用现有的RobotSim方法
        self.robot_sim.update(dt, linear_vel, angular_vel)
    
    @property
    def pose(self) -> RobotPose:
        """获取仿真机器人位姿"""
        # 直接返回RobotSim的pose，它已经是Pose2D对象
        return self.robot_sim.pose
    
    def disconnect(self) -> None:
        """仿真机器人断开连接（无操作）"""
        pass  # 仿真机器人不需要断开连接
    
    def get_true_pose(self) -> Optional[RobotPose]:
        """
        获取真实位姿（仿真特权）
        
        Returns:
            真实位姿 - 仿真模式可以直接访问真实位姿
        """
        return self.robot_sim.pose  # 仿真器的位姿就是真实位姿
    
    @property
    def maze(self):
        """获取迷宫对象（用于地图状态检查）"""
        return self.robot_sim.maze 