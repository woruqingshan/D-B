# ================================
# file: code1/appio/real_robot_adapter.py
# ================================
from core.robot_adapter import RobotAdapter, LidarScan, RobotPose, OdometryData
from appio.bluetooth import BleInterface  # 现有的BleInterface类
from typing import Tuple, Optional
import time
import json

class RealRobotAdapter(RobotAdapter):
    """真实机器人适配器 - 包装现有的BleInterface"""
    
    def __init__(self, ble_interface: BleInterface):
        super().__init__()
        self.ble_interface = ble_interface
        self._last_scan = None
        self._last_odom = None
        self._connection_timeout = 5.0
    
    def get_lidar_scan(self) -> LidarScan:
        """获取真实LiDAR扫描数据"""
        # 从蓝牙接口获取数据
        scan_data, _ = self.ble_interface.get_latest_data()
        
        if scan_data is None:
            # 如果没有数据，返回默认扫描
            return LidarScan(
                angle_min=-2.356,
                angle_increment=0.01309,
                ranges=[8.0] * 360,  # 默认最大距离
                robot_pose=None,
                t=time.time()
            )
        
        # 直接返回BleInterface的LaserScan对象
        return scan_data
    
    def get_odometry(self) -> Tuple[float, float, float]:
        """获取真实里程计增量"""
        # 计算里程计增量
        current_odom = self._get_current_odometry()
        current_time = time.time()
        
        if self._last_abs_odom is not None:
            # 计算增量
            dt = current_time - self._last_timestamp
            dx = current_odom.x - self._last_abs_odom.x
            dy = current_odom.y - self._last_abs_odom.y
            d_trans = (dx**2 + dy**2) ** 0.5
            d_theta = current_odom.theta - self._last_abs_odom.theta
        else:
            # 第一次读取，无增量
            dt = 0.0
            d_trans = 0.0
            d_theta = 0.0
        
        # 更新状态
        self._last_abs_odom = current_odom
        self._last_timestamp = current_time
        
        return (d_trans, d_theta, dt)
    
    def get_latest_data(self) -> Tuple[Optional[LidarScan], Optional[OdometryData]]:
        """获取最新真实数据"""
        # 直接调用BleInterface的方法
        scan_data, odom_data = self.ble_interface.get_latest_data()
        
        # 转换里程计数据
        if odom_data is not None:
            odom = OdometryData(
                x=odom_data.x,
                y=odom_data.y,
                theta=odom_data.theta
            )
        else:
            odom = None
        
        return scan_data, odom
    
    def update(self, dt: float, linear_vel: float, angular_vel: float) -> None:
        """发送速度命令到真实机器人"""
        # 使用BleInterface的apply_control方法
        self.ble_interface.apply_control(linear_vel, angular_vel)
    
    @property
    def pose(self) -> RobotPose:
        """获取真实机器人位姿"""
        odom = self._get_current_odometry()
        return RobotPose(
            x=odom.x,
            y=odom.y,
            theta=odom.theta
        )
    
    def disconnect(self) -> None:
        """断开真实机器人连接"""
        if self.ble_interface:
            self.ble_interface.disconnect()
    
    def get_true_pose(self) -> Optional[RobotPose]:
        """
        获取真实位姿（真实模式不可用）
        
        Returns:
            None - 真实机器人无法获取真实位姿
        """
        return None  # 真实模式无法知道真实位姿
    
    def _get_current_odometry(self) -> OdometryData:
        """获取当前里程计数据"""
        _, odom_data = self.ble_interface.get_latest_data()
        
        if odom_data is None:
            # 如果没有数据，返回默认值
            return OdometryData(0.0, 0.0, 0.0)
        
        # 转换Pose2D为OdometryData格式
        return OdometryData(
            x=odom_data.x,
            y=odom_data.y,
            theta=odom_data.theta
        ) 