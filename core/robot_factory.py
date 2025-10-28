# ================================
# file: code1/core/robot_factory.py
# ================================
from core.robot_adapter import RobotAdapter
from sim.sim_robot_adapter import SimRobotAdapter
from appio.real_robot_adapter import RealRobotAdapter
from sim.robot_sim import RobotSim
from appio.bluetooth import BleInterface
from typing import Optional

class RobotFactory:
    """机器人工厂类 - 创建适配器实例"""
    
    @staticmethod
    def create_robot_adapter(use_real_robot: bool, 
                           robot_sim: Optional[RobotSim] = None,
                           ble_interface: Optional[BleInterface] = None) -> RobotAdapter:
        """创建机器人适配器"""
        
        if use_real_robot:
            if ble_interface is None:
                raise ValueError("BleInterface required for real robot mode")
            return RealRobotAdapter(ble_interface)
        else:
            if robot_sim is None:
                raise ValueError("RobotSim required for simulation mode")
            return SimRobotAdapter(robot_sim) 