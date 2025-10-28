# ================================
# file: core/system_initializer.py
# ================================
from __future__ import annotations
"""System initialization module for robot navigation system.
Handles robot selection, maze loading, module instantiation, and SLAM initialization.
"""
import os
import time
from typing import Optional, Tuple
from datetime import datetime


from core.config import (
    CONTROL_HZ, GOAL_TOLERANCES, V_MAX, W_MAX, ROBOT_RADIUS, 
    SAFE_BUFFER_M, MAP_RES, SLAM_MAP_SIZE_PIXELS, SLAM_MAP_SIZE_METERS, 
    ROBOT_START_THETA
)
from core.robot_factory import RobotFactory  # 添加机器人工厂
from sim import MazeMap, RobotSim
from slam import SlamSystem
from planning import AStarPlanner
from explore import FrontierExplorer
from nav import NavigatorFSM, NavState
from gui import Visualizer
from appio import DataLogger, BleInterface


class SystemInitializer:
    """Handles system initialization including robot selection, maze loading, and module setup."""
    
    def __init__(self, logger_func=None):
        self.logger_func = logger_func
        self.log_file = None
    
    def _log(self, message: str, module: str = "INIT") -> None:
        """Log message using the provided logger function"""
        if self.logger_func and self.log_file:
            self.logger_func(self.log_file, message, module)
    
    def initialize(self, use_real_robot: bool = False, json_map_path: Optional[str] = None, 
                   port: Optional[str] = None, log_file=None) -> Tuple:
        """Initialize the complete robot navigation system.
        
        Parameters
        ----------
        use_real_robot : bool
            If True, use BleInterface; otherwise RobotSim
        json_map_path : Optional[str]
            Path to maze JSON (SIM only). If None, a blank map is used
        port : Optional[str]
            Serial port for BLE (REAL only)
        log_file : file
            Log file handle for logging
            
        Returns
        -------
        Tuple
            (robot, maze, slam, planner, explorer, nav, gui, logger, actual_pose, maze_entry, maze_info)
        """
        self.log_file = log_file
        
        self._log("=" * 60)
        self._log("系统初始化开始")
        self._log(f"运行模式: {'真实机器人' if use_real_robot else '仿真模式'}")
        self._log(f"地图文件: {json_map_path if json_map_path else '默认空白地图'}")
        self._log("=" * 60)
        
        # Initialize robot and maze
        robot, maze, maze_entry, maze_info, actual_pose = self._initialize_robot_and_maze(
            use_real_robot, json_map_path, port
        )

        # Initialize coordinate system
        self._initialize_coordinate_system(log_file)
        
        # Initialize core modules
        slam, planner, explorer, nav, gui, logger = self._initialize_modules(
            robot, maze, log_file, use_real_robot
        )
        
        # Initialize SLAM pose
        self._initialize_slam_pose(slam, actual_pose, use_real_robot)
        
        self._log("系统初始化完成")
        self._log("=" * 60)
        
        return (robot, maze, slam, planner, explorer, nav, gui, logger, 
                actual_pose, maze_entry, maze_info)
    
    def _initialize_robot_and_maze(self, use_real_robot: bool, json_map_path: Optional[str], 
                                  port: Optional[str]) -> Tuple:
        """Initialize robot and maze based on mode."""
        if not use_real_robot:
            # SIM mode: Create maze map and robot simulator
            maze = MazeMap()
            maze_entry = (0.0, 0.0)  # Default entry point
            maze_info = None
            
            if json_map_path:
                maze.load_from_json(json_map_path, self.log_file)
                maze_entry = maze.get_maze_entry_position()
                maze_info = maze.get_maze_info()
                
                self._log("迷宫构建信息:")
                self._log(f"  JSON迷宫尺寸: {maze_info['json_size'][0]} x {maze_info['json_size'][1]}")
                if maze_info['bounds']:
                    self._log(f"  迷宫中心: {maze_info['bounds']['center']}")
                    self._log(f"  迷宫四角: 左下{maze_info['bounds']['bottom_left']}, 右上{maze_info['bounds']['top_right']}")
                else:
                    self._log(f"  迷宫中心: 未设置")
                    self._log(f"  迷宫四角: 未设置")
                self._log(f"  迷宫入口位置: ({maze_entry[0]:.3f}, {maze_entry[1]:.3f})")
                self._log(f"  墙壁网格数: {maze_info['wall_count']}")
                self._log(f"  自由空间网格数: {maze_info['free_count']}")
            else:
                self._log("未指定地图文件，使用空白地图")
            
            # 创建仿真机器人
            robot_sim = RobotSim(maze)
            
            # Set robot initial position
            car_start = maze.get_car_start_position()
            start_x = car_start[0]  # 从maze获取起始位置
            start_y = car_start[1]  # 从maze获取起始位置
            
            # Debug: record position setting process
            self._log("位置设置调试:")
            self._log(f"  - 从maze获取的起始位置: {car_start}")
            self._log(f"  - 设置机器人位置: ({start_x:.3f}, {start_y:.3f})")
            
            # Set robot position
            robot_sim.set_pose(pose=robot_sim.pose.__class__(start_x, start_y, ROBOT_START_THETA))  # 使用配置中的朝向角度
            
            # 使用工厂类创建适配器
            robot = RobotFactory.create_robot_adapter(False, robot_sim=robot_sim)
            
            # Verify position setting
            actual_pose = robot.pose
            self._log(f"  - 机器人实际位置: ({actual_pose.x:.3f}, {actual_pose.y:.3f})")
            self._log(f"  - 位置设置是否成功: {'是' if abs(actual_pose.x - start_x) < 0.01 and abs(actual_pose.y - start_y) < 0.01 else '否'}")
            
            # Record robot initial position and maze entry
            self._log("小车初始位置:")
            self._log(f"   - 世界坐标: ({start_x:.3f}, {start_y:.3f})")
            self._log(f"   - 朝向角度: {ROBOT_START_THETA:.3f} rad ({ROBOT_START_THETA*180/3.14159:.1f}°)")
            self._log(f"   - 设计位置: 迷宫下方中央")
            if json_map_path:
                self._log(f"   - 目标: 搜索并移动到迷宫入口 ({maze_entry[0]:.3f}, {maze_entry[1]:.3f})")
        else:
            # REAL mode: Use BLE interface
            if port is None:
                raise SystemExit("--port is required in --real mode")
            
            # 创建蓝牙接口
            ble_interface = BleInterface(port=port)
            
            # 使用工厂类创建适配器
            robot = RobotFactory.create_robot_adapter(True, ble_interface=ble_interface)
            
            maze = None  # no ground-truth in real mode
            maze_entry = (0.0, 0.0)  # Default for real mode - will be detected by smart detector
            maze_info = None
            actual_pose = None
            self._log(f"连接真实机器人端口: {port}")
            self._log("真实模式：将使用智能入口检测器自动识别迷宫入口")
        
        return robot, maze, maze_entry, maze_info, actual_pose
    
    def _initialize_modules(self, robot, maze, log_file, use_real_robot=False):
        """Initialize all core modules."""
        self._log("初始化核心模块...")
        
        slam = SlamSystem(
            map_size_pixels=SLAM_MAP_SIZE_PIXELS, 
            map_size_meters=SLAM_MAP_SIZE_METERS,
            mode='simulation' if not use_real_robot else 'real',  # Explicitly specify mode
            logger_func=self.logger_func, 
            log_file=log_file
        )
        
        # Note: New architecture doesn't use odom_is_world attribute
        planner = AStarPlanner(logger_func=self.logger_func, log_file=log_file)
        # Fix: FrontierExplorer(planner, slam_system, logger_func, log_file)
        # Must pass slam_system first, then logger_func and log_file
        explorer = FrontierExplorer(planner, slam_system=slam, logger_func=self.logger_func, log_file=log_file)
        
        # 确保导航器正确接收日志函数和日志文件（不再需要DWA参数）
        nav = NavigatorFSM(slam, planner, explorer, robot, self.logger_func, log_file)  # Removed dwa parameter
        
        # Initialize GUI - always use Matplotlib visualizer
        from gui.visualizer import Visualizer
        gui = Visualizer(maze, self.logger_func, log_file)
        self._log("✅ 使用Matplotlib可视化器")
        logger = DataLogger()
        
        self._log("核心模块初始化完成")
        return slam, planner, explorer, nav, gui, logger
    
    def _initialize_coordinate_system(self, log_file):
        """Initialize coordinate system with logging function."""
        from core.coords import coord_system
        coord_system.logger_func = self.logger_func
        coord_system.log_file = log_file
        self._log("坐标系统初始化完成")
    
    def _initialize_slam_pose(self, slam, actual_pose, use_real_robot):
        """Initialize SLAM pose."""
        if not use_real_robot and actual_pose:
            self._log("设置SLAM初始位姿:")
            self._log(f"  - 使用机器人位置: ({actual_pose.x:.3f}, {actual_pose.y:.3f}, {actual_pose.theta:.3f})")
            slam.set_initial_pose(actual_pose)
            self._log(f"  - SLAM初始化状态: {'已初始化' if slam.is_initialized() else '未初始化'}")
        else:
            self._log("真实机器人模式：SLAM将从当前位置开始初始化") 