# ================================
# file: code/run_main.py
# ================================
from __future__ import annotations
"""Project entrypoint: runs SIM by default with a real-time loop and GUI.
- In SIM mode: uses RobotSim + MazeMap + Matplotlib GUI.
- In REAL mode: uses BleInterface; GUI will still show SLAM map (no GT map).

Usage (SIM):
    python main.py --map ./data/maze.json
Usage (REAL):
    python main.py --real --port /dev/rfcomm0
"""
import argparse
import time
import os
import math
from typing import Optional
from datetime import datetime


from core.config import (
    CONTROL_HZ, GOAL_TOLERANCES, V_MAX, W_MAX, ROBOT_RADIUS, 
    SAFE_BUFFER_M, MAP_RES, SLAM_RESOLUTION, DWA_MIN_V, SLAM_DECAY_FACTOR
)
from core.coords import map_to_world
from core.system_initializer import SystemInitializer
from core.coords import coord_system  # 添加坐标系统导入
from core.map_validator import MapValidator  # 添加地图验证模块
from core.robot_factory import RobotFactory  # 添加机器人工厂
# 删除智能入口检测模块导入（不再需要）
from sim import RobotSim
from nav import NavState
from appio import DataLogger, BleInterface


def log_to_file(log_file, message, module="MAIN"):
    """Write message to log file with timestamp and module"""
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    log_entry = f"[{timestamp}] [{module}] {message}\n"
    log_file.write(log_entry)
    log_file.flush()  # Ensure immediate write
    print(log_entry.strip())  # Also print to console


def run(use_real_robot: bool = False, json_map_path: Optional[str] = None, port: Optional[str] = None) -> None:
    """Wire modules and start the real-time loop.
    Parameters
    ----------
    use_real_robot : If True, use BleInterface; otherwise RobotSim.
    json_map_path  : Path to maze JSON (SIM only). If None, a blank map is used.
    port           : Serial port for BLE (REAL only).
    """
    # Create log file
    log_filename = f"robot_movement_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    log_filepath = os.path.join(os.path.dirname(__file__), log_filename)
    
    # Open log file for the entire function duration
    log_file = open(log_filepath, 'w', encoding='utf-8')
    
    try:
        log_to_file(log_file, "=" * 60)
        log_to_file(log_file, "机器人导航系统运行日志")
        log_to_file(log_file, f"开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        log_to_file(log_file, f"地图文件: {json_map_path if json_map_path else '默认空白地图'}")
        log_to_file(log_file, f"运行模式: {'真实机器人' if use_real_robot else '仿真模式'}")
        log_to_file(log_file, f"可视化方式: Matplotlib")
        log_to_file(log_file, "=" * 60)
        
        # --- System initialization ---
        initializer = SystemInitializer(logger_func=log_to_file)
        (robot, maze, slam, planner, explorer, nav, gui, logger, 
         actual_pose, maze_entry, maze_info) = initializer.initialize(
            use_real_robot=use_real_robot,
            json_map_path=json_map_path,
            port=port,
            log_file=log_file
        )
        
        # 确保前沿点探索器使用SLAM系统
        if explorer and hasattr(explorer, 'slam_system'):
            explorer.slam_system = slam
            log_to_file(log_file, "前沿点探索器已配置SLAM系统")

        # 初始化MapValidator（用于地图验证）
        map_validator = MapValidator(logger_func=log_to_file)
        
        
        # 简化的入口检测逻辑 - 直接使用已知的迷宫入口坐标
        if json_map_path:
            # 仿真模式：直接使用JSON文件中的迷宫入口坐标
            log_to_file(log_file, "仿真模式：使用JSON文件中的迷宫入口坐标")
            if maze_info is None:
                maze_entry = maze.get_maze_entry_position()
                maze_info = maze.get_maze_info()
            nav.set_initial_goal(maze_entry)
            log_to_file(log_file, f"导航状态: 使用JSON文件的迷宫入口 ({maze_entry[0]:.3f}, {maze_entry[1]:.3f})")
            
            # 设置迷宫出口位置（从JSON文件读取）
            import json
            with open(json_map_path, 'r') as f:
                maze_data = json.load(f)
            
            # 获取goal_point并转换为世界坐标
            goal_json = maze_data.get('goal_point', [2.5, 4.0])
            cell_size = maze_data['metadata'].get('cell_size_m', 0.45)
            origin_offset = maze_data['metadata'].get('origin_offset_m', [0.1, 0.1])
            
            goal_world_x = origin_offset[0] + goal_json[0] * cell_size
            goal_world_y = origin_offset[1] + goal_json[1] * cell_size
            maze_exit = (goal_world_x, goal_world_y)
            
            nav.set_maze_exit(maze_exit)
            log_to_file(log_file, f"导航状态: 使用JSON文件的迷宫出口 ({maze_exit[0]:.3f}, {maze_exit[1]:.3f})")
        else:
            # 真实模式：直接开始探索模式
            log_to_file(log_file, "真实模式：直接开始探索模式")
            nav.set_state(NavState.EXPLORING)
            log_to_file(log_file, "导航状态: 开始探索模式")

        dt = 1.0 / CONTROL_HZ
        last_gui = time.time()
        last_map_snap = time.time()
        last_abs_odom = None  # type: ignore
        step_count = 0
        
        # 添加时间步长监控
        last_step_time = time.time()
        
        # 添加位置跟踪变量（优先级1修复）
        last_new_pose = None
        last_slam_pose = None
        
        # 添加速度平滑变量（双控制通道修复）
        last_linear_vel = 0.0
        last_angular_vel = 0.0
        
        # English: carry odom to next frame
        odom_delta_buf = (0.0, 0.0, 0.0)
        
        # Initialize SLAM system properly
        if slam and slam.is_initialized():
            log_to_file(log_file, "SLAM系统已初始化，准备开始更新")
        else:
            log_to_file(log_file, "警告：SLAM系统未正确初始化")

        log_to_file(log_file, f"开始主循环 @ {CONTROL_HZ:.1f} Hz")
        log_to_file(log_file, "=" * 60)

        try:
            while True:
                step_count += 1
                current_time = time.time()
                actual_dt = current_time - last_step_time
                last_step_time = current_time
                
                # 修复：确保dt不为0，避免控制频率异常
                if actual_dt <= 0 or actual_dt > 1.0:  # 防止异常大的dt
                    actual_dt = dt  # 使用预设的dt
                    if step_count % 100 == 1:
                        log_to_file(log_file, f"时间步长异常，使用预设dt: {actual_dt:.3f}s")
                
                # 只在每50步输出一次详细日志，大幅减少输出频率以提高性能
                if step_count % 100 == 1:
                    log_to_file(log_file, f"\n步骤 {step_count} - 时间: {current_time:.3f} - 实际dt: {actual_dt:.3f}s")
                log_to_file(log_file, "-" * 40)
                    
                # --- Sensor ingest ---
                if not use_real_robot:
                    # 仿真模式：生成机体系里程计增量
                    scan = robot.get_lidar_scan()
                    current_pose = robot.pose
                    
                    # 计算并转换为机体系里程计增量
                    if hasattr(robot, '_last_pose') and robot._last_pose is not None:
                        # 1. 计算世界坐标系下的真值位移
                        dx_world_true = current_pose.x - robot._last_pose.x
                        dy_world_true = current_pose.y - robot._last_pose.y
                        dtheta_true = current_pose.theta - robot._last_pose.theta
                        dtheta_true = math.atan2(math.sin(dtheta_true), math.cos(dtheta_true))
                        
                        # 2. 将世界坐标系位移转换为机体系增量
                        # English: Convert world-frame displacement to body-frame using previous robot heading
                        last_theta = robot._last_pose.theta
                        c, s = math.cos(last_theta), math.sin(last_theta)
                        
                        # 机体系增量计算
                        dx_body = dx_world_true * c + dy_world_true * s
                        dy_body = -dx_world_true * s + dy_world_true * c
                        
                        # 3. 添加传感器噪声（可选）
                        import numpy as np
                        NOISE_LIN_PERCENT = 0  # 2% linear noise
                        NOISE_ANG_PERCENT = 0  # 5% angular noise
                        
                        dist_moved = math.hypot(dx_body, dy_body)
                        noise_dx = np.random.normal(0, NOISE_LIN_PERCENT * dist_moved)
                        noise_dtheta = np.random.normal(0, NOISE_ANG_PERCENT * abs(dtheta_true))
                        
                        # 4. 生成机体系里程计增量
                        odom_delta = (
                            dx_body + noise_dx,
                            dy_body,  # 简化：忽略横向噪声
                            dtheta_true + noise_dtheta
                        )
                        
                        # 更新参考位姿
                        robot._last_pose = current_pose.copy()
                    else:
                        # 首次调用，初始化
                        robot._last_pose = current_pose.copy()
                        odom_delta = (0.0, 0.0, 0.0)
                    
                    # BUG: This makes _last_pose == current_pose so consistency becomes 0 every frame.
                    # robot._last_pose = current_pose.copy()
                    # English: Do not overwrite _last_pose before control. It zeroes the consistency check and confuses odom accounting.
                else:
                    # 真实机器人模式：处理累积距离
                    scan, abs_odom = robot.get_latest_data()
                    if scan is None:
                        time.sleep(0.005)
                        continue
                    
                    # 处理累积距离格式
                    if abs_odom is not None and last_abs_odom is not None:
                        # 计算累积距离变化
                        dx = abs_odom.x - last_abs_odom.x
                        dy = abs_odom.y - last_abs_odom.y
                        d_trans = math.hypot(dx, dy)
                        
                        # 转换为Δs沿当前朝向
                        current_theta = robot.pose.theta
                        dx_world = d_trans * math.cos(current_theta)
                        dy_world = d_trans * math.sin(current_theta)
                        
                        odom_delta = (dx_world, dy_world, 0.0)  # 角度变化设为0
                    else:
                        odom_delta = (0.0, 0.0, 0.0)
                    
                    last_abs_odom = abs_odom
                    
                    # English: ensure anchor exists in REAL mode too
                    if not hasattr(robot, '_last_pose') or robot._last_pose is None:
                        robot._last_pose = robot.pose.copy()

                # 计算当前速度（使用固定周期dt，避免用odom_delta[2]做除数）
                robot_v = math.hypot(odom_delta[0], odom_delta[1]) / max(1e-6, dt)
                robot_omega = odom_delta[2] / max(1e-6, dt)

                # --- SLAM update ---
                # English: feed THIS-frame odom delta
                # Get true pose (simulation only, returns None for real robot)
                true_pose = robot.get_true_pose() if hasattr(robot, 'get_true_pose') else None
                
                # Update SLAM with ground truth (mapping phase) or without (localization phase)
                slam.update(scan, odom_delta, true_pose=true_pose)
                current_pose = slam.get_pose()
                occ_grid = slam.get_occ_grid()
                
                # Check if mapping phase should be completed (simulation mode)
                if slam.mapping_phase and not use_real_robot:
                    # Trigger mapping completion after certain time or coverage
                    mapping_duration = slam.get_mapping_duration()
                    coverage_ratio = slam.get_coverage_ratio()
                    
                    # Complete mapping after 60 seconds or 80% coverage
                    if mapping_duration > 10000.0:
                        log_to_file(log_file, f"建图完成触发: 时长={mapping_duration:.1f}s, 覆盖率={coverage_ratio:.2%}")
                    
                # FIX: Use consistent pose for navigation and display
                actual_robot_pose = robot.pose  # 获取实际机器人位置
                slam_pose = slam.get_pose()     # 获取SLAM估计位置
                    
                # 只在每50步记录一次SLAM更新结果，减少输出频率
                if step_count % 2 == 1:
                    log_to_file(log_file, "SLAM地图更新:")
                    log_to_file(log_file, f"   - SLAM位姿: ({current_pose.x:.3f}, {current_pose.y:.3f}, {current_pose.theta:.3f})")
                    log_to_file(log_file, f"   - 实际机器人位姿: ({actual_robot_pose.x:.3f}, {actual_robot_pose.y:.3f}, {actual_robot_pose.theta:.3f})")
                    log_to_file(log_file, f"   - 地图尺寸: {occ_grid.shape}")
                    # SLAM阶段状态
                    if slam.mapping_phase:
                        log_to_file(log_file, f"   - 当前阶段: 建图阶段")
                        log_to_file(log_file, f"   - 建图时长: {slam.get_mapping_duration():.1f}s")
                        log_to_file(log_file, f"   - 地图覆盖率: {slam.get_coverage_ratio():.2%}")
                    else:
                        log_to_file(log_file, f"   - 当前阶段: 定位阶段")
                        if slam.localizer and hasattr(slam.localizer, '_last_match_quality'):
                            log_to_file(log_file, f"   - ICP质量: {slam.localizer._last_match_quality:.3f}")
                    log_to_file(log_file, f"   - 已知区域: {(occ_grid != 2).sum()} 个网格")
                    log_to_file(log_file, f"   - 未知区域: {(occ_grid == 2).sum()} 个网格")
                    log_to_file(log_file, f"   - 障碍物: {(occ_grid == 1).sum()} 个网格")
                    log_to_file(log_file, f"   - 自由空间: {(occ_grid == 0).sum()} 个网格")

                # 添加详细的ICP-SLAM状态监控
                if step_count % 2 == 1:
                    log_to_file(log_file, "ICP-SLAM状态:")
                    log_to_file(log_file, f"   - SLAM位姿: ({slam_pose.x:.3f}, {slam_pose.y:.3f}, {slam_pose.theta:.3f})")
                    log_to_file(log_file, f"   - 实际位姿: ({actual_robot_pose.x:.3f}, {actual_robot_pose.y:.3f}, {actual_robot_pose.theta:.3f})")
                    
                    # 计算位姿误差
                    pose_error_dx = slam_pose.x - actual_robot_pose.x
                    pose_error_dy = slam_pose.y - actual_robot_pose.y
                    pose_error_dtheta = slam_pose.theta - actual_robot_pose.theta
                    pose_error_dtheta = math.atan2(math.sin(pose_error_dtheta), math.cos(pose_error_dtheta))
                    pose_error_distance = math.hypot(pose_error_dx, pose_error_dy)
                    
                    log_to_file(log_file, f"   - 位姿误差: dx={pose_error_dx:.3f}, dy={pose_error_dy:.3f}, dtheta={pose_error_dtheta:.3f}")
                    log_to_file(log_file, f"   - 位姿误差距离: {pose_error_distance:.3f} m")
                    
                    # SLAM系统状态监控（新架构）
                    if slam.mapping_phase:
                        # 建图阶段
                        if hasattr(slam.mapper, '_update_count'):
                            log_to_file(log_file, f"   - 建图更新次数: {slam.mapper._update_count}")
                        if hasattr(slam.mapper, '_total_rays'):
                            log_to_file(log_file, f"   - 总光束数: {slam.mapper._total_rays}")
                        if hasattr(slam.mapper, '_total_hits'):
                            log_to_file(log_file, f"   - 总命中数: {slam.mapper._total_hits}")
                    else:
                        # 定位阶段
                        if slam.localizer:
                            if hasattr(slam.localizer, '_tracking_good'):
                                log_to_file(log_file, f"   - 跟踪状态: {'良好' if slam.localizer._tracking_good else '丢失'}")
                            if hasattr(slam.localizer, '_last_match_quality'):
                                log_to_file(log_file, f"   - 匹配质量: {slam.localizer._last_match_quality:.3f}")
                    
                    # 里程计数据验证
                    odom_consistency_distance = math.hypot(odom_delta[0], odom_delta[1])
                    log_to_file(log_file, f"   - 里程计一致性: 距离={odom_consistency_distance:.3f} m")
                # --- Navigation / control ---
                nav.update()
                    
                # 只在每50步记录导航状态，减少输出频率
                if step_count % 100 == 1:
                    log_to_file(log_file, "导航状态:")
                    log_to_file(log_file, f"   - 当前状态: {nav.state.name}")
                    log_to_file(log_file, f"   - 全局目标: {nav.current_goal}")
                    log_to_file(log_file, f"   - 路径长度: {len(nav.current_path) if nav.current_path else 0}")
                    
                    # 添加三阶段导航状态监控
                    log_to_file(log_file, "三阶段导航状态监控:")
                    log_to_file(log_file, f"   - 入口位置: {nav.entrance_xy}")
                    log_to_file(log_file, f"   - 入口已到达: {nav.entrance_reached}")
                    log_to_file(log_file, f"   - 探索已开始: {nav.exploration_started}")
                    log_to_file(log_file, f"   - 探索已完成: {nav.exploration_complete}")
                    log_to_file(log_file, f"   - 出口位置: {nav.exit_xy}")
                    
                    # 添加入口检测详细信息
                    if nav.entrance_xy:
                        entrance_distance = math.sqrt((actual_robot_pose.x - nav.entrance_xy[0])**2 + 
                                                    (actual_robot_pose.y - nav.entrance_xy[1])**2)
                        log_to_file(log_file, f"   - 距离入口: {entrance_distance:.3f} 米")
                        log_to_file(log_file, f"   - 入口检测阈值: 0.5 米")
                        log_to_file(log_file, f"   - 是否应到达入口: {entrance_distance <= 0.5}")
                
                # 只在每50步添加目标到达检测，减少输出频率
                if step_count % 100 == 1:
                    goal_region = nav.get_goal_region()
                    if goal_region is not None:
                        xmin, xmax, ymin, ymax = goal_region
                        log_to_file(log_file, f"   - 目标区域: x∈[{xmin:.3f},{xmax:.3f}], y∈[{ymin:.3f},{ymax:.3f}]")
                        in_region = nav.is_pose_in_goal_region(actual_robot_pose)
                        log_to_file(log_file, f"   - 已进入目标区域: {in_region}")
                        if in_region:
                            log_to_file(log_file, f"   已进入目标区域！")
                    elif nav.current_goal:
                        goal_distance = math.sqrt((actual_robot_pose.x - nav.current_goal[0])**2 + 
                                                (actual_robot_pose.y - nav.current_goal[1])**2)
                        log_to_file(log_file, f"   - 到全局目标距离: {goal_distance:.3f} 米")
                            
                        # 检查是否到达全局目标
                        if goal_distance <= GOAL_TOLERANCES['default']:  # Use default tolerance from merged config
                            log_to_file(log_file, f"   已到达全局目标！")
                            log_to_file(log_file, f"   - 目标位置: {nav.current_goal}")
                            log_to_file(log_file, f"   - 当前位置: ({actual_robot_pose.x:.3f}, {actual_robot_pose.y:.3f})")
                            log_to_file(log_file, f"   - 距离: {goal_distance:.3f} 米")
                        else:
                            log_to_file(log_file, f"   - 距离目标还有: {goal_distance:.3f} 米")
                    else:
                        log_to_file(log_file, f"   - 无全局目标")
                
                # 入口检测逻辑已简化，直接使用已知坐标
                
                # 只在每50步添加导航系统详细调试，减少输出频率
                if step_count % 100 == 1:
                    log_to_file(log_file, "导航系统详细调试:")
                    log_to_file(log_file, f"   - 探索器状态: {'已初始化' if nav.explorer else '未初始化'}")
                    if nav.explorer:
                        log_to_file(log_file, f"   - 前沿点数量: {len(nav.explorer.frontiers) if hasattr(nav.explorer, 'frontiers') else '未知'}")
                    log_to_file(log_file, f"   - 控制器: {'已初始化' if getattr(nav, 'controller', None) else '未初始化'}")
                    if getattr(nav, 'controller', None):
                        log_to_file(log_file, f"   - 控制器索引: {nav.controller.idx}")
                        tracker_active = nav.controller.idx < len(nav.current_path) if nav.current_path else False
                        log_to_file(log_file, f"   - 跟踪器状态: {'活跃' if tracker_active else '完成'}")
                
                # 删除详细的路径规划调试日志
                    
                # 只在每50步记录路径规划结果，减少输出频率
                if step_count % 100 == 1:
                    if getattr(nav, 'controller', None) and nav.current_path:
                        local_goal = nav.controller.current_waypoint()  # English: use controller's current gate
                        log_to_file(log_file, "路径规划:")
                        log_to_file(log_file, f"   - 局部目标: {local_goal}")
                        log_to_file(log_file, f"   - 路径点数量: {len(nav.current_path)}")
                        if len(nav.current_path) > 0 and getattr(nav, 'controller', None):
                            k = min(nav.controller.idx, len(nav.current_path)-1)
                            log_to_file(log_file, f"   - 下一个路径点: {nav.current_path[k]}")
                                
                            # 添加路径跟踪进度分析
                            path_progress = (nav.controller.idx / len(nav.current_path) * 100) if getattr(nav, 'controller', None) else 0.0
                            log_to_file(log_file, f"   - 路径跟踪进度: {path_progress:.1f}%")
                                
                            # 检查路径跟踪状态
                            if not getattr(nav, 'controller', None) or nav.controller.idx >= len(nav.current_path):
                                log_to_file(log_file, f"   - 路径跟踪完成")
                            else:
                                remaining_points = len(nav.current_path) - nav.controller.idx
                                log_to_file(log_file, f"   - 剩余路径点: {remaining_points}")
                        else:
                            log_to_file(log_file, "🛣️ 路径规划: 无路径")
                    else:
                        log_to_file(log_file, "路径规划状态:")
                        log_to_file(log_file, f"   - 导航状态: {nav.state.name}")
                        log_to_file(log_file, f"   - 当前目标: {nav.current_goal}")
                        log_to_file(log_file, f"   - 控制器: {'已初始化' if getattr(nav, 'controller', None) else '未初始化'}")
                        log_to_file(log_file, f"   - 路径: {'有路径' if nav.current_path else '无路径'}")

                # --- Robot control ---
                # 获取Navigator的控制命令
                nav_cmd = nav.get_control_command()
                
                # 决定最终控制命令
                if nav_cmd and nav_cmd != (0.0, 0.0):
                    # 优先级1：使用Navigator的控制命令
                    linear_vel, angular_vel = nav_cmd
                    
                    if step_count % 100 == 1:
                        log_to_file(log_file, f"使用Navigator控制命令: v={linear_vel:.3f}, w={angular_vel:.3f}")
                else:
                    # 优先级2：Main循环备用控制
                    linear_vel = 0.0
                    angular_vel = 0.0
                    
                    # English: minimal fallback using controller's current waypoint, not the legacy tracker
                    if getattr(nav, 'controller', None) and nav.current_path:
                        wp = nav.controller.current_waypoint()
                        if wp:
                            dx = wp[0] - actual_robot_pose.x
                            dy = wp[1] - actual_robot_pose.y
                            distance = math.hypot(dx, dy)
                            target_angle = math.atan2(dy, dx)
                            angle_diff = math.atan2(math.sin(target_angle - actual_robot_pose.theta), 
                                                   math.cos(target_angle - actual_robot_pose.theta))
                            # P-only minimal fallback
                            linear_vel = min(V_MAX, max(DWA_MIN_V, distance * 0.8))
                            angular_vel = max(-W_MAX, min(W_MAX, angle_diff * 2.0))

                # 速度平滑 (防止突变)
                linear_vel = 0.3 * last_linear_vel + 0.7 * linear_vel
                angular_vel = 0.3 * last_angular_vel + 0.7 * angular_vel

                last_linear_vel = linear_vel
                last_angular_vel = angular_vel

                # 修复：在应用控制前记录当前位置
                pre_control_pose = robot.pose.copy()
                
                # 长时间窗限速：防止单帧大位移
                if actual_dt > 0.15:
                    scale = 0.15 / actual_dt
                    linear_vel *= scale
                    angular_vel *= scale
                    if step_count % 100 == 1:
                        log_to_file(log_file, f"长时间窗限速: actual_dt={actual_dt:.3f}s, 限速比例={scale:.3f}")
                
                # 长帧分块积分：防止长时间窗单次积分造成位姿跳跃
                MAX_INT_DT = 0.10  # 最大单次积分时间
                if actual_dt > MAX_INT_DT:
                    # 将长时间窗分割成多个小步
                    n = int(math.ceil(actual_dt / MAX_INT_DT))
                    sub_dt = actual_dt / n
                    for _ in range(n):
                        robot.update(sub_dt, linear_vel, angular_vel)
                else:
                    # 正常时间窗，直接积分
                    robot.update(actual_dt, linear_vel, angular_vel)
                
                # 获取更新后的位置
                new_pose = robot.pose
                
                # 计算控制后的真实增量
                control_dx = new_pose.x - pre_control_pose.x
                control_dy = new_pose.y - pre_control_pose.y
                control_dtheta = math.atan2(math.sin(new_pose.theta - pre_control_pose.theta),
                                           math.cos(new_pose.theta - pre_control_pose.theta))
                
                # English: do not overwrite odometry
                odom_cmd_delta = (control_dx, control_dy, control_dtheta)  # English: do not overwrite odometry
                
                # 更新 anchor 供 GUI/日志等其他用途
                #robot._last_pose = new_pose.copy()
                
                # 控制效果已在上面计算并存入odom_delta_buf
                
                # 调试信息：控制效果验证
                if step_count % 100 == 1:
                    log_to_file(log_file, "控制效果验证:")
                    log_to_file(log_file, f"   - 控制前位置: ({pre_control_pose.x:.3f}, {pre_control_pose.y:.3f}, {pre_control_pose.theta:.3f})")
                    log_to_file(log_file, f"   - 控制后位置: ({new_pose.x:.3f}, {new_pose.y:.3f}, {new_pose.theta:.3f})")
                    log_to_file(log_file, f"   - 控制增量: (dx={control_dx:.3f}, dy={control_dy:.3f}, dtheta={control_dtheta:.3f})")
                    log_to_file(log_file, f"   - 控制距离: {math.hypot(control_dx, control_dy):.3f} m")
                    log_to_file(log_file, f"   - 速度命令: v={linear_vel:.3f} m/s, w={angular_vel:.3f} rad/s")
                
                # 修复位置变化检测逻辑（优先级1修复）
                position_changed = False  # 默认值
                if last_new_pose is not None:
                    # 比较新位置和上一次位置
                    dx = new_pose.x - last_new_pose.x
                    dy = new_pose.y - last_new_pose.y
                    position_changed = abs(dx) > 0.001 or abs(dy) > 0.001
                    
                    # 只在每50步记录详细的位置信息
                    if step_count % 100 == 1:
                        log_to_file(log_file, "位置变化检测:")
                        log_to_file(log_file, f"   - 当前位置: ({new_pose.x:.3f}, {new_pose.y:.3f})")
                        log_to_file(log_file, f"   - 上次位置: ({last_new_pose.x:.3f}, {last_new_pose.y:.3f})")
                        log_to_file(log_file, f"   - 位置变化: {'是' if position_changed else '否'}")
                        if position_changed:
                            log_to_file(log_file, f"   - 变化量: dx={dx:.3f}, dy={dy:.3f}")
                            log_to_file(log_file, f"   - 移动距离: {math.sqrt(dx*dx + dy*dy):.3f} m")
                        else:
                            log_to_file(log_file, f"   - 位置未变化")
                        
                        # 比较真实位置和SLAM位置
                        slam_dx = new_pose.x - slam_pose.x
                        slam_dy = new_pose.y - slam_pose.y
                        log_to_file(log_file, f"   - SLAM位置: ({slam_pose.x:.3f}, {slam_pose.y:.3f})")
                        log_to_file(log_file, f"   - 位置差异: dx={slam_dx:.3f}, dy={slam_dy:.3f}")
                        log_to_file(log_file, f"   - 位置差异距离: {math.sqrt(slam_dx*slam_dx + slam_dy*slam_dy):.3f} m")
                
                # 更新位置记录
                last_new_pose = new_pose
                
                # 简化的位置更新日志（每步都记录）
                log_to_file(log_file, "小车位置更新:")
                log_to_file(log_file, f"   - 新位置: ({new_pose.x:.3f}, {new_pose.y:.3f})")
                log_to_file(log_file, f"   - 新朝向: {new_pose.theta:.3f} rad ({new_pose.theta*180/3.14159:.1f}°)")
                # 使用真实控制位移而不是里程计增量
                true_move = math.hypot(control_dx, control_dy)
                log_to_file(log_file, f"   - 移动距离: {true_move:.3f} m")
                log_to_file(log_file, f"   - 旋转角度: {control_dtheta:.3f} rad")
                
                # 确认GUI显示的是真实位置
                if step_count % 100 == 1:
                    log_to_file(log_file, f"   - GUI显示位置: ({new_pose.x:.3f}, {new_pose.y:.3f})")
                    log_to_file(log_file, f"   - GUI显示正确: {'是' if position_changed else '否'}")
                    log_to_file(log_file, f"   - 速度命令: 线速度={linear_vel:.3f} m/s, 角速度={angular_vel:.3f} rad/s")
                    
                # 删除详细的地图状态检查日志
                
                # 移除原有的错误位置检测逻辑（优先级1修复）
                # 新的位置检测逻辑已在上方实现

                # --- Logging (lightweight) ---
                logger.log_scan(scan)
                logger.log_pose(actual_robot_pose)  # 使用实际机器人位置
                if time.time() - last_map_snap > 1.0:
                    logger.log_map(slam.get_occ_grid())
                    last_map_snap = time.time()

                # --- GUI (10 Hz) - Phase 2 Optimization: Use blit for fast updates ---
                # English: With blit optimization, 10 Hz is now fast enough (was slow before)
                if time.time() - last_gui > 0.1 and gui is not None:
                    # 确保GUI使用真实机器人位置（优先级1修复）
                    actual_robot_pose = robot.pose  # 获取真实位置
                    local_goal = nav.controller.current_waypoint() if getattr(nav, 'controller', None) else None
                    
                    # 根据导航状态确定全局目标显示
                    if nav.state == NavState.TO_ENTRANCE:
                        global_goal = nav.entrance_xy  # 显示入口目标
                    elif nav.state == NavState.EXPLORING:
                        global_goal = nav.current_goal if nav.current_goal else None  # 显示当前探索目标
                    elif nav.state == NavState.TO_EXIT:
                        global_goal = nav.exit_xy  # 显示出口目标
                    elif nav.state == NavState.RETURN_HOME:
                        global_goal = nav.entrance_xy  # 显示返回入口目标
                    else:
                        global_goal = None
                    
                    # 获取SLAM位姿
                    slam_pose = slam.get_pose()
                    
                    # 设置到达区域显示
                    goal_band_rect = nav.get_goal_band_rect()
                    gui.set_goal_band(goal_band_rect)
                    
                    # 传递统一的 C-space 给 GUI 可视化
                    gui.set_cspace(nav._get_safebuffer_cspace(), SLAM_RESOLUTION)
                    
                    # 获取终点可达区域用于GUI显示
                    target_region = nav.get_target_reachable_region()
                    
                    # 更新GUI（使用真实位置和SLAM位姿）
                    gui.update(actual_robot_pose, slam.get_occ_grid(), nav.current_path, local_goal, global_goal, scan, slam_pose, nav.start_xy, nav.exit_xy, nav.state.name, target_region)
                    
                    # === Phase 1 Optimization: Reduce logging frequency ===
                    # English: Log every 100 steps instead of 50 to reduce I/O overhead
                    if step_count % 100 == 1:
                        log_to_file(log_file, "GUI更新确认:")
                        log_to_file(log_file, f"   - 真实机器人位置: ({actual_robot_pose.x:.3f}, {actual_robot_pose.y:.3f}, {actual_robot_pose.theta:.3f})")
                        log_to_file(log_file, f"   - SLAM估计位置: ({slam_pose.x:.3f}, {slam_pose.y:.3f}, {slam_pose.theta:.3f})")
                        
                        # 计算位姿差异
                        pose_diff = math.sqrt((actual_robot_pose.x - slam_pose.x)**2 + (actual_robot_pose.y - slam_pose.y)**2)
                        angle_diff = abs(actual_robot_pose.theta - slam_pose.theta)
                        angle_diff = min(angle_diff, 2*math.pi - angle_diff)  # Wrap to [0, π]
                        log_to_file(log_file, f"   - 位姿差异: 距离={pose_diff:.3f}m, 角度={angle_diff:.3f}rad ({angle_diff*180/math.pi:.1f}°)")
                        
                        log_to_file(log_file, f"   - 导航状态: {nav.state.name}")
                        log_to_file(log_file, f"   - 全局目标: {global_goal}")
                        log_to_file(log_file, f"   - 局部目标: {local_goal}")
                        log_to_file(log_file, f"   - GUI显示正确: 是")
                    
                    last_gui = time.time()

                # --- Termination ---
                if nav.state == NavState.FINISHED:
                    log_to_file(log_file, "任务完成！三阶段导航已完成")
                    break

                # 帧预算休眠：只在"本帧总耗时 < dt"时睡眠，避免长帧时额外拖长周期
                spent = time.time() - current_time
                if spent < dt:
                    time.sleep(dt - spent)
                    
        except KeyboardInterrupt:
            log_to_file(log_file, "用户中断程序")
        finally:
            logger.save("run_log.npz")
            # 使用适配器的disconnect方法
            robot.disconnect()
            
            # 显示详细栅格地图诊断窗口（在关闭主窗口前）
            if gui is not None and slam is not None:
                print("\n" + "="*60)
                print("正在生成详细栅格地图诊断...")
                print("="*60)
                
                try:
                    # 获取当前地图数据
                    occ_grid = slam.get_occ_grid()
                    
                    # 尝试获取log-odds网格
                    log_odds_grid = None
                    if hasattr(slam, 'mapper') and slam.mapper:
                        if hasattr(slam.mapper, '_lgrid'):
                            log_odds_grid = slam.mapper._lgrid
                    
                    # 获取当前位姿
                    current_pose = slam.pose if hasattr(slam, 'pose') else actual_pose
                    
                    # 显示诊断窗口
                    gui.show_detailed_grid_analysis(occ_grid, log_odds_grid, current_pose)
                    
                except Exception as e:
                    print(f"⚠️ 显示诊断窗口时出错: {e}")
                    import traceback
                    traceback.print_exc()
            
            # Close the GUI properly
            if gui is not None:
                gui.close()
            
            log_to_file(log_file, "=" * 60)
            log_to_file(log_file, f"程序结束 - 总步数: {step_count}")
            log_to_file(log_file, f"日志文件: {log_filename}")
            log_to_file(log_file, "=" * 60)
    
    finally:
        # Ensure log file is closed
        log_file.close()


def _parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--real", action="store_true", help="use real robot via BLE")
    ap.add_argument("--port", type=str, default=None, help="BLE serial port (REAL mode)")
    ap.add_argument("--map", type=str, default=None, help="maze JSON path (SIM mode)")
    return ap.parse_args()


def main():
    args = _parse_args()
    run(use_real_robot=args.real, json_map_path=args.map, port=args.port)


if __name__ == "__main__":
    main()
