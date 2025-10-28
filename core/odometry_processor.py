# ================================
# file: core/odometry_processor.py  
# ================================
"""
Odometry data processing and coordinate transformation utilities
For handling robot odometry data and converting between coordinate systems
"""

from __future__ import annotations
from typing import Optional, List, Dict, Tuple
import math
import time

from core.config import ROBOT_START_X, ROBOT_START_Y, ROBOT_START_THETA


class OdometryState:
    """保存绝对里程计用于差分；保存最近一次增量 - 与test_lidar_mapping.py完全一致"""
    def __init__(self):
        self.has_abs = False
        self.abs_x = 0.0
        self.abs_y = 0.0
        self.abs_th_deg = 0.0
        self.last_ts: Optional[float] = None
        self.delta = (0.0, 0.0, 0.0)  # dx_body, dy_body, dtheta
        # 添加累计里程计角度跟踪，避免双重累加
        self.cumulative_odom_angle_deg = 0.0

    def update_from_absolute(self, x: float, y: float, th_deg: float, t: Optional[float]) -> None:
        if not self.has_abs:
            self.abs_x, self.abs_y, self.abs_th_deg = x, y, th_deg
            self.cumulative_odom_angle_deg = th_deg  # 初始化累计角度
            self.has_abs = True
            self.last_ts = t or time.time()
            self.delta = (0.0, 0.0, 0.0)
            return
        # compute world delta then rotate into previous body frame
        dx_w = x - self.abs_x
        dy_w = y - self.abs_y
        dth = math.radians(th_deg - self.abs_th_deg)
        # normalize
        dth = math.atan2(math.sin(dth), math.cos(dth))
        th_prev = math.radians(self.abs_th_deg)
        c, s = math.cos(th_prev), math.sin(th_prev)
        dx_b =  c * dx_w + s * dy_w
        dy_b = -s * dx_w + c * dy_w
        self.delta = (dx_b, dy_b, dth)
        self.abs_x, self.abs_y, self.abs_th_deg = x, y, th_deg
        self.cumulative_odom_angle_deg = th_deg  # 更新累计角度
        self.last_ts = t or time.time()

    def update_from_delta(self, dx: float, dy: float, dtheta_deg: float) -> None:
        self.delta = (dx, dy, math.radians(dtheta_deg))
        # 更新累计里程计角度
        self.cumulative_odom_angle_deg += dtheta_deg


def parse_odometry_line(line: str) -> Optional[Dict]:
    """解析单行里程计数据 - 与test_lidar_mapping.py完全一致
    返回: {"time": 9834, "l_speed": 0.0, "r_speed": 0.0, "dx": 0.0, "dy": 0.0, 
           "dt": 0.099, "angle": 0.03, "x": 0.0, "y": 0.0, "lidar_status": "OFF"}
    """
    line = line.strip()
    if not line:
        return None
    
    data = {}
    
    # Parse different field types
    if line.startswith("Time:"):
        try:
            data["time"] = float(line.split(":", 1)[1])
            return data
        except (ValueError, IndexError):
            return None
    
    elif line.startswith("L_Speed:") and ",R_Speed:" in line:
        try:
            # L_Speed:0.00,R_Speed:0.00
            parts = line.replace(" ", "").split(",")
            l_speed = float(parts[0].split(":")[1])
            r_speed = float(parts[1].split(":")[1])
            data["l_speed"] = l_speed
            data["r_speed"] = r_speed
            return data
        except (ValueError, IndexError):
            return None
    
    elif line.startswith("ΔX:") or line.startswith("DX:") or line.startswith("dX:"):
        try:
            # ΔX:0.0,ΔY:0.0,dt:0.099
            parts = line.replace(" ", "").replace("Δ", "d").split(",")
            dx_mm = float(parts[0].split(":")[1])
            dy_mm = float(parts[1].split(":")[1])
            dt = float(parts[2].split(":")[1])
            # 里程计数据是毫米，转换为米
            data["dx"] = dx_mm / 1000.0
            data["dy"] = dy_mm / 1000.0
            data["dt"] = dt
            return data
        except (ValueError, IndexError):
            return None
    
    elif line.startswith("Angle:"):
        try:
            data["angle"] = float(line.split(":", 1)[1])
            return data
        except (ValueError, IndexError):
            return None
    
    elif line.startswith("X:") and ",Y:" in line:
        try:
            # X:0.0,Y:0.0
            parts = line.replace(" ", "").split(",")
            x_mm = float(parts[0].split(":")[1])
            y_mm = float(parts[1].split(":")[1])
            # 里程计数据是毫米，转换为米
            data["x"] = x_mm / 1000.0
            data["y"] = y_mm / 1000.0
            return data
        except (ValueError, IndexError):
            return None
    
    elif line.startswith("Lidar:"):
        try:
            status = line.split(":", 1)[1].strip()
            data["lidar_status"] = status
            return data
        except IndexError:
            return None
    
    return None


def accumulate_odometry_data(odom_buffer: List[Dict], odom: OdometryState) -> bool:
    """累积里程计数据，当收集到完整一帧时更新里程计状态 - 与test_lidar_mapping.py完全一致
    返回 True 表示更新了 odom.delta
    """
    if not odom_buffer:
        return False

    # Check if we have complete data (all essential fields)
    has_time = any("time" in item for item in odom_buffer)
    has_position = any("x" in item and "y" in item for item in odom_buffer)
    has_angle = any("angle" in item for item in odom_buffer)
    has_delta = any("dx" in item and "dy" in item for item in odom_buffer)
    
    # Try to use delta data first (more reliable for motion estimation)
    if has_delta:
        for item in odom_buffer:
            if "dx" in item and "dy" in item:
                dx = item["dx"]
                dy = item["dy"]
                dt = item.get("dt", 0.1)
                
                # Get absolute angle if available, turn it into delta-theta (CW positive)
                angle_abs = None
                for angle_item in odom_buffer:
                    if "angle" in angle_item:
                        angle_abs = angle_item["angle"]
                        break
                dtheta_deg = 0.0
                if angle_abs is not None:
                    dtheta_deg = angle_abs - odom.cumulative_odom_angle_deg  # CW delta
                # Update odometry with delta data (body-frame dx,dy + delta-theta)
                odom.update_from_delta(dx, dy, dtheta_deg)
                odom_buffer.clear()
                return True
    
    # Fallback to absolute position data
    if has_time and has_position and has_angle:
        time_val = None
        x_val = None
        y_val = None
        angle_val = None
        
        for item in odom_buffer:
            if "time" in item:
                time_val = item["time"]
            if "x" in item and "y" in item:
                x_val = item["x"]
                y_val = item["y"]
            if "angle" in item:
                angle_val = item["angle"]
        
        if all(v is not None for v in [x_val, y_val, angle_val]):
            odom.update_from_absolute(x_val, y_val, angle_val, time_val)
            odom_buffer.clear()
            return True
    
    return False


def parse_odom_line_or_block(line: str, block_buf: List[Dict], odom: OdometryState) -> bool:
    """解析里程计：支持逐行解析MCU格式 - 与test_lidar_mapping.py完全一致
    返回 True 表示更新了 odom.delta
    """
    # Parse single line
    parsed_data = parse_odometry_line(line)
    if parsed_data is None:
        return False
    
    # Add to buffer
    block_buf.append(parsed_data)
    
    # Try to accumulate and update odometry
    return accumulate_odometry_data(block_buf, odom)


# 世界坐标系统一配置 - 使用ROBOT_START_THETA作为初始朝向基准
START_THETA_RAD = float(ROBOT_START_THETA)  # 统一使用弧度，避免度/弧混用
INIT_XY = (ROBOT_START_X, ROBOT_START_Y)  # 世界坐标初始位置


def odom_to_world_pose(odom_abs_x: float, odom_abs_y: float, odom_theta_cw_deg: float) -> Tuple[float, float, float]:
    """将里程计坐标转换为世界坐标 - 与test_lidar_mapping.py完全一致
    
    Args:
        odom_abs_x: 里程计X坐标（沿初始车头方向）
        odom_abs_y: 里程计Y坐标（沿初始右手方向）
        odom_theta_cw_deg: 里程计角度（顺时针，度）
        
    Returns:
        (x, y, theta) 世界坐标，theta为弧度
    """
    phi = START_THETA_RAD  # 初始朝向偏角，统一使用弧度
    c, s = math.cos(phi), math.sin(phi)
    
    # 位置转换：R(φ0)·[X, Y_right]
    x = INIT_XY[0] + c * odom_abs_x + s * odom_abs_y
    y = INIT_XY[1] + s * odom_abs_x - c * odom_abs_y
    
    # 角度转换：θw = φ0 - θodom (里程计顺时针转世界逆时针)
    theta_world = phi - math.radians(odom_theta_cw_deg)
    
    return (x, y, theta_world)


def update_robot_position_from_odometry(odom: OdometryState, current_pose: List[float]) -> List[float]:
    """基于里程计更新机器人位置 - 使用统一的坐标转换 - 与test_lidar_mapping.py完全一致"""
    # 检查是否有绝对位置数据
    if odom.has_abs:
        # 使用统一的里程计到世界坐标转换
        world_pose = odom_to_world_pose(odom.abs_x, odom.abs_y, odom.cumulative_odom_angle_deg)
        return list(world_pose)
    else:
        # No absolute pose: integrate body-frame delta into world frame
        dx_b, dy_b, dtheta = odom.delta
        th = current_pose[2]               # world heading (CCW, rad)
        c, s = math.cos(th), math.sin(th)
        # body (forward/right) -> world (+X/+Y)
        dx_w =  c * dx_b - s * dy_b
        dy_w =  s * dx_b + c * dy_b
        new_x = current_pose[0] + dx_w
        new_y = current_pose[1] + dy_w
        new_theta = current_pose[2] - dtheta  # odom CW positive → world CCW: minus
        return [new_x, new_y, new_theta]
