# -*- coding: utf-8 -*-
"""
稀疏雷达数据处理和建图器
Sparse LiDAR Data Processor and Mapper

功能：
- 从JSON/NPZ文件加载雷达数据
- 从稀疏雷达点直接构建占用栅格地图
- 支持log-odds更新和Bresenham射线追踪
- 处理logs文件夹下的原始数据
"""

import json
import os
import numpy as np
import math
import time
from typing import List, Dict, Tuple, Optional, Union
from core.config import (
    ROBOT_START_X, ROBOT_START_Y, ROBOT_START_THETA,
    SLAM_RESOLUTION, SLAM_MAP_SIZE_METERS, SLAM_MAP_SIZE_PIXELS,
    LOG_ODDS_HIT, LOG_ODDS_MISS, LOG_ODDS_MIN, LOG_ODDS_MAX,
    LIDAR_MIN_RANGE, LIDAR_RANGE, SAFE_BUFFER_M
)


class SparseLidarProcessor:
    """稀疏雷达数据处理和建图器"""
    
    def __init__(self, map_size_pixels: int = None, map_size_meters: float = None, 
                 resolution: float = None, robot_pose: Tuple[float, float, float] = None):
        """
        初始化处理器
        
        Args:
            map_size_pixels: 地图像素尺寸 (NxN)，如果为None则从config.py获取
            map_size_meters: 地图物理尺寸 (米)，如果为None则从config.py获取
            resolution: 分辨率 (米/像素)，如果为None则从config.py获取
            robot_pose: 机器人初始位姿 (x, y, theta)，如果为None则从config.py获取
        """
        # 从config.py获取默认参数
        if resolution is None:
            resolution = SLAM_RESOLUTION
        if map_size_meters is None:
            map_size_meters = SLAM_MAP_SIZE_METERS
        if map_size_pixels is None:
            map_size_pixels = SLAM_MAP_SIZE_PIXELS
        if robot_pose is None:
            robot_pose = (ROBOT_START_X, ROBOT_START_Y, ROBOT_START_THETA)
        
        self.N = map_size_pixels
        self.L = map_size_meters
        self.res = resolution
        
        # Log-odds参数 - 从config.py获取
        self.log_hit = LOG_ODDS_HIT      # 命中障碍物的log-odds增量
        self.log_miss = LOG_ODDS_MISS    # 穿越自由空间的log-odds增量
        self.log_min = LOG_ODDS_MIN      # 最小log-odds值
        self.log_max = LOG_ODDS_MAX      # 最大log-odds值
        
        # Log-odds栅格地图
        self.lgrid = np.zeros((self.N, self.N), dtype=np.float32)
        
        # 机器人位姿 (世界坐标系)
        self.robot_pose = robot_pose
        
        # 统计信息
        self.update_count = 0
        self.total_points = 0
        self.total_rays = 0
        
        # 高波特率优化：批量处理参数
        from core.config import HIGH_BAUDRATE_MODE, HIGH_BAUDRATE_BATCH_SIZE
        if HIGH_BAUDRATE_MODE:
            self.batch_size = HIGH_BAUDRATE_BATCH_SIZE
            self.pending_points = []  # 待处理的点缓存
            print(f"[HIGH_BAUD] 稀疏雷达处理器启用批量处理模式，批大小: {self.batch_size}")
        else:
            self.batch_size = 1
            self.pending_points = []
        
        # ========== 优化方案3：时间窗口未知清除策略 ==========
        # 追踪每个角度扇区的最近命中时间，用于无回波扇区清扫
        self.angle_bucket_size = 5  # 每5度一个bucket
        self.angle_buckets = int(360 / self.angle_bucket_size)  # 72个bucket
        self.last_hit_time = np.full(self.angle_buckets, time.time())  # 初始化为当前时间
        self.clear_timeout_s = 2.0  # 2秒内无命中则清扫该扇区
        
        # ========== 优化方案4：角度查找表 ==========
        # 预计算三角函数值，避免重复计算
        self._deg_step = 1.0  # 1度步长
        ds = self._deg_step
        ang = np.deg2rad(np.arange(-180.0, 180.0+ds, ds))
        self._lut_cos = np.cos(ang)
        self._lut_sin = np.sin(ang)
        self._lut_off = 180  # 角度到索引的偏移量
        
        print(f"[PROCESSOR] 初始化稀疏雷达处理器:")
        print(f"  - 地图尺寸: {self.N}x{self.N} pixels")
        print(f"  - 物理尺寸: {self.L:.2f}m x {self.L:.2f}m")
        print(f"  - 分辨率: {self.res:.4f} m/pixel")
        print(f"  - 机器人位姿: ({robot_pose[0]:.3f}, {robot_pose[1]:.3f}, {robot_pose[2]:.3f})")
    
    def load_json_data(self, json_path: str) -> Dict:
        """从JSON文件加载雷达数据"""
        print(f"[PROCESSOR] 加载JSON文件: {json_path}")
        
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        print(f"JSON数据统计:")
        print(f"  - 采集时间: {data.get('time', 'N/A')}")
        print(f"  - 原始行数: {data.get('statistics', {}).get('total_raw_lines', 0)}")
        print(f"  - 雷达点解析: {data.get('statistics', {}).get('lidar_points_parsed', 0)}")
        print(f"  - 稀疏点收集: {data.get('statistics', {}).get('sparse_points_collected', 0)}")
        
        return data
    
    def extract_sparse_points_from_json(self, json_data: Dict) -> List[Dict]:
        """从JSON数据中提取稀疏雷达点"""
        # 方法1：尝试从JSON中的sparse_lidar_points字段获取数据
        sparse_points = json_data.get('sparse_lidar_points', [])
        print(f"[PROCESSOR] 方法1 - 从JSON字段获取稀疏点: {len(sparse_points)} 个")
        
        # 方法2：如果sparse_lidar_points为空，从laser.ranges中提取
        if not sparse_points:
            print("[PROCESSOR] 方法1失败，尝试从laser.ranges中提取...")
            laser_data = json_data.get('laser', {})
            ranges = laser_data.get('ranges', [])
            angle_min = laser_data.get('angle_min', -math.pi)
            angle_increment = laser_data.get('angle_increment', 0.01)
            
            sparse_points = self._extract_sparse_points_from_ranges(ranges, angle_min, angle_increment)
            print(f"[PROCESSOR] 方法2 - 从ranges提取稀疏点: {len(sparse_points)} 个")
        
        # 方法3：如果ranges也为空，从原始行数据中提取
        if not sparse_points:
            print("[PROCESSOR] 方法2失败，尝试从原始行数据中提取...")
            raw_lines = json_data.get('last_raw_lines', [])
            sparse_points = self._extract_sparse_points_from_raw_lines(raw_lines)
            print(f"[PROCESSOR] 方法3 - 从原始行提取稀疏点: {len(sparse_points)} 个")
        
        return sparse_points
    
    def _extract_sparse_points_from_ranges(self, ranges: List[float], angle_min: float, angle_increment: float) -> List[Dict]:
        """从laser.ranges中提取稀疏雷达点"""
        sparse_points = []
        
        for i, range_val in enumerate(ranges):
            # 跳过无效距离
            if not np.isfinite(range_val) or range_val <= 0:
                continue
            
            # 计算角度
            angle_rad = angle_min + i * angle_increment
            angle_deg = math.degrees(angle_rad)
            
            sparse_points.append({
                "angle_deg": angle_deg,
                "distance_m": float(range_val),
                "quality": 50  # 默认质量
            })
        
        return sparse_points
    
    def _extract_sparse_points_from_raw_lines(self, raw_lines: List[str]) -> List[Dict]:
        """从原始行数据中提取稀疏雷达点"""
        sparse_points = []
        
        # --- warm-up gating state (for raw log replay) ---
        WARMUP_S = 1.4  # seconds
        warmup_on = False
        t_on = None
        t_now = None
        # --- end ---
        
        print(f"[PROCESSOR] 开始从 {len(raw_lines)} 行原始数据中提取LIDAR数据...")
        
        for line in raw_lines:
            line = line.strip()
            
            # track MCU time if present: "Time:xxxxx"
            if line.startswith("Time:"):
                # log can be ms or ticks; treat as seconds if value > 1000 then divide by 1000.0
                try:
                    v = float(line.split(":")[1])
                    t_now = v / 1000.0 if v > 1000 else v
                except Exception:
                    pass

            if "Lidar: ON" in line or "Lidar started successfully" in line:
                warmup_on = True
                t_on = t_now  # may be None until next "Time:" appears
            
            if line.startswith("LIDAR"):
                # drop if warm-up is active and we have elapsed time
                if warmup_on and (t_on is not None) and (t_now is not None):
                    if (t_now - t_on) < WARMUP_S:
                        continue
                    else:
                        warmup_on = False  # warm-up finished
                
                # --- direct garbage point filtering ---
                try:
                    # Parse format: "LIDAR Q25 A46.8 D643.25"
                    parts = line.split()
                    if len(parts) >= 4:
                        quality = int(parts[1][1:])      # Q25 -> 25
                        angle_deg = float(parts[2][1:])  # A46.8 -> 46.8
                        distance_mm = float(parts[3][1:]) # D643.25 -> 643.25
                        distance_m = distance_mm / 1000.0  # Convert mm to meters
                        
                        # --- direct garbage point filtering ---
                        if self._is_garbage_point_offline(quality, angle_deg, distance_m):
                            continue  # Skip this garbage point
                        # --- end direct filtering ---
                        
                        # 距离和质量检查
                        if distance_m > 0.001 and quality >= 10:  # 距离大于1mm，质量大于10
                            sparse_points.append({
                                "angle_deg": angle_deg,
                                "distance_m": distance_m,
                                "quality": quality
                            })
                except (ValueError, IndexError) as e:
                    # print(f"[PROCESSOR] 解析失败: {line} - {e}")
                    continue  # Skip invalid lines
        
        print(f"[PROCESSOR] 成功提取 {len(sparse_points)} 个有效稀疏雷达点")
        return sparse_points
    
    def _is_garbage_point_offline(self, quality: int, angle_deg: float, distance_m: float) -> bool:
        """离线解析中的垃圾点识别
        
        Args:
            quality: 雷达点质量
            angle_deg: 雷达点角度（度）
            distance_m: 雷达点距离（米）
            
        Returns:
            True if 是垃圾点，False otherwise
        """
        # 垃圾点模式1: Q16 A64.6 D7764.75 (约7.76米)
        if (abs(quality - 16) <= 2 and 
            abs(angle_deg - 64.6) <= 1.0 and 
            abs(distance_m - 7.76475) <= 0.5):
            return True
            
        # 垃圾点模式2: Q25 A92.8 D643.25 (约0.64米)  
        if (abs(quality - 25) <= 2 and 
            abs(angle_deg - 92.8) <= 1.0 and 
            abs(distance_m - 0.64325) <= 0.1):
            return True
            
        return False
    
    def _is_garbage_point_realtime(self, quality: int, angle_deg: float, distance_m: float) -> bool:
        """实时建图中的垃圾点识别
        
        Args:
            quality: 雷达点质量
            angle_deg: 雷达点角度（度）
            distance_m: 雷达点距离（米）
            
        Returns:
            True if 是垃圾点，False otherwise
        """
        # 垃圾点模式1: Q16 A64.6 D7764.75 (约7.76米)
        if (abs(quality - 16) <= 2 and 
            abs(angle_deg - 64.6) <= 1.0 and 
            abs(distance_m - 7.76475) <= 0.5):
            return True
            
        # 垃圾点模式2: Q25 A92.8 D643.25 (约0.64米)  
        if (abs(quality - 25) <= 2 and 
            abs(angle_deg - 92.8) <= 1.0 and 
            abs(distance_m - 0.64325) <= 0.1):
            return True
            
        return False
    
    def load_npz_data(self, npz_path: str) -> Dict:
        """从NPZ文件加载地图数据"""
        print(f"[PROCESSOR] 加载NPZ文件: {npz_path}")
        
        data = np.load(npz_path, allow_pickle=True)
        
        # 获取地图数据
        occ_grid = data.get('occupancy_grid', data.get('occ_grid', None))
        if occ_grid is None:
            raise ValueError("NPZ文件中没有找到occupancy_grid或occ_grid")
        
        # 获取元数据
        metadata = {}
        if 'metadata' in data:
            metadata = data['metadata'].item()
        
        log_odds_grid = data.get('log_odds_grid', None)
        
        map_info = {
            'occupancy_grid': occ_grid,
            'log_odds_grid': log_odds_grid,
            'metadata': metadata,
            'resolution': metadata.get('resolution', 0.01),
            'map_size_meters': metadata.get('size_meters', 2.0)
        }
        
        print(f"NPZ地图统计:")
        print(f"  - 地图尺寸: {occ_grid.shape}")
        print(f"  - 分辨率: {map_info['resolution']:.4f} m/pixel")
        print(f"  - 物理尺寸: {map_info['map_size_meters']:.2f} m")
        
        return map_info
    
    def process_sparse_points(self, sparse_points: List[Dict], robot_pose: Optional[Tuple[float, float, float]] = None) -> Dict:
        """处理稀疏雷达点并建图"""
        if robot_pose is None:
            robot_pose = self.robot_pose
        
        if not sparse_points:
            print(f"[PROCESSOR] 警告: 没有稀疏雷达点数据")
            return self.export_map()
        
        print(f"[PROCESSOR] 处理 {len(sparse_points)} 个稀疏雷达点")
        print(f"[PROCESSOR] 机器人位姿: ({robot_pose[0]:.3f}, {robot_pose[1]:.3f}, {robot_pose[2]:.3f})")
        
        # 统计有效点
        valid_points = 0
        for point in sparse_points:
            if self._mark_obstacle_point(robot_pose, point):
                valid_points += 1
            self._mark_free_space_ray(robot_pose, point)
        
        self.total_points += valid_points
        self.update_count += 1
        
        print(f"[PROCESSOR] 有效点: {valid_points}/{len(sparse_points)}")
        print(f"[PROCESSOR] 累计更新: {self.update_count} 次，总点数: {self.total_points}")
        
        return self.export_map()
    
    def process_json_file(self, json_path: str, output_dir: str = None) -> Tuple[str, Dict, Dict]:
        """
        处理JSON文件并创建地图
        
        Returns:
            Tuple[str, Dict, Dict]: (输出文件路径, 地图数据, JSON数据)
        """
        print(f"=== 处理JSON文件: {json_path} ===")
        
        # 加载JSON数据
        json_data = self.load_json_data(json_path)
        
        # 提取稀疏点
        sparse_points = self.extract_sparse_points_from_json(json_data)
        
        if not sparse_points:
            print("❌ 无法提取到稀疏雷达点数据")
            return None, None, None
        
        # 处理稀疏点并建图
        print(f"\n=== 开始稀疏雷达建图 ===")
        map_data = self.process_sparse_points(sparse_points)
        
        # 保存地图
        if output_dir is None:
            output_dir = os.path.dirname(json_path)
        
        timestamp = json_data.get('time', time.strftime("%Y%m%d_%H%M%S"))
        output_file = os.path.join(output_dir, f"sparse_map_from_json_{timestamp}.npz")
        os.makedirs(output_dir, exist_ok=True)
        np.savez(output_file, **map_data)
        
        # 打印统计信息
        stats = self.get_statistics()
        print(f"\n建图统计:")
        for key, value in stats.items():
            print(f"  - {key}: {value}")
        
        print(f"\n✅ 地图已保存到: {output_file}")
        
        return output_file, map_data, json_data
    
    def process_npz_file(self, npz_path: str, output_dir: str = None) -> Tuple[str, Dict]:
        """
        处理NPZ文件并转换为稀疏建图格式
        
        Returns:
            Tuple[str, Dict]: (输出文件路径, 地图数据)
        """
        print(f"=== 处理NPZ文件: {npz_path} ===")
        
        # 加载NPZ数据
        map_info = self.load_npz_data(npz_path)
        
        # 保存为稀疏建图格式
        if output_dir is None:
            output_dir = os.path.dirname(npz_path)
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        output_file = os.path.join(output_dir, f"sparse_map_from_npz_{timestamp}.npz")
        os.makedirs(output_dir, exist_ok=True)
        
        sparse_map_data = {
            'occupancy_grid': map_info['occupancy_grid'],
            'log_odds_grid': map_info['log_odds_grid'],
            'metadata': map_info['metadata']
        }
        np.savez(output_file, **sparse_map_data)
        
        print(f"[PROCESSOR] NPZ稀疏地图保存 -> {output_file}")
        
        return output_file, sparse_map_data
    
    def _mark_obstacle_point(self, robot_pose: Tuple[float, float, float], point: Dict) -> bool:
        """标记障碍点"""
        angle_deg = point['angle_deg']
        distance_m = point['distance_m']
        quality = point.get('quality', 0)
        
        # --- direct garbage point filtering ---
        if self._is_garbage_point_realtime(quality, angle_deg, distance_m):
            return False  # Skip garbage points
        # --- end direct filtering ---
        
        # 质量检查
        if quality < 5:  # 质量太低，跳过
            return False
        
        # 距离检查 - 使用config.py参数
        if distance_m < LIDAR_MIN_RANGE or distance_m > LIDAR_RANGE:
            return False
        
        # 统一角度合成：θw + α (只做一次角度转换)
        rx, ry, rtheta = robot_pose  # world pose, rtheta in radians
        alpha = -math.radians(angle_deg)  # hardware CW → math CCW
        world_angle = rtheta + alpha  # only here we add heading
        
        # 调试信息：前5个点的角度转换
        if self.total_points < 5:
            print(f"[ANGLE_FIX] 点{self.total_points}: 原始={angle_deg:.1f}° → 世界角度={math.degrees(world_angle):.1f}°")
        
        # 转换到世界坐标系 - 使用角度查找表优化
        # 将角度转换为查找表索引
        angle_deg_normalized = math.degrees(world_angle) % 360
        if angle_deg_normalized < 0:
            angle_deg_normalized += 360
        idx = int(angle_deg_normalized / self._deg_step) % len(self._lut_cos)
        
        # 使用查找表获取cos和sin值
        c, s = self._lut_cos[idx], self._lut_sin[idx]
        x_world = distance_m * c + rx
        y_world = distance_m * s + ry
        
        # 转换到栅格坐标
        gx = int(round(x_world / self.res))
        gy = int(round(y_world / self.res))
        
        # 检查边界并更新
        if 0 <= gx < self.N and 0 <= gy < self.N:
            self._update_log_odds(gx, gy, self.log_hit)
            return True
        
        return False
    
    def _mark_free_space_ray(self, robot_pose: Tuple[float, float, float], point: Dict):
        """标记自由空间射线（Bresenham算法）"""
        angle_deg = point['angle_deg']
        distance_m = point['distance_m']
        
        # 机器人栅格位置
        rx, ry, rtheta = robot_pose
        gx0 = int(round(rx / self.res))
        gy0 = int(round(ry / self.res))
        
        # 统一角度合成：θw + α (只做一次角度转换)
        alpha = -math.radians(angle_deg)  # hardware CW → math CCW
        world_angle = rtheta + alpha  # only here we add heading
        
        # 使用角度查找表优化
        angle_deg_normalized = math.degrees(world_angle) % 360
        if angle_deg_normalized < 0:
            angle_deg_normalized += 360
        idx = int(angle_deg_normalized / self._deg_step) % len(self._lut_cos)
        
        # 使用查找表获取cos和sin值
        c, s = self._lut_cos[idx], self._lut_sin[idx]
        x_world = distance_m * c + rx
        y_world = distance_m * s + ry
        
        gx1 = int(round(x_world / self.res))
        gy1 = int(round(y_world / self.res))
        
        # Bresenham射线追踪
        self._bresenham_ray(gx0, gy0, gx1, gy1)
    
    def _bresenham_ray(self, x0: int, y0: int, x1: int, y1: int):
        """Bresenham射线追踪算法"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            # 跳过起点和终点
            if (x != x0 or y != y0) and (x != x1 or y != y1):
                if 0 <= x < self.N and 0 <= y < self.N:
                    self._update_log_odds(x, y, self.log_miss)
            
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def _bresenham_ray_optimized(self, x0: int, y0: int, x1: int, y1: int, mark_endpoint: bool = False):
        """优化的Bresenham射线追踪算法 - 解决"只打点不挖线"问题
        
        Args:
            x0, y0: 起点坐标（机器人位置）
            x1, y1: 终点坐标（命中点或最大距离点）
            mark_endpoint: 是否标记终点（障碍物需要，自由区域不需要）
        """
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            # 优化：只跳过起点（机器人位置），不跳过终点
            # 这样确保整条射线路径都被标记为FREE
            if x != x0 or y != y0:  # 跳过起点，但不跳过终点
                if 0 <= x < self.N and 0 <= y < self.N:
                    # 如果终点是障碍物，只有在不是终点时才标记为FREE
                    if not (x == x1 and y == y1 and mark_endpoint):
                        self._update_log_odds(x, y, self.log_miss)
            
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
    
    def _update_log_odds(self, gx: int, gy: int, delta: float):
        """更新log-odds值"""
        old_val = self.lgrid[gy, gx]
        new_val = np.clip(old_val + delta, self.log_min, self.log_max)
        self.lgrid[gy, gx] = new_val
    
    def _clear_robot_footprint(self, pose: Tuple[float, float, float], r_clear_m: float = 0.18):
        """
        机器人脚印清空功能
        
        作用：强制机器人周围区域为自由空间，解决"前向黑洞"导致的连通性断裂问题
        确保BFS连通域大于1个自由格，前沿点探索不会因机器人周围未知而失败
        
        Args:
            pose: 机器人位姿 (x, y, theta_rad)
            r_clear_m: 清空半径（米），默认18cm
        """
        rx, ry, _ = pose
        
        # 转换到栅格坐标
        gx = int(round(rx / self.res))
        gy = int(round(ry / self.res))
        r = max(1, int(round(r_clear_m / self.res)))
        
        # 计算清空区域边界
        x0 = max(0, gx - r)
        x1 = min(self.N - 1, gx + r)
        y0 = max(0, gy - r)
        y1 = min(self.N - 1, gy + r)
        
        # 在圆形区域内清空障碍 - 向量化优化
        import numpy as np
        
        # 创建坐标网格
        yy, xx = np.ogrid[y0:y1+1, x0:x1+1]
        
        # 计算圆形掩码
        mask = (xx - gx)**2 + (yy - gy)**2 <= r*r
        
        # 向量化更新log-odds值
        if np.any(mask):
            # 批量更新log-odds值
            self.lgrid[y0:y1+1, x0:x1+1][mask] = np.clip(
                self.lgrid[y0:y1+1, x0:x1+1][mask] + self.log_miss, 
                self.log_min, 
                self.log_max
            )
            cleared_count = np.sum(mask)
        else:
            cleared_count = 0
        
        if cleared_count > 0 and self.total_points % 50 == 0:  # 每50个点打印一次调试信息
            print(f"[FOOTPRINT_CLEAR] 清空机器人周围 {cleared_count} 个栅格，半径={r_clear_m:.2f}m")
    
    def add_virtual_free_cone(self, pose: Tuple[float, float, float], heading_rad: float, 
                             d_free: float = 0.35, fov_deg: int = 60, step_deg: int = 2):
        """
        前向无回波扇区补偿功能
        
        作用：对330°~30°范围内无雷达点的扇区，以保守距离d_free标记为自由空间
        打通前向局部连通性，解决稀疏雷达数据导致的"未知=不可通行"问题
        
        Args:
            pose: 机器人位姿 (x, y, theta_rad)
            heading_rad: 机器人朝向角度（弧度）
            d_free: 自由空间假设距离（米），建议≤走廊半宽0.35m
            fov_deg: 前方视野角度范围（度），默认±30°
            step_deg: 角度步长（度），默认2°精度
        """
        rx, ry, _ = pose
        
        # 转换到栅格坐标
        gx0 = int(round(rx / self.res))
        gy0 = int(round(ry / self.res))
        
        # 计算角度范围
        half_fov = math.radians(fov_deg / 2.0)
        ang_start = heading_rad - half_fov
        ang_end = heading_rad + half_fov
        step_rad = math.radians(step_deg)
        
        # 在指定角度范围内创建虚拟自由射线
        virtual_rays = 0
        ang = ang_start
        while ang <= ang_end:
            # 计算射线终点世界坐标
            xw = rx + d_free * math.cos(ang)
            yw = ry + d_free * math.sin(ang)
            
            # 转换到栅格坐标
            gx1 = int(round(xw / self.res))
            gy1 = int(round(yw / self.res))
            
            # 只画自由空间射线，不标记终点为障碍物
            # 这确保了在无回波区域也标记为自由空间，但真实点命中时会用log_hit覆盖
            if (0 <= gx1 < self.N and 0 <= gy1 < self.N and 
                0 <= gx0 < self.N and 0 <= gy0 < self.N):
                self._bresenham_ray(gx0, gy0, gx1, gy1)
                virtual_rays += 1
            
            ang += step_rad
        
        if virtual_rays > 0:
            print(f"[VIRTUAL_FREE_CONE] 添加 {virtual_rays} 条虚拟自由射线，"
                  f"角度范围={math.degrees(ang_start):.1f}°~{math.degrees(ang_end):.1f}°，"
                  f"距离={d_free:.2f}m")
    
    def get_occupancy_grid(self) -> np.ndarray:
        """获取占用栅格地图"""
        # 转换为占用栅格：0=自由, 1=障碍, 2=未知
        occ_grid = np.full((self.N, self.N), 2, dtype=np.uint8)
        
        # 根据log-odds阈值判断
        free_mask = self.lgrid < -0.5
        occ_mask = self.lgrid > 0.5
        
        occ_grid[free_mask] = 0  # 自由
        occ_grid[occ_mask] = 1   # 障碍
        
        # ========== 优化方案2：结构化后处理 ==========
        # 应用闭运算和安全膨胀，消除噪声和薄缝
        return self._postprocess_binary_map(occ_grid)
    
    def _postprocess_binary_map(self, occ_bin: np.ndarray) -> np.ndarray:
        """结构化后处理：闭运算 + 安全膨胀
        
        Args:
            occ_bin: 原始二值地图 (0=自由, 1=障碍, 2=未知)
            
        Returns:
            处理后的二值地图
        """
        # 只处理障碍区域，保持未知区域不变
        processed = occ_bin.copy()
        
        # 提取障碍区域 (1)
        obstacle_mask = (occ_bin == 1).astype(np.uint8)
        
        if np.any(obstacle_mask):
            # 1) 闭运算：先膨胀后腐蚀，消除小孔和薄缝
            closed = self._closing_operation(obstacle_mask)
            
            # 2) 安全膨胀：根据SAFE_BUFFER_M进行膨胀
            inflate_px = max(1, int(np.ceil(SAFE_BUFFER_M / self.res)))
            inflated = self._dilate(closed, inflate_px)
            
            # 更新处理后的地图
            # 将膨胀后的障碍区域标记为1，其他保持不变
            processed[inflated == 1] = 1
            
            # 调试信息
            if self.total_points % 100 == 0:  # 每100个点打印一次
                original_obs = np.sum(obstacle_mask)
                processed_obs = np.sum(processed == 1)
                print(f"[POST_PROCESS] 障碍膨胀: {original_obs} -> {processed_obs} 像素, 膨胀半径={inflate_px}px")
        
        return processed
    
    def _maxfilter3(self, a: np.ndarray) -> np.ndarray:
        """3x3膨胀（最大值滤波）"""
        if a.size == 0:
            return a
        
        # 使用padding避免边界问题
        p = np.pad(a, 1, mode='constant', constant_values=0)
        out = a.copy()
        
        # 3x3膨胀核
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                neighbor = p[1+dy:1+dy+a.shape[0], 1+dx:1+dx+a.shape[1]]
                out = np.maximum(out, neighbor)
        
        return out
    
    def _minfilter3(self, a: np.ndarray) -> np.ndarray:
        """3x3腐蚀（最小值滤波）"""
        if a.size == 0:
            return a
        
        # 使用padding避免边界问题，pad with 1 to avoid edge effects
        p = np.pad(a, 1, mode='constant', constant_values=1)
        out = a.copy()
        
        # 3x3腐蚀核
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                neighbor = p[1+dy:1+dy+a.shape[0], 1+dx:1+dx+a.shape[1]]
                out = np.minimum(out, neighbor)
        
        return out
    
    def _closing_operation(self, binary_map: np.ndarray) -> np.ndarray:
        """闭运算：先膨胀后腐蚀，消除小孔和薄缝"""
        # 膨胀
        dilated = self._maxfilter3(binary_map)
        # 腐蚀
        closed = self._minfilter3(dilated)
        return closed
    
    def _dilate(self, binary_map: np.ndarray, iterations: int) -> np.ndarray:
        """多轮膨胀"""
        result = binary_map.copy()
        for _ in range(iterations):
            result = self._maxfilter3(result)
        return result
    
    def get_log_odds_grid(self) -> np.ndarray:
        """获取log-odds栅格"""
        return self.lgrid.copy()
    
    def get_statistics(self) -> Dict:
        """获取建图统计信息"""
        occ_grid = self.get_occupancy_grid()
        
        return {
            'update_count': self.update_count,
            'total_points': self.total_points,
            'total_rays': self.total_rays,
            'free_cells': int(np.sum(occ_grid == 0)),
            'occupied_cells': int(np.sum(occ_grid == 1)),
            'unknown_cells': int(np.sum(occ_grid == 2)),
            'coverage_ratio': float(np.sum(occ_grid != 2) / occ_grid.size),
            'resolution': self.res,
            'map_size_meters': self.L,
            'map_size_pixels': self.N
        }
    
    def export_map(self) -> Dict:
        """导出地图数据"""
        return {
            'occupancy_grid': self.get_occupancy_grid(),
            'log_odds_grid': self.get_log_odds_grid(),
            'metadata': {
                'resolution': self.res,
                'size': (self.N, self.N),
                'size_meters': self.L,
                'origin': (0.0, 0.0),
                'update_count': self.update_count,
                'total_points': self.total_points,
                'coverage_ratio': self.get_statistics()['coverage_ratio']
            }
        }
    
    def create_map_info_for_visualization(self) -> Dict:
        """为可视化创建地图信息"""
        return {
            'occ_grid': self.get_occupancy_grid(),
            'resolution': self.res,
            'map_size_meters': self.L,
            'log_odds_grid': self.get_log_odds_grid()
        }
    
    def add_single_sparse_point(self, angle_deg: float, distance_m: float, quality: int = 50) -> np.ndarray:
        """实时添加单个稀疏雷达点并更新地图
        
        Args:
            angle_deg: 雷达角度（度）
            distance_m: 距离（米）
            quality: 质量值
            
        Returns:
            更新后的占用栅格地图
        """
        # 创建点字典
        point = {
            'angle_deg': angle_deg,
            'distance_m': distance_m,
            'quality': quality
        }
        
        # 使用现有的处理逻辑
        if self._mark_obstacle_point(self.robot_pose, point):
            self.total_points += 1
        self._mark_free_space_ray(self.robot_pose, point)
        
        # 返回更新后的地图 - 优化：避免每点都做全图处理
        # return self.get_occupancy_grid()  # 删除：避免O(N²)复杂度
        return self.lgrid  # 直接返回log-odds网格，GUI更新时再调用get_occupancy_grid()
    
    def set_robot_pose(self, pose: Tuple[float, float, float]):
        """更新机器人位姿
        
        Args:
            pose: (x, y, theta) in world frame, theta in radians
        """
        self.robot_pose = (float(pose[0]), float(pose[1]), float(pose[2]))
    
    def add_single_sparse_point_with_pose(self, angle_deg: float, distance_m: float, 
                                        quality: int, robot_pose: Tuple[float, float, float]) -> np.ndarray:
        """带位姿的实时建图方法 - 优化版本：射线刻蚀 + 无命中处理
        
        Args:
            angle_deg: 雷达角度（度），车体系，逆时针为正
            distance_m: 距离（米）
            quality: 质量值
            robot_pose: 机器人世界位姿 (x, y, theta_rad)
            
        Returns:
            更新后的占用栅格地图
        """
        # 保存旧位姿
        old_pose = self.robot_pose
        
        # 更新位姿
        self.robot_pose = robot_pose
        
        # ========== 修改点1：每N个点清空一次机器人脚印 ==========
        # 目的：解决"前向黑洞"导致的连通性断裂，确保BFS连通域>1个自由格
        if self.total_points % 80 == 0:  # 优化：从每5个点改为每80个点，减少频繁操作
            self._clear_robot_footprint(robot_pose)
        
        # ========== 优化方案1：射线刻蚀 + 无命中处理 ==========
        rx, ry, rtheta = robot_pose
        gx0 = int(round(rx / self.res))
        gy0 = int(round(ry / self.res))
        
        # 统一角度合成
        alpha = -math.radians(angle_deg)  # hardware CW → math CCW
        world_angle = rtheta + alpha
        
        # 检查是否为有效命中点
        is_valid_hit = (quality >= 5 and 
                       LIDAR_MIN_RANGE <= distance_m <= LIDAR_RANGE and 
                       np.isfinite(distance_m))
        
        # ========== 优化方案3：时间窗口未知清除策略 ==========
        current_time = time.time()
        
        # 计算角度bucket索引 (标准化到0-360度范围)
        normalized_angle = angle_deg % 360
        bucket_idx = int(normalized_angle / self.angle_bucket_size) % self.angle_buckets
        
        if is_valid_hit:
            # 有效命中：先进行射线FREE刻蚀，再标记障碍点
            x_world = distance_m * math.cos(world_angle) + rx
            y_world = distance_m * math.sin(world_angle) + ry
            gx_hit = int(round(x_world / self.res))
            gy_hit = int(round(y_world / self.res))
            
            # 射线FREE刻蚀：从机器人位置到命中点的整条射线
            self._bresenham_ray_optimized(gx0, gy0, gx_hit, gy_hit, mark_endpoint=False)
            
            # 标记障碍点
            if 0 <= gx_hit < self.N and 0 <= gy_hit < self.N:
                self._update_log_odds(gx_hit, gy_hit, self.log_hit)
                self.total_points += 1
            
            # 更新该角度bucket的最后命中时间
            self.last_hit_time[bucket_idx] = current_time
        else:
            # 无命中处理：检查是否需要清扫该角度扇区
            time_since_last_hit = current_time - self.last_hit_time[bucket_idx]
            
            if time_since_last_hit > self.clear_timeout_s:
                # 超过时间阈值，清扫该角度扇区
                max_range_world = LIDAR_RANGE
                gx_far = int(round((rx + max_range_world * math.cos(world_angle)) / self.res))
                gy_far = int(round((ry + max_range_world * math.sin(world_angle)) / self.res))
                self._bresenham_ray_optimized(gx0, gy0, gx_far, gy_far, mark_endpoint=False)
                
                # 重置该bucket的时间
                self.last_hit_time[bucket_idx] = current_time
                
                # 调试信息
                if self.total_points % 50 == 0:
                    print(f"[TIME_CLEAR] 角度{angle_deg:.1f}°超时清扫，距离上次命中{time_since_last_hit:.1f}s")
        
        # 返回更新后的地图 - 优化：避免每点都做全图处理
        # return self.get_occupancy_grid()  # 删除：避免O(N²)复杂度
        return self.lgrid  # 直接返回log-odds网格，GUI更新时再调用get_occupancy_grid()
    
    def get_realtime_map_info(self) -> Dict:
        """获取实时建图信息（用于可视化）"""
        occ_grid = self.get_occupancy_grid()
        stats = self.get_statistics()
        
        return {
            'occupancy_grid': occ_grid,
            'log_odds_grid': self.get_log_odds_grid(),
            'resolution': self.res,
            'map_size_meters': self.L,
            'robot_pose': self.robot_pose,
            'statistics': stats,
            'update_count': self.update_count,
            'total_points': self.total_points
        }
    
    def flush_pending_points(self) -> bool:
        """强制处理所有待处理的点（用于建图窗口结束时）"""
        if not self.pending_points:
            return True
        
        result = self._process_batch()
        return result
    
    def _process_batch(self) -> bool:
        """批量处理雷达点（高波特率优化）"""
        if not self.pending_points:
            return False
        
        # 批量转换到世界坐标系
        world_points = []
        for angle_deg, distance_m, quality in self.pending_points:
            alpha = -math.radians(angle_deg)
            world_angle = self.robot_pose[2] + alpha
            world_x = self.robot_pose[0] + distance_m * math.cos(world_angle)
            world_y = self.robot_pose[1] + distance_m * math.sin(world_angle)
            world_points.append((world_x, world_y))
        
        # 批量栅格坐标转换
        gx0 = int(round(self.robot_pose[0] / self.res))
        gy0 = int(round(self.robot_pose[1] / self.res))
        
        valid_count = 0
        for i, (world_x, world_y) in enumerate(world_points):
            gx = int(round(world_x / self.res))
            gy = int(round(world_y / self.res))
            
            if 0 <= gx < self.N and 0 <= gy < self.N:
                # 标记障碍物点
                self._mark_obstacle_point(gx, gy)
                # 标记自由空间射线
                self._mark_free_space_ray(gx0, gy0, gx, gy)
                valid_count += 1
        
        # 更新统计
        self.total_points += valid_count
        self.update_count += 1
        
        # 清空缓存
        self.pending_points.clear()
        
        return valid_count > 0
    


if __name__ == "__main__":
    print("SparseLidarProcessor - 稀疏雷达数据处理和建图器")
    print("使用方法:")
    print("  from sparse_lidar_processor import SparseLidarProcessor")
    print("  processor = SparseLidarProcessor()")
    print("  result = processor.process_json_file('your_file.json')")
