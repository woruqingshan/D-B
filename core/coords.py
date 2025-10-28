# ================================
# file: code/core/coords.py
# ================================
from __future__ import annotations
from typing import Tuple, Dict, Optional
import math

# Import configuration parameters
from core.config import WORLD_SIZE, GRID_SIZE, MAP_RES, SLAM_RESOLUTION, SLAM_MAP_SIZE_PIXELS

class CoordinateSystem:
    """统一的坐标系统管理"""
    
    def __init__(self, world_size: float = WORLD_SIZE, grid_size: int = SLAM_MAP_SIZE_PIXELS, logger_func=None, log_file=None):
        self.world_size = world_size
        self.grid_size = grid_size
        self.maze_bounds: Optional[Dict] = None  # 迷宫边界，在加载JSON后设置
        self.logger_func = logger_func
        self.log_file = log_file
        self.res = SLAM_RESOLUTION  # Use SLAM resolution for accurate mapping
        
    def set_maze_bounds(self, maze_bounds: Dict) -> None:
        """设置迷宫边界"""
        self.maze_bounds = maze_bounds
        print(f"[COORD_DEBUG] 设置迷宫边界: {maze_bounds}")
    
    # 世界坐标 ↔ 网格坐标
    def world_to_grid(self, x_world: float, y_world: float) -> Tuple[int, int]:
        """世界坐标转网格坐标 - 使用SLAM分辨率"""
        grid_x = int(round(x_world / self.res))  # Use SLAM_RESOLUTION
        grid_y = int(round(y_world / self.res))
        # 边界检查
        grid_x = max(0, min(grid_x, self.grid_size - 1))
        grid_y = max(0, min(grid_y, self.grid_size - 1))
        
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """网格坐标转世界坐标 - 使用SLAM分辨率"""
        world_x = float(grid_x * self.res)  # Use SLAM_RESOLUTION
        world_y = float(grid_y * self.res)
        
        # 添加调试信息 - 集成到main日志系统
        debug_msg = f"网格坐标({grid_x}, {grid_y}) -> 世界坐标({world_x:.2f}, {world_y:.2f})"
        print(f"[COORDS_DEBUG] {debug_msg}")
        
        # 如果设置了日志函数，则记录到main日志
        if hasattr(self, 'logger_func') and self.logger_func and hasattr(self, 'log_file') and self.log_file:
            self.logger_func(self.log_file, debug_msg, "COORDS")
        
        return (world_x, world_y)
    
    # 迷宫坐标 ↔ 世界坐标
    def maze_to_world(self, maze_x: int, maze_y: int) -> Tuple[float, float]:
        """迷宫坐标转世界坐标"""
        if not self.maze_bounds:
            raise ValueError("迷宫边界未设置")
        
        maze_left = self.maze_bounds['bottom_left'][0]
        maze_bottom = self.maze_bounds['bottom_left'][1]
        
        world_x = maze_left + maze_x
        world_y = maze_bottom + maze_y
        return (world_x, world_y)
    
    def world_to_maze(self, x_world: float, y_world: float) -> Tuple[int, int]:
        """世界坐标转迷宫坐标"""
        if not self.maze_bounds:
            raise ValueError("迷宫边界未设置")
        
        maze_left = self.maze_bounds['bottom_left'][0]
        maze_bottom = self.maze_bounds['bottom_left'][1]
        
        maze_x = int(x_world - maze_left)
        maze_y = int(y_world - maze_bottom)
        return (maze_x, maze_y)
    
    # 迷宫坐标 ↔ 网格坐标
    def maze_to_grid(self, maze_x: int, maze_y: int) -> Tuple[int, int]:
        """迷宫坐标转网格坐标"""
        world_x, world_y = self.maze_to_world(maze_x, maze_y)
        return self.world_to_grid(world_x, world_y)
    
    def grid_to_maze(self, grid_x: int, grid_y: int) -> Tuple[int, int]:
        """网格坐标转迷宫坐标"""
        world_x, world_y = self.grid_to_world(grid_x, grid_y)
        return self.world_to_maze(world_x, world_y)
    
    # 验证函数
    def is_in_maze(self, x_world: float, y_world: float) -> bool:
        """检查世界坐标是否在迷宫内"""
        if not self.maze_bounds:
            return False
        
        bl = self.maze_bounds['bottom_left']
        tr = self.maze_bounds['top_right']
        
        return bl[0] <= x_world <= tr[0] and bl[1] <= y_world <= tr[1]
    
    def is_in_grid(self, grid_x: int, grid_y: int) -> bool:
        """检查网格坐标是否在有效范围内"""
        return 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size
    
    # 调试函数
    def debug_conversion(self, coord_type: str, coord: Tuple, target_type: str) -> Tuple:
        """调试坐标转换"""
        print(f"[COORD_DEBUG] {coord_type} {coord} -> {target_type}")
        
        if coord_type == "world" and target_type == "grid":
            result = self.world_to_grid(*coord)
        elif coord_type == "grid" and target_type == "world":
            result = self.grid_to_world(*coord)
        elif coord_type == "maze" and target_type == "world":
            result = self.maze_to_world(*coord)
        elif coord_type == "world" and target_type == "maze":
            result = self.world_to_maze(*coord)
        elif coord_type == "maze" and target_type == "grid":
            result = self.maze_to_grid(*coord)
        elif coord_type == "grid" and target_type == "maze":
            result = self.grid_to_maze(*coord)
        else:
            raise ValueError(f"不支持的转换: {coord_type} -> {target_type}")
        
        print(f"[COORD_DEBUG] 结果: {result}")
        return result


# 创建全局坐标系统实例
_global_coord_system = CoordinateSystem()

def get_coord_system():
    """获取全局坐标系统实例"""
    return _global_coord_system

# 为了向后兼容，直接导出全局实例
coord_system = _global_coord_system

# 模块级别的便捷函数 - 使用现有的CoordinateSystem方法
def world_to_map(x_world: float, y_world: float) -> Tuple[int, int]:
    """世界坐标转网格坐标 - 使用CoordinateSystem"""
    return _global_coord_system.world_to_grid(x_world, y_world)

def map_to_world(grid_x: int, grid_y: int) -> Tuple[float, float]:
    """网格坐标转世界坐标 - 使用CoordinateSystem"""
    return _global_coord_system.grid_to_world(grid_x, grid_y)