# ================================
# file: core/map_validator.py
# ================================
from __future__ import annotations
"""Map validation module for robot navigation system.
Handles maze construction validation, entry detection, and coordinate conversion verification.
"""
import math
from typing import Optional, Tuple, Dict, Any

from core.coords import coord_system


class MapValidator:
    """Handles map validation, entry detection, and coordinate verification."""

    def __init__(self, logger_func=None):
        self.logger_func = logger_func
        self.log_file = None

    def _log(self, message: str, module: str = "MAP_VALID") -> None:
        """Log message using the provided logger function"""
        if self.logger_func and self.log_file:
            self.logger_func(self.log_file, message, module)

    def validate_map_and_entry(self, maze, maze_info: Dict[str, Any], 
                              maze_entry: Tuple[float, float], 
                              start_x: float, start_y: float,
                              log_file=None) -> Tuple[bool, Optional[Tuple[float, float]]]:
        """Validate map construction and detect maze entry intelligently.
        
        Parameters
        ----------
        maze : MazeMap
            The maze object containing the grid
        maze_info : Dict[str, Any]
            Maze information dictionary
        maze_entry : Tuple[float, float]
            Original maze entry coordinates
        start_x, start_y : float
            Robot start position
        log_file : file
            Log file handle for logging
            
        Returns
        -------
        Tuple[bool, Optional[Tuple[float, float]]]
            (validation_success, corrected_entry)
        """
        self.log_file = log_file
        
        self._log("=" * 60)
        self._log("地图构建验证和智能入口检测")
        self._log("=" * 60)
        
        # Coordinate conversion verification
        self._validate_coordinate_conversion(maze)
        
        # Map construction validation
        map_valid = self._validate_map_construction(maze_info)
        
        # Entry point validation and correction
        entry_valid, corrected_entry = self._validate_entry_point(
            maze, maze_entry, start_x, start_y
        )
        
        # Path accessibility check
        path_accessible = self._check_path_accessibility(
            maze, start_x, start_y, corrected_entry or maze_entry
        )
        
        # Overall validation result
        validation_success = map_valid and entry_valid and path_accessible
        
        self._log("=" * 60)
        self._log(f"地图验证结果: {'通过' if validation_success else '失败'}")
        if validation_success:
            self._log("智能入口检测成功，可以安全导航")
        else:
            self._log("智能入口检测失败，需要检查地图构建或手动设置导航目标")
        self._log("=" * 60)
        
        return validation_success, corrected_entry

    def _validate_coordinate_conversion(self, maze) -> None:
        """Validate coordinate conversion accuracy."""
        self._log("=" * 60)
        self._log("坐标转换验证和障碍物膨胀检查")
        self._log("=" * 60)
        
        # Key positions to verify
        key_positions = [
            (6.0, 2.0, "机器人起始位置"),
            (6.0, 3.0, "路径点1"),
            (6.0, 4.0, "路径点2"),
            (6.0, 5.0, "迷宫入口"),
            (5.0, 5.0, "迷宫左下角"),
            (20.0, 20.0, "迷宫右上角"),
            (12.5, 12.5, "迷宫中心")
        ]
        
        self._log("坐标转换验证:")
        for world_x, world_y, desc in key_positions:
            grid_x, grid_y = coord_system.world_to_grid(world_x, world_y)
            back_world_x, back_world_y = coord_system.grid_to_world(grid_x, grid_y)
            
            self._log(f"  {desc}:")
            self._log(f"    世界坐标: ({world_x:.3f}, {world_y:.3f})")
            self._log(f"    网格坐标: ({grid_x}, {grid_y})")
            self._log(f"    反向转换: ({back_world_x:.3f}, {back_world_y:.3f})")
            
            # Check if grid coordinates are within valid range
            if 0 <= grid_x < 25 and 0 <= grid_y < 25:
                cell_value = maze.grid[grid_y, grid_x]
                status = "障碍" if cell_value == 1 else "自由" if cell_value == 0 else "未知"
                self._log(f"    网格状态: {status} (值:{cell_value})")
            else:
                self._log(f"    网格状态: 超出范围")
            
            # Check conversion precision
            x_error = abs(world_x - back_world_x)
            y_error = abs(world_y - back_world_y)
            self._log(f"    转换误差: dx={x_error:.6f}, dy={y_error:.6f}")
        
        # Obstacle inflation check
        self._log("障碍物膨胀检查:")
        from core.config import ROBOT_RADIUS, SAFE_BUFFER_M, MAP_RES
        self._log(f"  - ROBOT_RADIUS: {ROBOT_RADIUS} m (planning radius, point mass model)")
        self._log(f"  - SAFE_BUFFER_M: {SAFE_BUFFER_M} m (includes physical radius)")
        self._log(f"  - MAP_RES: {MAP_RES} m")
        
        # Check inflation parameter calculation
        inflate_cells_old = max(1, int(SAFE_BUFFER_M / MAP_RES))
        inflate_cells_new = max(0, int(SAFE_BUFFER_M / MAP_RES) - 1)
        self._log(f"  - 膨胀网格数(旧): {inflate_cells_old}")
        self._log(f"  - 膨胀网格数(新): {inflate_cells_new}")
        
        # Check obstacle inflation effects at key positions
        test_positions = [(6.0, 3.5, "机器人卡死位置")]
        for world_x, world_y, desc in test_positions:
            grid_x, grid_y = coord_system.world_to_grid(world_x, world_y)
            self._log(f"  {desc} ({world_x:.1f}, {world_y:.1f}):")
            self._log(f"    网格坐标: ({grid_x}, {grid_y})")
            
            # Check obstacle detection with different inflation parameters
            for inflate in [0, 1, 2]:
                is_obstacle = maze.is_obstacle_world(world_x, world_y, inflate_cells=inflate)
                self._log(f"    膨胀{inflate}网格: {'障碍' if is_obstacle else '自由'}")
            
            # Check surrounding area
            self._log(f"    周围区域检查:")
            for dy in [-2, -1, 0, 1, 2]:
                for dx in [-2, -1, 0, 1, 2]:
                    check_x, check_y = grid_x + dx, grid_y + dy
                    if 0 <= check_x < 25 and 0 <= check_y < 25:
                        cell_value = maze.grid[check_y, check_x]
                        status = "障碍" if cell_value == 1 else "自由" if cell_value == 0 else "未知"
                        if cell_value == 1:  # Only show obstacles
                            self._log(f"      网格({check_x},{check_y}): {status}")
        
        self._log("=" * 60)

    def _validate_map_construction(self, maze_info: Dict[str, Any]) -> bool:
        """Validate map construction parameters."""
        self._log("1. 地图构建验证:")
        self._log(f"   - 迷宫尺寸: {maze_info['json_size'][0]} x {maze_info['json_size'][1]}")
        self._log(f"   - 墙壁网格数: {maze_info['wall_count']}")
        self._log(f"   - 自由空间网格数: {maze_info['free_count']}")
        self._log(f"   - 迷宫边界: 左下{maze_info['bounds']['bottom_left']}, 右上{maze_info['bounds']['top_right']}")
        
        # Basic validation checks
        if maze_info['wall_count'] <= 0:
            self._log("   ⚠️  墙壁数量异常")
            return False
        
        if maze_info['free_count'] <= 0:
            self._log("   ⚠️  自由空间数量异常")
            return False
        
        self._log("   ✅ 地图构建参数正常")
        return True

    def _validate_entry_point(self, maze, maze_entry: Tuple[float, float], 
                             start_x: float, start_y: float) -> Tuple[bool, Optional[Tuple[float, float]]]:
        """Validate and potentially correct the maze entry point."""
        # Check entry point status
        entry_grid_x, entry_grid_y = coord_system.world_to_grid(maze_entry[0], maze_entry[1])
        self._log(f"2. 入口点检查:")
        self._log(f"   - JSON入口坐标: {maze_entry}")
        self._log(f"   - 入口网格坐标: ({entry_grid_x}, {entry_grid_y})")
        
        # Check if entry point is accessible
        if 0 <= entry_grid_x < 25 and 0 <= entry_grid_y < 25:
            entry_value = maze.grid[entry_grid_y, entry_grid_x]
            entry_accessible = entry_value == 0 or entry_value == 2  # Free or unknown
            self._log(f"   - 入口点状态: {entry_value} (0=自由, 1=障碍, 2=未知)")
            self._log(f"   - 入口可达性: {'是' if entry_accessible else '否'}")
        else:
            entry_accessible = False
            self._log(f"   - 入口点超出地图范围")
        
        # Try to correct entry point if it's blocked
        corrected_entry = None
        if not entry_accessible and entry_value == 1:
            self._log(f"   - 尝试修复入口点")
            if 0 <= entry_grid_x < 25 and 0 <= entry_grid_y < 25:
                maze.grid[entry_grid_y, entry_grid_x] = 0
                self._log(f"   - 已将入口点({entry_grid_x},{entry_grid_y})设置为自由空间")
                corrected_entry = maze_entry
                entry_accessible = True
        
        return entry_accessible, corrected_entry

    def _check_path_accessibility(self, maze, start_x: float, start_y: float, 
                                 entry_point: Tuple[float, float]) -> bool:
        """Check if there's a clear path from start to entry point."""
        self._log("3. 智能入口检测:")
        
        # Check path from start to entry
        start_grid_x, start_grid_y = coord_system.world_to_grid(start_x, start_y)
        entry_grid_x, entry_grid_y = coord_system.world_to_grid(entry_point[0], entry_point[1])
        
        self._log("4. 起点到入口路径检查:")
        self._log(f"   - 起点网格坐标: ({start_grid_x}, {start_grid_y})")
        self._log(f"   - 入口网格坐标: ({entry_grid_x}, {entry_grid_y})")
        self._log(f"   - 直线距离: {((entry_grid_x-start_grid_x)**2 + (entry_grid_y-start_grid_y)**2)**0.5:.1f} 网格")
        
        # Check key points along the path
        path_points = []
        path_obstacles = 0
        steps = max(abs(entry_grid_x - start_grid_x), abs(entry_grid_y - start_grid_y))
        if steps > 0:
            for i in range(1, steps + 1):
                t = i / steps
                check_x = int(start_grid_x + t * (entry_grid_x - start_grid_x))
                check_y = int(start_grid_y + t * (entry_grid_y - start_grid_y))
                path_points.append((check_x, check_y))
                
                # Check path point status
                if 0 <= check_x < 25 and 0 <= check_y < 25:
                    path_value = maze.grid[check_y, check_x]
                    if path_value == 1:
                        path_obstacles += 1
        
        self._log(f"   - 路径关键点: {path_points}")
        self._log(f"   - 路径上障碍物数量: {path_obstacles}")
        
        # Check start and entry point status
        start_value = maze.grid[start_grid_y, start_grid_x] if 0 <= start_grid_x < 25 and 0 <= start_grid_y < 25 else -1
        entry_value = maze.grid[entry_grid_y, entry_grid_x] if 0 <= entry_grid_x < 25 and 0 <= entry_grid_y < 25 else -1
        
        self._log(f"   - 起点({start_grid_x},{start_grid_y})状态: {start_value} (0=自由, 1=障碍, 2=未知)")
        self._log(f"   - 入口({entry_grid_x},{entry_grid_y})状态: {entry_value} (0=自由, 1=障碍, 2=未知)")
        
        # Smart entry validation: based on path accessibility
        self._log("5. 智能入口验证:")
        
        # Check accessibility
        path_accessible = path_obstacles == 0
        start_accessible = start_value == 0
        entry_accessible = entry_value == 0 or entry_value == 2  # Entry can be free or unknown
        
        self._log(f"   - 路径可达性: {'是' if path_accessible else '否'}")
        self._log(f"   - 起点可达性: {'是' if start_accessible else '否'}")
        self._log(f"   - 入口可达性: {'是' if entry_accessible else '否'}")
        
        # Smart entry detection logic
        if path_accessible and start_accessible and entry_accessible:
            self._log(f"   智能入口检测通过：路径畅通，可以导航")
            return True
        else:
            self._log(f"   智能入口检测发现问题")
            
            if not path_accessible:
                self._log(f"   - 路径上有{path_obstacles}个障碍物")
            if not start_accessible:
                self._log(f"   - 起点被障碍物占据")
            if not entry_accessible:
                self._log(f"   - 入口被障碍物占据")
            
            return False

    def detect_maze_entry_from_lidar(self, scan, robot_pose, maze_info) -> Optional[Tuple[float, float]]:
        """
        Detect maze entry from LiDAR scan data.
        Identifies entry by analyzing "channel" patterns in scan data.
        """
        if scan is None or len(scan.ranges) == 0:
            return None
        
        # Analyze scan data to find possible entry channels
        # Entry characteristics: far distance ahead, obstacles on both sides
        front_angles = range(len(scan.ranges) // 3, 2 * len(scan.ranges) // 3)  # Front 120 degrees
        front_ranges = [scan.ranges[i] for i in front_angles if i < len(scan.ranges)]
        
        if not front_ranges:
            return None
        
        # Find the farthest point (possible entry)
        max_range_idx = front_angles[front_ranges.index(max(front_ranges))]
        max_range = scan.ranges[max_range_idx]
        
        # Check if it meets entry conditions: moderate distance (not too far or too close)
        if 2.0 < max_range < 8.0:
            # Calculate position in robot coordinate system
            angle = scan.angle_min + max_range_idx * scan.angle_increment
            entry_x = robot_pose.x + max_range * math.cos(angle + robot_pose.theta)
            entry_y = robot_pose.y + max_range * math.sin(angle + robot_pose.theta)
            
            return (entry_x, entry_y)
        
        return None 