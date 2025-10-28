# ================================
# file: code/sim/maze_map.py
# ================================
from __future__ import annotations
from typing import Tuple, Sequence, Dict, Optional
import json
import math
import numpy as np
# Import directly from config to avoid circular import
from core.config import (
    MAP_SIZE, MAP_RES, ROBOT_START_X, ROBOT_START_Y,
    WORLD_SIZE, SLAM_RESOLUTION, MAZE_CENTER_OFFSET,  # English: for mask anchor
    MAZE_W_M, MAZE_H_M  # 新增迷宫尺寸参数
)
from core.coords import coord_system

class MazeMap:
    """Maze map built from a JSON description.

    The grid uses values: 0=free, 1=obstacle, 2=unknown (for consistency).
    For ground-truth view we set unknown to 0 for free areas that are not walls.
    """
    def __init__(self) -> None:
        # English: build GT grid at SLAM resolution so planners/collision checks align
        N = int(round(WORLD_SIZE / SLAM_RESOLUTION))  # e.g., 2.0/0.01=200
        self.height: int = N
        self.width: int = N
        self.grid: np.ndarray = np.full((self.height, self.width), 2, dtype=np.uint8)
        
        # Car start position: from config
        self.car_start_world: Tuple[float, float] = (ROBOT_START_X, ROBOT_START_Y)
        self.car_start_cell: Tuple[int, int] = (0, 0)  # Will be set later
        
        # Maze entry point: will be set from JSON
        self.maze_entry_world: Tuple[float, float] = (0.0, 0.0)
        self.maze_entry_cell: Tuple[int, int] = (0, 0)
        # exit from JSON (for planner/visualizer/navigator)
        self.maze_exit_world: Tuple[float, float]  = (0.0, 0.0)   # world meters
        self.maze_exit_cell:  Tuple[int, int]      = (0, 0)       # (gx, gy)
        
        # Maze information
        self.maze_bounds: Dict = {}
        self.json_maze_size: Tuple[int, int] = (0, 0)
        self.interface_center: Tuple[float, float] = (WORLD_SIZE/2.0, WORLD_SIZE/2.0)
        
        # Grid information
        self.grid_width: int = self.width
        self.grid_height: int = self.height
        self.grid_bounds: Dict = {}
        
        self._gt_ready: bool = False
        # English: JSON meta cache
        self._cell_m: float = float(MAP_RES)   # 默认0.70
        self._offx: float = MAZE_CENTER_OFFSET[0]  # 默认0.0
        self._offy: float = MAZE_CENTER_OFFSET[1]  # 默认0.0
        self._world_size: float = float(WORLD_SIZE)

    def _world_to_grid(self, x_world: float, y_world: float) -> Tuple[int, int]:
        """World[m] -> grid index (x,y), origin at (0,0), res = SLAM_RESOLUTION."""
        # NOTE: do not call coord_system here during maze building.
        gx = int(round(x_world / SLAM_RESOLUTION))
        gy = int(round(y_world / SLAM_RESOLUTION))
        return gx, gy

    def _grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Grid index -> world[m], inverse of _world_to_grid."""
        xw = float(grid_x) * SLAM_RESOLUTION
        yw = float(grid_y) * SLAM_RESOLUTION
        return xw, yw

    def _calculate_maze_bounds(self, json_maze_size: Tuple[int, int]) -> Dict:
        """English: bounds in WORLD meters using JSON meta (_cell_m, _offx,_offy)."""
        w_cells, h_cells = json_maze_size
        maze_w = (w_cells - 1) * self._cell_m
        maze_h = (h_cells - 1) * self._cell_m
        # 优先按JSON cell尺寸推导；若meta不可靠则回退到配置常量
        maze_w = max(MAZE_W_M, (w_cells - 1) * self._cell_m) if w_cells > 1 else MAZE_W_M
        maze_h = max(MAZE_H_M, (h_cells - 1) * self._cell_m) if h_cells > 1 else MAZE_H_M
        bl = (self._offx, self._offy)              # English: lower-left origin from JSON/meta
        tr = (self._offx + maze_w, self._offy + maze_h)
        cx = 0.5 * (bl[0] + tr[0]); cy = 0.5 * (bl[1] + tr[1])
        return {
            'center': (cx, cy),
            'bottom_left': bl,
            'bottom_right': (tr[0], bl[1]),
            'top_left': (bl[0], tr[1]),
            'top_right': tr,
            'width': maze_w,
            'height': maze_h,
        }

    def _convert_json_to_world_coords(self, json_x: int, json_y: int, 
                                    json_maze_size: Tuple[int, int]) -> Tuple[float, float]:
        """Convert JSON maze coordinates to world coordinates.
        
        Args:
            json_x, json_y: Coordinates relative to JSON maze (0 to maze_size-1)
            json_maze_size: (width, height) of the JSON maze
            
        Returns:
            (x_world, y_world): World coordinates in the 25x25 map
        """
        # 使用新的坐标系统
        
        # JSON坐标就是迷宫坐标，直接转换
        # English: world = offset + json * cell_size
        world_x = self._offx + float(json_x) * self._cell_m
        world_y = self._offy + float(json_y) * self._cell_m
        
        print(f"[MAZE_DEBUG] JSON坐标[{json_x},{json_y}] -> 世界坐标({world_x:.1f},{world_y:.1f})")
        
        return (world_x, world_y)

    def _validate_maze_entry(self, entry_world: Tuple[float, float], 
                           maze_bounds: Dict) -> Tuple[float, float]:
        """Validate and adjust maze entry position to be on maze boundary.
        
        Args:
            entry_world: Proposed entry position in world coordinates
            maze_bounds: Maze boundary information
            
        Returns:
            Validated entry position on maze boundary
        """
        entry_x, entry_y = entry_world
        bl_x, bl_y = maze_bounds['bottom_left']
        tr_x, tr_y = maze_bounds['top_right']
        
        # For maze entry, we want it to be exactly on the boundary
        # Check if entry is exactly on the boundary (with small tolerance)
        tolerance = 0.1  # 10cm tolerance
        
        # Check if entry is on any boundary
        on_left = abs(entry_x - bl_x) < tolerance
        on_right = abs(entry_x - tr_x) < tolerance
        on_bottom = abs(entry_y - bl_y) < tolerance
        on_top = abs(entry_y - tr_y) < tolerance
        
        # If entry is on boundary, keep it as is
        if on_left or on_right or on_bottom or on_top:
            print(f"入口点已在边界上: {entry_world}")
            return entry_world
        
        # If entry is inside maze bounds, move to nearest boundary
        if bl_x < entry_x < tr_x and bl_y < entry_y < tr_y:
            # Find closest boundary
            dist_to_left = abs(entry_x - bl_x)
            dist_to_right = abs(entry_x - tr_x)
            dist_to_bottom = abs(entry_y - bl_y)
            dist_to_top = abs(entry_y - tr_y)
            
            min_dist = min(dist_to_left, dist_to_right, dist_to_bottom, dist_to_top)
            
            if min_dist == dist_to_left:
                adjusted_entry = (bl_x, entry_y)  # Move to left boundary
            elif min_dist == dist_to_right:
                adjusted_entry = (tr_x, entry_y)  # Move to right boundary
            elif min_dist == dist_to_bottom:
                adjusted_entry = (entry_x, bl_y)  # Move to bottom boundary
            else:
                adjusted_entry = (entry_x, tr_y)  # Move to top boundary
            
            print(f"入口点从 {entry_world} 调整到边界: {adjusted_entry}")
            return adjusted_entry
        
        print(f"入口点保持在原位置: {entry_world}")
        return entry_world  # Entry is already on boundary or outside

    def _snap_inside_maze(self, pt: Tuple[float, float]) -> Tuple[float, float]:
        """Snap a boundary point slightly INSIDE the maze box for planning.
        English: move by +/− 1 pixel (SLAM_RESOLUTION) along the outward normal."""
        x, y = pt
        bl_x, bl_y = self.maze_bounds['bottom_left']
        tr_x, tr_y = self.maze_bounds['top_right']
        eps = SLAM_RESOLUTION  # 1 pixel
        # left / right
        if abs(x - bl_x) < 1e-6: x = bl_x + eps
        if abs(x - tr_x) < 1e-6: x = tr_x - eps
        # bottom / top
        if abs(y - bl_y) < 1e-6: y = bl_y + eps
        if abs(y - tr_y) < 1e-6: y = tr_y - eps
        return (x, y)

    def _infer_maze_size_from_segments(self, segments: list) -> Tuple[int, int]:
        """Infer maze size from segment coordinates.
        
        Args:
            segments: List of wall segments with start/end coordinates
            
        Returns:
            (width, height): Inferred maze dimensions
        """
        if not segments:
            return (14, 14)  # Default size if no segments
        
        # Find the maximum x and y coordinates
        max_x = max_y = 0
        for seg in segments:
            start_x, start_y = seg["start"]
            end_x, end_y = seg["end"]
            max_x = max(max_x, start_x, end_x)
            max_y = max(max_y, start_y, end_y)
        
        # Maze size is max coordinate + 1 (since coordinates are 0-based)
        width = max_x + 1
        height = max_y + 1
        
        return (width, height)

    def load_from_json(self, path: str, log_file=None) -> None:
        """Load maze from a JSON file with centered positioning and detailed logging.
        
        JSON expected keys: 
        - "segments": [{"start":[x,y], "end":[x,y]}] (maze-relative coordinates)
        - "map_size": [W,H] (optional, may not be accurate)
        - "wall_width_cells": int (optional)
        - "start_point": [x,y] (maze entry point in maze-relative coordinates)
        """
        if log_file:
            from main import log_to_file
            log_to_file(log_file, f"加载JSON迷宫文件: {path}")
        
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        
        # Get segments first
        # Get metadata and segments
        meta = data.get("metadata", {})
        self._cell_m = float(meta.get("cell_size_m", MAP_RES))        # 0.45
        self._offx, self._offy = map(float, meta.get("origin_offset_m", [0.0, 0.0]))
        self._world_size = float(meta.get("world_size_m", WORLD_SIZE))
        segments = data.get("segments", [])
        
        if log_file:
            log_to_file(log_file, f"迷宫信息: {len(segments)}个墙壁段, 入口: {data.get('start_point', '未指定')}")
        
        # Infer maze size from segments instead of using map_size
        self.json_maze_size = self._infer_maze_size_from_segments(segments)
        
        # Automatically update configuration based on maze size
        maze_width, maze_height = self.json_maze_size
        print(f"[MazeMap] 检测到迷宫尺寸: {maze_width}x{maze_height}")
        
        
        # Validate maze size - ensure it fits within 25x25 interface
        max_allowed_size = 20  # Leave some margin
        if self.json_maze_size[0] > max_allowed_size or self.json_maze_size[1] > max_allowed_size:
            print(f"⚠️ 警告: 迷宫尺寸 {self.json_maze_size} 过大，可能超出界面范围")
            print(f"   建议迷宫尺寸不超过 {max_allowed_size}x{max_allowed_size}")
            if log_file:
                log_to_file(log_file, f"⚠️ 警告: 迷宫尺寸 {self.json_maze_size} 过大，可能超出界面范围")
                log_to_file(log_file, f"   建议迷宫尺寸不超过 {max_allowed_size}x{max_allowed_size}")
        
        # Bounds in WORLD meters, then inform coord_system
        self.maze_bounds = self._calculate_maze_bounds(self.json_maze_size)
        coord_system.set_maze_bounds(self.maze_bounds)
        if hasattr(coord_system, "set_world_box"):
            coord_system.set_world_box(0.0, 0.0, self._world_size, self._world_size)
        
        # Calculate grid size
        maze_size = self.json_maze_size[0]  # Assume square maze
        # English: GT grid covers the whole WORLD box at SLAM resolution
        self.grid_width = self.width
        self.grid_height = self.height
        self.grid_bounds = {
            'bottom_left': (0, 0),
            'top_right': (self.width-1, self.height-1)
        }
        
        # Maze bounds in grid coordinates
        self.maze_grid_bounds = {
            'bottom_left': (int(round(self.maze_bounds['bottom_left'][0] / SLAM_RESOLUTION)),
                            int(round(self.maze_bounds['bottom_left'][1] / SLAM_RESOLUTION))),
            'top_right':   (int(round(self.maze_bounds['top_right'][0]   / SLAM_RESOLUTION)),
                            int(round(self.maze_bounds['top_right'][1]   / SLAM_RESOLUTION))),
        }
        
        if log_file:
            log_to_file(log_file, "=" * 60)
            log_to_file(log_file, "迷宫构建信息:")
            log_to_file(log_file, f"JSON文件map_size: {data.get('map_size', '未指定')}")
            log_to_file(log_file, f"推断迷宫尺寸: {self.json_maze_size[0]} x {self.json_maze_size[1]}")
            log_to_file(log_file, f"网格尺寸: {self.grid_width} x {self.grid_height}")
            log_to_file(log_file, f"界面中心: {self.interface_center}")
            log_to_file(log_file, f"迷宫中心: {self.maze_bounds['center']}")
            log_to_file(log_file, "迷宫四角位置:")
            log_to_file(log_file, f"  左下角: {self.maze_bounds['bottom_left']}")
            log_to_file(log_file, f"  右下角: {self.maze_bounds['bottom_right']}")
            log_to_file(log_file, f"  左上角: {self.maze_bounds['top_left']}")
            log_to_file(log_file, f"  右上角: {self.maze_bounds['top_right']}")
            log_to_file(log_file, f"迷宫边界区域: [{self.maze_bounds['bottom_left']}] 到 [{self.maze_bounds['top_right']}]")
            log_to_file(log_file, f"网格坐标范围: {self.grid_bounds['bottom_left']} 到 {self.grid_bounds['top_right']}")
            log_to_file(log_file, f"迷宫网格范围: {self.maze_grid_bounds['bottom_left']} 到 {self.maze_grid_bounds['top_right']}")
        
        # Initialize: mark everything as free (0) for GT map, then draw walls as 1
        self.grid.fill(0)
        
        # Process wall segments with coordinate conversion
        wall_count = 0
        if log_file:
            log_to_file(log_file, f"\n=== 开始处理墙壁段 ===")
        
        for i, seg in enumerate(segments):
            # Convert segment endpoints from JSON coordinates to world coordinates
            start_json = seg["start"]
            end_json = seg["end"]
            
            start_world = self._convert_json_to_world_coords(
                start_json[0], start_json[1], self.json_maze_size)
            end_world = self._convert_json_to_world_coords(
                end_json[0], end_json[1], self.json_maze_size)
            
            # Convert world coordinates to grid coordinates
            start_grid = self._world_to_grid(start_world[0], start_world[1])
            end_grid = self._world_to_grid(end_world[0], end_world[1])
            
            # Debug: Print coordinate conversion for first few segments
            if i < 5:
                if log_file:
                    log_to_file(log_file, f"墙壁段 {i}:")
                    log_to_file(log_file, f"  JSON坐标: start{start_json} -> end{end_json}")
                    log_to_file(log_file, f"  世界坐标: start{start_world} -> end{end_world}")
                    log_to_file(log_file, f"  网格坐标: start{start_grid} -> end{end_grid}")
            
            # Draw the wall segment in grid
            # Use correct coordinate order: (x, y) format
            sx, sy = start_grid[0], start_grid[1]  # (grid_x, grid_y)
            ex, ey = end_grid[0], end_grid[1]      # (grid_x, grid_y)
            
            # Debug coordinate order
            # 删除详细的坐标转换检查日志
            
            # Clamp to grid bounds
            sx = max(0, min(self.grid_width-1, sx))
            ex = max(0, min(self.grid_width-1, ex))
            sy = max(0, min(self.grid_height-1, sy))
            ey = max(0, min(self.grid_height-1, ey))
            
            if sx == ex:
                # Vertical wall - draw as thin line (0 width)
                y0, y1 = sorted((sy, ey))
                for y in range(y0, y1+1):
                    if 0 <= y < self.grid_height:
                        self.grid[y, sx] = 1
                        wall_count += 1
                # 删除详细的墙壁绘制日志
            elif sy == ey:
                # Horizontal wall - draw as thin line (0 width)
                x0, x1 = sorted((sx, ex))
                for x in range(x0, x1+1):
                    if 0 <= x < self.grid_width:
                        self.grid[sy, x] = 1
                        wall_count += 1
                # 删除详细的墙壁绘制日志
            else:
                # Diagonal segment - draw as thin line using Bresenham algorithm
                dx = abs(ex - sx)
                dy = -abs(ey - sy)
                sx1 = 1 if sx < ex else -1
                sy1 = 1 if sy < ey else -1
                err = dx + dy
                x, y = sx, sy
                diagonal_count = 0
                while True:
                    if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
                        self.grid[y, x] = 1
                        wall_count += 1
                        diagonal_count += 1
                    if x == ex and y == ey:
                        break
                    e2 = 2 * err
                    if e2 >= dy:
                        err += dy
                        x += sx1
                    if e2 <= dx:
                        err += dx
                        y += sy1
                # 删除详细的墙壁绘制日志
        
        if log_file:
            log_to_file(log_file, f"迷宫构建完成: {wall_count}个墙壁网格")
        
        # Set maze entry point from JSON
        sp = data.get("start_point")
        if isinstance(sp, Sequence) and len(sp) == 2:            
            # Convert JSON maze entry point to world coordinates
            self.maze_entry_world = self._convert_json_to_world_coords(
                sp[0], sp[1], self.json_maze_size)
            print(f"转换后世界坐标: {self.maze_entry_world}")
            
            if log_file:
                log_to_file(log_file, f"转换后世界坐标: {self.maze_entry_world}")
            
            # Validate and adjust entry position
            original_entry = self.maze_entry_world
            self.maze_entry_world = self._validate_maze_entry(
                self.maze_entry_world, self.maze_bounds)
            if original_entry != self.maze_entry_world:
                print(f"入口点被调整: {original_entry} -> {self.maze_entry_world}")
                if log_file:
                    log_to_file(log_file, f"入口点被调整: {original_entry} -> {self.maze_entry_world}")
            
            self.maze_entry_cell = self._world_to_grid(
                self.maze_entry_world[0], self.maze_entry_world[1])

            # 防御性处理：确保网格索引在合法范围内，避免后续 numpy 索引越界
            ex, ey = self.maze_entry_cell
            ex = max(0, min(self.grid_width - 1, ex))
            ey = max(0, min(self.grid_height - 1, ey))
            # 更新为已裁剪的网格坐标
            self.maze_entry_cell = (ex, ey)

            print(f"最终网格坐标: {self.maze_entry_cell}")
            
            if log_file:
                log_to_file(log_file, f"最终网格坐标: {self.maze_entry_cell}")
            
            # Ensure entry point is marked as free space (not wall)
            entry_x, entry_y = self.maze_entry_cell  # (grid_x, grid_y)
            if 0 <= entry_x < self.grid_width and 0 <= entry_y < self.grid_height:
                original_value = self.grid[entry_y, entry_x]
                self.grid[entry_y, entry_x] = 0  # Mark as free space
                print(f"入口点网格[{entry_x},{entry_y}]从值{original_value}设置为0(自由空间)")
                if log_file:
                    log_to_file(log_file, f"入口点网格[{entry_x},{entry_y}]从值{original_value}设置为0(自由空间)")
            else:
                print(f"警告: 入口点网格坐标[{entry_x},{entry_y}]超出地图范围")
                if log_file:
                    log_to_file(log_file, f"警告: 入口点网格坐标[{entry_x},{entry_y}]超出地图范围")
        else:
            # Default maze entry point if not specified in JSON
            self.maze_entry_world = self.maze_bounds['bottom_left']
            self.maze_entry_cell = self._world_to_grid(
                self.maze_entry_world[0], self.maze_entry_world[1])
            print(f"使用默认入口位置: {self.maze_entry_world}")
            if log_file:
                log_to_file(log_file, f"使用默认入口位置: {self.maze_entry_world}")
                
        if log_file:
            log_to_file(log_file, f"入口点设置: 世界坐标{self.maze_entry_world}")
            log_to_file(log_file, "=" * 60)
        
        # 读取出口并设置：既供可视化/导航使用，也把出口外侧涂白
        self._mark_exit_and_free_outside(data, log_file)
        
        self._gt_ready = True

    # ===== Module-level helpers (kept inside file for single-source-of-truth) =====
    @staticmethod
    def get_maze_mask(shape: Tuple[int, int],
                      res: float = SLAM_RESOLUTION,
                      origin_xy: Optional[Tuple[float, float]] = None) -> np.ndarray:
        """Boolean mask of legal maze rectangle (True=inside)."""
        H, W = shape
        offx, offy = origin_xy if origin_xy is not None else MAZE_CENTER_OFFSET
        w_pix = int(round(MAZE_W_M / res))
        h_pix = int(round(MAZE_H_M / res))
        j0 = max(0, int(round(offx / res)))           # x -> col
        i0 = max(0, int(round(offy / res)))           # y -> row
        j1 = min(W, j0 + w_pix)
        i1 = min(H, i0 + h_pix)
        mask = np.zeros((H, W), dtype=bool)
        if i0 < i1 and j0 < j1:
            mask[i0:i1, j0:j1] = True
        return mask

    @staticmethod
    def paint_white_outside(occ: np.ndarray,
                            res: float = SLAM_RESOLUTION,
                            origin_xy: Optional[Tuple[float, float]] = None) -> np.ndarray:
        """
        English: Return a COPY for GUI where outside-maze cells are forced FREE(0).
        Planning must NOT use this; A* should block outside via mask in planner.
        """
        out = occ.copy()
        mask = MazeMap.get_maze_mask(out.shape, res, origin_xy)
        out[~mask] = 0  # 0=free -> white on GUI
        return out

    @staticmethod
    def coverage_ratio_in_maze(occ: np.ndarray,
                               res: float = SLAM_RESOLUTION,
                               origin_xy: Optional[Tuple[float, float]] = None) -> float:
        """
        English: Coverage = known / maze_cells using ONLY the maze mask.
        Known := occ != 2 (not unknown).
        """
        mask = MazeMap.get_maze_mask(occ.shape, res, origin_xy)
        denom = int(mask.sum())
        if denom == 0:
            return 0.0
        known = np.logical_and(mask, occ != 2).sum()
        return float(known) / float(denom)

    def _mark_exit_and_free_outside(self, data: dict, log_file=None) -> None:
        """Parse exit from JSON, store exit for others, and paint outside area white."""
        # 获取goal_point并转换为世界坐标
        meta = data.get('metadata', {})
        cell_size = float(meta.get('cell_size_m', self._cell_m))
        origin_offset = meta.get('origin_offset_m', [self._offx, self._offy])
        # Accept common keys: goal_point / goal / exit (flat or under "maze")
        goal_json = data.get('goal_point') or data.get('goal') or data.get('exit')
        if goal_json is None:
            maze_blk = data.get('maze', {})
            goal_json = maze_blk.get('goal_point') or maze_blk.get('goal') or maze_blk.get('exit')
        if not (isinstance(goal_json, (list, tuple)) and len(goal_json) == 2):
            # No goal info -> nothing to free
            if log_file:
                from main import log_to_file
                log_to_file(log_file, "未在JSON中找到 goal/exit，跳过出口外区域置白")
            return
        goal_world_x = origin_offset[0] + float(goal_json[0]) * cell_size
        goal_world_y = origin_offset[1] + float(goal_json[1]) * cell_size

        # 1) 存储出口世界/网格坐标；若在边界上，内缩1个像素用于规划
        exit_w_on_edge = (goal_world_x, goal_world_y)
        exit_w_inside  = self._snap_inside_maze(self._validate_maze_entry(exit_w_on_edge, self.maze_bounds))
        self.maze_exit_world = exit_w_inside
        self.maze_exit_cell  = self._world_to_grid(exit_w_inside[0], exit_w_inside[1])
        if log_file:
            from main import log_to_file
            log_to_file(log_file, f"出口(世界): {exit_w_on_edge} → 规划点(内缩): {exit_w_inside}, 网格: {self.maze_exit_cell}")
        
        # 不再修改主网格的出口外区域，仅存储出口坐标和写日志
        if log_file:
            from main import log_to_file
            log_to_file(log_file, f"出口已设置: 世界({goal_world_x:.2f}, {goal_world_y:.2f}) 网格({self.maze_exit_cell})")

    def is_obstacle_world(self, x: float, y: float, inflate_cells: int = 0) -> bool:
        """Check if world (x,y) collides with obstacle in grid.
        inflate_cells expands wall footprint to emulate robot radius.
        """
        # 使用新的坐标系统进行转换
        grid_x, grid_y = coord_system.world_to_grid(x, y)
        
        # 边界检查
        if not coord_system.is_in_grid(grid_x, grid_y):
            return True
            
        if inflate_cells <= 0:
            return self.grid[grid_y, grid_x] == 1  # 注意：numpy数组是 [row, col] 格式
        
        # Check a box neighborhood
        y0 = max(0, grid_y - inflate_cells)
        y1 = min(self.grid_height-1, grid_y + inflate_cells)
        x0 = max(0, grid_x - inflate_cells)
        x1 = min(self.grid_width-1, grid_x + inflate_cells)
        return (self.grid[y0:y1+1, x0:x1+1] == 1).any()

    def get_car_start_position(self) -> Tuple[float, float]:
        """Get the car start position in world coordinates."""
        # 添加调试信息
        print(f"[MAZE_DEBUG] 获取小车起始位置: {self.car_start_world}")
        print(f"[MAZE_DEBUG] 小车起始位置类型: {type(self.car_start_world)}")
        print(f"[MAZE_DEBUG] 小车起始位置值: x={self.car_start_world[0]}, y={self.car_start_world[1]}")
        return self.car_start_world

    def set_car_start_position(self, x: float, y: float) -> None:
        """Set the car start position in world coordinates."""
        print(f"[MAZE_DEBUG] 设置小车起始位置: 从 {self.car_start_world} 到 ({x}, {y})")
        self.car_start_world = (x, y)
        print(f"[MAZE_DEBUG] 设置后的小车起始位置: {self.car_start_world}")

    def get_maze_entry_position(self) -> Tuple[float, float]:
        """Get the maze entry position in world coordinates."""
        return self.maze_entry_world

    def get_maze_info(self) -> Dict:
        """Get comprehensive maze information."""
        if not self._gt_ready:
            return {
                'json_size': self.json_maze_size,
                'bounds': {},
                'entry_position': self.maze_entry_world,
                'car_start': self.car_start_world,
                'wall_count': 0,
                'free_count': 0,
                'unknown_count': (self.grid == 2).sum()
            }
        
        return {
            'json_size': self.json_maze_size,
            'bounds': self.maze_bounds,
            'entry_position': self.maze_entry_world,
            'car_start': self.car_start_world,
            'wall_count': (self.grid == 1).sum(),
            'free_count': (self.grid == 0).sum(),
            'unknown_count': (self.grid == 2).sum()
        }


