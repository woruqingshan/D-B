# ================================
# file: code/nav/navigator.py
# ================================
from __future__ import annotations
from typing import Optional, Tuple, Sequence, List
from enum import Enum
import time

from core import Pose2D, GOAL_TOLERANCES, world_to_map  # Use GOAL_TOLERANCES['default'] after config merge
from core.config import (
    GOAL_TOLERANCES, GOAL_TIMEOUT_SECONDS, GOAL_STUCK_DISTANCE, GOAL_STUCK_TIME,
    PATH_RESAMPLE_M, LOOKAHEAD_BASE_M, GOAL_GATE_AHEAD_M, GOAL_PROGRESS_EPS_M,
    SPIN_YAW_THRESHOLD, SPIN_STOP_YAW, SPIN_W, SPIN_TIMEOUT_S,
    CORRIDOR_GOAL_ENABLE, CORRIDOR_WIDTH_M, CORRIDOR_GOAL_TOL_M, CORNER_BOX_HALF_M,
    SLAM_RESOLUTION, SAFE_BUFFER_M, GOAL_BAND_BOUNDARY_TOLERANCE_M,
    FRONTIER_FAILURE_THRESHOLD, SIMPLE_RETREAT_DISTANCE_M
)
from slam import SlamSystem
from planning import AStarPlanner
from explore import FrontierExplorer
from .motion_controller import MotionController

# === 严格路径跟踪参数 ===
import math
from core.config import SLAM_RESOLUTION
STRICT_PATH_TOLERANCE = 0.03  # 5cm路径偏差容忍度

# === Arrival band outer margin (L∞ expansion) ===
# English: When arrival is checked, treat the goal band as an outward-expanded
# rectangle by this margin (L∞ metric). Points inside this expanded rectangle
# are accepted as "arrived" even if the robot stopped a few centimeters outside
# due to discretization or controller quantization. Typical 0.02–0.04 m.
GOAL_BAND_OUTER_MARGIN_M = 0.04  # 4 cm; set to 0.02 for 2 cm if desired
unknown_threshold = 0.003
FINAL_REGION_SIZE = 0.50
# === Primitive/Path micro-segment filter (消除小折线) ===
# English: any collinear segment whose length is below this threshold will be
# removed during path cleanup. This trims tiny "tails/zig-zags" caused by
# float rounding and repeated re-sampling near corners.
# 建议阈值：max(2 * 分辨率, 3cm)
PRIM_MIN_SEG_LEN_M = max(2 * SLAM_RESOLUTION, 0.03)

# === 探索完成控制参数 ===
EXPLORATION_COVERAGE_THRESHOLD = 99.5  # 探索完成阈值（%）- 可调节的全局参数

class NavState(Enum):
    IDLE = 0
    TO_ENTRANCE = 1      # 阶段1：前往迷宫入口
    EXPLORING = 2        # 阶段2：在迷宫内探索
    TO_EXIT = 3          # 阶段3：前往迷宫出口
    RETURN_HOME = 4      # 阶段4：返回入口（可选）
    FINISHED = 5


class NavigatorFSM:
    """High-level mission controller with a finite-state machine.
    It depends on a `robot` object exposing apply_control(v,w).
    For SIM: `robot` is RobotSim. For REAL: `robot` is an interface wrapper
    (e.g., appio.BleInterface) providing the same method signature.
    """
    def __init__(self, slam: SlamSystem, planner: AStarPlanner,
                 explorer: FrontierExplorer, robot, logger_func=None, log_file=None) -> None:
        self.slam = slam
        self.planner = planner
        self.explorer = explorer
        self.robot = robot
        self.logger_func = logger_func  # 日志函数
        self.log_file = log_file  # 日志文件对象
        
        # 先初始化debug_log，再调用_log_debug
        self.debug_log = []
        self.last_state = NavState.IDLE
        self.state_change_time = time.time()
        
        # 状态切换保护 - 减少冷却时间，允许正常状态切换
        self.state_change_cooldown = 0.5  # 状态切换冷却时间（秒）
        self.last_state_change_time = 0.0
        
        # 确保前沿点探索器使用SLAM系统
        if self.explorer and hasattr(self.explorer, 'slam_system'):
            self.explorer.slam_system = self.slam
            self._log_debug("前沿点探索器已配置SLAM系统")
        
        self.state = NavState.IDLE
        self.current_goal: Optional[Tuple[float,float]] = None
        self.current_path: Sequence[Tuple[float,float]] = []
        self.controller: Optional[MotionController] = None
        self.start_xy: Optional[Tuple[float,float]] = None
        self.exit_xy: Optional[Tuple[float,float]] = None
        
        # 导入闸门配置
        from core.config import GATE_RADIUS_M
        self.gate_radius = GATE_RADIUS_M
        
        # 掉头相关标志
        self._turn_around_completed = False
        self._pending_return_home = False
        
        # === NEW: primitive execution state (axis-aligned motion) ===
        self._primitive_queue = []      # List[dict]
        self._active_primitive = None   # current primitive dict
        self._prim_start_xy = None      # (x,y) at primitive fetch
        self._band_rect = None          # (xmin,xmax,ymin,ymax)
        self._primitive_mode = False
        # 原语执行统计
        self._primitive_stats = {
            'total_executed': 0,
            'turn_count': 0,
            'move_count': 0,
            'gate_count': 0
        }
        
        # 三阶段导航管理变量
        self.entrance_xy: Optional[Tuple[float,float]] = None  # 迷宫入口位置
        self.entrance_reached = False                          # 是否已到达入口
        self.exploration_started = False                       # 是否开始探索
        self.exploration_complete = False                      # 探索是否完成
        self.coverage_threshold = EXPLORATION_COVERAGE_THRESHOLD  # 探索完成阈值（%）- 使用顶部全局参数
        
        # 到达判定超时保护
        self.goal_arrival_timeout = 10.0  # 目标到达超时时间（秒）
        self.goal_start_time = None       # 开始向目标移动的时间
        
        # 简化恢复策略计数器
        self.frontier_failure_count = 0   # 连续前沿搜索失败次数
        
        # === Simple retreat recovery state ===
        self._recovery_mode = False
        self._recovery_params = {
            "v": 0.10,                         # 恢复期线速度 m/s
            "w": 0.0,                          # 恢复期角速度 rad/s
            "duration": 0.5,                   # 恢复持续时间 s
            "start_time": None,                # 恢复开始时间
            "start_pose": None                 # 恢复开始时的位姿
        }
        
        # === 到达带机制相关变量 ===
        self._goal_band_rect = None            # 当前目标的到达带矩形 (xmin,xmax,ymin,ymax)
        self._goal_band_owner = None           # English: which state created current band ('EXPLORE','TO_EXIT','RETURN')
        self._band_guard_ticks = 0             # English: small guard to ignore stale band-cross right after state switch
        self._band_arrival_timeout = 5.0       # 到达带判定超时时间（秒）
        self._band_start_time = None           # 开始向到达带移动的时间
        self._last_known_cells = 0             # 上次已知栅格数量（用于检测覆盖度下降）
        self._last_dir_vec: Optional[Tuple[float,float]] = None  # 上次运动方向向量
        self._target_reachable_region = None   # English: target reachable region for GUI display
        
    def is_within_gate(self, robot_pose: Tuple[float, float], gate_center: Tuple[float, float]) -> bool:
        """检查机器人是否在闸门区域内"""
        import math
        
        rx, ry = robot_pose
        gx, gy = gate_center
        
        distance = math.sqrt((rx - gx)**2 + (ry - gy)**2)
        return distance <= self.gate_radius
    
    def check_gate_reached(self, robot_pose: Tuple[float, float]) -> bool:
        """根据当前状态检查是否到达相应闸门"""
        if self.state == NavState.TO_EXIT:
            # 检查是否到达终点闸门
            if self.exit_xy is not None:
                return self.is_within_gate(robot_pose, self.exit_xy)
        elif self.state == NavState.RETURN_HOME:
            # 检查是否到达起点闸门
            if self.start_xy is not None:
                return self.is_within_gate(robot_pose, self.start_xy)
        return False

    def set_maze_exit(self, exit_xy: Tuple[float, float]) -> None:
        """设置迷宫出口位置（从JSON加载）
        
        English: Set the maze exit position from JSON file.
        This is called during initialization with the known exit location.
        """
        self.exit_xy = exit_xy
        self._log_debug(f"迷宫出口已设置: {exit_xy}")

    def _arm_controller_with_path(self, path_pts: List[Tuple[float,float]]):
        """English: bind new path to MotionController and arm it."""
        self.current_path = path_pts
        if self.controller is None:
            self.controller = MotionController()
        self.controller.set_path(path_pts)


    def get_control_command(self) -> Tuple[float, float]:
        """获取当前控制命令"""
        return self.current_control_cmd
    

    def _align_path_to_axis(self, path_points: Sequence[Tuple[float, float]]) -> Sequence[Tuple[float, float]]:
        """
        对A*输出进行轴对齐处理，确保路径严格水平或垂直
        
        English: Align A* path points to grid axis to ensure strict Manhattan movement.
        This prevents numerical drift and ensures paths are strictly horizontal or vertical.
        
        Args:
            path_points: Raw A* path points
            
        Returns:
            Axis-aligned path points
        """
        if not path_points:
            return path_points
        
        aligned_points = []
        for i, (x, y) in enumerate(path_points):
            # 按分辨率取整
            x_rounded = round(x / SLAM_RESOLUTION) * SLAM_RESOLUTION
            y_rounded = round(y / SLAM_RESOLUTION) * SLAM_RESOLUTION
            
            # 轴向压正：确保路径严格水平或垂直
            if i > 0:
                prev_x, prev_y = aligned_points[-1]
                dx = x_rounded - prev_x
                dy = y_rounded - prev_y
                
                # 如果移动方向不是严格水平或垂直，进行轴向压正
                if abs(dx) > abs(dy):
                    # 水平移动，y坐标保持不变
                    y_rounded = prev_y
                elif abs(dy) > abs(dx):
                    # 垂直移动，x坐标保持不变
                    x_rounded = prev_x
                else:
                    # 如果dx和dy相等，保持原值（理论上不应该发生）
                    pass
            
            aligned_points.append((x_rounded, y_rounded))
        
        return aligned_points

    def _check_path_deviation(self, pose: Pose2D, path_points: Sequence[Tuple[float, float]]) -> Tuple[bool, float]:
        """
        检查机器人是否偏离规划路径
        
        English: Check if robot has deviated from the planned path beyond tolerance.
        
        Args:
            pose: Current robot pose
            path_points: Planned path points
            
        Returns:
            (is_deviated, deviation_distance)
        """
        if not path_points or len(path_points) < 2:
            return False, 0.0
        
        # 找到最近的路径点
        min_dist = float('inf')
        for point in path_points:
            dist = pose.distance_to(Pose2D(point[0], point[1], 0.0))
            min_dist = min(min_dist, dist)
        
        is_deviated = min_dist > STRICT_PATH_TOLERANCE
        return is_deviated, min_dist


    def _is_exploration_complete(self, occ_grid) -> bool:
        """
        检查探索是否完成：使用全图未知区域比例 < 1% 作为判据
        并要求连续3帧都满足条件才返回True
        """
        import numpy as np
        if not self.exploration_started:
            return False

        # 1. 计算全图未知区域比例
        H, W = occ_grid.shape
        total_cells = H * W
        unknown_cells = int((occ_grid == 2).sum())
        unknown_ratio = unknown_cells / total_cells

        # 2. 连续帧稳定性检查
        if not hasattr(self, "_unknown_ok_streak"):
            self._unknown_ok_streak = 0
        
        if unknown_ratio <= unknown_threshold:
            self._unknown_ok_streak += 1
        else:
            self._unknown_ok_streak = 0

        # 3. 记录日志（每50帧）
        if len(self.debug_log) % 50 == 0:
            self._log_debug(f"探索进度: {unknown_ratio*100:.2f}% 未知, 连续={self._unknown_ok_streak}帧达标")

        return self._unknown_ok_streak >= 3  # 要求连续3帧都满足条件

    def _compute_interior_coverage_percent(self, occ_grid) -> float:
        """
        保留此函数仅用于统计展示，不再用于判定
        返回 known_cells/total_cells 的百分比
        """
        import numpy as np
        H, W = occ_grid.shape
        known_cells = np.sum(occ_grid != 2)  # 0或1都算已知
        total_cells = H * W
        if total_cells == 0:
            return 0.0
        return float(known_cells) / float(total_cells) * 100.0

    def _check_entrance_reached(self, pose: Pose2D) -> bool:
        """检查是否到达迷宫入口"""
        if self.entrance_xy is None:
            return False
        
        distance = pose.distance_to(Pose2D(self.entrance_xy[0], self.entrance_xy[1], 0.0))
        
        # 减少检测阈值，避免过早触发入口检测
        entrance_tolerance = 0.3  # 入口检测容差
        
        # 记录检测信息（每50次调用记录一次）
        if len(self.debug_log) % 50 == 0:
            self._log_debug(f"入口检测: 距离={distance:.3f}m, 阈值={entrance_tolerance}m, 到达={distance <= entrance_tolerance}")
        
        return distance <= entrance_tolerance

    def _is_stuck_at_goal(self, pose: Pose2D) -> bool:
        """English: treat 'stuck' as being inside goal box while controller considers path done."""
        if not self.current_path or self.controller is None:
            return False
        # Check if robot is near goal band or at corner
        near = False
        if self._goal_band_rect is not None:
            near = self._in_rect(pose.x, pose.y, self._goal_band_rect)
        no_progress = self.controller.is_done()
        return near and no_progress

    def _log_debug(self, message: str) -> None:
        """Add debug message to log and integrate with main log_to_file system"""
        timestamp = time.time()
        self.debug_log.append(f"[{timestamp:.3f}] {message}")
        
        # 输出到控制台
        print(f"[NAV_DEBUG] {message}")
        
        # 集成到main函数的log_to_file系统
        if self.logger_func and self.log_file:
            self.logger_func(self.log_file, message, "NAV")

    # --- NEW: utility helpers for band-shaped goals ---
    @staticmethod
    def _segment_axis(dx: float, dy: float) -> Tuple[str, int]:
        """Return axis ('x' or 'y') and direction sign (+1/-1)."""
        if abs(dx) >= abs(dy):
            return ('x', +1 if dx >= 0 else -1)
        return ('y', +1 if dy >= 0 else -1)

    @staticmethod
    def _axis_heading(axis: str, dir_sign: int) -> float:
        """English: canonical heading for axis-aligned motion."""
        if axis == 'x':
            return 0.0 if dir_sign >= 0 else math.pi
        # axis == 'y'
        return math.pi/2 if dir_sign >= 0 else -math.pi/2

    @staticmethod
    def _wrap_pi(a: float) -> float:
        """English: wrap angle to [-pi, pi]."""
        while a >  math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a


    # === Corridor goal-band helpers ===
    def _compute_border_goal_band(self, x0: float, y0: float,
                                  half_width_m: float = 0.30,
                                  depth_m: float = 0.30,
                                  log: bool = True) -> Optional[Tuple[float,float,float,float]]:
        """Compute entry/exit goal-band on maze border using configurable half-width and depth.
        English:
          - Detect which border the gate lies on using an epsilon tied to map resolution.
          - Return (xmin, xmax, ymin, ymax) in meters, clamped to [0, WORLD_SIZE].
          - If not on any border, return None and let fallback logic run."""
        from core.config import WORLD_SIZE, SLAM_RESOLUTION
        HALF_WIDTH_M = half_width_m
        DEPTH_M      = depth_m
        eps = max(3 * SLAM_RESOLUTION, 0.03)  # tolerant border test (≥2px or 1cm)

        def clamp(v, lo, hi): return max(lo, min(hi, v))

        if y0 <= eps:  # bottom edge (y=0)
            xmin, xmax = x0 - HALF_WIDTH_M, x0 + HALF_WIDTH_M
            ymin, ymax = 0.0, DEPTH_M
        elif abs(y0 - WORLD_SIZE) <= eps:  # top edge (y=WORLD_SIZE)
            xmin, xmax = x0 - HALF_WIDTH_M, x0 + HALF_WIDTH_M
            ymin, ymax = WORLD_SIZE - DEPTH_M, WORLD_SIZE
        elif x0 <= eps:  # left edge (x=0)
            xmin, xmax = 0.0, DEPTH_M
            ymin, ymax = y0 - HALF_WIDTH_M, y0 + HALF_WIDTH_M
        elif abs(x0 - WORLD_SIZE) <= eps:  # right edge (x=WORLD_SIZE)
            xmin, xmax = WORLD_SIZE - DEPTH_M, WORLD_SIZE
            ymin, ymax = y0 - HALF_WIDTH_M, y0 + HALF_WIDTH_M
        else:
            return None  # not on border; let existing band logic handle

        # clamp into world and ensure non-degenerate thickness ≥1px
        xmin, xmax = clamp(xmin, 0.0, WORLD_SIZE), clamp(xmax, 0.0, WORLD_SIZE)
        ymin, ymax = clamp(ymin, 0.0, WORLD_SIZE), clamp(ymax, 0.0, WORLD_SIZE)
        if xmax - xmin < SLAM_RESOLUTION:  # ensure at least 1 pixel
            cx = 0.5 * (xmin + xmax)
            xmin, xmax = clamp(cx - SLAM_RESOLUTION, 0.0, WORLD_SIZE), clamp(cx + SLAM_RESOLUTION, 0.0, WORLD_SIZE)
        if ymax - ymin < SLAM_RESOLUTION:
            cy = 0.5 * (ymin + ymax)
            ymin, ymax = clamp(cy - SLAM_RESOLUTION, 0.0, WORLD_SIZE), clamp(cy + SLAM_RESOLUTION, 0.0, WORLD_SIZE)
        
        if log:
            self._log_debug(f"边界到达带: ({xmin:.2f}, {xmax:.2f}, {ymin:.2f}, {ymax:.2f})")
        return (xmin, xmax, ymin, ymax)

    def _compute_goal_band(self, pose: Pose2D, goal_xy: Tuple[float, float]) -> Tuple[float,float,float,float]:
        """
        方案B实现：按"容差轴段 × 正交交集"生成到达带，并对 UNKNOWN 采用方向性宽容：
        - Ignore UNKNOWN slits whose length ≤ gap_pix along the scanning direction;
          never ignore inflated obstacles.
        - Build two temporary binary views only for this routine:
          gridH = obstacle | opening_x(UNKNOWN), gridV = obstacle | opening_y(UNKNOWN).
        返回: (xmin, xmax, ymin, ymax) in meters.
        """
        gx, gy = goal_xy
        
        # English: if goal is on a border, use custom fixed-size band and return immediately.
        band = self._compute_border_goal_band(gx, gy)
        if band is not None:
            self._log_debug(f"使用边界到达带: {band}")
            return band
        
        # ... 原有内部走廊/常规带的计算继续 ...
        import numpy as _np
        import math
        from scipy.ndimage import maximum_filter1d, binary_opening

        # --- base SAFE_BUFFER C-space for reference (0=free,1=blocked) ---
        base = self._get_safebuffer_cspace()
        H, W = base.shape

        # --- index helpers ---
        def world_to_ij(x: float, y: float) -> Tuple[int,int]:
            i = int(_np.floor(y / SLAM_RESOLUTION))
            j = int(_np.floor(x / SLAM_RESOLUTION))
            return max(0, min(H-1, i)), max(0, min(W-1, j))
        def idx_to_world_x(j0:int, j1:int) -> Tuple[float,float]:
            if j0 > j1: return gx, gx
            return (j0*SLAM_RESOLUTION, (j1+1)*SLAM_RESOLUTION)
        def idx_to_world_y(i0:int, i1:int) -> Tuple[float,float]:
            if i0 > i1: return gy, gy
            return (i0*SLAM_RESOLUTION, (i1+1)*SLAM_RESOLUTION)

        ig, jg = world_to_ij(gx, gy)

        # If goal cell is blocked, snap to the nearest free cell (local spiral search).
        if base[ig, jg] != 0:
            found = False
            for rad in range(1, max(H, W)):
                i0 = max(0, ig-rad); i1 = min(H-1, ig+rad)
                j0 = max(0, jg-rad); j1 = min(W-1, jg+rad)
                for ii in (i0, i1):
                    for jj in range(j0, j1+1):
                        if base[ii, jj] == 0:
                            ig, jg = ii, jj; found = True; break
                    if found: break
                if not found:
                    for ii in range(i0+1, i1):
                        for jj in (j0, j1):
                            if base[ii, jj] == 0:
                                ig, jg = ii, jj; found = True; break
                        if found: break
                if found: break
            if not found:
                return (gx, gx, gy, gy)

        # --- Build relaxed views: keep inflated obstacles, open UNKNOWN along each axis ---
        occ = self.slam.get_occ_grid()             # 0=free, 1=occ, 2=unknown
        obst = (occ == 1).astype(_np.uint8)
        unk  = (occ == 2)

        inflate_cells = getattr(self, "_cspace_cells",
                                int(math.ceil(SAFE_BUFFER_M / SLAM_RESOLUTION)))
        if inflate_cells > 0:
            k = 2*inflate_cells + 1
            tmp = maximum_filter1d(obst, size=k, axis=0, mode="nearest")
            inflated = maximum_filter1d(tmp,  size=k, axis=1, mode="nearest")
        else:
            inflated = obst

        # English: remove short UNKNOWN slits differently for vertical/horizontal scans
        gap_pix = max(1, int(round(0.5 * SAFE_BUFFER_M / SLAM_RESOLUTION)))
        unk_v = binary_opening(unk, structure=_np.ones((gap_pix, 1), dtype=bool)).astype(_np.uint8)  # tolerant for up/down
        unk_h = binary_opening(unk, structure=_np.ones((1, gap_pix), dtype=bool)).astype(_np.uint8)  # tolerant for left/right

        gridV = ((inflated > 0) | (unk_v > 0)).astype(_np.uint8)  # 0=free for vertical scans
        gridH = ((inflated > 0) | (unk_h > 0)).astype(_np.uint8)  # 0=free for horizontal scans

        # --- scanners with supplied grid (0=free) ---
        def scan_left(g:_np.ndarray, i:int, j:int) -> int:
            k = 0
            while j-k-1 >= 0 and g[i, j-k-1] == 0: k += 1
            return k
        def scan_right(g:_np.ndarray, i:int, j:int) -> int:
            k = 0
            while j+k+1 < W and g[i, j+k+1] == 0: k += 1
            return k
        def scan_up(g:_np.ndarray, i:int, j:int) -> int:
            k = 0
            while i-k-1 >= 0 and g[i-k-1, j] == 0: k += 1
            return k
        def scan_down(g:_np.ndarray, i:int, j:int) -> int:
            k = 0
            while i+k+1 < H and g[i+k+1, j] == 0: k += 1
            return k

        # Local free extents at (ig, jg) with relaxed views
        rL = scan_left(gridH,  ig, jg);  rR = scan_right(gridH, ig, jg)
        rU = scan_up(gridV,    ig, jg);  rD = scan_down(gridV, ig, jg)
        tol_pix = max(1, int(round(CORRIDOR_GOAL_TOL_M / SLAM_RESOLUTION)))

        # === Horizontal-band candidate (axis = x; orth = vertical intersection) ===
        j0 = jg - min(tol_pix, rL)
        j1 = jg + min(tol_pix, rR)
        i0_list, i1_list = [], []
        for jj in range(j0, j1+1):
            ii0 = ig
            while ii0-1 >= 0 and gridV[ii0-1, jj] == 0: ii0 -= 1
            ii1 = ig
            while ii1+1 < H and gridV[ii1+1, jj] == 0: ii1 += 1
            i0_list.append(ii0); i1_list.append(ii1)
        i0_h = int(max(i0_list)) if i0_list else ig
        i1_h = int(min(i1_list)) if i1_list else ig

        # === Vertical-band candidate (axis = y; orth = horizontal intersection) ===
        i0v = ig - min(tol_pix, rU)
        i1v = ig + min(tol_pix, rD)
        j0_list, j1_list = [], []
        for ii in range(i0v, i1v+1):
            jj0 = jg
            while jj0-1 >= 0 and gridH[ii, jj0-1] == 0: jj0 -= 1
            jj1 = jg
            while jj1+1 < W and gridH[ii, jj1+1] == 0: jj1 += 1
            j0_list.append(jj0); j1_list.append(jj1)
        j0_v = int(max(j0_list)) if j0_list else jg
        j1_v = int(min(j1_list)) if j1_list else jg

        # to world
        xL_h, xR_h = idx_to_world_x(j0,   j1)
        yB_h, yT_h = idx_to_world_y(i0_h, i1_h)
        xL_v, xR_v = idx_to_world_x(j0_v, j1_v)
        yB_v, yT_v = idx_to_world_y(i0v,  i1v)

        def area(rect):
            xmin, xmax, ymin, ymax = rect
            return max(0.0, xmax - xmin) * max(0.0, ymax - ymin)

        cand_h = (xL_h, xR_h, yB_h, yT_h)
        cand_v = (xL_v, xR_v, yB_v, yT_v)
        a_h, a_v = area(cand_h), area(cand_v)

        # Choose by: valid area → larger area → tie-breaker by smaller gate distance from pose
        def gate_distance(rect):
            xmin, xmax, ymin, ymax = rect
            xw, yw = xmax - xmin, ymax - ymin
            if xw < yw:  # vertical band
                gate = xmin if abs(pose.x - xmin) <= abs(pose.x - xmax) else xmax
                return abs(pose.x - gate)
            else:        # horizontal band
                gate = ymin if abs(pose.y - ymin) <= abs(pose.y - ymax) else ymax
                return abs(pose.y - gate)

        if a_h <= 0 and a_v <= 0:
            return (gx, gx, gy, gy)
        if a_h > 0 and (a_h >= a_v):
            if a_v <= 0:
                return cand_h
            if a_h >= a_v * 1.10:
                return cand_h
            return cand_h if gate_distance(cand_h) <= gate_distance(cand_v) else cand_v
        else:
            if a_h <= 0:
                return cand_v
            if a_v >= a_h * 1.10:
                return cand_v
            return cand_v if gate_distance(cand_v) <= gate_distance(cand_h) else cand_h

    @staticmethod
    def _in_rect(x: float, y: float, rect: Tuple[float,float,float,float]) -> bool:
        xmin,xmax,ymin,ymax = rect
        return (xmin <= x <= xmax) and (ymin <= y <= ymax)

    def get_goal_region(self) -> Optional[Tuple[float,float,float,float]]:
        """
        English: expose current goal band rectangle (if any) to external callers.
        If strict band tracking is active we return that rectangle; otherwise,
        when heading to the exit we synthesize the fixed-width border band so
        monitoring code can still treat the finish area as a region instead of
        a single point.
        """
        if self._goal_band_rect is not None:
            return self._goal_band_rect
        if self.state == NavState.TO_EXIT and self.exit_xy is not None:
            x0, y0 = self.exit_xy
            return self._compute_border_goal_band(x0, y0, half_width_m=0.20, log=False)
        return None

    def has_goal_region(self) -> bool:
        """English: helper for callers that only need to know if a region is available."""
        return self.get_goal_region() is not None

    def is_pose_in_goal_region(self, pose: Pose2D) -> bool:
        """Check whether the given pose lies within the active goal band (if any)."""
        rect = self.get_goal_region()
        if rect is None:
            return False
        return self._reached_band_by_edge_cross(pose, rect)

    @staticmethod
    def _reached_band_by_edge_cross(pose: Pose2D, rect: Tuple[float,float,float,float]) -> bool:
        """
        Arrival test with three tiers:
        1) Inside the rectangle  -> reached.
        2) Cross an INSET gate line (pulled inward) with relaxed orthogonal range.
        3) L∞-expanded rectangle (outward by GOAL_BAND_OUTER_MARGIN_M).
           This accepts points that stopped a few centimeters outside the band.
           (Optional Tier-4: Near-rectangle Euclidean fallback.)
        """
        xmin, xmax, ymin, ymax = rect

        # Tier-1: already inside
        if (xmin <= pose.x <= xmax) and (ymin <= pose.y <= ymax):
            return True

        # Tolerances for gate-crossing
        eps_axis  = 0.5 * SLAM_RESOLUTION
        eps_ortho = max(0.5 * SLAM_RESOLUTION, 0.5 * SAFE_BUFFER_M)
        inset_m   = max(0.5 * SLAM_RESOLUTION, 0.5 * SAFE_BUFFER_M)

        xw = xmax - xmin
        yw = ymax - ymin

        # Tier-2: cross inward gate with relaxed orthogonal range
        if xw < yw:
            # vertical band -> gate on x
            gxL = xmin + inset_m
            gxR = xmax - inset_m
            gate = gxL if abs(pose.x - gxL) <= abs(pose.x - gxR) else gxR
            in_y = (ymin - eps_ortho <= pose.y <= ymax + eps_ortho)
            if gate == gxL:
                if in_y and (pose.x >= gate - eps_axis):
                    return True
            else:
                if in_y and (pose.x <= gate + eps_axis):
                    return True
        else:
            # horizontal band -> gate on y
            gyB = ymin + inset_m
            gyT = ymax - inset_m
            gate = gyB if abs(pose.y - gyB) <= abs(pose.y - gyT) else gyT
            in_x = (xmin - eps_ortho <= pose.x <= xmax + eps_ortho)
            if gate == gyB:
                if in_x and (pose.y >= gate - eps_axis):
                    return True
            else:
                if in_x and (pose.y <= gate + eps_axis):
                    return True

        # --- Tier-3: L∞ outward-expanded rectangle by configured margin ---
        # English: accept arrival if pose lies inside the rectangle expanded
        # outward by GOAL_BAND_OUTER_MARGIN_M in both axes.
        delta = GOAL_BAND_OUTER_MARGIN_M
        if (xmin - delta <= pose.x <= xmax + delta) and (ymin - delta <= pose.y <= ymax + delta):
            return True

        # --- Optional Tier-4: Euclidean near-rect fallback (kept as weaker backup) ---
        near_eps = max(4 * SLAM_RESOLUTION, 0.5 * SAFE_BUFFER_M)
        dx = 0.0 if xmin <= pose.x <= xmax else (xmin - pose.x if pose.x < xmin else pose.x - xmax)
        dy = 0.0 if ymin <= pose.y <= ymax else (ymin - pose.y if pose.y < ymin else pose.y - ymax)
        if (dx*dx + dy*dy) ** 0.5 <= near_eps:
            return True

        return False

    def get_goal_band_rect(self) -> Optional[Tuple[float,float,float,float]]:
        return self._goal_band_rect

    def get_target_reachable_region(self) -> Optional[Tuple[float,float,float,float]]:
        """获取终点可达区域用于GUI显示"""
        return self._target_reachable_region

    # === 新增工具函数 ===
    def _reset_goal_band(self):
        """English: clear goal-band and its ownership."""
        self._goal_band_rect = None
        self._goal_band_owner = None

    def _reset_motion(self):
        """English: clear controller and current path without touching pose."""
        self.controller = None
        self.current_path = []

    def _create_target_reachable_region(self, target_xy, region_size_m):
        """
        1) 40×40 初始框
        2) 裁剪到世界边界 [0, WORLD_SIZE]
        3) 存 GUI
        """
        from core.config import WORLD_SIZE
        cx, cy = target_xy
        h = region_size_m * 0.5
        xmin, xmax, ymin, ymax = cx - h, cx + h, cy - h, cy + h
        xmin = max(0.0, min(WORLD_SIZE, xmin))
        xmax = max(0.0, min(WORLD_SIZE, xmax))
        ymin = max(0.0, min(WORLD_SIZE, ymin))
        ymax = max(0.0, min(WORLD_SIZE, ymax))
        # Unified rect format: (xmin, xmax, ymin, ymax)
        # Defensive normalization: ensure ordering in case a caller passed mixed values
        xmin, xmax = (min(xmin, xmax), max(xmin, xmax))
        ymin, ymax = (min(ymin, ymax), max(ymin, ymax))
        region_rect = (xmin, xmax, ymin, ymax)
        # sanity assertion to catch regressions early
        assert region_rect[0] <= region_rect[1] and region_rect[2] <= region_rect[3], "bad rect ordering"
        self._target_reachable_region = region_rect
        return region_rect

    # 删除 _can_reach_destination（冗余，已被新判定逻辑取代）
    def _validate_band_on_cspace(self, band_rect, cspace_bin) -> bool:
        """
        band 内必须存在 >=1 个 free 像素，且这些 free 像素与机器人所在的 free 连通域连通
        """
        import numpy as np
        from core.config import SLAM_RESOLUTION
        pose = self.slam.get_pose()
        H, W = cspace_bin.shape

        def to_ij(x, y):
            i = int(round(y / SLAM_RESOLUTION))
            j = int(round(x / SLAM_RESOLUTION))
            return max(0, min(H-1, i)), max(0, min(W-1, j))

        # 机器人起点
        si, sj = to_ij(pose.x, pose.y)
        if cspace_bin[si, sj] != 0:
            return False  # 机器人不在 free，直接不合法

        # band 像素窗口
        # NOTE: rect tuple unified format is (xmin, xmax, ymin, ymax)
        # Historically some helpers returned (xmin, ymin, xmax, ymax).
        xmin, xmax, ymin, ymax = band_rect
        i0, j0 = to_ij(xmin, ymin)
        i1, j1 = to_ij(xmax, ymax)
        i0, i1 = min(i0, i1), max(i0, i1)
        j0, j1 = min(j0, j1), max(j0, j1)

        band_mask = np.zeros_like(cspace_bin, dtype=np.uint8)
        band_mask[i0:i1+1, j0:j1+1] = 1
        band_free = (band_mask & (cspace_bin == 0)).astype(np.uint8)
        if band_free.sum() == 0:
            return False  # band 内完全不可行

        # 计算从 (si,sj) 的 free 连通域（4-邻接 BFS）
        from collections import deque
        q = deque([(si, sj)])
        seen = np.zeros_like(cspace_bin, dtype=np.uint8)
        seen[si, sj] = 1
        hit = False
        while q:
            i, j = q.popleft()
            if band_free[i, j]:
                hit = True
                break
            for di, dj in ((1,0),(-1,0),(0,1),(0,-1)):
                ni, nj = i+di, j+dj
                if 0 <= ni < H and 0 <= nj < W and not seen[ni, nj] and cspace_bin[ni, nj] == 0:
                    seen[ni, nj] = 1
                    q.append((ni, nj))
        return hit

    def _try_finish_exploration_by_goal_reachability(self, pose: Pose2D) -> bool:
        """
        严格判定终点可达性，连续多帧可达才切 TO_EXIT
        """
        if self.exit_xy is None:
            self._log_debug("终点坐标未设置，无法进行可达性判定")
            return False

        # 1. 构造目标带（裁剪到世界边界）
        band_rect = self._create_target_reachable_region(self.exit_xy, region_size_m=FINAL_REGION_SIZE)

        # 2. 获取标准 C-space（障碍膨胀 + UNKNOWN=blocked）
        c_cached = getattr(self.explorer, "get_latest_cspace_bin", lambda: None)() if self.explorer else None
        occ_shape = self.slam.get_occ_grid().shape
        if c_cached is not None and c_cached.shape == occ_shape:
            cspace_bin = (c_cached > 0).astype('uint8')
        else:
            # 用 _get_safebuffer_cspace 并把 UNKNOWN=2 也视为 blocked
            raw_cspace = self._get_safebuffer_cspace()
            cspace_bin = (raw_cspace != 0).astype('uint8')

        # 3. 连通性判定
        ok = self._validate_band_on_cspace(band_rect, cspace_bin)

        # 4. 再用 planner 跑一次 plan_to_band（严格 C-space）
        from core.config import SLAM_RESOLUTION
        start_ij = (int(round(pose.y / SLAM_RESOLUTION)), int(round(pose.x / SLAM_RESOLUTION)))
        path = []
        if ok:
            path = self.planner.plan_to_band(
                start_ij, band_rect, cspace_bin,
                grid_is_cspace=True, safe_buffer_m=0.0
            )
            ok = bool(path)

        # 5. 连续帧可达判定
        streak = getattr(self, '_exit_reachable_streak', 0)
        if ok:
            streak += 1
        else:
            streak = 0
        self._exit_reachable_streak = streak

        self._log_debug(f"[TO_EXIT判定] streak={streak}, path_len={len(path)}")

        # 6. 达到阈值才切换
        if streak >= 3:
            self._plan_to_band(self.exit_xy, band_rect)
            self._goal_band_owner = 'TO_EXIT'
            self._reset_motion()
            self._band_guard_ticks = 5
            self.set_state(NavState.TO_EXIT)
            self._log_debug("探索完成：出口带在严格C-space上稳定可达 → 切换 TO_EXIT")
            self._exit_reachable_streak = 0
            return True
        return False


    # === SAFE_BUFFER-only C-space ===
    def _get_safebuffer_cspace(self):
        """
        English:
          1) Reuse explorer's C-space if available (no re-inflation).
          2) Otherwise, inflate ONLY obstacles by SAFE_BUFFER_M.
             Do NOT add any world-border padding on 2.8×2.8m maps.
        """
        import numpy as _np
        occ = self.slam.get_occ_grid()  # 0=free,1=occ,2=unknown
        
        # Try to reuse explorer's C-space first
        try:
            from explore.frontier_explorer import FrontierExplorer  # local import to avoid cycles
            if hasattr(self, "frontier") and isinstance(self.frontier, FrontierExplorer):
                c_cached = getattr(self.frontier, "latest_cspace_bin", None)
                if c_cached is not None and c_cached.shape == occ.shape:
                    self._log_debug(f"[CSPACE] Reusing explorer's C-space cache")
                    return c_cached  # 0=free, 1=blocked
        except Exception:
            pass
        
        # Fallback: build C-space by inflating obstacles only
        from core.config import SAFE_BUFFER_M, SLAM_RESOLUTION
        inflate_px = max(1, int(round(SAFE_BUFFER_M / SLAM_RESOLUTION)))
        
        # Only obstacles (occ==1) are inflated. Unknown(2) stays unknown. No border padding.
        obs = (occ == 1).astype(_np.uint8)
        from scipy.ndimage import binary_dilation, generate_binary_structure
        st = generate_binary_structure(2, 1)
        inflated = binary_dilation(obs, structure=st, iterations=inflate_px)
        
        # Keep unknown areas as unknown (2)
        result = inflated.astype(_np.uint8)
        result[occ == 2] = 2
        
        self._log_debug(f"[CSPACE] Built obstacle-only C-space, SAFE_BUFFER={SAFE_BUFFER_M:.3f}m, cells={inflate_px}")
        
        # Optional: push to visualizer if available
        try:
            vis = getattr(self, "visualizer", None)
            if vis and hasattr(vis, "set_cspace"):
                vis.set_cspace(result, SLAM_RESOLUTION)  # no hard dependency
        except Exception:
            pass
        
        return result

    # --- Simple recovery strategy (v3.3) ------------------------------------
    def _find_nearest_obstacle_direction(self, pose: Pose2D) -> Tuple[float, float]:
        """
        Find the nearest obstacle and return the retreat direction (away from obstacle).
        Returns: (dx, dy) unit vector pointing away from nearest obstacle.
        """
        import math
        import numpy as np
        
        # Get occupancy grid
        occ = self.slam.get_occ_grid()  # 0=free, 1=obstacle, 2=unknown
        H, W = occ.shape
        
        # Robot position in grid coordinates
        ri = int(round(pose.y / SLAM_RESOLUTION))
        rj = int(round(pose.x / SLAM_RESOLUTION))
        
        # Search radius (in pixels) - search within reasonable range
        search_radius = int(round(0.5 / SLAM_RESOLUTION))  # 50cm search radius
        
        min_dist = float('inf')
        nearest_obstacle_ij = None
        
        # Search for nearest obstacle in a square region
        for di in range(-search_radius, search_radius + 1):
            for dj in range(-search_radius, search_radius + 1):
                ni, nj = ri + di, rj + dj
                if 0 <= ni < H and 0 <= nj < W:
                    # Check if this cell is an obstacle (1=known obstacle, 2=unknown)
                    if occ[ni, nj] in [1, 2]:
                        dist = math.sqrt(di*di + dj*dj)
                        if dist < min_dist:
                            min_dist = dist
                            nearest_obstacle_ij = (ni, nj)
        
        if nearest_obstacle_ij is None:
            # No obstacle found nearby, retreat in current heading direction
            return (math.cos(pose.theta), math.sin(pose.theta))
        
        # Calculate retreat direction (away from nearest obstacle)
        obs_i, obs_j = nearest_obstacle_ij
        dx = rj - obs_j  # robot_j - obstacle_j (world x direction)
        dy = ri - obs_i  # robot_i - obstacle_i (world y direction)
        
        # Normalize to unit vector
        length = math.sqrt(dx*dx + dy*dy)
        if length > 0:
            dx /= length
            dy /= length
        
        self._log_debug(f"[SIMPLE_RECOVER] 最近障碍: grid({obs_i},{obs_j}) 距离: {min_dist*SLAM_RESOLUTION:.3f}m")
        self._log_debug(f"[SIMPLE_RECOVER] 退避方向: ({dx:.3f}, {dy:.3f})")
        
        return (dx, dy)

    def _simple_retreat_recovery(self, pose: Pose2D) -> bool:
        """
        固定退避：找最近障碍的反方向，量化到主轴，执行 TURN→MOVE(0.10m)。
        不看C-space与连通性，只做一次0.1m的轴向脱困。
        """
        # 1) 最近障碍方向（已把 unknown 当作障碍）
        dx, dy = self._find_nearest_obstacle_direction(pose)

        # 2) 量化到主轴 {x+/x-/y+/y-}
        axis, dir_sign = self._segment_axis(dx, dy)
        theta_star = self._axis_heading(axis, dir_sign)

        # 3) 注入恢复原语队列：TURN → MOVE(0.10m)
        self._primitive_queue = [
            {"type": "TURN", "theta": float(theta_star), "recovery": True},
            {"type": "MOVE", "axis": axis, "dir": int(dir_sign),
             "distance": float(SIMPLE_RETREAT_DISTANCE_M), "recovery": True},
        ]
        self._active_primitive = None
        self._prim_start_xy = None
        self._band_rect = None
        self._primitive_mode = True

        # 进入恢复态：禁用路径/目标，避免并行干扰
        self._recovery_mode = True
        self.current_goal = None
        self.current_path = []
        self.controller = None  # 不使用常规门控

        self._log_debug(f"[RECOVER] 主轴量化: axis={axis}, dir={dir_sign}, θ*={theta_star:.3f}rad")
        return True

    def _generate_primitive_path(self, pose: Pose2D) -> List[Tuple[float, float]]:
        """
        从原语队列生成可视化路径点
        返回：路径点列表 [(x1,y1), (x2,y2), ...]
        """
        if not self._primitive_queue or not self._band_rect:
            return []
        
        import math
        path_points = []
        current_x, current_y = pose.x, pose.y
        
        # 添加起始点
        path_points.append((current_x, current_y))
        
        # 遍历原语队列，计算每个原语的终点
        for primitive in self._primitive_queue:
            if primitive["type"] == "TURN":
                # TURN原语不改变位置，只改变朝向
                continue
            elif primitive["type"] == "MOVE":
                # MOVE原语：沿轴向移动指定距离
                axis = primitive["axis"]
                distance = abs(primitive["distance"])  # English: visualize with magnitude
                dir_sign = primitive["dir"]
                
                if axis == "x":
                    current_x += dir_sign * distance
                else:  # axis == "y"
                    current_y += dir_sign * distance
                
                path_points.append((current_x, current_y))
            elif primitive["type"] == "GATE":
                # GATE原语：移动到闸门位置
                axis = primitive["axis"]
                gate_val = primitive["gate"]
                
                if axis == "x":
                    current_x = gate_val
                else:  # axis == "y"
                    current_y = gate_val
                
                path_points.append((current_x, current_y))
        
        # 清理蓝线：对齐 + 去重 + 共线压缩
        path_points = self._clean_primitive_path(path_points)
        return path_points

    def _decompose_path_to_primitives(self, pts: Sequence[Tuple[float,float]], pose_theta: float) -> List[dict]:
        """Decompose axis-aligned polyline to TURN+MOVE primitives.
        Final point is inside band; no GATE is required."""
        prims: List[dict] = []
        if not pts or len(pts) < 2:
            return prims

        EPS_MOVE = max(0.5 * SLAM_RESOLUTION, 0.01)  # same as planner
        theta = float(pose_theta)

        for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
            dx, dy = x1 - x0, y1 - y0
            axis, dsgn = self._segment_axis(dx, dy)
            d = abs(dx) if axis == "x" else abs(dy)
            if d <= EPS_MOVE:
                continue  # ignore tiny segment

            # heading for this leg
            t_star = self._axis_heading(axis, dsgn)
            err = self._wrap_pi(t_star - theta)
            if abs(err) > 1e-3:
                prims.append({"type": "TURN", "heading_rad": float(t_star)})
                theta = t_star  # apply

            prims.append({"type": "MOVE", "axis": axis, "distance": float(d), "dir": int(dsgn)})

        return prims


    def _clean_primitive_path(self, path_points: Sequence[Tuple[float,float]]) -> List[Tuple[float,float]]:
        """Grid-align, deduplicate, compress collinear, then shave micro segments."""
        if not path_points:
            return []
        # 1) align to grid
        aligned = []
        for x, y in path_points:
            gx = round(x / SLAM_RESOLUTION) * SLAM_RESOLUTION
            gy = round(y / SLAM_RESOLUTION) * SLAM_RESOLUTION
            aligned.append((gx, gy))
        # 2) dedup consecutive
        ded = [aligned[0]]
        for p in aligned[1:]:
            if p != ded[-1]:
                ded.append(p)
        # 3) compress collinear
        if len(ded) <= 2:
            base = ded
        else:
            out = [ded[0]]
            for p in ded[1:]:
                if len(out) >= 2:
                    a, b, c = out[-2], out[-1], p
                    if (a[0] == b[0] == c[0]) or (a[1] == b[1] == c[1]):
                        out[-1] = c
                        continue
                out.append(p)
            base = out
        # 4) shave micro segments (最短段剃除): 丢弃长度 < PRIM_MIN_SEG_LEN_M 的小段
        if len(base) <= 2:
            return base
        shaved = [base[0]]
        for idx, p in enumerate(base[1:], start=1):
            px, py = shaved[-1]
            dx, dy = p[0] - px, p[1] - py
            seg = (dx*dx + dy*dy) ** 0.5
            # 最后一个点必须保留；中间点若距离过短则延后合并
            if seg >= PRIM_MIN_SEG_LEN_M or idx == len(base) - 1:
                shaved.append(p)
            else:
                # 把"短段"吸收至下一点：这里先跳过，下一轮会用更远的点来形成更长一段
                continue
        # 若剃除后只剩一个点，补上终点保证至少两点
        if len(shaved) == 1 and len(base) >= 2:
            shaved.append(base[-1])
        return shaved

    def _plan_to_band(self, goal_xy: Tuple[float,float], band_rect: Tuple[float,float,float,float], meta: dict = None) -> None:
        """Plan on strict C-space to the band, then decompose polyline to primitives."""
        import time
        self._band_rect = band_rect  # for arrival check
        self._goal_band_rect = band_rect   # English: keep arrival checks consistent
        self._primitive_mode = True
        self._active_primitive = None
        self._prim_start_xy = None

        pose = self.slam.get_pose()
        from core.config import SLAM_RESOLUTION, SAFE_BUFFER_M

        # start, band → pixels
        start_ij = (int(round(pose.y / SLAM_RESOLUTION)), int(round(pose.x / SLAM_RESOLUTION)))

        # Use cached strict C-space if explorer exposes it; otherwise fall back to occ
        use_cspace = False
        grid_for_plan = None
        if self.explorer:
            cached = getattr(self.explorer, "get_latest_cspace_bin", lambda: None)()
            if cached is not None and cached.shape == self.slam.get_occ_grid().shape:
                grid_for_plan = cached            # 0/1 C-space
                use_cspace = True
            else:
                grid_for_plan = self.slam.get_occ_grid()
        else:
            grid_for_plan = self.slam.get_occ_grid()

        # strict band planning on C-space
        eff_buf = 0.0 if use_cspace else SAFE_BUFFER_M
        blocked = meta.get("blocked_mask", None) if meta else None  # 若为 None，planner 内部会自己构造 C-space
        safe_poly = self.planner.plan_to_band(
            start_ij, band_rect, grid_for_plan,
            safe_buffer_m=eff_buf, grid_is_cspace=use_cspace,
            blocked_mask=blocked   # ← 关键：有就用，不再二次膨胀
        )

        if not safe_poly:
            # fallback：维持空队列，交给探索器重选
            self._primitive_queue = []
            self.current_path = []
            self._log_debug("[PRIM] plan_to_band 失败，保持原地等待")
            return

        # 轴向对齐 + 重采样（与全局一致）
        safe_poly = self._align_path_to_axis(safe_poly)

        # 可视化路径（先清理小抖动）
        self.current_path = self._clean_primitive_path(safe_poly)

        # 折线 → 原语队列（TURN + MOVE；不强制 GATE，终点已在带内）
        self._primitive_queue = self._decompose_path_to_primitives(self.current_path, pose_theta=pose.theta)

        self.current_goal = goal_xy
        self.goal_start_time = time.time()
        self._primitive_mode = True      # English: ensure primitive engine runs
        self.controller = None           # no tracker in primitive mode
        self._log_debug(f"切换到原语模式(严格C-space)，原语数={len(self._primitive_queue)}，折线路径点={len(self.current_path)}")

    def _update_motion(self, pose):
        """Run either primitive follower or legacy tracker."""
        if self._primitive_mode and (self._band_rect or self._primitive_queue):
            # fetch next primitive if none active
            if self._active_primitive is None and self._primitive_queue:
                self._active_primitive = self._primitive_queue.pop(0)
                self._prim_start_xy = (pose.x, pose.y)
                try:
                    self._log_debug(f"[PRIM] 激活原语: {self._active_primitive}")
                except (AttributeError, KeyError, TypeError) as e:
                    self._log_debug(f"[PRIM] 日志记录失败: {e}")
            if self._active_primitive:
                prim = self._active_primitive
                # Compute remaining distance for MOVE using start pose
                remaining = None
                try:
                    if prim.get("type") == "MOVE" and self._prim_start_xy is not None:
                        sx, sy = self._prim_start_xy
                        moved = abs(pose.x - sx) if prim["axis"] == "x" else abs(pose.y - sy)
                        dist_mag = abs(float(prim["distance"]))  # English: planner may pass negative for -x/-y; use magnitude
                        remaining = max(0.0, dist_mag - moved)
                except (KeyError, TypeError, ValueError) as e:
                    try: self._log_debug(f"[PRIM] remaining-dist failed: {e}")
                    except Exception: pass
                    remaining = None
                
                # === Recovery primitive execution: bypass controller gating ===
                if prim.get("recovery", False):
                    # 参数
                    v_rec = self._recovery_params["v"]
                    k_w   = self._recovery_params["w_gain"]
                    yaw_allow = self._recovery_params["yaw_allow"]
                    yaw_done  = self._recovery_params["yaw_done"]

                    v_cmd, w_cmd, done = 0.0, 0.0, False

                    if prim["type"] == "TURN":
                        theta_star = float(prim["theta"])
                        err = self._wrap_pi(theta_star - pose.theta)
                        w_cmd = max(-1.0, min(1.0, k_w * err))
                        v_cmd = 0.0
                        done = abs(err) <= yaw_done

                    elif prim["type"] == "MOVE":
                        # 计算目标朝向（主轴 + 方向）
                        theta_star = self._axis_heading(prim["axis"], prim["dir"])
                        err = self._wrap_pi(theta_star - pose.theta)
                        # 航向门控：误差大于阈值只转不走；否则以固定速度直行
                        if abs(err) > yaw_allow:
                            w_cmd = max(-1.0, min(1.0, k_w * err))
                            v_cmd = 0.0
                        else:
                            w_cmd = max(-1.0, min(1.0, k_w * err * 0.2))  # 微调保持对准
                            v_cmd = v_rec
                        # 距离完成判定（用 _prim_start_xy 与标称距离）
                        if self._prim_start_xy is None:
                            self._prim_start_xy = (pose.x, pose.y)
                        sx, sy = self._prim_start_xy
                        traveled = abs(pose.x - sx) if prim["axis"] == "x" else abs(pose.y - sy)
                        dist_mag = abs(float(prim["distance"]))
                        remaining = max(0.0, dist_mag - traveled)
                        done = remaining <= 1e-3

                    elif prim["type"] == "GATE":
                        # 恢复策略不使用GATE，直接视为完成
                        done = True

                    # 发令并处理完成
                    try:
                        self._log_debug(f"[PRIM:RECOVER] v={v_cmd:.2f}, w={w_cmd:.2f}, done={done}")
                    except Exception:
                        pass

                    if done:
                        # 统计与收尾
                        self._active_primitive = None
                        self._prim_start_xy = None
                        self._primitive_stats['total_executed'] += 1
                        # 若队列空了，退出原语模式与恢复态
                        if not self._primitive_queue:
                            self._exit_primitive_mode()
                            self._recovery_mode = False
                    return v_cmd, w_cmd
                
                # Always update GATE direction from current pose vs gate
                if prim.get("type") == "GATE":
                    try:
                        if prim["axis"] == "x":
                            prim["dir"] = +1 if pose.x <= float(prim["gate"]) else -1
                        else:
                            prim["dir"] = +1 if pose.y <= float(prim["gate"]) else -1
                    except (KeyError, TypeError, ValueError) as e:
                        try: self._log_debug(f"[PRIM] gate-dir failed: {e}")
                        except Exception: pass
                # Ensure controller has the latest methods by reimporting if needed
                if self.controller is None or not hasattr(self.controller, 'follow_primitive'):
                    from .motion_controller import MotionController
                    self.controller = MotionController()
                v, w, done = self.controller.follow_primitive(pose, prim, remaining_dist=remaining)
                # 日志在 remaining 为 None 时安全打印
                rem_str = f"{remaining:.3f}m" if (remaining is not None) else "N/A"
                try:
                    self._log_debug(f"[PRIM] 执行: type={prim.get('type')} rem={rem_str} → v={v:.2f}, w={w:.2f}, done={done}")
                except (AttributeError, KeyError, TypeError) as e:
                    self._log_debug(f"[PRIM] 执行日志失败: {e}")
                if done:
                    # 更新统计信息
                    prim_type = prim.get('type', 'UNKNOWN')
                    self._primitive_stats['total_executed'] += 1
                    if prim_type == 'TURN':
                        self._primitive_stats['turn_count'] += 1
                    elif prim_type == 'MOVE':
                        self._primitive_stats['move_count'] += 1
                    elif prim_type == 'GATE':
                        self._primitive_stats['gate_count'] += 1
                    
                    try:
                        stats = self._primitive_stats
                        self._log_debug(f"[PRIM] 完成: {prim} (总计: {stats['total_executed']}, T:{stats['turn_count']}, M:{stats['move_count']}, G:{stats['gate_count']})")
                    except (AttributeError, KeyError, TypeError) as e:
                        self._log_debug(f"[PRIM] 完成日志失败: {e}")
                    # 检查是否是掉头操作完成
                    if prim.get("turn_around", False):
                        self._log_debug("掉头操作完成，切换到返回起点状态")
                        self._turn_around_completed = True
                        self._pending_return_home = False
                        self.set_state(NavState.RETURN_HOME)
                        self._plan_to(self.entrance_xy)  # 立即规划返回路径
                        self._exit_primitive_mode()
                        return (0.0, 0.0)
                    
                    self._active_primitive = None
                    self._prim_start_xy = None
                    
                    # 原语执行完成后，更新可视化路径
                    self.current_path = self._generate_primitive_path(pose)
                    
                    if not self._primitive_queue:
                        self._on_band_arrived(pose)
                return v, w
            # queue exhausted but arrival not yet confirmed:
            # 若已经越过闸门或已在带内，直接判定到达；否则短暂停车等待下一帧
            
            # 超时保护：原语队列清空但未到达，检查超时
            if self.goal_start_time is not None:
                elapsed_time = time.time() - self.goal_start_time
                if elapsed_time > self.goal_arrival_timeout:
                    self._log_debug(f"[PRIM] 原语执行超时({elapsed_time:.1f}s > {self.goal_arrival_timeout}s) → 强制退出原语模式")
                    self._exit_primitive_mode()
                    self.goal_start_time = None
                    return 0.0, 0.0
            
            if self._band_rect and self._check_goal_band_arrival(pose):
                self._on_band_arrived(pose)
            return 0.0, 0.0
        # === 传统路径跟踪：使用严格"先旋转后直行"的 step()，并传入当前段轴向 ===
        if self.controller is None or not hasattr(self.controller, 'step'):
            from .motion_controller import MotionController
            self.controller = MotionController()
        # 告知控制器当前段的 axis / dir（用于四向锁定）
        Pk, Pk1 = self._current_segment_points()
        ax, dr = self._segment_axis(Pk1[0]-Pk[0], Pk1[1]-Pk[1])
        if hasattr(self.controller, "set_segment_axis"):
            self.controller.set_segment_axis(ax, dr)
        v, w = self.controller.step(pose)
        return v, w

    def _check_goal_band_arrival(self, pose):
        """
        Arrival test for corridor band using 'gate' rule.
        Horizontal: y in [ymin,ymax] and x >= xmin - eps
        Vertical:   x in [xmin,xmax] and y >= ymin - eps
        """
        if not self._band_rect:
            return False
        # 统一用"越过闸门边界"判定（已含正交方向宽松容差）
        return self._reached_band_by_edge_cross(pose, self._band_rect)

    def _execute_turn_around(self):
        """
        执行原地180度掉头
        
        English: Execute a 180-degree turn in place to face the opposite direction.
        This is called when the robot reaches the exit and needs to turn around
        before planning the return path to the entrance.
        """
        current_pose = self.slam.get_pose()
        target_theta = current_pose.theta + math.pi  # 180度掉头
        
        # 角度归一化到 [-π, π]
        while target_theta > math.pi:
            target_theta -= 2 * math.pi
        while target_theta < -math.pi:
            target_theta += 2 * math.pi
        
        self._log_debug(f"执行原地掉头：当前朝向 {current_pose.theta:.3f}rad → 目标朝向 {target_theta:.3f}rad")
        
        # 创建掉头原语队列
        self._primitive_queue = [{
            "type": "TURN", 
            "heading_rad": float(target_theta),
            "turn_around": True  # 标记这是掉头操作，用于日志识别
        }]
        self._active_primitive = None
        self._primitive_mode = True
        self._recovery_mode = False
        self._band_rect = None  # 掉头不需要到达带
        
        # 清理当前路径和目标，专注于掉头
        self.current_goal = None
        self.current_path = []
        self.controller = None
        
        self._log_debug("掉头原语已创建，开始执行180度旋转")

    def _exit_primitive_mode(self):
        """统一退出原语模式，清理所有相关状态"""
        if self._primitive_mode:
            try:
                stats = self._primitive_stats
                self._log_debug(f"[PRIM] 退出原语模式 (本次统计: T:{stats['turn_count']}, M:{stats['move_count']}, G:{stats['gate_count']})")
            except Exception:
                pass
        
        self._primitive_mode = False
        self._band_rect = None
        self._active_primitive = None
        self._prim_start_xy = None
        self._primitive_queue = []
        self._recovery_mode = False  # 清理恢复态
        # 清理可视化蓝线，避免画面残留一小截
        self.current_path = []
        
        # 重置统计信息（可选：保留累计统计）
        # self._primitive_stats = {'total_executed': 0, 'turn_count': 0, 'move_count': 0, 'gate_count': 0}

    def _on_band_arrived(self, pose):
        """Reset primitive mode and notify upper logic that band reached."""
        if self._check_goal_band_arrival(pose):
            arrival_rect = self._band_rect or self._goal_band_rect
            try:
                self._log_debug("[PRIM] 到达：越过闸门/进入到达带，退出原语模式")
            except Exception:
                pass
            # 使用统一的退出方法
            self._exit_primitive_mode()
            # 原语模式下没有传统路径，直接清理当前探索目标
            self.current_goal = None
            self.current_path = []
            self.controller = None
            if arrival_rect is not None:
                self._goal_band_rect = arrival_rect  # 记录最后一次成功的到达带，供全局逻辑判定

            if self.state == NavState.TO_EXIT:
                self._log_debug("阶段3完成：出口已到达，准备掉头返回起点")
                self._execute_turn_around()
                self._turn_around_completed = False
                self._pending_return_home = True
            elif self.state == NavState.RETURN_HOME:
                self._log_debug("阶段4完成：已返回入口，任务完成")
                self.set_state(NavState.FINISHED)

    def set_initial_goal(self, goal_xy: Tuple[float, float]) -> None:
        """Set initial goal (e.g., maze entry point) before starting exploration."""
        self._log_debug(f"阶段1：设置迷宫入口为目标 {goal_xy}")
        self.entrance_xy = goal_xy  # 保存入口位置
        self.start_xy = goal_xy     # 设置起点闸门位置为迷宫入口
        self.current_goal = goal_xy
        self._plan_to(goal_xy)
        if self.current_path:
            self._log_debug(f"阶段1路径规划成功，路径长度: {len(self.current_path)}")
            self._log_debug(f"路径点: {self.current_path[:3]}...")  # Show first 3 points
            self.state = NavState.TO_ENTRANCE  # 明确设置为前往入口状态
        else:
            self._log_debug("阶段1路径规划失败，目标可能不可达")
            # 即使路径规划失败，也保持目标，让探索模式尝试
            self.state = NavState.TO_ENTRANCE

    def set_state(self, s: NavState) -> None:
        current_time = time.time()
        
        # 检查状态切换冷却时间 - 只对重复切换进行保护
        if (self.state != s and 
            current_time - self.last_state_change_time < self.state_change_cooldown and
            self.last_state == s):  # 只阻止回到上一个状态
            self._log_debug(f"状态切换被冷却时间阻止: {self.state.name} -> {s.name}")
            return
        
        if self.state != s:
            self._log_debug(f"状态转换: {self.state.name} -> {s.name}")
            self.last_state = self.state
            self.state_change_time = current_time
            self.last_state_change_time = current_time
        
        # FIX: reset motion & band state when entering goal phases
        if s in (NavState.TO_EXIT, NavState.RETURN_HOME):
            self._log_debug("状态切换：清空上一阶段的控制器与到达带状态")
            self._reset_motion()       # English: drop old controller/path
            self._reset_goal_band()    # English: drop stale band from exploring
            self._band_guard_ticks = 5 # English: ignore band-cross for first few updates
            self._primitive_queue = []        # clear primitive executor
            self._primitive_mode = False      # 清空原语模式
            self._active_primitive = None     # 清空活跃原语
            self._prim_start_xy = None        # 清空原语起始位置
            self._band_rect = None            # 清空到达带
            self._pending_return_home = False
            self._turn_around_completed = False
            self._recovery_mode = False       # 清空恢复模式
        
        self.state = s
        if s == NavState.EXPLORING:
            # 进入探索模式时，确保目标已清除
            self._log_debug("EXPLORING状态: 进入探索模式，准备前沿点探索")
            
            # 记录起始位置
            if self.start_xy is None:
                p = self.slam.get_pose()
                self.start_xy = (p.x, p.y)
                self._log_debug(f"记录起始位置: {self.start_xy}")
        elif s == NavState.TO_EXIT:
            self._log_debug(f"TO_EXIT状态: 目标出口 {self.exit_xy}")
            self.current_goal = self.exit_xy
        elif s == NavState.RETURN_HOME:
            self._log_debug(f"RETURN_HOME状态: 目标起始位置 {self.start_xy}")
            if self.start_xy is not None:
                self._plan_to(self.start_xy)
        elif s == NavState.FINISHED:
            self._log_debug("FINISHED状态: 停止机器人")
            self.current_control_cmd = (0.0, 0.0)

    def _plan_to(self, goal_xy: Tuple[float,float]) -> None:
        import time
        
        # 任何时候显式走传统规划时，先退出原语模式，避免并存状态互相覆盖
        if self._primitive_mode:
            self._log_debug("[PRIM] 传统规划触发，退出原语模式")
            self._exit_primitive_mode()
        
        # === Performance optimization: replanning throttling ===
        # English: Add cooldown to avoid excessive A* replanning
        now = time.time()
        # 只有在"已经有有效tracker/路径"时才执行节流；否则必须立即规划
        if (self.controller is not None and self.current_path) and (now - self._last_plan_t < self._replan_cooldown):
            self._log_debug(f"重规划节流: 距离上次规划仅{now - self._last_plan_t:.2f}s < {self._replan_cooldown}s")
            return
        
        self._log_debug(f"开始路径规划: 目标 {goal_xy}")
        p = self.slam.get_pose()
        
        # 直接使用SLAM像素坐标进行高精度路径规划
        # A*期望 (row, col) 格式，其中 row=y_pixel, col=x_pixel
        from core.config import SLAM_RESOLUTION
        start_ij = (int(round(p.y / SLAM_RESOLUTION)), int(round(p.x / SLAM_RESOLUTION)))
        goal_ij = (int(round(goal_xy[1] / SLAM_RESOLUTION)), int(round(goal_xy[0] / SLAM_RESOLUTION)))

        # 防御性裁剪：确保像素索引在占据网格范围内，避免传入越界索引导致 A* 崩溃
        try:
            occ_shape = self.slam.get_occ_grid().shape
            H, W = occ_shape
            si, sj = start_ij
            gi, gj = goal_ij
            si = max(0, min(H-1, si))
            sj = max(0, min(W-1, sj))
            gi = max(0, min(H-1, gi))
            gj = max(0, min(W-1, gj))
            start_ij = (si, sj)
            goal_ij = (gi, gj)
        except Exception:
            # 若获取占据网格失败，退回到原始索引（让下游处理报错）
            pass

        self._log_debug(f"规划参数: 起点({p.x:.2f},{p.y:.2f})->SLAM像素{start_ij}, 目标{goal_xy}->SLAM像素{goal_ij}")
        
        # === Key Fix: Use cached C-space to avoid double inflation ===
        use_cspace = False
        grid_for_astar = None

        if self.explorer:
            cached_cspace = getattr(self.explorer, "get_latest_cspace_bin", lambda: None)()
            if cached_cspace is not None and cached_cspace.shape == self.slam.get_occ_grid().shape:
                grid_for_astar = cached_cspace          # 0/1，已是 C-space
                use_cspace = True

        if grid_for_astar is None:
            grid_for_astar = self.slam.get_occ_grid()  # 0/1/2 原始占据。A* 内部一次性膨胀。

        # 关键修复：若已是C-space（二值），A* 不再二次膨胀
        eff_buffer = 0.0 if use_cspace else SAFE_BUFFER_M
        path = self.planner.plan(
            start_ij, goal_ij, grid_for_astar,
            safe_buffer_m=eff_buffer,
            grid_is_cspace=use_cspace  # Tell A* whether to skip inflation
        )
        
        if path:
            self._log_debug(f"路径规划成功: {len(path)}个路径点")
            self._log_debug(f"路径前3点: {path[:3]}")
            self.current_goal = goal_xy
            
            # === 严格路径跟踪：A*输出轴对齐处理 ===
            path_aligned = self._align_path_to_axis(path)
            self._log_debug(f"A*轴对齐处理: {len(path)}点 → {len(path_aligned)}点")
            
            # === Key Enhancement: Resample path to fixed step for stable tracking ===
            # English: Resample polyline at fixed step ds to get smooth arc-length progress
            # This ensures uniform spacing for progress-based arrival detection
            def _resample_pts(pts: Sequence[Tuple[float, float]], ds: float) -> Sequence[Tuple[float, float]]:
                """Resample path at fixed arc-length step ds."""
                if not pts:
                    return []
                
                out = [pts[0]]
                
                for k in range(1, len(pts)):
                    x0, y0 = out[-1]
                    x1, y1 = pts[k]
                    vx, vy = x1 - x0, y1 - y0
                    seg = (vx*vx + vy*vy) ** 0.5
                    
                    if seg < 1e-6:
                        continue
                    
                    ux, uy = vx / seg, vy / seg
                    remain = seg
                    
                    # Insert intermediate points at ds spacing
                    while remain >= ds:
                        x0 += ux * ds
                        y0 += uy * ds
                        out.append((x0, y0))
                        remain -= ds
                    
                    # Add segment endpoint if not too close to last point
                    if remain > 1e-6:
                        out.append((x1, y1))
                
                return out
            
            path_resampled = _resample_pts(path_aligned, PATH_RESAMPLE_M)
            self._log_debug(f"路径重采样: {len(path_aligned)}点 → {len(path_resampled)}点 (步长{PATH_RESAMPLE_M}m)")
            
            # 关键：再做一次清理，去掉采样/数值带来的微小折线与尾巴
            self.current_path = self._clean_primitive_path(path_resampled)
            # 交给严格运动控制器（内部会二次做共线压缩，只保留角点）
            # Ensure controller has the latest methods by reimporting if needed
            if self.controller is None or not hasattr(self.controller, 'follow_primitive'):
                from .motion_controller import MotionController
                self.controller = MotionController()
            self.controller.set_path(self.current_path)
            
            # 根据当前状态选择不同的目标容差和超时机制
            if self.state == NavState.EXPLORING:
                tolerance = GOAL_TOLERANCES['exploration']  # Use exploration tolerance from dict
                enable_timeout = False  # 探索模式禁用超时机制 - 只依赖距离判定
                self._log_debug(f"使用探索容差: {tolerance}m, 禁用超时机制")
            elif self.state == NavState.TO_ENTRANCE:
                tolerance = GOAL_TOLERANCES['entry']  # Use entry tolerance from dict
                enable_timeout = False  # 入口导航禁用超时机制
                self._log_debug(f"使用入口容差: {tolerance}m, 禁用超时机制")
            elif self.state == NavState.TO_EXIT:
                tolerance = GOAL_TOLERANCES['exit']  # Use exit tolerance from dict
                enable_timeout = False  # 出口导航禁用超时机制
                self._log_debug(f"使用出口容差: {tolerance}m, 禁用超时机制")
            else:
                tolerance = GOAL_TOLERANCES['default']
                enable_timeout = False  # 其他状态禁用超时机制
                self._log_debug(f"使用默认容差: {tolerance}m, 禁用超时机制")
            
            # === Critical Fix: Use unified controller for path tracking ===
            # English: Use MotionController for unified path tracking
            self._arm_controller_with_path(path_resampled)
            
            # === Performance optimization: update planning timestamp ===
            # English: Record successful planning time to enable throttling
            self._last_plan_t = now
        else:
            self._log_debug("路径规划失败: 无法找到路径")
            self.current_goal = None
            self.current_path = []
            self.controller = None

    def _current_segment_points(self) -> Tuple[Tuple[float,float], Tuple[float,float]]:
        """English: robustly fetch the active path segment as two points."""
        if not self.current_path or not self.controller:
            return ((0.0,0.0),(0.0,0.0))
        # Use controller's idx instead of tracker's idx
        k = max(0, min(self.controller.idx, len(self.current_path)-1))
        Pk1 = self.current_path[k]
        Pk  = self.current_path[k-1] if k-1 >= 0 else (self.slam.get_pose().x, self.slam.get_pose().y)
        return (Pk, Pk1)

    def update(self) -> None:
        if self.state == NavState.IDLE:
            self.current_control_cmd = (0.0, 0.0)
            return
            
        pose = self.slam.get_pose()
        occ = self.slam.get_occ_grid()
        # Watch known coverage drop → hint explorer to disable ROI once
        known_now = int((occ != 2).sum())
        if known_now + int(0.05 * occ.size) < self._last_known_cells:
            self._log_debug("⚠ Coverage dropped; request explorer to use full map once.")
            if hasattr(self.explorer, "_roi_force_full"):
                self.explorer._roi_force_full = True
        self._last_known_cells = known_now

        # 添加导航状态详细调试
        self._log_debug(f"导航状态详细调试:")
        self._log_debug(f"  - 当前状态: {self.state.name}")
        self._log_debug(f"  - 当前位姿: ({pose.x:.3f}, {pose.y:.3f}, {pose.theta:.3f})")
        self._log_debug(f"  - 当前目标: {self.current_goal}")
        self._log_debug(f"  - 路径长度: {len(self.current_path)}")
        self._log_debug(f"  - 控制器: {'已初始化' if getattr(self, 'controller', None) else '未初始化'}")
        if getattr(self, 'controller', None):
            self._log_debug(f"  - 控制器索引: {self.controller.idx}")
            self._log_debug(f"  - 控制器状态: {'活跃' if self.controller.idx < len(self.current_path) else '完成'}")

        # 三阶段导航系统 - 更新控制命令
        if self.state == NavState.TO_ENTRANCE:
            self.current_control_cmd = self._update_to_entrance(pose, occ)
        elif self.state == NavState.EXPLORING:
            self.current_control_cmd = self._update_exploring(pose, occ)
        elif self.state == NavState.TO_EXIT:
            self.current_control_cmd = self._update_to_exit(pose, occ)
        elif self.state == NavState.RETURN_HOME:
            self.current_control_cmd = self._update_return_home(pose, occ)
        elif self.state == NavState.FINISHED:
            self.current_control_cmd = (0.0, 0.0)

    def _update_to_entrance(self, pose: Pose2D, occ_grid) -> Tuple[float, float]:
        """阶段1：前往迷宫入口"""
        self._log_debug(f"阶段1更新：前往入口 {self.entrance_xy}")
        
        # 如果没有入口目标，等待
        if self.entrance_xy is None:
            self._log_debug("阶段1：等待入口目标设置")
            return (0.0, 0.3)
        
        # 添加详细的入口检测信息
        distance = pose.distance_to(Pose2D(self.entrance_xy[0], self.entrance_xy[1], 0.0))
        self._log_debug(f"阶段1：距离入口 {distance:.3f}m")
        
        # 添加控制器状态信息
        if self.controller:
            self._log_debug(f"阶段1：控制器索引 {self.controller.idx}/{len(self.current_path)}")
            self._log_debug(f"阶段1：控制器完成 {self.controller.is_done()}")
        
        # 检查是否到达入口（多种方式）
        entrance_reached = False
        
        # 方式1：距离检测
        if self._check_entrance_reached(pose):
            entrance_reached = True
            self._log_debug("阶段1完成：距离检测到达入口")
        
        # 方式2：控制器完成检测
        elif self.controller and self.controller.is_done():
            entrance_reached = True
            self._log_debug("阶段1完成：控制器检测到达入口")
        
        if entrance_reached:
            self._log_debug("阶段1完成：已到达迷宫入口，切换到探索模式")
            self.entrance_reached = True
            self.exploration_started = True
            
            # 先清除当前目标，再切换状态
            self.current_goal = None
            self.current_path = []
            self.controller = None
            
            # 切换到探索模式
            self.set_state(NavState.EXPLORING)
            return (0.0, 0.0)  # 状态切换时停止
        
        # 跟踪路径到入口
        if self.controller and self.current_path:
            # === 严格路径跟踪：检查路径偏差 ===
            is_deviated, deviation_dist = self._check_path_deviation(pose, self.current_path)
            if is_deviated:
                self._log_debug(f"⚠️ 路径偏差过大: {deviation_dist:.3f}m > {STRICT_PATH_TOLERANCE}m，重新规划")
                if self.entrance_xy:
                    self._plan_to(self.entrance_xy)
                return (0.0, 0.0)
            
            # Use controller for path tracking
            self._log_debug(f"阶段1：使用控制器跟踪路径")
            
            # 直接使用严格运动控制器（先旋转后移动）
            cmd = self._update_motion(pose)
            self._log_debug(f"阶段1：控制器命令 v={cmd[0]:.2f}, w={cmd[1]:.2f}")
            return cmd
        else:
            self._log_debug("阶段1：控制器不可用，重新规划路径")
            if self.entrance_xy:
                self._plan_to(self.entrance_xy)
            return (0.0, 0.0)  # 重新规划时停止
 
    def _update_exploring(self, pose: Pose2D, occ_grid) -> Tuple[float, float]:
        """阶段2：在迷宫内探索"""
        self._log_debug(f"阶段2更新：探索迷宫")

        if self._try_finish_exploration_by_goal_reachability(pose):
                self._log_debug("！！！！阶段2完成：基础探索完成且出口已验证可达,前往出口")
                self.exploration_complete = True
                return (0.0, 0.0)  # 状态切换当帧不再运动

        # 1a. 基础探索完成检查: 优先通过未知区域比例判定
        if self._is_exploration_complete(occ_grid):
            self._log_debug("阶段2: 探索基本完成 - 未知区域已在阈值以下")
            if self.exit_xy is None:
                self._log_debug("错误：出口位置未设置！请在初始化时调用 set_maze_exit()")
                self.set_state(NavState.FINISHED)
                return (0.0, 0.0)

            # 1b. 出口可达性次级判定: 在基础完成的基础上,确认出口区域可达
            if self._try_finish_exploration_by_goal_reachability(pose):
                self._log_debug("阶段2完成：基础探索完成且出口已验证可达,前往出口")
                self.exploration_complete = True
                return (0.0, 0.0)  # 状态切换当帧不再运动
            else:
                self._log_debug("阶段2: 基础探索完成但出口不可达,继续探索")
                # 继续正常探索流程

        # 2) 正常探索：挑选前沿或触发兜底
        if self.current_goal is None:
            self._log_debug("阶段2：尝试选择新的前沿点")
            hint = {"last_dir": self._last_dir_vec, "exit_xy": self.exit_xy}
            sel = self.explorer.choose_next_frontier(occ_grid, pose, hint=hint)
            if sel is None:
                # 前沿搜索失败，增加计数器
                self.frontier_failure_count += 1
                self._log_debug(f"阶段2：没有找到前沿点 (失败次数: {self.frontier_failure_count}/{FRONTIER_FAILURE_THRESHOLD})")
                
                # 检查是否达到恢复阈值
                if self.frontier_failure_count >= FRONTIER_FAILURE_THRESHOLD:
                    self._log_debug(f"阶段2：达到恢复阈值({FRONTIER_FAILURE_THRESHOLD}) → 启动简化恢复策略")
                    self.frontier_failure_count = 0
                    
                    if self._simple_retreat_recovery(pose):
                        return (0.0, 0.0)
                    else:
                        self._log_debug("阶段2：简化恢复策略失败 → 恢复模式-慢速旋转")
                        return (0.0, 0.5)
                else:
                    self._log_debug("阶段2：未达到恢复阈值 → 恢复模式-慢速旋转")
                    return (0.0, 0.5)
            
            # 兼容 {frontier, goal_band} 或 (x,y)
            if isinstance(sel, dict) and 'frontier' in sel and 'goal_band' in sel:
                goal = tuple(sel['frontier'])
                band = sel['goal_band']
                meta = sel.get('meta', {})
                
                if band is None:
                    self._log_debug("阶段2：Explorer返回的goal-band为None，拒绝该前沿点")
                    return (0.0, 0.5)
                
                self._log_debug(f"阶段2：选择前沿点 {goal}")
                self._goal_band_rect = band
                self.goal_start_time = time.time()
                self.frontier_failure_count = 0
                
                # 记录来向信息
                if goal is not None:
                    gpx, gpy = goal
                    vx, vy = (gpx - pose.x, gpy - pose.y)
                    n = (vx*vx + vy*vy) ** 0.5
                    self._last_dir_vec = None if n < 1e-6 else (vx/n, vy/n)
                    self._last_dispatched_goal = goal
                    if hasattr(self.explorer, "note_committed_goal"):
                        self.explorer.note_committed_goal((pose.x, pose.y), goal, band)
                
                self._plan_to_band(goal, band, meta)
                return (0.0, 0.0)
            else:
                self._log_debug("阶段2：Explorer返回格式不正确，拒绝该前沿点")
                return (0.0, 0.5)
        
        # 3) 正常路径跟踪与到达判定
        if self._primitive_mode and self._band_rect is not None:
            return self._update_motion(pose)

        if (not self._primitive_mode) and self.current_goal is not None and (not self.controller or not self.current_path):
            self._log_debug("阶段2：有全局目标但无路径/控制器 → 触发重规划当前目标")
            self._plan_to(self.current_goal)
            return (0.0, 0.0)

        # 先做到达带判定，避免多余控制
        if self.controller and self.current_path:
            if self._goal_band_rect is not None and (
                self._reached_band_by_edge_cross(pose, self._goal_band_rect) or
                self._in_rect(pose.x, pose.y, self._goal_band_rect)
            ):
                self._log_debug("阶段2：已进入到达带，清理目标并继续选点")
                if self._last_dispatched_goal is not None and hasattr(self.explorer, "note_arrival"):
                    self.explorer.note_arrival((pose.x, pose.y))
                self._last_dispatched_goal = None
                self.current_goal, self.current_path = None, []
                self.controller = None
                self._goal_band_rect = None
                return (0.0, 0.0)
            
            if self.controller.is_done():
                self._log_debug("阶段2：前沿点已到达 → 清理目标并继续探索")
                if self._last_dispatched_goal is not None and hasattr(self.explorer, "note_arrival"):
                    self.explorer.note_arrival((pose.x, pose.y))
                self._last_dispatched_goal = None
                self.current_goal, self.current_path = None, []
                self.controller = None
                self._goal_band_rect = None
                return (0.0, 0.0)

        # 路径偏差检查与控制
        is_dev, dev_d = self._check_path_deviation(pose, self.current_path)
        if is_dev:
            self._log_debug(f"⚠️ 路径偏差过大: {dev_d:.3f}m > {STRICT_PATH_TOLERANCE}m，重新规划")
            self._plan_to(self.current_goal)
            return (0.0, 0.0)

        return self._update_motion(pose)

    def _update_to_exit(self, pose: Pose2D, occ_grid) -> Tuple[float, float]:
        """阶段3：前往迷宫出口（到达判定优先；到则立刻切阶段4，不做掉头）"""
        self._log_debug(f"阶段3更新：前往出口 {self.exit_xy}")

        # ===== 0) 工具：带矩形扩张（用于 in-rect 容差判定） =====
        from core.config import GOAL_BAND_BOUNDARY_TOLERANCE_M
        def _expand_rect(rect, tol: float):
            xmin, xmax, ymin, ymax = rect
            return (xmin - tol, xmax + tol, ymin - tol, ymax + tol)

        # ===== 1) 到达判定 —— 必须放在最前，先于任何“原语早返回” =====
        band = self._goal_band_rect if getattr(self, "_goal_band_rect", None) else getattr(self, "_band_rect", None)
        reached = False
        if band is not None:
            # 两条路都算到达：①穿越边界线 ②位姿落在带内（+2cm 容差）
            try:
                cross_edge = self._reached_band_by_edge_cross(pose, band)
            except Exception:
                cross_edge = False
            in_band = self._in_rect(pose.x, pose.y, _expand_rect(band, GOAL_BAND_BOUNDARY_TOLERANCE_M))
            reached = bool(cross_edge or in_band)

        if reached:
            # —— 到达后：清理阶段3上下文 → 直接切换到阶段4（不在此处做掉头）——
            self._log_debug("阶段3完成：出口到达带已满足 → 直接切换阶段4(ReturnHome)")

            # 清理原语/控制/目标上下文
            if hasattr(self, "_exit_primitive_mode"):
                try:
                    self._exit_primitive_mode()
                except Exception:
                    # 若无完善接口，则手动复位关键标志
                    self._primitive_mode = False
                    self._band_rect = None
            else:
                self._primitive_mode = False
                self._band_rect = None

            self.current_goal = None
            self.current_path = []
            self.controller = None
            self._goal_band_rect = None

            # 切状态：阶段4负责掉头与最短路返家
            self.set_state(NavState.RETURN_HOME)
            return (0.0, 0.0)

        # ===== 2) 未到达 → 若原语已武装，则直接执行原语（此处不再短路到达判定，因为已在前面做过）=====
        if self._primitive_mode and self._band_rect:
            return self._update_motion(pose)

        # ===== 3) 非原语且缺控制器/路径 → 用“出口到达带”进行补规划 =====
        if (not self._primitive_mode) and (not self.controller or not self.current_path):
            if self.exit_xy is None:
                self._log_debug("阶段3：无出口目标，任务完成")
                self.set_state(NavState.FINISHED)
                return (0.0, 0.0)

            self._log_debug("阶段3：使用与EXPLORING相同的目标区域生成逻辑")
            band_rect = self._create_target_reachable_region(self.exit_xy, region_size_m=FINAL_REGION_SIZE)
            
            # 使用 _validate_band_on_cspace 验证区域可达性
            c_cached = getattr(self.explorer, "get_latest_cspace_bin", lambda: None)() if self.explorer else None
            occ_shape = self.slam.get_occ_grid().shape
            if c_cached is not None and c_cached.shape == occ_shape:
                cspace_bin = (c_cached > 0).astype('uint8')
            else:
                raw_cspace = self._get_safebuffer_cspace()
                cspace_bin = (raw_cspace != 0).astype('uint8')
            
            if self._validate_band_on_cspace(band_rect, cspace_bin):
                self._plan_to_band(self.exit_xy, band_rect)
                self._log_debug("阶段3：目标区域验证可达，开始规划路径")
            else:
                self._log_debug("阶段3：目标区域验证不可达，保持等待")
            return (0.0, 0.0)

        # ===== 4) 偏差重规划（优先到达带方案）=====
        deviated, d = self._check_path_deviation(pose, self.current_path)
        if deviated:
            from core.config import CORRIDOR_LANE_PITCH_M  # 仅用于推导合理 half_width（如需）
            self._log_debug(f"⚠️ 路径偏差过大: {d:.3f}m > {STRICT_PATH_TOLERANCE}m，使用到达带机制重新规划")
            ex, ey = self.exit_xy if self.exit_xy else (None, None)
            if ex is None:
                self._log_debug("阶段3：无出口坐标，无法重规划 → 停止")
                return (0.0, 0.0)

            # 使用与EXPLORING相同的目标区域生成逻辑进行重规划
            band_rect = self._create_target_reachable_region((ex, ey), region_size_m=FINAL_REGION_SIZE)
            
            # 使用 _validate_band_on_cspace 验证区域可达性
            c_cached = getattr(self.explorer, "get_latest_cspace_bin", lambda: None)() if self.explorer else None
            occ_shape = self.slam.get_occ_grid().shape
            if c_cached is not None and c_cached.shape == occ_shape:
                cspace_bin = (c_cached > 0).astype('uint8')
            else:
                raw_cspace = self._get_safebuffer_cspace()
                cspace_bin = (raw_cspace != 0).astype('uint8')
            
            if self._validate_band_on_cspace(band_rect, cspace_bin):
                self._plan_to_band((ex, ey), band_rect)
                self._log_debug("阶段3：重规划目标区域验证可达，开始规划路径")
            else:
                self._log_debug("阶段3：重规划时目标区域验证不可达，保持等待")
                return (0.0, 0.0)
            return (0.0, 0.0)

        # ===== 5) 常规控制 =====
        cmd = self._update_motion(pose)
        self._log_debug(f"阶段3：控制器命令 v={cmd[0]:.2f}, w={cmd[1]:.2f}")
        return cmd

    def _update_return_home(self, pose: Pose2D, occ_grid) -> Tuple[float, float]:
        """阶段4：返回起点"""
        self._log_debug(f"阶段4更新：返回起点 {self.entrance_xy}")

        # English: give priority to primitive executor if armed
        if self._primitive_mode and self._band_rect:
            return self._update_motion(pose)

        # 若控制器/路径缺失，补规划（仅在非原语模式下）
        if (not self._primitive_mode) and (not self.controller or not self.current_path):
            if self.entrance_xy is None:
                self._log_debug("阶段4：无起点位置，任务完成")
                self.set_state(NavState.FINISHED)
                return (0.0, 0.0)

            self._log_debug("阶段4：使用统一到达带机制规划到起点")
            ex, ey = self.entrance_xy
            band_rect = self._compute_border_goal_band(ex, ey)
            if band_rect is None:
                band_rect = self._compute_goal_band(pose, (ex, ey))
            if band_rect[0] != band_rect[1] or band_rect[2] != band_rect[3]:
                self._plan_to_band((ex, ey), band_rect)
            else:
                self._log_debug("阶段4：起点到达带计算失败，使用传统规划")
                self._plan_to((ex, ey))
            return (0.0, 0.0)

        # 到达判定：只使用带跨越检测，不使用旧控制器
        reached = False
        
        # Guard-1: ignore band-cross right after switching state
        if self._band_guard_ticks > 0:
            self._band_guard_ticks -= 1
            self._log_debug(f"TO_EXIT保护计数: {self._band_guard_ticks}")
        else:
            # Guard-2: only accept band built by this state
            if self._goal_band_rect is not None and self._goal_band_owner == 'TO_EXIT':
                reached = self._reached_band_by_edge_cross(pose, self._goal_band_rect)
                if reached:
                    self._log_debug("TO_EXIT: 成功到达目标区域")

        if reached:
            self._log_debug("阶段4完成：已返回入口，任务完成")
            self.set_state(NavState.FINISHED)
            return (0.0, 0.0)

        # 偏差重规划：也使用到达带机制
        deviated, d = self._check_path_deviation(pose, self.current_path)
        if deviated:
            self._log_debug(f"⚠️ 路径偏差过大: {d:.3f}m > {STRICT_PATH_TOLERANCE}m，使用到达带机制重新规划到起点")
            band_rect = self._compute_goal_band(pose, self.entrance_xy)
            if band_rect[0] != band_rect[1] or band_rect[2] != band_rect[3]:
                self._plan_to_band(self.entrance_xy, band_rect)
            else:
                self._log_debug("阶段4：重新规划时起点周围无可达区域")
                return (0.0, 0.0)
            return (0.0, 0.0)

        return self._update_motion(pose)

    # 统一出口/入口到达带逻辑：直接用 _compute_border_goal_band 或 _compute_goal_band
    # 所有相关调用已在 _update_to_exit 和 _update_return_home 逻辑中合并，无需自定义冗余函数。
        

