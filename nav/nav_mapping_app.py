# ================================
# file: nav/nav_mapping_app.py
# ================================
"""
Modular Navigation and Mapping Application
Refactored from test_nav_exploring.py to use proper module structure
"""

from __future__ import annotations
from typing import Optional, Tuple, List, Dict
import time
import math
import os
import threading
import signal
import sys
import atexit
from enum import Enum, auto
from dataclasses import dataclass
import numpy as np

# Core imports
from core.config import (
    ROBOT_START_X, ROBOT_START_Y, ROBOT_START_THETA,
    SLAM_RESOLUTION, CONTROL_HZ, WORLD_SIZE,
    MAX_MOVE_CHUNK_M, MAX_TURN_CHUNK_DEG, MAP_AFTER_CHUNK_S,
    DEFAULT_MAX_SEG_S, DEFAULT_MAPPING_DUR_S, INTERMEDIATE_MAPPING_S, COMMAND_TIMEOUT_S,
    MAIN_MAPPING_POINTS, POST_SEGMENT_MAPPING_POINTS, DEFAULT_MAPPING_POINTS,
    SINGLE_THREAD_BATCH_SIZE, SINGLE_THREAD_EMPTY_BATCH_LIMIT, 
    SINGLE_THREAD_PREFILTER_ENABLED, SINGLE_THREAD_IMPORTANT_PREFIXES
)
from core.odometry_processor import (
    OdometryState, parse_odom_line_or_block, 
    odom_to_world_pose, update_robot_position_from_odometry,
    START_THETA_RAD
)
from core.lidar_parser import parse_lidar_line

# Hardware interface
from appio.nav_bluetooth import NavBleInterface

# Mapping
from sparse_lidar_processor import SparseLidarProcessor

# Visualization
from gui.nav_visualizer import NavVisualizer

# Navigation modules
try:
    from planning.global_planner import AStarPlanner
    from explore.frontier_explorer import FrontierExplorer
    from nav.navigator import NavigatorFSM
    from nav.nav_adapters import (
        compute_frontier_and_band, plan_manhattan_path,
        decompose_to_primitives, primitives_to_ble_commands,
        compute_frontier_and_band_with_viz, split_long_moves, split_large_turns,
        reached_band
    )
    NAV_MODULES_AVAILABLE = True
except ImportError:
    AStarPlanner = None
    FrontierExplorer = None
    NavigatorFSM = None
    compute_frontier_and_band = None
    plan_manhattan_path = None
    decompose_to_primitives = None
    primitives_to_ble_commands = None
    compute_frontier_and_band_with_viz = None
    NAV_MODULES_AVAILABLE = False


class State(Enum):
    MAPPING = auto()
    NAVIGATING = auto()
    TURNING = auto()
    MOVING = auto()
    COMPLETE = auto()


# [ADD] mission states for end-to-end flow
class MissionState(Enum):
    EXPLORING = auto()      # frontier exploration
    TO_EXIT   = auto()      # go to exit band
    RETURN_HOME = auto()    # go back to start band
    COMPLETE  = auto()


@dataclass
class RobotContext:
    pose_world: Tuple[float, float, float]  # (x,y,theta[rad])
    odom: OdometryState
    mapper: SparseLidarProcessor
    ble: NavBleInterface
    viz: Optional[NavVisualizer]
    planner: Optional[AStarPlanner]
    explorer: Optional[FrontierExplorer]
    nfsm: Optional[NavigatorFSM]
    occ_grid: Optional[object] = None  # np.ndarray
    last_event_ts: float = 0.0


class NavMappingApp:
    """Top-level modular coordinator for navigation and mapping."""
    
    def __init__(self, port="COM4", baud=921600, size_m: float=2.8, map_res: float=SLAM_RESOLUTION):
        self.port, self.baud = port, baud
        self.size_m, self.map_res = size_m, map_res
        
        # Initialize hardware interface
        self.ble = NavBleInterface(port=port, baud=baud)
        self.ble.connect()
        print(f"[INFO] 串口连接成功: {port} @ {baud}")
        
        # Initialize mapper
        try:
            self.mapper = SparseLidarProcessor(
                map_size_pixels=None,
                map_size_meters=size_m,
                resolution=map_res
            )
        except TypeError:
            # Fallback to original constructor
            self.mapper = SparseLidarProcessor(size_m=size_m, resolution=map_res)
        
        # Initialize visualizer
        try:
            self.viz = NavVisualizer(size_m=size_m, res=map_res)
            print("[INFO] GUI初始化成功")
        except Exception as e:
            print(f"[ERROR] GUI初始化失败: {e}")
            self.viz = None
        
        # Initialize navigation modules
        self._init_navigation_modules()
        
        # Initialize state management
        self.odom = OdometryState()
        self.odom_buffer = []
        self.pose_world = [ROBOT_START_X, ROBOT_START_Y, START_THETA_RAD]
        self._odom_updated = False
        self.mapper.set_robot_pose(tuple(self.pose_world))
        
        # State machine
        self.state = State.MAPPING
        
        # High baudrate optimization
        from core.config import HIGH_BAUDRATE_MODE, HIGH_BAUDRATE_MAPPING_DURATION, HIGH_BAUDRATE_GUI_UPDATE_INTERVAL
        if HIGH_BAUDRATE_MODE and baud >= 921600:
            self.mapping_dur_s = HIGH_BAUDRATE_MAPPING_DURATION
            self.gui_update_interval = HIGH_BAUDRATE_GUI_UPDATE_INTERVAL
            print(f"[HIGH_BAUD] 建图时间优化: {self.mapping_dur_s}s, GUI更新间隔: {self.gui_update_interval}")
        else:
            self.mapping_dur_s = DEFAULT_MAPPING_DUR_S
            self.gui_update_interval = 25
        
        self.control_dt = 1.0 / float(CONTROL_HZ if CONTROL_HZ > 0 else 20)
        self._rebase_yaw_on_first_angle = False
        
        # LiDAR warm-up gating
        self.LIDAR_WARMUP_S = 1.4          # seconds; from your logs
        self._lidar_warmup_until = 0.0     # monotonic timestamp to stop dropping
        self._lidar_dropped_warmup = 0     # for diagnostics only
        
        # Direct garbage point filtering - specific startup garbage points
        self.garbage_patterns = [
            {"quality": 16, "angle_deg": 64.6, "distance_m": 7.76475, "tolerance": {"quality": 2, "angle": 1.0, "distance": 0.5}},
            {"quality": 25, "angle_deg": 92.8, "distance_m": 0.64325, "tolerance": {"quality": 2, "angle": 1.0, "distance": 0.1}},
        ]
        
        # Current goal tracking for post-segment checks
        self._current_goal_xy = None
        self._current_goal_band = None
        self._replanned_primitives = None  # 缓存重规划的原语序列
        self._replan_count = 0  # 重规划计数器，防止死循环
        # 重规划停滞检测
        self._last_replan_path_len = None
        self._stagnation_hits = 0
        # 当前目标的初始中心距离，用于进展判断
        self._goal_initial_dist = None
        
        # [ADD] record home pose/band and placeholders for exit
        self.home_xyth = (self.pose_world[0], self.pose_world[1], self.pose_world[2])  # start pose
        band_w = 0.08  # 8 cm tolerance band for arrival
        self.home_band = (self.home_xyth[0]-band_w, self.home_xyth[0]+band_w,
                          self.home_xyth[1]-band_w, self.home_xyth[1]+band_w)

        self.exit_band = None     # will be filled by detector or manual config
        self.mission_state = MissionState.EXPLORING

        # exploration completion thresholds
        self.coverage_threshold = 0.95   # map known ratio to stop exploring
        self.unknown_threshold  = 0.05   # unknown ratio allowed
        self._exit_path_streak  = 0      # consecutive reachable checks
        self._need_streak       = 3
        # 多段目标追踪
        self._cur_goal_has_turn = False      # 当前目标是否包含转向
        self._cur_goal_segments_total = 0    # 当前目标总段数
        self._segments_done = 0              # 已执行段数
        
        # Initialize data logging
        self.raw_data_log = None
        self.mapping_window_count = 0  # 跟踪建图窗口编号
        self._shutdown_requested = False  # 用于标记是否需要关闭
        self._force_save_called = False  # 防止重复强制保存
        
        # High baudrate memory monitoring
        from core.config import HIGH_BAUDRATE_MEMORY_MONITOR, HIGH_BAUDRATE_MAX_MEMORY_MB
        if HIGH_BAUDRATE_MEMORY_MONITOR and baud >= 921600:
            self.memory_monitor_enabled = True
            self.max_memory_mb = HIGH_BAUDRATE_MAX_MEMORY_MB
            self._last_memory_check = time.time()
            print(f"[HIGH_BAUD] 内存监控已启用，最大内存: {self.max_memory_mb}MB")
        else:
            self.memory_monitor_enabled = False
            
        self._init_data_logging()
        
        # Setup signal handlers for graceful shutdown
        self._setup_signal_handlers()
        
        # Create robot context for clean interface
        self.robot_context = RobotContext(
            pose_world=tuple(self.pose_world),
            odom=self.odom,
            mapper=self.mapper,
            ble=self.ble,
            viz=self.viz,
            planner=self.planner if hasattr(self, 'planner') else None,
            explorer=self.explorer if hasattr(self, 'explorer') else None,
            nfsm=self.nfsm if hasattr(self, 'nfsm') else None
        )

    # ---------- 目标/缓存管理与辅助判断 ----------
    def _clear_goal_and_replan_cache(self):
        """清理当前目标与重规划缓存，保证下一轮会重新选点"""
        self._current_goal_xy = None
        self._current_goal_band = None
        self._replanned_primitives = None
        self._replan_count = 0
        self._last_replan_path_len = None
        self._stagnation_hits = 0
        self._goal_initial_dist = None
        # 清理多段目标追踪
        self._cur_goal_has_turn = False
        self._cur_goal_segments_total = 0
        self._segments_done = 0

    @staticmethod
    def _band_center(band):
        xmin, xmax, ymin, ymax = band
        return (0.5 * (xmin + xmax), 0.5 * (ymin + ymax))

    @staticmethod
    def _dist(p, q):
        return math.hypot(p[0] - q[0], p[1] - q[1])
    
    @staticmethod
    def _inside_band_enhanced(xy, band_rect, margin=0.0):
        """Enhanced band checking with margin tolerance"""
        x, y = xy
        xmin, xmax, ymin, ymax = band_rect
        return (x >= xmin + margin and x <= xmax - margin and
                y >= ymin + margin and y <= ymax - margin)
    
    @staticmethod
    def _reached_band_by_edge_cross(pose_xy, rect):
        """
        Enhanced arrival test with three tiers (adapted from navigator.py):
        1) Inside the rectangle -> reached.
        2) Cross an INSET gate line (pulled inward) with relaxed orthogonal range.
        3) L∞-expanded rectangle (outward by GOAL_BAND_OUTER_MARGIN_M).
        """
        x, y = pose_xy
        xmin, xmax, ymin, ymax = rect
        
        # Tier-1: already inside
        if (xmin <= x <= xmax) and (ymin <= y <= ymax):
            return True
        
        # Tolerances for gate-crossing (adapted from navigator.py)
        from core.config import SLAM_RESOLUTION, SAFE_BUFFER_M
        eps_axis = 0.5 * SLAM_RESOLUTION
        eps_ortho = max(0.5 * SLAM_RESOLUTION, 0.5 * SAFE_BUFFER_M)
        inset_m = max(0.5 * SLAM_RESOLUTION, 0.5 * SAFE_BUFFER_M)
        
        xw = xmax - xmin
        yw = ymax - ymin
        
        # Tier-2: cross inward gate with relaxed orthogonal range - check BOTH gates
        if xw < yw:
            # vertical band -> gates on x; check BOTH inset lines
            gxL = xmin + inset_m
            gxR = xmax - inset_m
            in_y = (ymin - eps_ortho <= y <= ymax + eps_ortho)
            if in_y and ((x >= gxL - eps_axis) or (x <= gxR + eps_axis)):
                return True
        else:
            # horizontal band -> gates on y; check BOTH inset lines
            gyB = ymin + inset_m
            gyT = ymax - inset_m
            in_x = (xmin - eps_ortho <= x <= xmax + eps_ortho)
            if in_x and ((y >= gyB - eps_axis) or (y <= gyT + eps_axis)):
                return True
        
        # Tier-3: L∞ outward-expanded rectangle by configured margin
        GOAL_BAND_OUTER_MARGIN_M = 0.04  # 4cm margin
        delta = GOAL_BAND_OUTER_MARGIN_M
        if (xmin - delta <= x <= xmax + delta) and (ymin - delta <= y <= ymax + delta):
            return True
        
        # Tier-4: Euclidean near-rect fallback
        near_eps = max(4 * SLAM_RESOLUTION, 0.5 * SAFE_BUFFER_M)
        dx = 0.0 if xmin <= x <= xmax else (xmin - x if x < xmin else x - xmax)
        dy = 0.0 if ymin <= y <= ymax else (ymin - y if y < ymin else y - ymax)
        if (dx*dx + dy*dy) ** 0.5 <= near_eps:
            return True
        
        return False
    
    @staticmethod
    def _compute_border_goal_band(x0: float, y0: float,
                                  half_width_m: float = 0.30,
                                  depth_m: float = 0.30,
                                  log: bool = True) -> Optional[Tuple[float,float,float,float]]:
        """
        Compute entry/exit goal-band on maze border using configurable half-width and depth.
        Adapted from navigator.py logic.
        """
        from core.config import WORLD_SIZE, SLAM_RESOLUTION
        HALF_WIDTH_M = half_width_m
        DEPTH_M = depth_m
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
            print(f"[BORDER_BAND] 边界到达带: ({xmin:.2f}, {xmax:.2f}, {ymin:.2f}, {ymax:.2f})")
        return (xmin, xmax, ymin, ymax)
    
    def _compute_smart_goal_band(self, goal_xy: Tuple[float, float], pose_xy: Tuple[float, float]) -> Tuple[float,float,float,float]:
        """
        Smart goal band computation: use border band for edge goals, 
        otherwise use existing band logic.
        Adapted from navigator.py strategy.
        """
        gx, gy = goal_xy
        
        # First try border band computation
        border_band = self._compute_border_goal_band(gx, gy, half_width_m=0.30, depth_m=0.30, log=False)
        if border_band is not None:
            print(f"[SMART_BAND] 使用边界到达带: {border_band}")
            return border_band
        
        # Fallback to existing band logic (if available)
        # Create a thicker square band around the goal to avoid edge-hugging
        from core.config import SLAM_RESOLUTION, SAFE_BUFFER_M
        # Use SAFE_BUFFER_M as minimum band size to ensure clearance from obstacles
        band_size = max(0.15, SAFE_BUFFER_M + 0.05)  # At least 15cm, or SAFE_BUFFER + 5cm
        xmin, xmax = gx - band_size, gx + band_size
        ymin, ymax = gy - band_size, gy + band_size
        
        # Clamp to world bounds
        from core.config import WORLD_SIZE
        xmin = max(0.0, min(WORLD_SIZE, xmin))
        xmax = max(0.0, min(WORLD_SIZE, xmax))
        ymin = max(0.0, min(WORLD_SIZE, ymin))
        ymax = max(0.0, min(WORLD_SIZE, ymax))
        
        print(f"[SMART_BAND] 使用安全方形到达带: ({xmin:.2f}, {xmax:.2f}, {ymin:.2f}, {ymax:.2f}) (尺寸: {band_size:.2f}m)")
        return (xmin, xmax, ymin, ymax)

    def _init_navigation_modules(self):
        """Initialize navigation modules with error handling"""
        if not NAV_MODULES_AVAILABLE:
            print("[WARN] Navigation modules not available. Running in mapping-only mode.")
            self.planner = None
            self.explorer = None
            self.nfsm = None
        else:
            try:
                self.planner = AStarPlanner()
                self.explorer = FrontierExplorer(self.planner)
                self.nfsm = NavigatorFSM(
                    slam=None, planner=self.planner, 
                    explorer=self.explorer, robot=None, 
                    logger_func=None, log_file=None
                )
                print("[INFO] Navigation modules initialized successfully")
            except Exception as e:
                print(f"[WARN] Failed to initialize navigation modules: {e}")
                self.planner = None
                self.explorer = None
                self.nfsm = None

    def _init_data_logging(self):
        """Initialize raw data logging and terminal output logging"""
        try:
            self.log_timestamp = time.strftime("%Y%m%d_%H%M%S")
            # 创建 nav_exploring 目录结构
            self.base_logdir = os.path.join("logs", "nav_exploring")
            self.raw_data_dir = os.path.join(self.base_logdir, "raw_data")
            self.maps_dir = os.path.join(self.base_logdir, "maps")
            self.txt_output_dir = os.path.join(self.base_logdir, "txt_output")
            
            os.makedirs(self.raw_data_dir, exist_ok=True)
            os.makedirs(self.maps_dir, exist_ok=True)
            os.makedirs(self.txt_output_dir, exist_ok=True)
            
            # 创建原始数据日志文件
            log_file = os.path.join(self.raw_data_dir, f"raw_data_{self.log_timestamp}.txt")
            self.raw_data_log = open(log_file, 'w', encoding='utf-8')
            print(f"[INFO] 原始数据日志文件: {log_file}")
            
            # 创建建图统计日志文件
            stats_file = os.path.join(self.base_logdir, f"mapping_stats_{self.log_timestamp}.txt")
            self.stats_log = open(stats_file, 'w', encoding='utf-8')
            print(f"[INFO] 建图统计日志文件: {stats_file}")
            
            # 创建终端输出日志文件
            terminal_log_file = os.path.join(self.txt_output_dir, f"terminal_output_{self.log_timestamp}.txt")
            self.terminal_log = open(terminal_log_file, 'w', encoding='utf-8')
            print(f"[INFO] 终端输出日志文件: {terminal_log_file}")
            
            # 设置输出重定向
            self._setup_terminal_logging()
            
        except Exception as e:
            print(f"[WARN] 无法创建数据日志文件: {e}")
            self.raw_data_log = None
            self.stats_log = None
            self.terminal_log = None

    def _setup_terminal_logging(self):
        """Setup terminal output logging by redirecting stdout and stderr"""
        if self.terminal_log is None:
            return
            
        try:
            # 检查是否已经有自定义输出重定向在运行（来自test_nav_modular.py）
            current_stdout = sys.stdout
            current_stderr = sys.stderr
            
            # 如果stdout已经有write方法且不是原始输出，说明已经有重定向
            if (hasattr(current_stdout, 'terminal') and hasattr(current_stdout, 'file')):
                print(f"[TERMINAL_LOG] 检测到已有日志重定向，将同时写入NavMappingApp日志文件")
                
                # 在这种情况下，我们需要扩展现有的重定向来同时写入我们的文件
                class ExtendedTeeOutput:
                    def __init__(self, existing_output, additional_file):
                        self.existing = existing_output
                        self.additional_file = additional_file
                        
                    def write(self, message):
                        # 先调用现有的输出（终端+原有日志文件）
                        self.existing.write(message)
                        # 同时写入我们的额外日志文件
                        if message.strip():
                            timestamp = time.strftime("%H:%M:%S")
                            self.additional_file.write(f"[{timestamp}] {message}")
                            self.additional_file.flush()
                            
                    def flush(self):
                        if hasattr(self.existing, 'flush'):
                            self.existing.flush()
                        self.additional_file.flush()
                        
                    def __getattr__(self, name):
                        return getattr(self.existing, name)
                
                sys.stdout = ExtendedTeeOutput(current_stdout, self.terminal_log)
                sys.stderr = ExtendedTeeOutput(current_stderr, self.terminal_log)
                
            else:
                # 没有现有重定向，设置新的重定向
                # 保存原始stdout和stderr
                self._original_stdout = sys.stdout
                self._original_stderr = sys.stderr
                
                # 创建自定义的输出类，同时写入终端和文件
                class TeeOutput:
                    def __init__(self, terminal_output, file_output):
                        self.terminal = terminal_output
                        self.file = file_output
                        
                    def write(self, message):
                        # 写入终端
                        self.terminal.write(message)
                        self.terminal.flush()
                        # 写入文件（带时间戳）
                        if message.strip():  # 只记录非空消息
                            timestamp = time.strftime("%H:%M:%S")
                            self.file.write(f"[{timestamp}] {message}")
                            self.file.flush()
                            
                    def flush(self):
                        self.terminal.flush()
                        self.file.flush()
                        
                    def __getattr__(self, name):
                        return getattr(self.terminal, name)
                
                # 重定向stdout和stderr
                sys.stdout = TeeOutput(self._original_stdout, self.terminal_log)
                sys.stderr = TeeOutput(self._original_stderr, self.terminal_log)
            
            print(f"[TERMINAL_LOG] NavMappingApp 终端输出日志记录已启用")
            
        except Exception as e:
            print(f"[WARN] 设置终端日志失败: {e}")
            self.terminal_log = None

    def _restore_terminal_output(self):
        """Restore original stdout and stderr"""
        try:
            current_stdout = sys.stdout
            current_stderr = sys.stderr
            
            # 如果当前输出是我们设置的ExtendedTeeOutput，需要恢复到底层输出
            if (hasattr(current_stdout, 'existing')):
                sys.stdout = current_stdout.existing
            elif hasattr(self, '_original_stdout'):
                sys.stdout = self._original_stdout
                
            if (hasattr(current_stderr, 'existing')):
                sys.stderr = current_stderr.existing
            elif hasattr(self, '_original_stderr'):
                sys.stderr = self._original_stderr
                
        except Exception as e:
            print(f"[WARN] 恢复终端输出失败: {e}")

    def _setup_signal_handlers(self):
        """Setup signal handlers for graceful shutdown"""
        def signal_handler(signum, frame):
            print(f"\n[INFO] 收到信号 {signum}，开始优雅关闭...")
            self._shutdown_requested = True
            self._force_save_all_data()
        
        # Register signal handlers
        signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
        signal.signal(signal.SIGTERM, signal_handler)  # Termination signal
        
        # Register atexit handler for final cleanup
        atexit.register(self._force_save_all_data)

    def _force_save_all_data(self):
        """Force save all data regardless of current state"""
        # Prevent duplicate saves
        if self._force_save_called:
            print("[SAVE] 强制保存已执行过，跳过重复保存")
            return
            
        self._force_save_called = True
        
        try:
            print("[SAVE] 强制保存所有数据...")
            
            # Save current mapping results
            try:
                self._save_mapping_results_incremental()
            except Exception as e:
                print(f"[WARN] 增量保存失败: {e}")
                
            try:
                self._save_final_results()
            except Exception as e:
                print(f"[WARN] 最终保存失败: {e}")
            
            # 恢复终端输出（避免在关闭日志文件时出现循环）
            self._restore_terminal_output()
            
            # Close log files properly
            if hasattr(self, 'raw_data_log') and self.raw_data_log:
                try:
                    self.raw_data_log.flush()
                    self.raw_data_log.close()
                    self.raw_data_log = None  # Prevent further access
                    print("[SAVE] 原始数据日志已强制保存并关闭")
                except Exception as e:
                    print(f"[WARN] 关闭原始数据日志失败: {e}")
                    
            if hasattr(self, 'stats_log') and self.stats_log:
                try:
                    self.stats_log.flush()
                    self.stats_log.close()
                    self.stats_log = None  # Prevent further access
                    print("[SAVE] 建图统计日志已强制保存并关闭")
                except Exception as e:
                    print(f"[WARN] 关闭统计日志失败: {e}")
            
            # Close terminal log file
            if hasattr(self, 'terminal_log') and self.terminal_log:
                try:
                    self.terminal_log.flush()
                    self.terminal_log.close()
                    self.terminal_log = None  # Prevent further access
                    print("[SAVE] 终端输出日志已强制保存并关闭")
                except Exception as e:
                    print(f"[WARN] 关闭终端输出日志失败: {e}")
            
            print("[SAVE] 所有数据保存完成")
            
        except Exception as e:
            print(f"[ERROR] 强制保存数据时出错: {e}")

    def _pump_serial_once(self) -> Tuple[List[Dict], bool, bool]:
        """Process serial data and return (lidar_points, arrival, turning_finished)"""
        pts = []
        arrived = False
        turn_ok = False
        
        # Get all available lines
        lines = self.ble.drain_all_lines()
        if not lines:
            return pts, arrived, turn_ok
        
        if len(lines) > 1:
            print(f"[QUEUE] 一次性处理 {len(lines)} 行数据")
        
        for ln in lines:
            line_clean = ln.strip()
            
            # Check for completion signals
            completion_signals = [
                "Arrival at target.", "Arrival at target", "arrival",
                "completed", "finished", "done", "turning finished"
            ]
            if any(signal.lower() in line_clean.lower() for signal in completion_signals):
                if "turning finished" in line_clean.lower():
                    print("[INFO] Turning finished")
                    turn_ok = True
                else:
                    arrived = True
            
            # Process odometry data
            odom_updated = parse_odom_line_or_block(ln, self.odom_buffer, self.odom)
            if odom_updated:
                self._odom_updated = True
            
            # Process lidar data
            p = parse_lidar_line(ln)
            if p and p.get("distance_m", 0) > 0.0:
                # Ignore startup points while warm-up window is active
                if time.monotonic() < getattr(self, "_lidar_warmup_until", 0.0):
                    continue
                pts.append(p)
        
        return pts, arrived, turn_ok

    def _is_garbage_lidar_point(self, point: Dict) -> bool:
        """直接识别雷达启动垃圾点
        
        Args:
            point: 雷达点字典，包含quality, angle_deg, distance_m等字段
            
        Returns:
            True if 是垃圾点，False otherwise
        """
        quality = point.get('quality', 0)
        angle_deg = point.get('angle_deg', 0)
        distance_m = point.get('distance_m', 0)
        
        # 检查是否匹配已知的垃圾点模式
        for pattern in self.garbage_patterns:
            if (abs(quality - pattern['quality']) <= pattern['tolerance']['quality'] and
                abs(angle_deg - pattern['angle_deg']) <= pattern['tolerance']['angle'] and
                abs(distance_m - pattern['distance_m']) <= pattern['tolerance']['distance']):
                return True
        
        return False

    def _check_memory_usage(self) -> bool:
        """检查内存使用情况，返回True表示内存正常，False表示需要清理"""
        if not self.memory_monitor_enabled:
            return True
            
        try:
            import psutil
            process = psutil.Process()
            memory_mb = process.memory_info().rss / 1024 / 1024
            
            if memory_mb > self.max_memory_mb:
                print(f"[MEMORY_WARN] 内存使用过高: {memory_mb:.1f}MB > {self.max_memory_mb}MB")
                return False
            elif memory_mb > self.max_memory_mb * 0.8:  # 80%阈值
                print(f"[MEMORY_INFO] 内存使用: {memory_mb:.1f}MB (80%阈值)")
                
            return True
        except ImportError:
            print(f"[WARN] psutil未安装，无法监控内存使用")
            return True
        except Exception as e:
            print(f"[WARN] 内存检查失败: {e}")
            return True

    def _force_memory_cleanup(self):
        """强制内存清理"""
        try:
            import gc
            gc.collect()  # 强制垃圾回收
            print(f"[MEMORY_CLEANUP] 执行内存清理")
        except Exception as e:
            print(f"[WARN] 内存清理失败: {e}")

    def do_mapping_window_by_points(self, target_points: int = DEFAULT_MAPPING_POINTS):
        """基于点数量的建图窗口 - 单线程批量处理优化"""
        # Increment mapping window counter
        self.mapping_window_count += 1
        print(f"[MAPPING] 开始第 {self.mapping_window_count} 个建图窗口 (目标: {target_points}个点)")
        
        # 单线程模式：停止后台线程，使用直接串口读取
        self.ble._running = False
        th = self.ble._rx_thread
        if th and th.is_alive():
            th.join(timeout=1.0)
            print(f"[MAPPING] 后台线程已停止，启用单线程批量处理模式")
        
        # 清空队列（如果还有残留数据）
        try:
            while True:
                self.ble.lines.get_nowait()
        except Exception:
            pass
        print(f"[MAPPING] 队列已清空，准备开始单线程批量数据采集")
        
        # Reset yaw rebase flag
        self._rebase_yaw_on_first_angle = True
        
        # Sync pose to mapper
        self.mapper.set_robot_pose(tuple(self.pose_world))
        print(f"[MAPPING] 初始位姿同步: ({self.pose_world[0]:.3f}, {self.pose_world[1]:.3f}, {math.degrees(self.pose_world[2]):.1f}°)")
        
        # Start lidar
        self.ble.send("S")
        # mark warm-up window using monotonic clock; do not rely on sleep only
        self._lidar_warmup_until = time.monotonic() + self.LIDAR_WARMUP_S
        self._lidar_dropped_warmup = 0
        time.sleep(0.05)  # small pause only to ensure command is accepted
        print(f"[MAPPING] 开始建图... (warm-up {self.LIDAR_WARMUP_S:.1f}s)")
        
        start_time = time.time()
        processed_points = 0
        raw_count = 0
        odom_count = 0
        lidar_line_count = 0
        gui_every = self.gui_update_interval  # 使用配置的GUI更新间隔
        
        # 单线程批量处理循环 - 优化版本
        batch_size = SINGLE_THREAD_BATCH_SIZE  # 使用配置的批量处理大小
        empty_batch_count = 0  # 空批次计数，用于检测数据结束
        
        while processed_points < target_points and not self._shutdown_requested:
            # 批量读取数据 - 根据配置决定是否预过滤
            if SINGLE_THREAD_PREFILTER_ENABLED:
                batch_lines = self.ble.read_batch_lines_filtered(
                    max_lines=batch_size, 
                    important_prefixes=SINGLE_THREAD_IMPORTANT_PREFIXES
                )
            else:
                batch_lines = self.ble.read_batch_lines(max_lines=batch_size)
            
            if not batch_lines:
                empty_batch_count += 1
                if empty_batch_count > SINGLE_THREAD_EMPTY_BATCH_LIMIT:  # 使用配置的空批次限制
                    print(f"[MAPPING] 连续{SINGLE_THREAD_EMPTY_BATCH_LIMIT}个空批次，可能数据结束")
                    break
                time.sleep(0.0001)  # 极短等待时间以最大化性能
                # 移除调试打印以提升性能
                continue
            
            empty_batch_count = 0  # 重置空批次计数
            
            # 批量处理数据
            for line in batch_lines:
                # Check for shutdown request during mapping
                if self._shutdown_requested:
                    print("[MAPPING] 收到关闭请求，终止建图窗口")
                    break
                    
                raw_count += 1
                
                # Save raw data to file if logging is enabled
                if self.raw_data_log:
                    try:
                        timestamp = time.time() - start_time
                        self.raw_data_log.write(f"{timestamp:.3f}\t{line}\n")
                        self.raw_data_log.flush()
                    except Exception as e:
                        print(f"[WARN] 写入原始数据失败: {e}")
                
                if raw_count <= 50:
                    print(f"  RAW: {line}")
                elif raw_count <= 200 and raw_count % 50 == 0:
                    print(f"  RAW[{raw_count}]: {line}")
                
                if line.strip().startswith("LIDAR"):
                    lidar_line_count += 1
            
                # Handle yaw rebase for first angle message
                if self._rebase_yaw_on_first_angle and line.startswith("Angle:"):
                    try:
                        angle_abs = float(line.split(":", 1)[1])
                        self.odom.cumulative_odom_angle_deg = angle_abs
                        self.odom.delta = (0.0, 0.0, 0.0)
                        
                        if self.odom.has_abs:
                            xw, yw, thw = odom_to_world_pose(
                                self.odom.abs_x, self.odom.abs_y, angle_abs
                            )
                            self.pose_world = [xw, yw, thw]
                        else:
                            self.pose_world[2] = START_THETA_RAD - math.radians(angle_abs)
                        
                        self.mapper.set_robot_pose(tuple(self.pose_world))
                        self._rebase_yaw_on_first_angle = False
                        print(f"[YAW_REBASE] 第一条Angle对齐: {angle_abs:.1f}° → 世界角度: {math.degrees(self.pose_world[2]):.1f}°")
                        continue
                    except Exception as e:
                        print(f"[YAW_REBASE] 角度对齐失败: {e}")
                
                # Process odometry updates
                odom_updated_this_cycle = False
                if parse_odom_line_or_block(line, self.odom_buffer, self.odom):
                    odom_count += 1
                    old_pose = self.pose_world.copy()
                    self.pose_world = update_robot_position_from_odometry(self.odom, self.pose_world)
                    self._odom_updated = True
                    odom_updated_this_cycle = True
                    self.mapper.set_robot_pose(tuple(self.pose_world))
                    
                    if odom_count <= 10:
                        print(f"  ✅ 里程计数据 #{odom_count}: dx={self.odom.delta[0]:.4f}, dy={self.odom.delta[1]:.4f}")
                
                # Process lidar data
                p = parse_lidar_line(line)
                if p and p.get("distance_m", 0) > 0.0:
                    # --- direct garbage point filtering ---
                    if self._is_garbage_lidar_point(p):
                        self._lidar_dropped_warmup += 1
                        if self._lidar_dropped_warmup <= 5:
                            print(f"  🗑️ drop garbage point #{self._lidar_dropped_warmup}: Q{p.get('quality', 0)} A{p.get('angle_deg', 0):.1f}° D{p.get('distance_m', 0):.1f}m")
                        continue
                    # --- end direct filtering ---
                    
                    # --- warm-up drop: ignore all lidar points during the warm-up window ---
                    now_mono = time.monotonic()
                    if now_mono < getattr(self, "_lidar_warmup_until", 0.0):
                        # Drop spurious startup points unconditionally
                        self._lidar_dropped_warmup += 1
                        # Optional tiny log for the first few drops
                        if self._lidar_dropped_warmup <= 3:
                            print(f"  ↩ drop startup lidar #{self._lidar_dropped_warmup}")
                        continue
                    # --- end warm-up drop ---
                    
                    processed_points += 1
                    if processed_points <= 20:
                        print(f"  ✅ 雷达数据 #{processed_points}: angle={p['angle_deg']:.1f}°, dist={p['distance_m']:.3f}m, qual={p.get('quality', 50)}")
                    
                    success = self.mapper.add_single_sparse_point_with_pose(
                        p['angle_deg'], p['distance_m'], p.get('quality', 50),
                        robot_pose=tuple(self.pose_world)
                    )
                    
                    # 移除调试输出以提升性能
                    # if processed_points <= 10:
                    #     print(f"  📊 建图结果: 成功={success is not None}")
                    
                    # 实时GUI更新：每处理若干雷达点就更新一次可视化 - 优化版本
                    if success is not None and (self.viz is not None) and (processed_points % gui_every == 0):
                        try:
                            # 获取当前地图和位姿 - 移除内存检查以提升性能
                            current_occ_grid = self.mapper.get_occupancy_grid()
                            current_pose = self.mapper.robot_pose
                            self.viz.update_mapping_realtime(current_occ_grid, current_pose)
                            # 移除调试打印以提升性能
                        except Exception as e:
                            print(f"[GUI_ERROR] 实时GUI更新失败: {e}")
                    
                    # 实时显示进度 - 减少频率以提升性能
                    if processed_points % 200 == 0:
                        print(f"  📊 已处理 {processed_points}/{target_points} 个点")
                
                # Ensure pose sync even without odometry update
                if not odom_updated_this_cycle:
                    self.mapper.set_robot_pose(tuple(self.pose_world))
                
                # 定期内存检查（每1000个原始行检查一次）
                if raw_count % 1000 == 0 and self.memory_monitor_enabled:
                    if not self._check_memory_usage():
                        print(f"[MEMORY_WARN] 定期检查发现内存使用过高，执行清理")
                        self._force_memory_cleanup()
        
        # Stop lidar and cleanup
        self.ble.send("T")
        time.sleep(0.8)
        flush_deadline = time.time() + 0.5
        while time.time() < flush_deadline:
            ln = self.ble.readline()
            if not ln:
                break
        
        # 虚拟自由扇区补偿
        try:
            self.mapper.add_virtual_free_cone(
                pose=tuple(self.pose_world),
                heading_rad=self.pose_world[2],
                d_free=0.35,
                fov_deg=60,
                step_deg=2
            )
        except Exception as e:
            print(f"[MAPPING] 虚拟自由扇区补偿失败: {e}")
        
        # 统计信息
        mapping_time = time.time() - start_time
        parse_rate = (processed_points / lidar_line_count * 100) if lidar_line_count > 0 else 0
        
        print(f"[MAPPING] 点数量控制建图完成:")
        print(f"  - 目标点数: {target_points}")
        print(f"  - 实际处理: {processed_points}")
        print(f"  - 建图用时: {mapping_time:.2f}s")
        print(f"  - 处理速度: {processed_points/mapping_time:.1f} 点/秒")
        print(f"  - 原始数据: {raw_count} 行")
        print(f"  - LIDAR行数: {lidar_line_count}")
        print(f"  - 里程计记录: {odom_count}")
        print(f"  - 解析成功率: {parse_rate:.1f}% ({processed_points}/{lidar_line_count})")
        print(f"  - 丢弃垃圾点: {self._lidar_dropped_warmup} 个")
        
        # 记录统计信息到文件
        if self.stats_log:
            try:
                stats_entry = f"[{time.strftime('%H:%M:%S')}] 点数量控制建图统计:\n"
                stats_entry += f"  目标点数: {target_points}\n"
                stats_entry += f"  实际处理: {processed_points}\n"
                stats_entry += f"  建图用时: {mapping_time:.2f}s\n"
                stats_entry += f"  处理速度: {processed_points/mapping_time:.1f} 点/秒\n"
                stats_entry += f"  原始数据: {raw_count} 行\n"
                stats_entry += f"  LIDAR行数: {lidar_line_count}\n"
                stats_entry += f"  里程计记录: {odom_count}\n"
                stats_entry += f"  解析成功率: {parse_rate:.1f}% ({processed_points}/{lidar_line_count})\n"
                stats_entry += f"  丢弃垃圾点: {self._lidar_dropped_warmup} 个\n"
                stats_entry += f"  机器人位姿: ({self.pose_world[0]:.3f}, {self.pose_world[1]:.3f}, {math.degrees(self.pose_world[2]):.1f}°)\n"
                stats_entry += f"{'='*50}\n"
                
                self.stats_log.write(stats_entry)
                self.stats_log.flush()
            except Exception as e:
                print(f"[WARN] 写入统计信息失败: {e}")
        
        # Final GUI update
        try:
            occ_export = getattr(self.mapper, 'export_occupancy_grid', None)
            if callable(occ_export):
                self.occ_grid = self.mapper.export_occupancy_grid()
            else:
                self.occ_grid = self.mapper.get_occupancy_grid()
        except AttributeError:
            self.occ_grid = None
            
        if self.occ_grid is not None and self.viz is not None:
            try:
                self.viz.update(self.occ_grid, tuple(self.pose_world))
            except Exception as e:
                print(f"[GUI_ERROR] 最终GUI更新失败: {e}")
        
        # Save mapping results after each window (incremental saves)
        self._save_mapping_results_incremental()
        
        # Restart background thread
        self.ble._running = True
        self.ble._rx_thread = threading.Thread(target=self.ble._reader, daemon=True)
        self.ble._rx_thread.start()
        
        return processed_points, mapping_time

    def do_mapping_window(self, window_s: float):
        """Perform mapping window with single-threaded data processing"""
        # Increment mapping window counter
        self.mapping_window_count += 1
        print(f"[MAPPING] 开始第 {self.mapping_window_count} 个建图窗口 ({window_s}s)")
        
        # Stop background thread for clean data collection
        self.ble._running = False
        th = self.ble._rx_thread
        if th and th.is_alive():
            th.join(timeout=1.0)
            print(f"[MAPPING] 后台线程已停止，确保单线程读取")
        
        # Clear queue
        try:
            while True:
                self.ble.lines.get_nowait()
        except Exception:
            pass
        print(f"[MAPPING] 队列已清空，准备开始纯净的建图数据采集")
        
        # Reset yaw rebase flag
        self._rebase_yaw_on_first_angle = True
        
        # Sync pose to mapper
        self.mapper.set_robot_pose(tuple(self.pose_world))
        print(f"[MAPPING] 初始位姿同步: ({self.pose_world[0]:.3f}, {self.pose_world[1]:.3f}, {math.degrees(self.pose_world[2]):.1f}°)")
        
        # Start lidar
        self.ble.send("S")
        # mark warm-up window using monotonic clock; do not rely on sleep only
        self._lidar_warmup_until = time.monotonic() + self.LIDAR_WARMUP_S
        self._lidar_dropped_warmup = 0
        time.sleep(0.05)  # small pause only to ensure command is accepted
        print(f"[MAPPING] 开始建图... (warm-up {self.LIDAR_WARMUP_S:.1f}s)")
        
        t0 = time.time()
        lidar_count = 0
        odom_count = 0
        raw_count = 0
        lidar_line_count = 0
        gui_every = self.gui_update_interval  # 使用配置的GUI更新间隔
        
        # Main mapping loop
        while time.time() - t0 < window_s and not self._shutdown_requested:
            line = self.ble.readline()
            if not line:
                continue
            
            # Check for shutdown request during mapping
            if self._shutdown_requested:
                print("[MAPPING] 收到关闭请求，终止建图窗口")
                break
                
            raw_count += 1
            
            # Save raw data to file if logging is enabled
            if self.raw_data_log:
                try:
                    timestamp = time.time() - t0
                    self.raw_data_log.write(f"{timestamp:.3f}\t{line}\n")
                    self.raw_data_log.flush()
                except Exception as e:
                    print(f"[WARN] 写入原始数据失败: {e}")
            
            if raw_count <= 50:
                print(f"  RAW: {line}")
            elif raw_count <= 200 and raw_count % 50 == 0:
                print(f"  RAW[{raw_count}]: {line}")
            
            if line.strip().startswith("LIDAR"):
                lidar_line_count += 1
            
            # Handle yaw rebase for first angle message
            if self._rebase_yaw_on_first_angle and line.startswith("Angle:"):
                try:
                    angle_abs = float(line.split(":", 1)[1])
                    self.odom.cumulative_odom_angle_deg = angle_abs
                    self.odom.delta = (0.0, 0.0, 0.0)
                    
                    if self.odom.has_abs:
                        xw, yw, thw = odom_to_world_pose(
                            self.odom.abs_x, self.odom.abs_y, angle_abs
                        )
                        self.pose_world = [xw, yw, thw]
                    else:
                        self.pose_world[2] = START_THETA_RAD - math.radians(angle_abs)
                    
                    self.mapper.set_robot_pose(tuple(self.pose_world))
                    self._rebase_yaw_on_first_angle = False
                    print(f"[YAW_REBASE] 第一条Angle对齐: {angle_abs:.1f}° → 世界角度: {math.degrees(self.pose_world[2]):.1f}°")
                    continue
                except Exception as e:
                    print(f"[YAW_REBASE] 角度对齐失败: {e}")
            
            # Process odometry updates
            odom_updated_this_cycle = False
            if parse_odom_line_or_block(line, self.odom_buffer, self.odom):
                odom_count += 1
                old_pose = self.pose_world.copy()
                self.pose_world = update_robot_position_from_odometry(self.odom, self.pose_world)
                self._odom_updated = True
                odom_updated_this_cycle = True
                self.mapper.set_robot_pose(tuple(self.pose_world))
                
                if odom_count <= 10:
                    print(f"  ✅ 里程计数据 #{odom_count}: dx={self.odom.delta[0]:.4f}, dy={self.odom.delta[1]:.4f}")
            
            # Process lidar data
            p = parse_lidar_line(line)
            if p and p.get("distance_m", 0) > 0.0:
                # --- direct garbage point filtering ---
                if self._is_garbage_lidar_point(p):
                    self._lidar_dropped_warmup += 1
                    if self._lidar_dropped_warmup <= 5:
                        print(f"  🗑️ drop garbage point #{self._lidar_dropped_warmup}: Q{p.get('quality', 0)} A{p.get('angle_deg', 0):.1f}° D{p.get('distance_m', 0):.1f}m")
                    continue
                # --- end direct filtering ---
                
                # --- warm-up drop: ignore all lidar points during the warm-up window ---
                now_mono = time.monotonic()
                if now_mono < getattr(self, "_lidar_warmup_until", 0.0):
                    # Drop spurious startup points unconditionally
                    self._lidar_dropped_warmup += 1
                    # Optional tiny log for the first few drops
                    if self._lidar_dropped_warmup <= 3:
                        print(f"  ↩ drop startup lidar #{self._lidar_dropped_warmup}")
                    continue
                # --- end warm-up drop ---
                
                lidar_count += 1
                if lidar_count <= 20:
                    print(f"  ✅ 雷达数据 #{lidar_count}: angle={p['angle_deg']:.1f}°, dist={p['distance_m']:.3f}m, qual={p.get('quality', 50)}")
                
                success = self.mapper.add_single_sparse_point_with_pose(
                    p['angle_deg'], p['distance_m'], p.get('quality', 50),
                    robot_pose=tuple(self.pose_world)
                )
                
                if lidar_count <= 10:
                    print(f"  📊 建图结果: 成功={success is not None}")
                
                # 实时GUI更新：每处理若干雷达点就更新一次可视化
                if success is not None and (self.viz is not None) and (lidar_count % gui_every == 0):
                    try:
                        # 检查内存使用情况
                        if not self._check_memory_usage():
                            print(f"[MEMORY_WARN] 内存使用过高，跳过GUI更新")
                            self._force_memory_cleanup()
                            continue
                            
                        # 获取当前地图和位姿
                        current_occ_grid = self.mapper.get_occupancy_grid()
                        current_pose = self.mapper.robot_pose
                        self.viz.update_mapping_realtime(current_occ_grid, current_pose)
                        if lidar_count <= 100:  # 只在前100个点打印调试信息
                            print(f"  🖥️  实时GUI更新 #{lidar_count}")
                    except Exception as e:
                        print(f"[GUI_ERROR] 实时GUI更新失败: {e}")
                        # GUI更新失败时也进行内存清理
                        self._force_memory_cleanup()
            
            # Ensure pose sync even without odometry update
            if not odom_updated_this_cycle:
                self.mapper.set_robot_pose(tuple(self.pose_world))
            
            # 定期内存检查（每1000个原始行检查一次）
            if raw_count % 1000 == 0 and self.memory_monitor_enabled:
                if not self._check_memory_usage():
                    print(f"[MEMORY_WARN] 定期检查发现内存使用过高，执行清理")
                    self._force_memory_cleanup()
        
        # Statistics and logging
        mapping_time = time.time() - t0
        parse_rate = (lidar_count / lidar_line_count * 100) if lidar_line_count > 0 else 0
        
        print(f"[MAPPING] 建图窗口结束统计:")
        print(f"  - 原始行数: {raw_count}")
        print(f"  - LIDAR行数: {lidar_line_count}")
        print(f"  - 解析成功雷达点数: {lidar_count}")
        print(f"  - 里程计记录: {odom_count}")
        print(f"  - 建图器总点数: {self.mapper.total_points}")
        print(f"  - 解析成功率: {parse_rate:.1f}% ({lidar_count}/{lidar_line_count})")
        print(f"  - 建图耗时: {mapping_time:.1f}s")
        
        # 记录统计信息到文件
        if self.stats_log:
            try:
                stats_entry = f"[{time.strftime('%H:%M:%S')}] 建图窗口统计:\n"
                stats_entry += f"  原始行数: {raw_count}\n"
                stats_entry += f"  LIDAR行数: {lidar_line_count}\n"
                stats_entry += f"  解析成功雷达点数: {lidar_count}\n"
                stats_entry += f"  里程计记录: {odom_count}\n"
                stats_entry += f"  建图器总点数: {self.mapper.total_points}\n"
                stats_entry += f"  解析成功率: {parse_rate:.1f}% ({lidar_count}/{lidar_line_count})\n"
                stats_entry += f"  建图耗时: {mapping_time:.1f}s\n"
                stats_entry += f"  机器人位姿: ({self.pose_world[0]:.3f}, {self.pose_world[1]:.3f}, {math.degrees(self.pose_world[2]):.1f}°)\n"
                stats_entry += f"{'='*50}\n"
                
                self.stats_log.write(stats_entry)
                self.stats_log.flush()
            except Exception as e:
                print(f"[WARN] 写入统计信息失败: {e}")
        
        # Stop lidar and cleanup
        self.ble.send("T")
        time.sleep(0.8)
        flush_deadline = time.time() + 0.5
        while time.time() < flush_deadline:
            ln = self.ble.readline()
            if not ln:
                break
        
        # ========== 修改点2：建图窗口结束后的前向无回波扇区补偿 ==========
        # 目的：对330°~30°无雷达点的扇区进行自由空间补偿，解决稀疏数据导致的连通性问题
        try:
            # 以当前机器人朝向为中心，给±30°方向添加虚拟自由射线
            # 距离限制在0.35m内，确保安全且不干扰后续真实点
            self.mapper.add_virtual_free_cone(
                pose=tuple(self.pose_world),
                heading_rad=self.pose_world[2],
                d_free=0.35,      # 保守距离，小于走廊半宽
                fov_deg=60,       # ±30°视野范围
                step_deg=2        # 2°精度
            )
        except Exception as e:
            print(f"[MAPPING] 虚拟自由扇区补偿失败: {e}")
        
        # Final GUI update
        try:
            occ_export = getattr(self.mapper, 'export_occupancy_grid', None)
            if callable(occ_export):
                self.occ_grid = self.mapper.export_occupancy_grid()
            else:
                self.occ_grid = self.mapper.get_occupancy_grid()
        except AttributeError:
            self.occ_grid = None
            
        if self.occ_grid is not None and self.viz is not None:
            try:
                self.viz.update(self.occ_grid, tuple(self.pose_world))
            except Exception as e:
                print(f"[GUI_ERROR] 最终GUI更新失败: {e}")
        
        # Save mapping results after each window (incremental saves)
        self._save_mapping_results_incremental()
        
        # 高波特率优化：强制处理所有待处理的点
        if hasattr(self.mapper, 'flush_pending_points'):
            try:
                self.mapper.flush_pending_points()
                print(f"[HIGH_BAUD] 批量处理完成，处理了所有待处理的雷达点")
            except Exception as e:
                print(f"[WARN] 批量处理失败: {e}")
        
        # Restart background thread
        self.ble._running = True
        self.ble._rx_thread = threading.Thread(target=self.ble._reader, daemon=True)
        self.ble._rx_thread.start()

    def _save_mapping_results_incremental(self):
        """Save mapping results after each mapping window"""
        try:
            if not hasattr(self, 'maps_dir'):
                return
                
            # 使用窗口编号和时间戳作为文件名
            window_timestamp = time.strftime("%H%M%S")
            window_id = f"w{self.mapping_window_count:03d}_{window_timestamp}"
            
            # 保存地图数据
            try:
                map_npz_path = os.path.join(self.maps_dir, f"map_incremental_{window_id}.npz")
                map_json_path = os.path.join(self.maps_dir, f"map_incremental_{window_id}.json")
                
                if hasattr(self.mapper, 'save_npz'):
                    self.mapper.save_npz(map_npz_path)
                    print(f"[SAVE] 增量地图保存: {map_npz_path}")
                    
                if hasattr(self.mapper, 'save_json'):
                    self.mapper.save_json(map_json_path)
                    print(f"[SAVE] 增量地图JSON保存: {map_json_path}")
            except Exception as e:
                print(f"[WARN] 保存增量地图失败: {e}")
            
            # 保存可视化图像
            try:
                if self.viz and hasattr(self.viz, 'fig'):
                    map_img_path = os.path.join(self.maps_dir, f"map_visual_{window_id}.png")
                    self.viz.fig.savefig(map_img_path, dpi=160, bbox_inches='tight')
                    print(f"[SAVE] 增量地图图像保存: {map_img_path}")
            except Exception as e:
                print(f"[WARN] 保存地图图像失败: {e}")
                
        except Exception as e:
            print(f"[WARN] 增量保存过程出错: {e}")

    def plan_next_and_emit_primitives(self) -> List[str]:
        """Compute frontier, plan path, decompose to primitives, return BLE commands"""
        if not self.explorer or not self.planner or not self.nfsm:
            print("[WARN] Navigation modules not available, cannot plan next action.")
            return []
            
        if self.occ_grid is None:
            try:
                occ_export = getattr(self.mapper, 'export_occupancy_grid', None)
                if callable(occ_export):
                    self.occ_grid = self.mapper.export_occupancy_grid()
                else:
                    self.occ_grid = self.mapper.get_occupancy_grid()
            except AttributeError:
                self.occ_grid = None
        
        if self.occ_grid is None:
            print("[WARN] No occupancy grid available for planning.")
            return []
            
        goal_xy, band = compute_frontier_and_band(self.explorer, self.occ_grid, self.pose_world)
        
        # 检测恢复需求
        if goal_xy == 'RECOVERY_NEEDED':
            print("[NAV_RECOVERY] 🚨 触发恢复模式：连通域大小为1")
            return 'RECOVERY_MODE'
            
        if goal_xy is None or band is None:
            # 没有目标时，清理残留缓存，避免下一轮继续"重规划"
            self._clear_goal_and_replan_cache()
            return []
        
        # 覆盖band：对边界目标给固定厚度，对普通目标给非零厚度方形
        if goal_xy is not None:
            print(f"[SMART_BAND] 原始band: {band}")
            band = self._compute_smart_goal_band(goal_xy, (self.pose_world[0], self.pose_world[1]))
            print(f"[SMART_BAND] 智能band: {band}")
        
        # 缓存当前目标用于段后检查，并重置重规划计数器与停滞检测
        self._current_goal_xy = tuple(goal_xy) if goal_xy else None
        self._current_goal_band = tuple(band) if band else None
        self._replan_count = 0
        self._replanned_primitives = None
        self._last_replan_path_len = None
        self._stagnation_hits = 0
        # 记录初始目标中心距离
        center0 = self._band_center(self._current_goal_band)
        self._goal_initial_dist = self._dist((self.pose_world[0], self.pose_world[1]), center0)
            
        path_xy = plan_manhattan_path(self.planner, self.occ_grid, 
                                    (self.pose_world[0], self.pose_world[1]), band)
        if not path_xy:
            # 规划失败也要清理，防止进入"无尽重规划"
            self._clear_goal_and_replan_cache()
            return []
        
        # 可视化导航元素：前沿点、可达区域、规划路径、终点
        if self.viz is not None and compute_frontier_and_band_with_viz is not None:
            try:
                # 获取完整的导航信息（包括候选前沿点）
                nav_info = compute_frontier_and_band_with_viz(self.explorer, self.occ_grid, self.pose_world)
                
                # 计算终点坐标（如果存在）
                exit_xy = None
                if self.exit_band is not None:
                    exit_xy = (0.5 * (self.exit_band[0] + self.exit_band[1]), 
                              0.5 * (self.exit_band[2] + self.exit_band[3]))
                
                self.viz.show_navigation_elements(
                    frontier_xy=nav_info['goal_xy'],
                    goal_band=nav_info['band'],
                    path_points=path_xy,
                    frontier_candidates=nav_info.get('candidates', []),
                    exit_xy=exit_xy,
                    exit_band=self.exit_band  # 传递终点区域
                )
                print(f"[NAV_VIZ] 已显示导航元素：前沿点、目标区域、规划路径、终点")
            except Exception as e:
                print(f"[GUI_ERROR] 显示导航元素失败: {e}")
            
        prims = decompose_to_primitives(self.nfsm, path_xy, self.pose_world[2])
        
        # 强制切片：使用配置参数
        try:
            prims = split_large_turns(prims, MAX_TURN_CHUNK_DEG)
            prims = split_long_moves(prims, MAX_MOVE_CHUNK_M)
            print(f"[SLICING] 原语切片后数量: {len(prims)}")
        except Exception as e:
            print(f"[WARN] 原语切片失败: {e}")
        
        # 记录目标结构信息：是否需要转向、总段数
        self._cur_goal_has_turn = any(p.get("type") == "TURN" for p in prims if isinstance(p, dict))
        self._cur_goal_segments_total = len(prims)
        self._segments_done = 0
        
        cmds = primitives_to_ble_commands(prims, self.pose_world[2])
        return cmds

    def execute_commands(self, cmds: List[str], max_seg_s: float = None) -> bool:
        """Execute command sequence with timeout-based odometry tracking and distance decomposition"""
        if not cmds:
            print(f"[EXECUTE] ❌ 无指令可执行")
            return False
        
        # Use config parameter if not provided
        if max_seg_s is None:
            max_seg_s = COMMAND_TIMEOUT_S
        
        print(f"[EXECUTE] 开始执行指令序列: {len(cmds)} 条指令")
        executed_count = 0
        self._segments_done = 0  # 重置已执行段数
        
        segment_check_stopped = False  # 跟踪段后检查是否要求停止
        for i, cmd in enumerate(cmds):
            print(f"[EXECUTE] 执行指令 [{i+1}/{len(cmds)}]: {cmd}")
            
            # Parse command to check if it's a long-distance move that needs decomposition
            decomposition_needed = False
            move_distance_mm = 0
            angle_deg = 0
            
            try:
                if cmd.startswith("(M,") and cmd.endswith(")"):
                    content = cmd[3:-1]
                    parts = content.split(",")
                    if len(parts) >= 2:
                        angle_deg = float(parts[0])
                        move_distance_mm = int(parts[1])  # 修正：distance在parts[1]位置
                        
                        # Check if it's a move > MAX_MOVE_CHUNK_M that needs decomposition
                        max_move_mm = int(MAX_MOVE_CHUNK_M * 1000)
                        decomposition_needed = (move_distance_mm > max_move_mm)
            except Exception as e:
                print(f"  ⚠️  指令解析失败: {e}")
            
            if decomposition_needed:
                print(f"  🔄 检测到长距离移动 ({move_distance_mm}mm > {max_move_mm}mm)，进行分解执行")
                segments_executed = self._execute_decomposed_move(angle_deg, move_distance_mm, max_seg_s)
                executed_count += segments_executed
                # 更新段数计数（分解执行计算多段）
                self._segments_done += segments_executed
                # 检查分解执行是否因为段后检查而提前停止
                if self._current_goal_band is None:  # 如果目标带被清理了，说明到达了
                    segment_check_stopped = True
                    print(f"  🛑 分解执行因段后检查停止，共执行 {segments_executed} 个段")
                    break
            else:
                # Execute single command normally
                was_turn = False
                # 预解析判定是否转向段
                try:
                    if cmd.startswith("(M,") and cmd.endswith(")"):
                        parts = cmd[3:-1].split(",")
                        if len(parts) >= 2:
                            was_turn = (float(parts[0]) != 0.0)
                except Exception:
                    pass
                executed_count += self._execute_single_command(cmd, max_seg_s)
                # 更新段数计数
                self._segments_done += 1
                # 移动段后才做小建图与到达判定；转向段直接跳过
                if (not was_turn) and self._post_segment_check():
                    print(f"  🛑 指令 [{i+1}] 段后检查表明应停止执行")
                    segment_check_stopped = True
                    break
        
        # 如果段后检查要求停止，确保缓存完全清理
        if segment_check_stopped:
            # 双重保险：确保即使段后检查中有遗漏，这里也彻底清理
            if self._current_goal_band is None:  # 到达判定应该已经清理了
                self._replanned_primitives = None  # 强制清理重规划缓存
                print(f"  🧹 执行结束：确认清理重规划缓存")
        
        print(f"[EXECUTE] ✅ 指令执行完成: {executed_count}/{len(cmds)} (包含分解执行)")
        
        # Prepare for next mapping window
        self._rebase_yaw_on_first_angle = True
        return executed_count > 0
    
    def _execute_single_command(self, cmd: str, max_seg_s: float) -> int:
        """Execute a single command with timeout-based tracking"""
        print(f"[EXECUTE_SINGLE] 执行单指令: {cmd}")
        self.ble.send(cmd)
        start_time = time.time()
            
        # Determine command type for logging
        turning = False
        try:
            if cmd.startswith("(M,") and cmd.endswith(")"):
                content = cmd[3:-1]
                parts = content.split(",")
                if len(parts) >= 2:
                    angle_deg = float(parts[0])
                    turning = (angle_deg != 0)
        except Exception:
            pass
            
        print(f"  ⏱️  等待执行 {max_seg_s:.1f}s (类型: {'转向' if turning else '移动'}) - 不等待确认信号")
        
        # Wait for timeout period, continuously update odometry
        while time.time() - start_time < max_seg_s:
            pts, _, _ = self._pump_serial_once()
            self.pose_world = update_robot_position_from_odometry(self.odom, self.pose_world)
            self.mapper.set_robot_pose(tuple(self.pose_world))
            time.sleep(self.control_dt)
        
        # Final pose update
        final_pose = update_robot_position_from_odometry(self.odom, self.pose_world)
        pose_changed = any(abs(final_pose[i] - self.pose_world[i]) > 1e-6 for i in range(3))
        if pose_changed:
            self.pose_world = final_pose
            self.mapper.set_robot_pose(tuple(self.pose_world))
            print(f"  ✅ 单指令执行完毕，位姿已更新: ({self.pose_world[0]:.3f}, {self.pose_world[1]:.3f}, {math.degrees(self.pose_world[2]):.1f}°)")
        else:
            print(f"  ✅ 单指令执行完毕，位姿无变化")
        
        return 1
    
    def _execute_decomposed_move(self, angle_deg: float, total_distance_mm: int, max_seg_s: float) -> int:
        """Execute decomposed move: 35cm chunks with mapping in between"""
        print(f"[EXECUTE_DECOMPOSE] 开始分解移动: 角度 {angle_deg:.1f}°, 总距离 {total_distance_mm}mm")
        
        remaining_distance = total_distance_mm
        executed_segments = 0
        segment_size = int(MAX_MOVE_CHUNK_M * 1000)  # Convert to mm
        
        while remaining_distance > 0:
            # Calculate current segment size
            current_segment = min(segment_size, remaining_distance)
            
            # For the first segment, include the turn if angle_deg != 0
            # For subsequent segments, only move (angle_deg = 0)
            if executed_segments == 0:
                segment_cmd = f"(M,{angle_deg:.1f},{current_segment})"
            else:
                segment_cmd = f"(M,0,{current_segment})"
            
            print(f"  📏 执行段 [{executed_segments + 1}]: {segment_cmd} (剩余: {remaining_distance - current_segment}mm)")
            
            # Execute current segment
            self.ble.send(segment_cmd)
            start_time = time.time()
            
            # Wait for segment completion with odometry tracking
            while time.time() - start_time < max_seg_s:
                pts, _, _ = self._pump_serial_once()
                self.pose_world = update_robot_position_from_odometry(self.odom, self.pose_world)
                self.mapper.set_robot_pose(tuple(self.pose_world))
                time.sleep(self.control_dt)
            
            # Update remaining distance
            remaining_distance -= current_segment
            executed_segments += 1
            
            # Final pose update for this segment
            final_pose = update_robot_position_from_odometry(self.odom, self.pose_world)
            if any(abs(final_pose[i] - self.pose_world[i]) > 1e-6 for i in range(3)):
                self.pose_world = final_pose
                self.mapper.set_robot_pose(tuple(self.pose_world))
            
            print(f"  ✅ 段 [{executed_segments}] 完成，当前位姿: ({self.pose_world[0]:.3f}, {self.pose_world[1]:.3f}, {math.degrees(self.pose_world[2]):.1f}°)")
            
            # 只有"移动"段后检查。第一段若含转向，先跳过检查；后续直行动作后检查
            do_check = not (executed_segments == 1 and abs(angle_deg) > 0.5)
            if do_check and self._post_segment_check():
                print(f"  🛑 段后检查表明应停止执行，提前结束")
                break
            
            # If there are more segments, do intermediate mapping (moved to _post_segment_check)
            if remaining_distance > 0:
                # 段后检查已经包含建图，这里只做延迟
                time.sleep(0.5)
        
        print(f"[EXECUTE_DECOMPOSE] ✅ 分解移动完成: {executed_segments} 个段")
        return executed_segments
    
    def _post_segment_check(self) -> bool:
        """
        段后流程：小建图→到达判定→贴边逃逸→可选重规划
        返回: True 表示应停止本批执行（到达/逃逸/重规划由下一轮处理）
        """
        try:
            # 小建图
            mapping_duration = INTERMEDIATE_MAPPING_S
            print(f"  🗺️  段后小建图 {mapping_duration}s")
            self.state = State.MAPPING
            points, mapping_time = self.do_mapping_window_by_points(target_points=POST_SEGMENT_MAPPING_POINTS)
            print(f"  🚀 段后小建图完成: {points}个点, 用时{mapping_time:.2f}s")
        except Exception as e:
            print(f"  ⚠️  段后小建图失败: {e}")

        # 到达判定 - 先做闸门/带判定，命中则直接到达
        goal_reached = False
        if self._current_goal_band is not None and NAV_MODULES_AVAILABLE:
            try:
                # 1) 先做闸门/带判定，命中则直接到达
                pos_ok = self._reached_band_by_edge_cross(
                    (self.pose_world[0], self.pose_world[1]), 
                    self._current_goal_band
                )
                
                # 进展判据保持（避免擦边误触）
                center = self._band_center(self._current_goal_band)
                cur_d = self._dist((self.pose_world[0], self.pose_world[1]), center)
                init_d = self._goal_initial_dist if self._goal_initial_dist is not None else cur_d + 1.0
                progressed = (init_d - cur_d) >= 0.03
                
                # 早到达：命中闸门/带且有最小进展 → 直接到达
                if pos_ok and progressed:
                    print(f"  🎯 早到达：已跨越闸门/带，停止剩余原语并清理目标缓存")
                    print(f"  📊 进展: {init_d - cur_d:.3f}m, 当前距离: {cur_d:.3f}m")
                    self._clear_goal_and_replan_cache()
                    goal_reached = True
                    return True
                
                # 2) 未命中闸门/带 → 才应用"多段未完成就不判到达"的保护
                if (self._cur_goal_has_turn and 
                    self._segments_done < self._cur_goal_segments_total):
                    print(f"  📊 多段目标检查: 已执行{self._segments_done}/{self._cur_goal_segments_total}段，包含转向，继续执行")
                    return False
                
                # 3) 如果三层判定失败，使用距离阈值作为兜底
                if not pos_ok:
                    reach_thr = max(0.07, min(0.10, 0.25 * max(init_d, 1e-3)))  # 自适应阈值
                    pos_ok = (cur_d <= reach_thr)
                
                if pos_ok and progressed:
                    print(f"  🎯 已到达目标可达带，停止剩余原语并清理目标缓存")
                    print(f"  📊 进展: {init_d - cur_d:.3f}m, 当前距离: {cur_d:.3f}m, 阈值: {reach_thr:.3f}m")
                    self._clear_goal_and_replan_cache()
                    goal_reached = True
                    return True
            except Exception as e:
                print(f"  ⚠️  到达判定失败: {e}")

        # 贴边逃逸
        try:
            occ = self.mapper.get_occupancy_grid()
            if self._need_escape(occ, tuple(self.pose_world)):
                print(f"  🚨 贴边过近，执行逃逸")
                self._perform_escape()
                return True
        except Exception as e:
            print(f"  ⚠️  贴边检测失败: {e}")

        # 原语重规划：基于更新后的地图和位姿重新规划路径
        # 只有在未到达目标且仍有目标时才进行重规划
        try:
            if not goal_reached and self._current_goal_band is not None and NAV_MODULES_AVAILABLE:
                # 更新占用网格
                try:
                    occ_export = getattr(self.mapper, 'export_occupancy_grid', None)
                    if callable(occ_export):
                        self.occ_grid = self.mapper.export_occupancy_grid()
                    else:
                        self.occ_grid = self.mapper.get_occupancy_grid()
                except AttributeError:
                    self.occ_grid = None
                
                if self.occ_grid is not None and self.planner is not None:
                    # 防止重规划死循环：限制同一目标的重规划次数
                    # 注意：由于goal_reached已经在前面检查过，这里不再重复检查
                    if self._replan_count < 3:
                        # 重新规划路径
                        new_path = plan_manhattan_path(self.planner, self.occ_grid, 
                                                     (self.pose_world[0], self.pose_world[1]), 
                                                     self._current_goal_band)
                        # 0/1 个点视为"已达"（A*退化为原地）
                        if new_path and len(new_path) <= 1:
                            print("  🎯 规划退化为原地/极短路径，按已到达处理")
                            self._clear_goal_and_replan_cache()
                            return True
                        if new_path and len(new_path) > 1:
                            # 停滞检测：路径长度未改善则累计命中
                            cur_len = len(new_path)
                            if self._last_replan_path_len is not None and cur_len >= self._last_replan_path_len:
                                self._stagnation_hits += 1
                            else:
                                self._stagnation_hits = 0
                            self._last_replan_path_len = cur_len
                            # 连续2次无改善，放弃该目标
                            if self._stagnation_hits >= 2:
                                print("  ⚠️ 重规划无改善达到阈值，释放当前目标以重新选点")
                                self._clear_goal_and_replan_cache()
                                return True
                            # 生成新的原语序列
                            new_prims = decompose_to_primitives(self.nfsm, new_path, self.pose_world[2])
                            if new_prims:
                                # 应用原语切片
                                try:
                                    from core.config import MAX_MOVE_CHUNK_M, MAX_TURN_CHUNK_DEG
                                    new_prims = split_large_turns(new_prims, MAX_TURN_CHUNK_DEG)
                                    new_prims = split_long_moves(new_prims, MAX_MOVE_CHUNK_M)
                                    
                                    # 转换为BLE命令并缓存
                                    new_cmds = primitives_to_ble_commands(new_prims, self.pose_world[2])
                                    if new_cmds:
                                        self._replanned_primitives = new_cmds
                                        self._replan_count += 1  # 增加重规划计数器
                                        print(f"  🔄 段后重规划成功，缓存 {len(new_cmds)} 条命令 (重规划次数: {self._replan_count})")
                                        # 由于原语已变更，建议返回True以停止当前批次，让下一轮处理新原语
                                        return True
                                    else:
                                        print(f"  ⚠️  重规划原语转换为BLE命令失败")
                                except Exception as e:
                                    print(f"  ⚠️  原语切片失败: {e}")
                    else:
                        print(f"  ⚠️  重规划次数超限 ({self._replan_count}/3)，清理目标缓存")
                        # 重规划失败或超限，清理目标缓存以探索新前沿点
                        self._clear_goal_and_replan_cache()
        except Exception as e:
            print(f"  ⚠️  段后重规划失败: {e}")
        
        return False

    def _need_escape(self, occ, pose_world, min_clear_m: float = 0.12) -> bool:
        """基于C-space的近距清空度判断是否贴边过近"""
        try:
            res = self.map_res
        except:
            res = SLAM_RESOLUTION
        i = int(round(pose_world[1] / res))
        j = int(round(pose_world[0] / res))
        if i < 1 or j < 1 or i >= occ.shape[0]-1 or j >= occ.shape[1]-1:
            return True  # 紧贴边界
        # 3x3邻域障碍/未知占比
        nb = occ[i-1:i+2, j-1:j+2]
        bad = ((nb == 1) | (nb == 2)).sum()
        if bad >= 5:
            return True
        # 半径r内是否有障碍/未知
        r_px = max(2, int(round(min_clear_m / res)))
        si = max(0, i-r_px); ei = min(occ.shape[0], i+r_px+1)
        sj = max(0, j-r_px); ej = min(occ.shape[1], j+r_px+1)
        win = occ[si:ei, sj:ej]
        if ((win == 1) | (win == 2)).any():
            return True
        return False

    def _perform_escape(self):
        """掉头180°并退/前进15cm，然后小建图"""
        try:
            # English: split 180° into two safer 90° rotations, move after the second
            self.ble.send("(M,90,0)")
            time.sleep(0.2)
            self.ble.send("(M,90,150)")
            start_time = time.time()
            
            # 等待执行
            while time.time() - start_time < COMMAND_TIMEOUT_S:
                pts, _, _ = self._pump_serial_once()
                self.pose_world = update_robot_position_from_odometry(self.odom, self.pose_world)
                self.mapper.set_robot_pose(tuple(self.pose_world))
                time.sleep(self.control_dt)
            
            # 逃逸后立刻小建图，刷新C-space
            escape_mapping_duration = max(5, int(INTERMEDIATE_MAPPING_S/2))
            self.state = State.MAPPING
            self.do_mapping_window(escape_mapping_duration)
        except Exception as e:
            print(f"  ⚠️  逃逸执行失败: {e}")

    def _find_nearest_obstacle_direction(self, occ: np.ndarray, pose_world) -> Optional[Tuple[float, float]]:
        """检测最近的障碍方向，返回远离障碍的方向向量 (dx, dy)"""
        try:
            res = getattr(self, 'map_res', SLAM_RESOLUTION)
        except:
            res = SLAM_RESOLUTION
            
        robot_i = int(round(pose_world[1] / res))  # y坐标对应行
        robot_j = int(round(pose_world[0] / res))  # x坐标对应列
        h, w = occ.shape
        
        if robot_i < 0 or robot_i >= h or robot_j < 0 or robot_j >= w:
            return None
            
        # 在局部范围内搜索最近障碍
        search_radius_px = min(50, min(h, w) // 4)  # 限制搜索范围，避免过远
        min_dist = float('inf')
        nearest_obstacle_i, nearest_obstacle_j = None, None
        
        for di in range(-search_radius_px, search_radius_px + 1):
            for dj in range(-search_radius_px, search_radius_px + 1):
                test_i = robot_i + di
                test_j = robot_j + dj
                
                if (0 <= test_i < h and 0 <= test_j < w and 
                    occ[test_i, test_j] == 1):  # 障碍物
                    
                    dist = math.sqrt(di*di + dj*dj)
                    if dist < min_dist and dist > 0:  # 避免除零
                        min_dist = dist
                        nearest_obstacle_i = test_i
                        nearest_obstacle_j = test_j
        
        if nearest_obstacle_i is None or nearest_obstacle_j is None:
            return None
            
        # 计算机器人到障碍的方向 (归一化)
        obstacle_dx = (nearest_obstacle_j - robot_j) * res  # 世界坐标系x
        obstacle_dy = (nearest_obstacle_i - robot_i) * res  # 世界坐标系y
        obstacle_dist = math.sqrt(obstacle_dx*obstacle_dx + obstacle_dy*obstacle_dy)
        
        if obstacle_dist < 1e-6:
            return None
            
        # 远离障碍的方向 (相反方向)
        escape_dx = -obstacle_dx / obstacle_dist
        escape_dy = -obstacle_dy / obstacle_dist
        
        return (escape_dx, escape_dy)
        
    def _perform_recovery_movement(self):
        """执行恢复移动：检测最近障碍，朝远离方向移动15cm"""
        try:
            occ = self.mapper.get_occupancy_grid()
            escape_dir = self._find_nearest_obstacle_direction(occ, tuple(self.pose_world))
            
            if escape_dir is None:
                print("  🔄 恢复模式：未检测到明确障碍，执行默认后退")
                # English: split 180° into two safer 90° rotations, move after the second
                self.ble.send("(M,90,0)")
                time.sleep(0.2)
                self.ble.send("(M,90,150)")
            else:
                # 检查方向计算是否可靠
                escape_dx, escape_dy = escape_dir
                if not (math.isfinite(escape_dx) and math.isfinite(escape_dy)):
                    print("  🔄 恢复模式：方向计算不可靠，执行兜底掉头")
                    # English: split 180° into two safer 90° rotations, move after the second
                    self.ble.send("(M,90,0)")
                    time.sleep(0.2)
                    self.ble.send("(M,90,150)")
                    # 等待执行
                    start_time = time.time()
                    while time.time() - start_time < COMMAND_TIMEOUT_S:
                        pts, _, _ = self._pump_serial_once()
                        self.pose_world = update_robot_position_from_odometry(self.odom, self.pose_world)
                        self.mapper.set_robot_pose(tuple(self.pose_world))
                        time.sleep(self.control_dt)
                    
                    # 恢复后立刻小建图，刷新C-space
                    recovery_mapping_duration = max(8, int(INTERMEDIATE_MAPPING_S*0.8))
                    self.state = State.MAPPING
                    self.do_mapping_window(recovery_mapping_duration)
                    print(f"  ✅ 兜底恢复移动完成，执行建图 {recovery_mapping_duration}s")
                    return
                # 计算目标角度（相对于当前朝向）
                escape_dx, escape_dy = escape_dir
                target_angle_rad = math.atan2(escape_dy, escape_dx)
                current_angle_rad = self.pose_world[2]  # ✅ 已经是弧度，不需要转换
                
                # 计算需要转向的角度
                angle_diff_rad = target_angle_rad - current_angle_rad
                # 归一化到[-π, π]
                angle_diff_rad = ((angle_diff_rad + math.pi) % (2*math.pi)) - math.pi
                angle_diff_deg = math.degrees(angle_diff_rad)
                
                # English: clamp to multiples of ≤90°. Two-step 90°+90° if large.
                sgn = 1.0 if angle_diff_deg >= 0 else -1.0
                abs_deg = abs(angle_diff_deg)
                
                if abs_deg > 90.0 + 1e-3:
                    self.ble.send(f"(M,{sgn*90:.1f},0)")
                    time.sleep(0.2)
                    self.ble.send(f"(M,{sgn*90:.1f},150)")
                else:
                    self.ble.send(f"(M,{angle_diff_deg:.1f},150)")
                
                print(f"  🔄 恢复模式：检测到障碍，朝远离方向移动 (角度差: {angle_diff_deg:.1f}°)")
            
            # 等待执行
            start_time = time.time()
            while time.time() - start_time < COMMAND_TIMEOUT_S:
                pts, _, _ = self._pump_serial_once()
                self.pose_world = update_robot_position_from_odometry(self.odom, self.pose_world)
                self.mapper.set_robot_pose(tuple(self.pose_world))
                time.sleep(self.control_dt)
            
            # 恢复后立刻小建图，刷新C-space
            recovery_mapping_duration = max(8, int(INTERMEDIATE_MAPPING_S*0.8))
            self.state = State.MAPPING
            self.do_mapping_window(recovery_mapping_duration)
            print(f"  ✅ 恢复移动完成，执行建图 {recovery_mapping_duration}s")
            
        except Exception as e:
            print(f"  ⚠️  恢复移动执行失败: {e}")

    def run(self, max_cycles: int = 999):
        """Main navigation loop"""
        cycles = 0
        total_mapping_time = 0
        total_points_processed = 0
        print(f"[NAV_MAIN] 开始导航循环 (任务完成或手动停止时结束)")
        
        while not self._shutdown_requested:
            cycles += 1
            print(f"\n{'='*60}")
            print(f"[NAV_MAIN] 循环 #{cycles}/{max_cycles}")
            print(f"{'='*60}")
            
            # Check for shutdown request before proceeding
            if self._shutdown_requested:
                print("[NAV_MAIN] 收到关闭请求，终止导航循环")
                break
            
            # 1) Mapping phase - 主循环建图 (800个点)
            print(f"[NAV_MAIN] 阶段1: 主循环建图 (MAPPING)")
            self.state = State.MAPPING
            
            # 进入建图前，清除导航覆盖层
            if self.viz is not None:
                try:
                    self.viz.clear_navigation_overlay()
                except Exception as e:
                    print(f"[GUI_ERROR] 清除导航覆盖层失败: {e}")
            
            # 显示终点区域
            self._show_exit_band_on_gui()
            
            try:
                points, mapping_time = self.do_mapping_window_by_points(target_points=MAIN_MAPPING_POINTS)
                total_points_processed += points
                total_mapping_time += mapping_time
                print(f"[NAV_MAIN] 主循环建图完成: {points}个点, 用时{mapping_time:.2f}s")
            except Exception as e:
                print(f"[ERROR] 建图阶段出错: {e}")
                self._force_save_all_data()
                if self._shutdown_requested:
                    break
            
            # Check for shutdown request before navigation planning
            if self._shutdown_requested:
                print("[NAV_MAIN] 收到关闭请求，终止导航循环")
                break
            
            # 2) Navigation planning
            print(f"\n[NAV_MAIN] 阶段2: 导航规划 (NAVIGATING)")
            self.state = State.NAVIGATING
            try:
                # [MODIFY] main loop decision making
                if self.mission_state == MissionState.EXPLORING:
                    # 统一的状态决策
                    next_state = self._determine_next_mission_state()
                    if next_state != MissionState.EXPLORING:
                        print(f"[NAV_MAIN] 🎯 状态切换: {self.mission_state.name} -> {next_state.name}")
                        self.mission_state = next_state
                        cmds = []  # force next loop to plan-to-exit
                    else:
                        # 正常前沿点探索
                        if self._replanned_primitives is not None:
                            cmds = self._replanned_primitives
                            self._replanned_primitives = None
                            print(f"[NAV_MAIN] 🔄 使用重规划的原语序列: {len(cmds)} 条指令")
                        else:
                            print(f"[NAV_MAIN] 🎯 寻找新的前沿点...")
                            cmds = self.plan_next_and_emit_primitives()

                elif self.mission_state == MissionState.TO_EXIT:
                    # plan to exit band, then switch to RETURN_HOME
                    if self.exit_band is None:
                        print(f"[NAV_MAIN] ⚠️  终点未设置，继续探索")
                        self.mission_state = MissionState.EXPLORING
                        cmds = self.plan_next_and_emit_primitives()
                    elif self._in_band(self.pose_world[:2], self.exit_band):
                        print(f"[NAV_MAIN] 🎯 已到达终点，开始返回起点")
                        self.mission_state = MissionState.RETURN_HOME
                        cmds = []
                    else:
                        print(f"[NAV_MAIN] 🎯 规划到终点的路径")
                        cmds = self._plan_path_to_target(self.exit_band)

                elif self.mission_state == MissionState.RETURN_HOME:
                    if self._in_band(self.pose_world[:2], self.home_band):
                        print(f"[NAV_MAIN] 🎯 已返回起点，任务完成")
                        self.mission_state = MissionState.COMPLETE
                        # 保存最终结果和日志
                        self._save_final_results_and_logs()
                        print(f"[NAV_MAIN] ✅ 任务完成，程序结束")
                        break
                    else:
                        print(f"[NAV_MAIN] 🎯 规划返回起点的路径")
                        cmds = self._plan_path_to_target(self.home_band)

                elif self.mission_state == MissionState.COMPLETE:
                    break
                
                # 检测恢复模式
                if cmds == 'RECOVERY_MODE':
                    print(f"[NAV_MAIN] 🚨 执行恢复移动...")
                    self._perform_recovery_movement()
                    print(f"[NAV_MAIN] ✅ 恢复完成，继续下一循环")
                    continue
                
                if not cmds:
                    if self.mission_state == MissionState.EXPLORING:
                        print(f"[NAV_MAIN] ⚠️  无有效路径/前沿点，继续建图探索")
                    else:
                        print(f"[NAV_MAIN] ⚠️  路径规划失败，尝试重新建图")
                    # 继续下一循环，而不是退出程序
                    continue
                
                # 3) Execute commands
                print(f"\n[NAV_MAIN] 阶段3: 指令执行 (MOVING/TURNING)")
                ok = self.execute_commands(cmds, max_seg_s=12.0)
                if not ok:
                    print(f"[NAV_MAIN] ⚠️  移动超时/失败，返回建图阶段")
                else:
                    print(f"[NAV_MAIN] ✅ 移动执行成功，继续下一循环")
                    
            except Exception as e:
                print(f"[ERROR] 导航阶段出错: {e}")
                self._force_save_all_data()
                if self._shutdown_requested:
                    break
        
        # Final cleanup and save - always called
        try:
            print("[NAV_MAIN] 进行最终数据保存...")
            self._save_final_results()
        except Exception as e:
            print(f"[ERROR] 最终保存失败: {e}")
            self._force_save_all_data()
        
        # 性能统计
        if total_mapping_time > 0:
            print(f"\n[PERFORMANCE] 总体性能统计:")
            print(f"  - 总循环数: {cycles}")
            print(f"  - 总建图时间: {total_mapping_time:.2f}s")
            print(f"  - 总处理点数: {total_points_processed}")
            print(f"  - 平均处理速度: {total_points_processed/total_mapping_time:.1f} 点/秒")
            print(f"  - 平均每循环: {total_mapping_time/cycles:.2f}s")

    def _save_final_results(self):
        """Save final mapping results to logs/nav_exploring"""
        try:
            occ_export = getattr(self.mapper, 'export_occupancy_grid', None)
            if callable(occ_export):
                occ = self.mapper.export_occupancy_grid()
            else:
                occ = self.mapper.get_occupancy_grid()
        except AttributeError:
            occ = None
            
        if occ is not None and self.viz is not None:
            try:
                self.viz.update(occ, tuple(self.pose_world))
            except Exception as e:
                print(f"[GUI_ERROR] 最终快照GUI更新失败: {e}")
        
        # Save final artifacts to logs/nav_exploring
        if hasattr(self, 'base_logdir') and hasattr(self, 'log_timestamp'):
            final_outdir = self.base_logdir
            final_ts = self.log_timestamp
        else:
            final_outdir = os.path.join("logs", "nav_exploring")
            final_ts = time.strftime("%Y%m%d_%H%M%S")
            os.makedirs(final_outdir, exist_ok=True)
        
        try:
            # 保存最终地图文件
            final_map_npz = os.path.join(final_outdir, f"final_map_{final_ts}.npz")
            final_map_json = os.path.join(final_outdir, f"final_map_{final_ts}.json")
            
            if hasattr(self.mapper, 'save_npz'):
                self.mapper.save_npz(final_map_npz)
                print(f"[SAVE] 最终地图NPZ保存: {final_map_npz}")
                
            if hasattr(self.mapper, 'save_json'):
                self.mapper.save_json(final_map_json)
                print(f"[SAVE] 最终地图JSON保存: {final_map_json}")
        except Exception as e:
            print(f"[WARN] 保存最终地图失败: {e}")
            
        try:
            # 保存最终可视化图像
            if self.viz and hasattr(self.viz, 'fig'):
                final_img_path = os.path.join(final_outdir, f"final_map_visual_{final_ts}.png")
                self.viz.fig.savefig(final_img_path, dpi=160, bbox_inches='tight')
                print(f"[SAVE] 最终地图图像保存: {final_img_path}")
        except Exception as e:
            print(f"[WARN] 保存最终地图图像失败: {e}")
            
        print(f"[SAVE] 所有文件已保存到: {final_outdir}")

    def close(self):
        """Cleanup and close connections with forced data saving"""
        try:
            print("[CLOSE] 开始关闭程序，强制保存所有数据...")
            
            # Force save all data before closing
            self._force_save_all_data()
            
            # Close hardware connections
            if hasattr(self, 'ble') and self.ble:
                self.ble.close()
                
            # Close visualization
            if hasattr(self, 'viz') and self.viz:
                self.viz.close()
                
            # 确保恢复终端输出
            self._restore_terminal_output()
                
            # Close log files (they should already be closed by _force_save_all_data, but be safe)
            if hasattr(self, 'raw_data_log') and self.raw_data_log:
                try:
                    self.raw_data_log.flush()
                    self.raw_data_log.close()
                    print("[INFO] 原始数据日志文件已关闭")
                except:
                    pass
                    
            if hasattr(self, 'stats_log') and self.stats_log:
                try:
                    self.stats_log.flush()
                    self.stats_log.close()
                    print("[INFO] 建图统计日志文件已关闭")
                except:
                    pass
            
            if hasattr(self, 'terminal_log') and self.terminal_log:
                try:
                    self.terminal_log.flush()
                    self.terminal_log.close()
                    print("[INFO] 终端输出日志文件已关闭")
                except:
                    pass
            
            print("[CLOSE] 所有连接和数据已安全关闭")
            
        except Exception as e:
            print(f"[ERROR] 关闭过程中出错: {e}")
            # Even if there's an error, try to save data one more time
            try:
                self._force_save_all_data()
            except:
                pass

    def _get_map_statistics(self) -> Tuple[float, float]:
        """获取地图统计信息：覆盖率、未知区域比例"""
        occ = self.mapper.get_occupancy_grid()
        total = occ.size
        unknown = (occ == 2).sum()
        known = total - unknown
        cov_ratio = known / max(total, 1)
        unk_ratio = unknown / max(total, 1)
        return cov_ratio, unk_ratio


    def _is_exploration_complete(self) -> bool:
        """判断探索是否完成"""
        cov_ratio, unk_ratio = self._get_map_statistics()
        is_complete = (cov_ratio >= self.coverage_threshold or 
                      unk_ratio <= self.unknown_threshold)
        
        print(f"[EXPLORATION_CHECK] 覆盖率: {cov_ratio:.3f} (阈值: {self.coverage_threshold})")
        print(f"[EXPLORATION_CHECK] 未知率: {unk_ratio:.3f} (阈值: {self.unknown_threshold})")
        print(f"[EXPLORATION_CHECK] 探索完成: {is_complete}")
        
        return is_complete

    def _can_reach_exit(self) -> bool:
        """判断是否能到达终点"""
        if self.exit_band is None:
            return False
        
        # 检查路径可达性
        path = self._path_to_band(self.exit_band)
        return len(path) > 0

    def _try_finish_exploration_by_goal_reachability(self) -> bool:
        """
        实时终点检测：严格判定终点可达性，连续多帧可达才切 TO_EXIT
        参考 navigator.py 的逻辑
        """
        if self.exit_band is None:
            return False
        
        # 1. 检查是否已经到达终点区域
        if self._in_band(self.pose_world[:2], self.exit_band):
            print(f"[REAL_TIME_CHECK] 🎯 已到达终点区域: {self.pose_world[:2]}")
            return True
        
        # 2. 检查路径可达性（连续帧判定）
        path = self._path_to_band(self.exit_band)
        reachable = len(path) > 0
        
        # 3. 连续帧判定（避免误判）
        if reachable:
            self._exit_path_streak += 1
        else:
            self._exit_path_streak = 0
        
        print(f"[REAL_TIME_CHECK] 路径可达性: {reachable}, 连续帧: {self._exit_path_streak}/{self._need_streak}")
        
        # 4. 达到阈值才切换
        if self._exit_path_streak >= self._need_streak:
            print(f"[REAL_TIME_CHECK] 🎯 连续{self._need_streak}帧可达，准备切换状态")
            self._exit_path_streak = 0  # 重置计数器
            return True
        
        return False

    def _show_exit_band_on_gui(self):
        """在GUI中显示终点区域"""
        if self.viz is not None and self.exit_band is not None:
            try:
                # 计算终点坐标
                exit_xy = (0.5 * (self.exit_band[0] + self.exit_band[1]), 
                          0.5 * (self.exit_band[2] + self.exit_band[3]))
                
                self.viz.show_navigation_elements(
                    exit_xy=exit_xy,
                    exit_band=self.exit_band
                )
                print(f"[GUI] 显示终点区域: {self.exit_band}")
            except Exception as e:
                print(f"[GUI_ERROR] 显示终点区域失败: {e}")

    def _show_path_and_primitives_on_gui(self, path_xy, prims, target_band):
        """在GUI中显示路径和原语信息"""
        if self.viz is not None:
            try:
                # 判断是终点还是起点
                is_exit = (target_band == self.exit_band)
                is_home = (target_band == self.home_band)
                
                # 准备显示参数
                exit_xy = None
                exit_band = None
                home_band = None
                
                if is_exit:
                    exit_xy = (0.5 * (self.exit_band[0] + self.exit_band[1]), 
                              0.5 * (self.exit_band[2] + self.exit_band[3]))
                    exit_band = self.exit_band
                elif is_home:
                    home_band = self.home_band
                
                # 生成原语信息字符串
                primitives_info = self._format_primitives_info(prims)
                
                # 显示导航元素
                self.viz.show_navigation_elements(
                    path_points=path_xy,
                    exit_xy=exit_xy,
                    exit_band=exit_band,
                    home_band=home_band,
                    primitives_info=primitives_info
                )
                
                print(f"[GUI] 显示路径和原语信息: 路径长度={len(path_xy)}, 原语数量={len(prims)}")
            except Exception as e:
                print(f"[GUI_ERROR] 显示路径和原语信息失败: {e}")

    def _format_primitives_info(self, prims) -> str:
        """格式化原语信息为字符串"""
        if not prims:
            return "无原语"
        
        info_lines = [f"原语数量: {len(prims)}"]
        for i, prim in enumerate(prims[:5]):  # 只显示前5个原语
            if isinstance(prim, dict):
                prim_type = prim.get('type', 'UNKNOWN')
                if prim_type == 'TURN':
                    angle = prim.get('angle', 0)
                    info_lines.append(f"{i+1}. 转向: {angle:.1f}°")
                elif prim_type == 'MOVE':
                    distance = prim.get('distance', 0)
                    info_lines.append(f"{i+1}. 移动: {distance:.3f}m")
                else:
                    info_lines.append(f"{i+1}. {prim_type}")
            else:
                info_lines.append(f"{i+1}. {str(prim)}")
        
        if len(prims) > 5:
            info_lines.append(f"... 还有{len(prims)-5}个原语")
        
        return "\n".join(info_lines)

    def _save_final_results_and_logs(self):
        """保存最终结果和日志，任务完成时调用"""
        try:
            print(f"[FINAL_SAVE] 开始保存最终结果和日志...")
            
            # 1. 保存最终地图和可视化
            self._save_final_results()
            
            # 2. 保存任务完成日志
            self._save_task_completion_log()
            
            # 3. 强制保存所有数据
            self._force_save_all_data()
            
            print(f"[FINAL_SAVE] ✅ 最终结果和日志保存完成")
            
        except Exception as e:
            print(f"[FINAL_SAVE] ❌ 保存最终结果失败: {e}")

    def _save_task_completion_log(self):
        """保存任务完成日志"""
        try:
            if hasattr(self, 'stats_log') and self.stats_log:
                completion_log = f"\n{'='*60}\n"
                completion_log += f"任务完成日志 - {time.strftime('%Y-%m-%d %H:%M:%S')}\n"
                completion_log += f"{'='*60}\n"
                completion_log += f"任务状态: {self.mission_state.name}\n"
                completion_log += f"最终位姿: ({self.pose_world[0]:.3f}, {self.pose_world[1]:.3f}, {math.degrees(self.pose_world[2]):.1f}°)\n"
                completion_log += f"起点区域: {self.home_band}\n"
                completion_log += f"终点区域: {self.exit_band}\n"
                completion_log += f"总建图时间: {getattr(self, 'total_mapping_time', 0):.2f}s\n"
                completion_log += f"总处理点数: {getattr(self, 'total_points_processed', 0)}\n"
                completion_log += f"任务完成时间: {time.strftime('%H:%M:%S')}\n"
                completion_log += f"{'='*60}\n"
                
                self.stats_log.write(completion_log)
                self.stats_log.flush()
                print(f"[TASK_LOG] 任务完成日志已保存")
                
        except Exception as e:
            print(f"[TASK_LOG] 保存任务完成日志失败: {e}")

    def _determine_next_mission_state(self) -> MissionState:
        """统一的任务状态决策"""
        # 1. 实时终点检测（探索过程中持续检查）
        if self._try_finish_exploration_by_goal_reachability():
            print(f"[NAV_MAIN] 🎯 实时检测：已到达终点区域，切换状态")
            return MissionState.TO_EXIT
        
        # 2. 探索完成判断
        if self._is_exploration_complete():
            if self._can_reach_exit():
                return MissionState.TO_EXIT
            # 否则继续探索
        
        return MissionState.EXPLORING

    def _in_band(self, xy, band) -> bool:
        """Use existing reached_band helper from nav_adapters."""
        from nav.nav_adapters import reached_band  # local import to avoid hard dep at import time
        return reached_band(xy, band, tol_m=0.06)  # 6 cm tolerance

    def _path_to_band(self, band):
        """Plan a manhattan path to a rectangular band using existing A*."""
        # NOTE: your docs declare AStarPlanner.plan_to_band().
        try:
            # Convert world coordinates to grid coordinates
            from core.config import SLAM_RESOLUTION
            start_ij = (int(self.pose_world[1] / SLAM_RESOLUTION),
                       int(self.pose_world[0] / SLAM_RESOLUTION))
            
            print(f"[PATH_PLANNING] 起点: {self.pose_world[:2]} -> 网格: {start_ij}")
            print(f"[PATH_PLANNING] 目标区域: {band}")
            print(f"[PATH_PLANNING] 地图尺寸: {self.occ_grid.shape if self.occ_grid is not None else 'None'}")
            
            # 传入已经膨胀的地图，避免二次膨胀
            # 使用 grid_is_cspace=True 告诉规划器地图已经是C-space
            path = self.planner.plan_to_band(
                start_ij, band, self.occ_grid, 
                grid_is_cspace=True  # 标记地图已经是C-space，避免二次膨胀
            )
            
            print(f"[PATH_PLANNING] 路径长度: {len(path) if path else 0}")
            return path
        except Exception as e:
            print(f"[PATH_PLANNING] 路径规划失败: {e}")
            return []

    def _plan_path_to_target(self, target_band) -> List[str]:
        """统一的路径规划方法：A* -> 原语 -> BLE命令"""
        path_xy = self._path_to_band(target_band)
        if not path_xy:
            return []

        # 1) decompose to motion primitives
        from nav.nav_adapters import decompose_to_primitives
        prims = decompose_to_primitives(self.nfsm, path_xy, self.pose_world[2])
        
        # 显示路径和原语信息到GUI
        self._show_path_and_primitives_on_gui(path_xy, prims, target_band)
        
        # 2) convert to BLE "(M,<deg>,<mm>)" commands
        from nav.nav_adapters import primitives_to_ble_commands
        return primitives_to_ble_commands(prims, self.pose_world[2])
