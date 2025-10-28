# ================================
# file: code/core/config.py
# ================================
"""
Global configuration for autonomous maze exploration system.
All units are SI (meters, radians, seconds).

Organization:
1. Map & Coordinate System
2. Robot Physical Parameters
3. Sensor Configuration
4. SLAM Configuration
5. Path Planning & Navigation
6. Control & Motion
7. Exploration Strategy
8. GUI & Logging
"""
from __future__ import annotations
import math

# ================================
# 1. MAP & COORDINATE SYSTEM
# ================================
# Legacy map parameters (for compatibility)
# 迷宫几何参数（统一外部真实与仿真）
MAZE_W_M: float = 2.8           # 迷宫宽度 (meters)
MAZE_H_M: float = 2.8           # 迷宫高度 (meters)
MAZE_CELL_M: float = 0.70       # JSON单格物理尺寸 (meters)
MAP_RES: float = MAZE_CELL_M    # 兼容旧名
MAP_SIZE: tuple = (5, 5)        # (H, W) units -> 保持兼容

# World coordinate system
WORLD_SIZE: float = 2.8         # World coordinate system size (meters)
GRID_SIZE: int = 5              # Grid coordinate system size (cells)
WORLD_VIEW_SIZE_METERS: float = WORLD_SIZE  # World view size (meters)

# 迷宫配置（动态计算偏移）
_ox = 0.5 * (WORLD_SIZE - MAZE_W_M)  # 0.5 * (2.8 - 2.8) = 0.0
_oy = 0.5 * (WORLD_SIZE - MAZE_H_M)  # 0.5 * (2.8 - 2.8) = 0.0
MAZE_CENTER_OFFSET: tuple = (_ox, _oy)   # (0.0, 0.0) 迷宫完全填充世界
MAZE_DEFAULT_SIZE: tuple = (4, 4)        # 2.8m/0.7m = 4格

# ================================
# 2. ROBOT PHYSICAL PARAMETERS
# ================================
# Robot geometry
ROBOT_RADIUS: float = 0.00       # Robot radius: 10cm (realistic for small robot on 2m map)
SAFE_BUFFER_M: float = 0.12     # Safety buffer for path planning: 20cm (优化后参数)

# Robot starting position (world coordinates)
ROBOT_START_X: float = 0.35   # Start X position (meters)
ROBOT_START_Y: float = 2.45      # Start Y position (meters)
ROBOT_START_THETA: float = 0
"""
json1 0.325 0.1 1.57
/ 0.775 0.1 1.57 交换起点终点 难

json2 1.9 1.225 3.14   难

json3 1.9 1.675 3.14   

"""


# ================================
# 3. SENSOR CONFIGURATION
# ================================
# LiDAR parameters
LIDAR_RANGE: float = 3.5        # Maximum range: 3m
LIDAR_MIN_RANGE: float = 0.05   # Minimum valid range: 5cm
LIDAR_FOV_DEG: float = 360.0    # Field of view: 360°
LIDAR_BEAMS: int = 360          # Number of beams: 360 (1° resolution)
LIDAR_SCAN_RATE_HZ: float = 5.5 # Scan rate: 5.5 Hz
LIDAR_YAW_BIAS: float = 0.0     # Sensor yaw bias (rad, positive = CCW)

# LiDAR calibration (for EKF augmentation)
LIDAR_YAW_BIAS_INIT: float = 0.0         # Initial guess for sensor yaw bias (rad)
EKF_BIAS_Q: float = 1e-6                 # Process noise for bias state (rad²)
EKF_BIAS_MAX_ABS: float = 0.2617993877991494  # Max bias: 15° (rad)
BIAS_UPDATE_SCORE_GATE: float = 0.02     # Minimum score to trust scan for bias update

# ================================
# 4. SLAM CONFIGURATION
# ================================
# SLAM map parameters (unified source)
SLAM_RESOLUTION: float = 0.01           # Resolution: 0.01 m/pixel (保持不变)
SLAM_MAP_SIZE_METERS: float = WORLD_SIZE  # Map physical size: 2.8×2.8 meters
SLAM_MAP_SIZE_PIXELS: int = int(round(SLAM_MAP_SIZE_METERS / SLAM_RESOLUTION))  # 280×280 pixels

# SLAM architecture
SLAM_USE_DECOUPLED_ARCHITECTURE: bool = True  # True: Decouple mapping and localization
SLAM_USE_GLOBAL_ORIGIN: bool = True           # Use global origin (0,0) for SLAM map

# SLAM completion triggers
MAPPING_TIME_THRESHOLD: float = 60.0    # Complete mapping after 60 seconds
MAPPING_COVERAGE_THRESHOLD: float = 0.80  # Complete mapping after 80% coverage

# BreezySLAM specific parameters
SLAM_MAP_QUALITY: int = 100             # Map quality parameter
SLAM_HOLE_WIDTH_MM: int = 300           # Hole width in mm
SLAM_SIGMA_XY_MM: int = 50              # XY search range in mm
SLAM_SIGMA_THETA_DEGREES: int = 7       # Angle search range in degrees
SLAM_MAX_SEARCH_ITER: int = 500         # Maximum search iterations

# SLAM export/transformation
SLAM_EXPORT_ENABLE: bool = True         # Enable pose/map export with transform
SLAM_EXPORT_ROTATE_DEG: int = 90        # CCW rotation at export (match GUI axes)
SLAM_EXPORT_SHIFT_M = (0.0, 0.0)        # Translation (auto-calibrated if enabled)
SLAM_EXPORT_AUTO_CALIBRATE: bool = True # Enable auto-calibration of translation

# EKF-SLAM noise parameters
EKF_Q_V: float = 0.02                   # Linear velocity noise std (m/s)
EKF_Q_W: float = 0.02                   # Angular velocity noise std (rad/s)
EKF_R_X: float = 0.03                   # X observation noise std (m)
EKF_R_Y: float = 0.03                   # Y observation noise std (m)
EKF_R_TH: float = 0.03                  # Theta observation noise std (rad)

# Inverse sensor model (occupancy grid mapping)
LOG_ODDS_HIT: float = +0.85             # Log-odds increment for hit
LOG_ODDS_MISS: float = -0.4             # Log-odds increment for miss
LOG_ODDS_MIN: float = -2.5              # Minimum log-odds value
LOG_ODDS_MAX: float = +3.5              # Maximum log-odds value
OCC_THRESH: float = 0.65                # Occupied threshold
FREE_THRESH: float = 0.35               # Free threshold

# Scan matching parameters
MATCH_SEARCH_DXY: float = 0.30          # Search window size (m)
MATCH_SEARCH_DTH: float = 10.0          # Angle search range (deg)
MATCH_STEP_DXY: float = 0.02            # Position search step (m)
MATCH_STEP_DTH: float = 2.0             # Angle search step (deg)
MATCH_SCORE_MIN: float = 0.10           # Minimum match score threshold

# SLAM optimization parameters (engineering-grade)
SLAM_JUMP_TH: float = 0.60              # Jump detection threshold (m)
SLAM_RMAX_EFF: float = LIDAR_RANGE - 0.15  # Effective max range (m)
SLAM_DECAY_PER_M: float = 0.12          # Distance decay coefficient
SLAM_NEIGH_CHECK: int = 2               # Neighborhood consistency radius (cells)
SLAM_OMEGA_HIGH_TH: float = 0.6         # High angular velocity threshold (rad/s)
SLAM_DECAY_PERIOD: int = 8              # Decay period (frames)
SLAM_DECAY_FACTOR: float = 0.995        # Decay factor

# Preprocessing parameters
DESKEW_ENABLE: bool = True              # Enable scan deskewing
SPIKE_JUMP_M: float = 1.0               # Spike detection threshold (m)
ROTATE_LIMIT_W: float = 0.35            # Max angular velocity limit (rad/s)
REPLAN_PERIOD: float = 2.0              # Replanning period (s)

# Map building guards (anti-erosion)
NEIGHBOR_CONSIST_TOL_M: float = 0.25    # Neighbor consistency tolerance (m)
TAIL_GAP_CELLS: int = 2                 # Free erosion tail gap (pixels)
CONFIRMED_OCC_TH: float = 2.0           # Confirmed obstacle threshold (log-odds)

# Odometry frame convention
ODOM_IN_WORLD: bool = True              # True if odom deltas are in world frame

# ================================
# 5. PATH PLANNING & NAVIGATION
# ================================
# Global planner (A*)

# Legacy spin parameters (duplicate removed, see lines 238-242 for active values)

PLANNER_INFLATE_RADIUS_M: float = 0.18  # Legacy parameter (use SAFE_BUFFER_M instead)

# Goal tolerances (state-dependent)
GOAL_TOLERANCES: dict = {
    'default': 0.08,        # Default: 8cm
    'exploration': 0.08,    # Exploration: 8cm (precise frontier reaching)
    'entry': 0.05,          # Entry: 5cm
    'exit': 0.30            # Exit: 30cm
}

# Goal timeout and stuck detection
GOAL_TIMEOUT_SECONDS: float = 40.0      # Goal reaching timeout (seconds)
GOAL_STUCK_DISTANCE: float = 0.1        # Stuck detection distance threshold (meters)
GOAL_STUCK_TIME: float = 40.0           # Stuck detection time threshold (seconds)

# Path tracking (arc-length based)
PATH_RESAMPLE_M: float = 0.02           # Path resampling step (meters)
LOOKAHEAD_BASE_M: float = 0.25          # Pure-Pursuit lookahead distance (meters) - increased from 0.15 for better preview
GOAL_GATE_AHEAD_M: float = 0.10         # Gate position before goal (meters)
GOAL_PROGRESS_EPS_M: float = 0.04       # Progress epsilon for arrival (meters)

# --- Corridor tolerance goal (band-shaped target) ---
# Enable band-shaped goal areas in corridors and at corners.
CORRIDOR_GOAL_ENABLE: bool = True
# Corridor feasible width = obstacle gap - 2*SAFE_BUFFER_M.
# If your maze pitch is 0.45m and SAFE_BUFFER_M=0.15 -> 0.15m band.
CORRIDOR_LANE_PITCH_M: float = 0.45
CORRIDOR_WIDTH_M: float = max(0.05, CORRIDOR_LANE_PITCH_M - 2*SAFE_BUFFER_M)
# Use exploration tolerance as X/Y half-span along the goal axis.
CORRIDOR_GOAL_TOL_M: float = GOAL_TOLERANCES.get('exploration', 0.10)
# Four-direction constraint: restrict headings to {0, ±pi/2, pi}.
CARDINAL_ONLY: bool = True
# Corner window half-size. Covers real robot size + tolerance.
CORNER_BOX_HALF_M: float = max(CORRIDOR_WIDTH_M * 0.5, CORRIDOR_GOAL_TOL_M)

# ================================
# 6. CONTROL & MOTION
# ================================
# Kinematic limits
V_MAX: float = 1.5              # Maximum linear velocity (m/s)
W_MAX: float = 1.2              # Maximum angular velocity (rad/s) - increased for faster rotation
ACC_MAX: float = 0.5            # Maximum linear acceleration (m/s²)
WACC_MAX: float = 1.5           # Maximum angular acceleration (rad/s²)
CONTROL_HZ: float = 20.0        # Control loop frequency (Hz)

# Motion Controller parameters (replaces DWA)
MOTION_CONTROLLER_YAW_ENTER: float = 0.087  # 5.0 degrees - rotation entry threshold (more lenient)
MOTION_CONTROLLER_YAW_EXIT: float = 0.035   # 2.0 degrees - rotation exit threshold (more lenient)
MOTION_CONTROLLER_W_SPIN: float = 2.0       # Rotation speed (rad/s) - increased for faster rotation
MOTION_CONTROLLER_V_MAX: float = 1.0        # Maximum linear speed (m/s) - increased for 2.8m maze
MOTION_CONTROLLER_WP_TOL: float = 1.5      # Waypoint tolerance (m)

# --- Unified path follower (segment-locked) ---
# English: gate tolerance to decide "corner line" crossing.
FOLLOWER_GATE_EPS_M: float = SLAM_RESOLUTION * 0.5
# English: ahead distance to probe in C-space before committing linear motion.
FOLLOWER_PROBE_AHEAD_M: float = 0.20
# English: lookahead along the current leg, clamped by remaining distance.
FOLLOWER_LOOKAHEAD_M: float = 0.25
# English: bypass smoothing while spinning; keep tiny smoothing for translation.
CONTROLLER_EMA_ALPHA: float = 0.15

# Legacy DWA parameters (for compatibility)
DWA_MIN_V: float = 0.05                     # Minimum linear velocity (m/s) - legacy parameter

# Planning costs
UNKNOWN_COST_ASTAR: float = 2.0 # Unknown area cost multiplier for A*
PLANNING_SAFETY_MARGIN: float = 0.1     # Safety margin for planning (m)
PLANNING_GRID_INFLATION: float = 0.0    # Legacy grid inflation (cells)

# ================================
# 7. EXPLORATION STRATEGY
# ================================
# Exploration completion criteria
EXPLORATION_MIN_TIME: float = 10.0              # Minimum exploration time (s)
EXPLORATION_MIN_COVERAGE_RATIO: float = 0.3     # Minimum coverage: 30%
EXPLORATION_MAX_COVERAGE_RATIO: float = 0.8     # Maximum coverage: 80%

# ================================
# 8. PERFORMANCE OPTIMIZATION
# ================================
# Frontier extraction throttling
FRONTIER_MIN_INTERVAL: float = 0.5     # Minimum interval between frontier extractions (s)
FRONTIER_ROI_MARGIN_M: float = 0.35    # ROI extra margin around robot (m)

# Replanning throttling
REPLAN_COOLDOWN: float = 0.6           # Replanning cooldown time (s)
MIN_PROGRESS_M: float = 0.10           # Minimum progress threshold for replanning (m)

# Spin-in-place parameters
SPIN_YAW_THRESHOLD: float = 0.5        # Yaw error threshold to start spinning (rad)
SPIN_STOP_YAW: float = 0.1             # Yaw error threshold to stop spinning (rad)
SPIN_W: float = 0.8                    # Spin angular velocity (rad/s)
SPIN_TIMEOUT_S: float = 3.0            # Spin timeout duration (s)

# Frontier detection parameters
MIN_FRONTIER_SIZE: int = 5              # Minimum frontier cluster size (cells)
MIN_FRONTIER_DISTANCE: float = 0.1      # Minimum distance to frontier (m)
MAX_FRONTIER_DISTANCE: float = 3.0     # Maximum distance to frontier (m) - temporarily increased to allow more frontiers

# === Corridor goal-band options (duplicate removed, see lines 171-182) ===

# Centerline strategy (corridor navigation)
CENTERLINE_TOLERANCE_M: float = 0.05            # Centerline tolerance: 5cm
CENTERLINE_PREFERENCE_WEIGHT: float = 2.0       # Centerline preference weight
MIN_CORRIDOR_WIDTH_M: float = 0.15              # Minimum corridor width: 15cm (45cm - 15cm - 15cm = 15cm可行区域)

# Frontier scoring weights
FRONTIER_DISTANCE_WEIGHT: float = 0.4   # Distance weight (lower priority)
FRONTIER_QUALITY_WEIGHT: float = 0.6    # Quality (centerline) weight (higher priority)

# Ring sampling fallback parameters moved to frontier_explorer.py for better organization

# Heading alignment (spin-in-place) - duplicate removed, see lines 238-242


# ================================
# 8. HIGH BAUDRATE OPTIMIZATION (921600)
# ================================
# High baudrate data processing optimization
HIGH_BAUDRATE_MODE: bool = True         # Enable high baudrate optimizations
HIGH_BAUDRATE_BAUD: int = 921600       # High baudrate threshold
HIGH_BAUDRATE_QUEUE_SIZE: int = 8192    # Increased queue size for high data rate
HIGH_BAUDRATE_TIMEOUT: float = 0.001   # Ultra reduced timeout for maximum processing speed
HIGH_BAUDRATE_BATCH_SIZE: int = 100    # Process data in larger batches
HIGH_BAUDRATE_MAPPING_DURATION: float = 8.0  # Reduced mapping duration for high baudrate
HIGH_BAUDRATE_GUI_UPDATE_INTERVAL: int = 500  # Update GUI every 500 points for ultra performance (only once per mapping)
HIGH_BAUDRATE_MEMORY_MONITOR: bool = False    # Disable memory monitoring for maximum performance
# HIGH_BAUDRATE_DISABLE_GUI: bool = True        # Removed: Keep GUI functionality enabled
HIGH_BAUDRATE_MAX_MEMORY_MB: int = 500        # Maximum memory usage (MB)

# ================================
# 9. GUI & LOGGING
# ================================
# GUI display parameters
GUI_UPDATE_RATE_HZ: float = 10.0        # GUI update frequency (Hz)
GUI_PANEL_COUNT: int = 3                # Number of display panels
GUI_SLAM_XLIM = (0.0, WORLD_SIZE)       # SLAM panel X-axis limits (m) - 动态计算
GUI_SLAM_YLIM = (0.0, WORLD_SIZE)       # SLAM panel Y-axis limits (m) - 动态计算

# Logging configuration
LOG_LEVEL: str = "INFO"                 # Log level (DEBUG, INFO, WARNING, ERROR)
LOG_MAX_SIZE_MB: int = 100              # Maximum log file size (MB)
LOG_BACKUP_COUNT: int = 5               # Number of backup log files

# ================================
# NOTES & DEPRECATION WARNINGS
# ================================
# - Coverage map functionality has been removed (not needed for core exploration)
# - Smart entry detection parameters removed (entry position is known from JSON)
# - Multi-sensor fusion (IMU, loop closure) disabled for current version
# - Some legacy parameters retained for backward compatibility but may be removed in future

# ===== Axis-primitive controller knobs (tunable) =====
# —— primitives / controller params ——
# TURN 完成容差（deg）
PRIM_TURN_TOL_DEG: float = 8.0            # TURN completion tolerance
# MOVE 直行允许的航向误差上限（deg）= θ_move
PRIM_MOVE_YAW_TOL_DEG: float = 6.0        # MOVE fast if |yaw_err| below this
# GATE 原语沿轴前进的航向误差上限（deg），默认与 MOVE 一致
PRIM_GATE_YAW_TOL_DEG: float = PRIM_MOVE_YAW_TOL_DEG
# Gains
PRIM_TURN_KP: float = 1.0                 # rad/s per rad for TURN
PRIM_MOVE_KP: float = 0.8                 # rad/s per rad for MOVE/GATE steering
# TURN fixed angular velocity (rad/s) - fast rotation
PRIM_TURN_W_FIXED: float = 0.6            # fixed max spin speed for fast rotation
# Speeds (m/s)
PRIM_MOVE_V_FAST: float = 0.25
PRIM_MOVE_V_SLOW: float = 0.12
PRIM_GATE_V_FAST: float = 0.22
PRIM_GATE_V_SLOW: float = 0.10
# Gate crossing tolerance (meters)
PRIM_GATE_EPS_M: float = 0.03  # ≈ 3 * SLAM_RES(0.01m), more robust to quantization

# Goal band arrival tolerance (meters) - v3.2 optimization
GOAL_BAND_BOUNDARY_TOLERANCE_M: float = 0.02  # 2cm tolerance at goal band boundaries for arrival detection

# Gate parameters for start/end point accessibility
GATE_SIZE_M: float = 0.40        # 40cm × 40cm gate size
GATE_RADIUS_M: float = GATE_SIZE_M / 2.0  # 15cm radius for circular gate

# Simple recovery strategy parameters - v3.3 optimization
FRONTIER_FAILURE_THRESHOLD: int = 7          # Number of consecutive frontier failures before triggering recovery
SIMPLE_RETREAT_DISTANCE_M: float = 0.1       # Fixed retreat distance: 0.1m away from nearest obstacle

# ===== Visual gate line =====
VIS_DRAW_GATE: bool = True
VIS_GATE_COLOR: str = "orange"
VIS_GATE_LINEWIDTH: float = 2.0
VIS_GATE_ALPHA: float = 0.95

# ================================
# TARGET REACHABILITY PARAMETERS
# ================================
TARGET_REACHABLE_REGION_SIZE_M: float = 0.40  # Target reachable region size (40cm)
BAND_GUARD_TICKS: int = 5                     # Guard ticks after state switch
MANHATTAN_PATH_ENABLED: bool = True           # Enable Manhattan path planning

#硬件参数！！


# ================================
# RING PRIMARY EXPLORATION PARAMETERS
# ================================
# Ring sampling as primary exploration strategy
USE_RING_PRIMARY: bool = True                 # Use ring sampling as primary strategy
RING_RADIUS_M: float = 0.40                  # Ring sampling radius (meters)
RING_K: int = 108                             # Number of angle samples
RING_AXIS_PREF_W: float = 0.08               # Axis preference weight for alignment
RING_MIN_FWD_DOT: float = 0.20               # Minimum forward dot product (cos(~78°) = 0.2)

# Ring sampling directional control parameters
FWD_SUPPRESS_CLEARANCE_M: float = 0.30       # Forward suppression when clearance < this value
FWD_SUPPRESS_CONE_DEG: float = 25            # Forward suppression cone angle (±deg)
SIDE_TURN_MIN_DELTA_DEG: float = 35          # Side turn minimum angle threshold
SIDE_TURN_BONUS: float = 0.15                # Side turn bonus score

# ================================
# MOTION DECOMPOSITION PARAMETERS
# ================================
# Motion primitive decomposition limits
MAX_MOVE_CHUNK_M: float = 0.35               # Maximum move chunk size (meters)
MAX_TURN_CHUNK_DEG: float = 90.0             # Maximum turn chunk size (degrees)
MAP_AFTER_CHUNK_S: float = 15.0              # Mapping duration after each chunk (seconds)

# ================================
# NAVIGATION TIMING PARAMETERS
# ================================
# Navigation control timing
DEFAULT_MAX_SEG_S: float = 15.0              # Default maximum segment execution time
DEFAULT_MAPPING_DUR_S: float = 15.0          # Default mapping duration
INTERMEDIATE_MAPPING_S: float = 15.0         # Intermediate mapping duration
COMMAND_TIMEOUT_S: float = 12.0              # Command execution timeout
CONTROL_DT_S: float = 0.05                   # Control loop time step

# ================================
# POINT-BASED MAPPING PARAMETERS
# ================================
# Point-based mapping control
MAIN_MAPPING_POINTS: int = 500               # Main loop mapping target points
POST_SEGMENT_MAPPING_POINTS: int = 300       # Post-segment mapping target points
DEFAULT_MAPPING_POINTS: int = 500            # Default mapping points (fallback)

# ================================
# BOUNDARY CONSTRAINT PARAMETERS
# ================================
# Boundary-aware frontier selection
BOUNDARY_PENALTY_ENABLED: bool = True        # Enable boundary penalty for frontier selection
BOUNDARY_SAFE_DISTANCE: float = 0.25         # Safe distance from boundary (25cm)
BOUNDARY_PENALTY_SEVERE: float = -1000.0     # Severe penalty for very close to boundary
BOUNDARY_PENALTY_MEDIUM: float = -500.0      # Medium penalty for close to boundary
BOUNDARY_PENALTY_LIGHT: float = -100.0       # Light penalty for near boundary

# ================================
# SINGLE-THREAD BATCH PROCESSING PARAMETERS
# ================================
# Single-thread batch processing optimization
SINGLE_THREAD_BATCH_SIZE: int = 300          # Batch size for single-thread processing (ultra performance)
SINGLE_THREAD_EMPTY_BATCH_LIMIT: int = 50   # Max empty batches before considering data end (increased from 10)
SINGLE_THREAD_PREFILTER_ENABLED: bool = False # Disable prefiltering temporarily for debugging
SINGLE_THREAD_IMPORTANT_PREFIXES: tuple = ("LIDAR", "Angle:", "ODOM", "Odom", "ODOMETRY")  # Important line prefixes
