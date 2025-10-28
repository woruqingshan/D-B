# ================================
# file: code/gui/visualizer.py
# ================================
from __future__ import annotations
from typing import Sequence, Tuple, Optional, List
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Configure Chinese font support
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans', 'Arial Unicode MS', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False  # Fix minus sign display issue

from core import Pose2D
from core.config import GATE_RADIUS_M

# Backend selection
backends_to_try = ["TkAgg", "Qt5Agg", "QtAgg", "MacOSX"]
for backend in backends_to_try:
    try:
        matplotlib.use(backend, force=True)
        print(f"✅ 使用matplotlib后端: {backend}")
        break
    except Exception as e:
        print(f"❌ 后端 {backend} 不可用: {e}")
else:
    try:
        matplotlib.use("Agg", force=True)
        print("✅ 使用非交互式后端: Agg")
    except Exception as e:
        print(f"❌ 设置后端失败: {e}")

from core.config import (
    SLAM_MAP_SIZE_METERS, WORLD_SIZE, LIDAR_RANGE, GUI_SLAM_XLIM, GUI_SLAM_YLIM,
    VIS_DRAW_GATE, VIS_GATE_COLOR, VIS_GATE_LINEWIDTH, VIS_GATE_ALPHA
)

class Visualizer:
    """Matplotlib-based 3-pane viewer."""
    
    def set_goal_band(self, rect: Optional[Tuple[float,float,float,float]]) -> None:
        """Update corridor goal-band rectangle (xmin,xmax,ymin,ymax) in world meters."""
        self._band_rect = rect

    def set_cspace(self, cspace_bin, resolution_m: float):
        """Set SAFE_BUFFER-only C-space for visualization (0=free,1=occ)."""
        self._cspace_img = cspace_bin
        self._cspace_res = resolution_m

    # ---- ADD: world extent helper (centered at robot start) ----
    def _world_extent_from_start(self):
        """计算世界坐标显示范围，使用全局原点"""
        from core.config import SLAM_MAP_SIZE_METERS
        
        # 使用全局原点，显示整个SLAM地图范围
        return [0.0, SLAM_MAP_SIZE_METERS, 0.0, SLAM_MAP_SIZE_METERS]
    
    def _initialize_astar_panel(self, maze_map):
        """Initialize A* panel with C-space obstacles and plan static path"""
        from core.config import SLAM_MAP_SIZE_PIXELS, SLAM_RESOLUTION
        from scipy.ndimage import binary_dilation
        from planning import AStarPlanner
        import json
        
        # Get maze grid (200x200, same as SLAM)
        maze_grid = maze_map.grid  # 0=free, 1=obstacle, 2=unknown
        
        # Create C-space by inflating obstacles
        # Get buffer from JSON (default 0.18m)
        from core.config import SAFE_BUFFER_M
        buffer_m = SAFE_BUFFER_M  # 使用0.08而不是0.18
        buffer_pixels = int(buffer_m / SLAM_RESOLUTION)
        
        # Create obstacle mask
        obstacle_mask = (maze_grid == 1)
        
        # Dilate obstacles by buffer
        from scipy.ndimage import maximum_filter1d
        if buffer_pixels > 0:
            k = 2 * buffer_pixels + 1
            tmp = maximum_filter1d(obstacle_mask.astype(np.uint8), size=k, axis=0, mode="nearest")
            dilated_obstacle = maximum_filter1d(tmp, size=k, axis=1, mode="nearest") > 0
        else:
            dilated_obstacle = obstacle_mask.copy()
        
        # Create visualization image
        # 0=free(white), 1=buffer(light gray with ×)
        cspace_img = np.zeros(maze_grid.shape, dtype=np.uint8)
        cspace_img[:] = 255  # Start with white (free)
        
        # 只显示缓冲区，不显示原始障碍物
        buffer_zone = dilated_obstacle & (~obstacle_mask)  # Buffer but not original obstacle
        cspace_img[buffer_zone] = 180  # Light gray for buffer
        
        # Display C-space
        extent = [0.0, WORLD_SIZE, 0.0, WORLD_SIZE]
        self.astar_image = self.axA.imshow(
            cspace_img, cmap='gray', origin='lower', 
            vmin=0, vmax=255, extent=extent, alpha=0.8
        )
        
        # Mark buffer zone with × pattern
        self._buffer_markers = []
        gy_coords, gx_coords = np.where(buffer_zone)
        # Sample every 5th pixel to avoid cluttering
        for i in range(0, len(gy_coords), 5):
            gy, gx = gy_coords[i], gx_coords[i]
            wx = gx * SLAM_RESOLUTION
            wy = gy * SLAM_RESOLUTION
            marker, = self.axA.plot(wx, wy, 'kx', ms=2, alpha=0.3)
            self._buffer_markers.append(marker)
        
        # Store for later use
        self._astar_cspace_grid = dilated_obstacle.astype(np.uint8)
        
        # Use coordinates provided by MazeMap instead of any hard-coded JSON.
        # English: Visualizer must not own data loading. Consume from MazeMap.
        # entry / exit are guaranteed to be inside the legal 1.8×1.8 area.
        try:
            start_world_x, start_world_y = getattr(maze_map, "maze_entry_world", (0.1, 0.1))
            goal_world_x,  goal_world_y  = getattr(maze_map, "maze_exit_world",
                                                   (maze_map.maze_bounds['top_right'][0]-SLAM_RESOLUTION,
                                                    maze_map.maze_bounds['top_right'][1]-SLAM_RESOLUTION))

            # Convert to grid (row=i=y, col=j=x)
            start_grid = (int(round(start_world_y / SLAM_RESOLUTION)), int(round(start_world_x / SLAM_RESOLUTION)))
            goal_grid  = (int(round(goal_world_y  / SLAM_RESOLUTION)), int(round(goal_world_x  / SLAM_RESOLUTION)))
            print(f"[A* Panel] 起点(世界→网格): ({start_world_x:.2f},{start_world_y:.2f}) -> {start_grid}")
            print(f"[A* Panel] 终点(世界→网格): ({goal_world_x:.2f},{goal_world_y:.2f}) -> {goal_grid}")
            
            # Create A* planner and plan path on ground truth map
            planner = AStarPlanner()
            
            # Use ground truth map for planning (not C-space)
            # Convert maze_grid to A* format: 0=free, 1=obstacle, 2=unknown
            planning_grid = maze_grid.copy()
            
            # Plan path with strict Manhattan distance (no diagonal moves)
            path = planner.plan(start_grid, goal_grid, planning_grid, grid_is_cspace=False)
            
            if path:
                print(f"[A* Panel] 成功规划路径，共{len(path)}个点")
                # Store path for visualization
                self._static_astar_path = path
                self._static_start = (start_world_x, start_world_y)
                self._static_goal = (goal_world_x, goal_world_y)
            else:
                print(f"[A* Panel] 路径规划失败")
                self._static_astar_path = None
                self._static_start = (start_world_x, start_world_y)
                self._static_goal = (goal_world_x, goal_world_y)
                
        except Exception as e:
            print(f"[A* Panel] 初始化路径规划失败: {e}")
            import traceback
            traceback.print_exc()
            self._static_astar_path = None
            self._static_start = None
            self._static_goal = None
        
        self._astar_initialized = True
        
        # Add legend
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], color='none', marker='s', markerfacecolor='lightgray', 
                   markersize=8, label='Safe Buffer'),
            Line2D([0], [0], color='b', linewidth=3, label='规划路径'),
            Line2D([0], [0], marker='o', color='r', linestyle='None', 
                   markersize=8, label='起点'),
            Line2D([0], [0], marker='*', color='g', linestyle='None', 
                   markersize=10, label='终点')
        ]
        self.axA.legend(handles=legend_elements, loc='upper right', fontsize=7)
        
        # Store data - will be drawn after plot objects are created
        # Drawing is done in __init__ after all plot objects exist
    
    def __init__(self, maze_map=None, logger_func=None, log_file=None) -> None:
        self.maze_map = maze_map
        self.logger_func = logger_func
        self.log_file = log_file
        H, W = (maze_map.grid.shape if maze_map is not None else (25, 25))

        # CoverageMap initialization removed - CoverageMap disabled
        self.slam_grid_shape = (H, W)

        # Create 4 panels in 2x2 layout: Ground Truth, SLAM Map, Scan Clouds, A* Planning
        self.fig, ((self.axL, self.axM), (self.axR, self.axA)) = plt.subplots(2, 2, figsize=(15, 12))
        plt.ion()

        # Left: Ground truth
        if maze_map is not None:
            gt_img = np.zeros_like(maze_map.grid, dtype=np.uint8)
            gt_img[maze_map.grid == 0] = 255  # free
            gt_img[maze_map.grid == 1] = 128  # occ
            gt_img[maze_map.grid == 2] = 0    # unknown
            self.imL = self.axL.imshow(
                gt_img, cmap="gray", origin="lower", vmin=0, vmax=255,
                extent=[0, WORLD_SIZE, 0, WORLD_SIZE]
            )
            title = "Ground Truth"
            if hasattr(maze_map, 'maze_bounds') and maze_map.maze_bounds:
                b = maze_map.maze_bounds
                title = f"Ground Truth - Maze {b['width']}x{b['height']}"
                if hasattr(maze_map, 'maze_entry_world'):
                    e = maze_map.maze_entry_world
                    title += f"\nEntry: ({e[0]:.1f}, {e[1]:.1f})"
            self.axL.set_title(title, fontsize=12, fontweight='bold')
        else:
            self.imL = self.axL.imshow(np.zeros((H, W), dtype=np.uint8), cmap="gray", origin="lower")
            self.axL.set_title("Ground Truth", fontsize=12, fontweight='bold')

        # 以起点为中心显示整幅 SLAM 地图（extent 由起点与地图物理边长决定）
        extent_world = [GUI_SLAM_XLIM[0], GUI_SLAM_XLIM[1], GUI_SLAM_YLIM[0], GUI_SLAM_YLIM[1]]
        
        # Debug: Print extent info
        print(f"SLAM地图显示范围:")
        print(f"  - 世界范围: {extent_world}")
        print(f"  - 地图尺寸: {H}x{W}")

        self.imM = self.axM.imshow(
            np.zeros((H, W), dtype=np.uint8),
            cmap="gray", origin="lower", vmin=0, vmax=255,
            extent=extent_world
        )
        self.axM.set_title("SLAM Map", fontsize=12, fontweight='bold')

        # 设置坐标轴范围，与左图一致
        self.axR.set_xlim(0.0, WORLD_SIZE)
        self.axR.set_ylim(0.0, WORLD_SIZE)
        self.axR.set_aspect('equal')


        # Right: Scan clouds comparison (prev=gray, curr=blue, xformed prev=green)
        self.axR.set_title("Scan Clouds (prev=gray, curr=blue, xform=green)", fontsize=12, fontweight='bold')

        # scatter artists
        self.sc_prev = self.axR.scatter([], [], s=6, c='#bbbbbb', marker='.', label='prev')
        self.sc_curr = self.axR.scatter([], [], s=8, c='C0', marker='.', label='curr')
        self.sc_xpre = self.axR.scatter([], [], s=10, c='C2', marker='.', label='xformed prev')
        self.axR.legend(loc='upper right', fontsize=8)
        # buffers for previous frame
        self._prev_scan_world = None     # np.ndarray (N,2)
        self._prev_pose_for_scan = None  # Pose2D
        
        # Fourth panel: A* C-space and path planning
        self.axA.set_title("A* C-space & Path Planning", fontsize=12, fontweight='bold')
        self.axA.set_xlim(0.0, WORLD_SIZE)
        self.axA.set_ylim(0.0, WORLD_SIZE)
        self.axA.set_aspect('equal')
        
        # A* panel visualization elements
        self.astar_image = None  # Will be initialized when maze info is available
        self._maze_info = None
        self._astar_initialized = False
        self._astar_cspace_grid = None  # C-space obstacles with buffer
        
        # Don't initialize A* panel yet - need to create plot objects first

        # Overlays
        (self.pL,) = self.axL.plot([], [], "b--", lw=2, alpha=0.8)
        (self.pM,) = self.axM.plot([], [], "b--", lw=2, alpha=0.8)
        (self.pR,) = self.axR.plot([], [], "b--", lw=2, alpha=0.8)
        (self.pA,) = self.axA.plot([], [], "b-", lw=3, alpha=0.9, label='Path')  # A* path

        (self.gL,) = self.axL.plot([], [], "go", ms=6, alpha=0.8)
        (self.gM,) = self.axM.plot([], [], "go", ms=6, alpha=0.8)
        (self.gR,) = self.axR.plot([], [], "go", ms=6, alpha=0.8)
        (self.gA,) = self.axA.plot([], [], "g*", ms=12, alpha=0.9, label='Goal')  # A* goal


        self.lL, = self.axL.plot([], [], "rx", ms=6, alpha=0.8)
        self.lM, = self.axM.plot([], [], "rx", ms=6, alpha=0.8)
        self.lR, = self.axR.plot([], [], "rx", ms=6, alpha=0.8)
        (self.sA,) = self.axA.plot([], [], "ro", ms=10, alpha=0.9, label='Start')  # A* start


        self.rL = self.axL.quiver([], [], [], [], color="r", scale=20, width=0.003)
        self.rM = self.axM.quiver([], [], [], [], color="r", scale=20, width=0.003)
        self.rR = self.axR.quiver([], [], [], [], color="r", scale=20, width=0.003)
        self.rA = self.axA.quiver([], [], [], [], color="r", scale=20, width=0.003)  # Robot on A* panel


        self.rL_dot, = self.axL.plot([], [], 'ro', markersize=4, zorder=10)
        self.rM_dot, = self.axM.plot([], [], 'ro', markersize=4, zorder=10)
        self.rR_dot, = self.axR.plot([], [], 'ro', markersize=4, zorder=10)
        self.rA_dot, = self.axA.plot([], [], 'ro', markersize=4, zorder=10)


        # SLAM pose cursors (blue color to distinguish from real robot pose)
        self.slamL = self.axL.quiver([], [], [], [], color="b", scale=20, width=0.003, alpha=0.7)
        self.slamM = self.axM.quiver([], [], [], [], color="b", scale=20, width=0.003, alpha=0.7)
        self.slamR = self.axR.quiver([], [], [], [], color="b", scale=20, width=0.003, alpha=0.7)
 

        self.slamL_dot, = self.axL.plot([], [], 'bo', markersize=3, zorder=9, alpha=0.7)
        self.slamM_dot, = self.axM.plot([], [], 'bo', markersize=3, zorder=9, alpha=0.7)
        self.slamR_dot, = self.axR.plot([], [], 'bo', markersize=3, zorder=9, alpha=0.7)


        self.radar_circle_L = None
        self.radar_circle_M = None
        self.radar_circle_R = None  # Initialize for right panel (Scan Clouds)


        for ax in (self.axL, self.axM, self.axR, self.axA):
            ax.set_xlim(GUI_SLAM_XLIM)
            ax.set_ylim(GUI_SLAM_YLIM)

        # Add legend for pose indicators
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], marker='o', color='r', linestyle='None', markersize=6, label='真实机器人位姿'),
            Line2D([0], [0], marker='o', color='b', linestyle='None', markersize=4, alpha=0.7, label='SLAM估计位姿'),
            Line2D([0], [0], marker='o', color='g', linestyle='None', markersize=6, label='目标点'),
            Line2D([0], [0], color='b', linestyle='--', linewidth=2, alpha=0.8, label='路径')
        ]
        self.axM.legend(handles=legend_elements, loc='upper right', fontsize=8)

        # Now initialize A* panel after all plot objects are created
        if maze_map is not None:
            self._initialize_astar_panel(maze_map)
            
            # Draw static path if available
            if hasattr(self, '_static_astar_path') and self._static_astar_path:
                path_x = [p[0] for p in self._static_astar_path]
                path_y = [p[1] for p in self._static_astar_path]
                self.pA.set_data(path_x, path_y)
            
            # Draw start and goal points
            if hasattr(self, '_static_start') and self._static_start:
                self.sA.set_data([self._static_start[0]], [self._static_start[1]])
            if hasattr(self, '_static_goal') and self._static_goal:
                self.gA.set_data([self._static_goal[0]], [self._static_goal[1]])

        # Corridor goal-band overlay
        self._band_rect = None  # (xmin,xmax,ymin,ymax)
        self._band_patch_M = None  # rectangle on SLAM panel
        self._band_patch_L = None  # rectangle on Ground Truth panel
        # NEW: gate lines
        self._gate_M = None
        self._gate_L = None
        self._gate_A = None
        
        # --- NEW: SAFE_BUFFER C-space overlay on planning panel ---
        self._cspace_img = None   # numpy 2D array (0=free,1=occ)
        self._cspace_imshow = None
        self._cspace_res = None
        self._cspace_key = None  # cache key for occ_grid

        plt.tight_layout()
        try:
            plt.show(block=False)
            print("图形界面已启动")
        except Exception as e:
            print(f"显示图形界面时出现问题: {e}")

    def _scan_to_world(self, pose: Pose2D, scan):
        """Convert (ranges, ang_min, ang_inc) to Nx2 world points, clip by LIDAR_RANGE."""
        if scan is None:
            return np.empty((0, 2), dtype=np.float32)
        try:
            ranges, ang_min, ang_inc = scan
        except Exception:
            return np.empty((0, 2), dtype=np.float32)
        r = np.asarray(ranges, dtype=np.float32)
        if r.size == 0:
            return np.empty((0, 2), dtype=np.float32)

        from core.config import LIDAR_RANGE
        # clamp/valid mask
        valid = np.isfinite(r) & (r > 0.0) & (r < (LIDAR_RANGE - 1e-6))
        if not np.any(valid):
            return np.empty((0, 2), dtype=np.float32)
        idx = np.nonzero(valid)[0]
        ang = ang_min + ang_inc * idx.astype(np.float32)
        # body -> world
        c, s = math.cos(pose.theta), math.sin(pose.theta)
        xb = r[valid] * np.cos(ang)
        yb = r[valid] * np.sin(ang)
        xw = c * xb - s * yb + pose.x
        yw = s * xb + c * yb + pose.y
        return np.stack([xw, yw], axis=1).astype(np.float32)

    def _update_right_scans(self, pose: Pose2D, scan):
        """Update right pane scatter: prev=gray, curr=blue, xformed prev=green."""
        # curr in world
        curr_w = self._scan_to_world(pose, scan)

        # transform previous scan to current world using odom delta (pose diff)
        xpre_w = np.empty((0, 2), dtype=np.float32)
        if self._prev_scan_world is not None and self._prev_pose_for_scan is not None:
            dx = pose.x - self._prev_pose_for_scan.x
            dy = pose.y - self._prev_pose_for_scan.y
            dth = pose.theta - self._prev_pose_for_scan.theta
            c, s = math.cos(dth), math.sin(dth)
            pw = self._prev_scan_world
            # world->world via relative SE2
            xpre_x = c * pw[:, 0] - s * pw[:, 1] + dx
            xpre_y = s * pw[:, 0] + c * pw[:, 1] + dy
            xpre_w = np.stack([xpre_x, xpre_y], axis=1).astype(np.float32)

        # update scatters
        self.sc_prev.set_offsets(self._prev_scan_world if self._prev_scan_world is not None else np.empty((0, 2)))
        self.sc_curr.set_offsets(curr_w)
        self.sc_xpre.set_offsets(xpre_w)

        # ring
        try:
            if self.radar_circle_R: 
                self.radar_circle_R.remove()
        except Exception:
            pass
        from core.config import LIDAR_RANGE
        angs = np.linspace(-135, 135, 64) * math.pi / 180.0
        arc_x = pose.x + LIDAR_RANGE * np.cos(angs + pose.theta)
        arc_y = pose.y + LIDAR_RANGE * np.sin(angs + pose.theta)
        (self.radar_circle_R,) = self.axR.plot(arc_x, arc_y, 'b-', alpha=0.25, linewidth=1)

        # persist for next frame
        self._prev_scan_world = curr_w
        self._prev_pose_for_scan = Pose2D(pose.x, pose.y, pose.theta)

    def draw_gate_circles(self, start_xy: tuple, exit_xy: tuple):
        """绘制起点和终点的闸门圆形区域"""
        import matplotlib.patches as patches
        
        # 清除之前的闸门圆圈
        for patch in self.axA.patches[:]:
            if hasattr(patch, '_is_gate_circle'):
                patch.remove()
        
        if start_xy is not None:
            # 起点闸门 - 蓝色虚线圆
            start_circle = patches.Circle(start_xy, GATE_RADIUS_M, 
                                         linewidth=2, edgecolor='blue', 
                                         facecolor='none', linestyle='--', alpha=0.8)
            start_circle._is_gate_circle = True
            self.axA.add_patch(start_circle)
        
        if exit_xy is not None:
            # 终点闸门 - 红色虚线圆
            exit_circle = patches.Circle(exit_xy, GATE_RADIUS_M, 
                                        linewidth=2, edgecolor='red', 
                                        facecolor='none', linestyle='--', alpha=0.8)
            exit_circle._is_gate_circle = True
            self.axA.add_patch(exit_circle)
        
        # 更新图例（如果还没有的话）
        if not hasattr(self, '_gate_legend_added'):
            self.axA.plot([], [], 'b--', linewidth=2, label='起点闸门')
            self.axA.plot([], [], 'r--', linewidth=2, label='终点闸门')
            self._gate_legend_added = True

    def update(self, pose: Pose2D, occ_grid, path: Sequence[Tuple[float, float]],
               local_goal: Optional[Tuple[float, float]],
               global_goal: Optional[Tuple[float, float]], scan=None, slam_pose: Optional[Pose2D] = None,
               start_xy: Optional[Tuple[float, float]] = None, exit_xy: Optional[Tuple[float, float]] = None,
               nav_state: str = None, target_region: Optional[Tuple[float,float,float,float]] = None) -> None:
        """Update visualization with world-coordinate overlays."""
        try:
            # === Phase 1 Optimization: Fast grayscale conversion ===
            # English: Direct uint8 mapping without multiple np.where calls
            img = np.full(occ_grid.shape, 0, dtype=np.uint8)  # Start with unknown (black)
            img[occ_grid == 0] = 255  # 自由空间 (white)
            img[occ_grid == 1] = 128  # 障碍物 (gray)
            # occ_grid == 2 stays 0 (black for unknown)
            
            # === Phase 1 Optimization: Reduce debug output frequency ===
            # English: Only print every 100 frames instead of 50 to reduce I/O overhead
            if not hasattr(self, '_update_count'):
                self._update_count = 0
            self._update_count += 1
                
            if self._update_count % 100 == 1:  # Reduced from 50 to 100
                print(f"GUI更新调试 (第{self._update_count}次):")
                print(f"  - 位姿: ({pose.x:.3f}, {pose.y:.3f}, {pose.theta:.3f})")
                print(f"  - 地图尺寸: {occ_grid.shape}")
                print(f"  - 自由空间: {np.sum(occ_grid == 0)}")
                print(f"  - 障碍物: {np.sum(occ_grid == 1)}")
                print(f"  - 未知区域: {np.sum(occ_grid == 2)}")
            
            # === Phase 1 Optimization: Use set_data() for existing Image object ===
            # English: Reuse existing Image object instead of recreating it
            # This is 10-20x faster than imshow()
            self.imM.set_data(img)
            # Note: extent is already set during initialization, no need to reset unless changed
            
            
            # --- Keep SAFE_BUFFER C-space overlay fresh even if producer didn't push it ---
            try:
                from core.config import SAFE_BUFFER_M, SLAM_RESOLUTION
                # Cheap change detector on occ_grid
                key = (occ_grid.shape,
                       int((occ_grid==0).sum()), int((occ_grid==1).sum()), int((occ_grid==2).sum()))
                if self._cspace_key != key:
                    # Inflate only known obstacles, then OR with unknown
                    from scipy.ndimage import maximum_filter1d
                    inflate_cells = int(math.ceil(SAFE_BUFFER_M / SLAM_RESOLUTION))
                    obst = (occ_grid == 1).astype(np.uint8)
                    unk  = (occ_grid == 2).astype(np.uint8)
                    if inflate_cells > 0:
                        k = 2 * inflate_cells + 1
                        tmp = maximum_filter1d(obst, size=k, axis=0, mode="nearest")
                        inflated = maximum_filter1d(tmp, size=k, axis=1, mode="nearest")
                    else:
                        inflated = obst
                    self._cspace_img = ((inflated > 0).astype(np.uint8) | unk).astype(np.uint8)
                    self._cspace_res = SLAM_RESOLUTION
                    self._cspace_key = key
            except Exception:
                pass

            # Draw goal-band rectangle overlays if provided
            try:
                # remove old patches
                if self._band_patch_M is not None:
                    self._band_patch_M.remove(); self._band_patch_M = None
                if self._band_patch_L is not None:
                    self._band_patch_L.remove(); self._band_patch_L = None
                if self._gate_M is not None:
                    self._gate_M.remove(); self._gate_M = None
                if self._gate_L is not None:
                    self._gate_L.remove(); self._gate_L = None
                if self._gate_A is not None:
                    self._gate_A.remove(); self._gate_A = None
                
                # Remove old target region patches
                if hasattr(self, '_target_region_patch_M') and self._target_region_patch_M is not None:
                    self._target_region_patch_M.remove(); self._target_region_patch_M = None
                if hasattr(self, '_target_region_patch_A') and self._target_region_patch_A is not None:
                    self._target_region_patch_A.remove(); self._target_region_patch_A = None
                if getattr(self, "_band_rect", None):
                    xmin,xmax,ymin,ymax = self._band_rect
                    import matplotlib.patches as patches
                    self._band_patch_M = patches.Rectangle((xmin,ymin), xmax-xmin, ymax-ymin,
                                                           linewidth=1.5, edgecolor='orange', facecolor='none', linestyle='--', alpha=0.9)
                    self._band_patch_L = patches.Rectangle((xmin,ymin), xmax-xmin, ymax-ymin,
                                                           linewidth=1.5, edgecolor='orange', facecolor='none', linestyle='--', alpha=0.9)
                    self.axM.add_patch(self._band_patch_M)
                    self.axL.add_patch(self._band_patch_L)
                    # NEW: draw the gate line (left edge for horizontal, bottom edge for vertical)
                    if VIS_DRAW_GATE:
                        is_horizontal = (xmax - xmin) >= (ymax - ymin)
                        if is_horizontal:
                            xs, ys = [xmin, xmin], [ymin, ymax]
                        else:
                            xs, ys = [xmin, xmax], [ymin, ymin]
                        (self._gate_M,) = self.axM.plot(xs, ys, color=VIS_GATE_COLOR,
                                                        linewidth=VIS_GATE_LINEWIDTH, alpha=VIS_GATE_ALPHA)
                        (self._gate_L,) = self.axL.plot(xs, ys, color=VIS_GATE_COLOR,
                                                        linewidth=VIS_GATE_LINEWIDTH, alpha=VIS_GATE_ALPHA)
                        (self._gate_A,) = self.axA.plot(xs, ys, color=VIS_GATE_COLOR,
                                                        linewidth=VIS_GATE_LINEWIDTH, alpha=VIS_GATE_ALPHA)
                
                # Draw target reachable region if provided
                if target_region is not None:
                    self._draw_target_reachable_region(target_region)
                # SAFE_BUFFER C-space overlay on planning panel (if provided)
                if self._cspace_img is not None and self._cspace_res is not None:
                    import numpy as _np
                    # draw as light gray mask; origin at (0,0)
                    if self._cspace_imshow is not None:
                        self._cspace_imshow.remove(); self._cspace_imshow = None
                    extent = (0.0, self._cspace_img.shape[1]*self._cspace_res,
                              0.0, self._cspace_img.shape[0]*self._cspace_res)
                    self._cspace_imshow = self.axA.imshow(_np.where(self._cspace_img>0, 0.7, _np.nan),
                                                          origin='lower', extent=extent, alpha=0.25, cmap='gray')
            except Exception:
                pass

            # Coverage update removed - no impact on core logic

            # Paths and goals are already in WORLD coordinates -> plot directly
            if path:
                xs = [x for (x, _) in path]
                ys = [y for (_, y) in path]
                self.pL.set_data(xs, ys)
                self.pM.set_data(xs, ys)
                self.pR.set_data(xs, ys)
                # Don't update pA - keep static path from initialization
            else:
                self.pL.set_data([], []); self.pM.set_data([], []); self.pR.set_data([], [])
                # Don't clear pA - keep static path

            if global_goal is not None:
                gx, gy = global_goal
                self.gL.set_data([gx], [gy])
                self.gM.set_data([gx], [gy])
                # Keep A* panel in sync with current global goal
                self.gA.set_data([gx], [gy])
            else:
                self.gL.set_data([], []); self.gM.set_data([], [])
                # do not clear gA when goal is None 

            # Robot pose in WORLD coordinates (red - real robot)
            rx, ry = pose.x, pose.y
            dx = math.cos(pose.theta) * 0.5
            dy = math.sin(pose.theta) * 0.5
            self.rL.set_offsets([[rx, ry]]); self.rL.set_UVC([dx], [dy])
            self.rM.set_offsets([[rx, ry]]); self.rM.set_UVC([dx], [dy])
            self.rA.set_offsets([[rx, ry]]); self.rA.set_UVC([dx], [dy])  # robot arrow on A* panel

            self.rL_dot.set_data([rx], [ry])
            self.rM_dot.set_data([rx], [ry])
            self.rA_dot.set_data([rx], [ry])  # robot dot on A* panel
            # Also show current start marker as the robot position on A* panel
            self.sA.set_data([rx], [ry])

            # SLAM pose in WORLD coordinates (blue - SLAM estimate)
            if slam_pose is not None:
                sx, sy = slam_pose.x, slam_pose.y
                sdx = math.cos(slam_pose.theta) * 0.5
                sdy = math.sin(slam_pose.theta) * 0.5
                self.slamL.set_offsets([[sx, sy]]); self.slamL.set_UVC([sdx], [sdy])
                self.slamM.set_offsets([[sx, sy]]); self.slamM.set_UVC([sdx], [sdy])

                self.slamL_dot.set_data([sx], [sy])
                self.slamM_dot.set_data([sx], [sy])
                
                # Debug: Print pose difference
                if self._update_count % 50 == 1:
                    pose_diff = math.sqrt((rx - sx)**2 + (ry - sy)**2)
                    angle_diff = abs(pose.theta - slam_pose.theta)
                    angle_diff = min(angle_diff, 2*math.pi - angle_diff)  # Wrap to [0, π]
                    print(f"  - 位姿差异: 距离={pose_diff:.3f}m, 角度={angle_diff:.3f}rad ({angle_diff*180/math.pi:.1f}°)")
            else:
                # Clear SLAM pose if not provided
                self.slamL.set_offsets([[]]); self.slamL.set_UVC([], [])
                self.slamM.set_offsets([[]]); self.slamM.set_UVC([], [])
                self.slamR.set_offsets([[]]); self.slamR.set_UVC([], [])
                
                self.slamL_dot.set_data([], [])
                self.slamM_dot.set_data([], [])
                self.slamR_dot.set_data([], [])

            self._update_radar_range(pose)
            self._update_right_scans(pose, scan)
            
            # A* panel: Update with real-time path based on navigation state
            if path and len(path) > 0:
                path_x = [p[0] for p in path]
                path_y = [p[1] for p in path]
                
                # Choose path color based on navigation state
                if nav_state == "RETURN_HOME":
                    # Use green color for return path
                    if not hasattr(self, 'pA_return'):
                        (self.pA_return,) = self.axA.plot([], [], "g-", lw=3, alpha=0.9, label='Return Path')
                    self.pA_return.set_data(path_x, path_y)
                    # Hide static path when showing return path
                    self.pA.set_data([], [])
                else:
                    # Use blue color for normal path
                    self.pA.set_data(path_x, path_y)
                    # Hide return path when showing normal path
                    if hasattr(self, 'pA_return'):
                        self.pA_return.set_data([], [])
            else:
                # Clear both paths when no path
                self.pA.set_data([], [])
                if hasattr(self, 'pA_return'):
                    self.pA_return.set_data([], [])
            
            # 绘制闸门圆圈
            if start_xy is not None or exit_xy is not None:
                self.draw_gate_circles(start_xy, exit_xy)

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        except Exception as e:
            print(f"⚠️ 更新可视化时出错: {e}")

    def _update_radar_range(self, pose: Pose2D):
        """Draw 360° radar arc in WORLD coordinates. Avoid grid conversions."""
        try:
            if self.radar_circle_L: self.radar_circle_L.remove()
            if self.radar_circle_M: self.radar_circle_M.remove()
            if self.radar_circle_R: self.radar_circle_R.remove()

            from core.config import LIDAR_RANGE
            angles = np.linspace(-180, 180, 50) * math.pi / 180.0
            arc_x = pose.x + LIDAR_RANGE * np.cos(angles + pose.theta)
            arc_y = pose.y + LIDAR_RANGE * np.sin(angles + pose.theta)

            self.radar_circle_L, = self.axL.plot(arc_x, arc_y, 'b-', alpha=0.3, linewidth=1)
            self.radar_circle_M, = self.axM.plot(arc_x, arc_y, 'b-', alpha=0.3, linewidth=1)
            self.radar_circle_R, = self.axR.plot(arc_x, arc_y, 'b-', alpha=0.3, linewidth=1)
        except Exception as e:
            print(f"⚠️ 更新雷达范围可视化时出错: {e}")

    def show_detailed_grid_analysis(self, occ_grid, log_odds_grid=None, pose=None):
        """
        显示详细的栅格地图分析窗口
        用于调试建图问题
        
        Args:
            occ_grid: 占用栅格 (0=free, 1=obstacle, 2=unknown)
            log_odds_grid: log-odds网格（可选）
            pose: 机器人当前位姿（可选）
        """
        # 创建新窗口，2×2布局
        fig_debug, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 14))
        fig_debug.suptitle('SLAM Map Builder 详细诊断', fontsize=16, fontweight='bold')
        
        from core.config import SLAM_RESOLUTION
        extent = [0.0, WORLD_SIZE, 0.0, WORLD_SIZE]
        
        # ========== 左上：原始占用栅格 ==========
        ax1.set_title('占用栅格地图 (Occupancy Grid)', fontsize=12, fontweight='bold')
        
        # 创建彩色图像：0=白色(free), 1=黑色(obstacle), 2=灰色(unknown)
        occ_img = np.zeros(occ_grid.shape, dtype=np.uint8)
        occ_img[occ_grid == 0] = 255  # Free = 白色
        occ_img[occ_grid == 1] = 0    # Obstacle = 黑色
        occ_img[occ_grid == 2] = 128  # Unknown = 灰色
        
        ax1.imshow(occ_img, cmap='gray', origin='lower', extent=extent, vmin=0, vmax=255)
        
        # 标记机器人位置
        if pose:
            ax1.plot(pose.x, pose.y, 'ro', markersize=10, label='机器人位置')
            ax1.arrow(pose.x, pose.y, 
                     0.1*math.cos(pose.theta), 0.1*math.sin(pose.theta),
                     head_width=0.05, head_length=0.05, fc='r', ec='r')
        
        # 统计信息
        free_count = np.sum(occ_grid == 0)
        occ_count = np.sum(occ_grid == 1)
        unknown_count = np.sum(occ_grid == 2)
        total = occ_grid.size
        
        stats_text = f'统计:\n'
        stats_text += f'自由: {free_count} ({free_count/total*100:.1f}%)\n'
        stats_text += f'障碍: {occ_count} ({occ_count/total*100:.1f}%)\n'
        stats_text += f'未知: {unknown_count} ({unknown_count/total*100:.1f}%)'
        
        ax1.text(0.02, 1.92, stats_text, fontsize=9, 
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
                verticalalignment='top')
        ax1.legend(loc='upper right', fontsize=8)
        ax1.set_xlim(0, WORLD_SIZE)
        ax1.set_ylim(0, WORLD_SIZE)
        ax1.set_xlabel('X (m)', fontsize=10)
        ax1.set_ylabel('Y (m)', fontsize=10)
        ax1.grid(True, alpha=0.3)
        
        # ========== 右上：Log-odds热图 ==========
        ax2.set_title('Log-odds 热图', fontsize=12, fontweight='bold')
        
        if log_odds_grid is not None:
            im2 = ax2.imshow(log_odds_grid, cmap='RdBu_r', origin='lower', 
                           extent=extent, vmin=-2.5, vmax=3.5)
            plt.colorbar(im2, ax=ax2, label='Log-odds值')
            
            # Log-odds统计
            lo_stats = f'Log-odds统计:\n'
            lo_stats += f'最小: {log_odds_grid.min():.2f}\n'
            lo_stats += f'最大: {log_odds_grid.max():.2f}\n'
            lo_stats += f'均值: {log_odds_grid.mean():.2f}\n'
            lo_stats += f'标准差: {log_odds_grid.std():.2f}'
            
            ax2.text(0.02, 1.92, lo_stats, fontsize=9,
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
                    verticalalignment='top')
        else:
            ax2.text(0.5, 0.5, '无Log-odds数据', ha='center', va='center', fontsize=14)
        
        if pose:
            ax2.plot(pose.x, pose.y, 'ro', markersize=10)
        
        ax2.set_xlim(0, WORLD_SIZE)
        ax2.set_ylim(0, WORLD_SIZE)
        ax2.set_xlabel('X (m)', fontsize=10)
        ax2.set_ylabel('Y (m)', fontsize=10)
        ax2.grid(True, alpha=0.3)
        
        # ========== 左下：起点周围局部放大 ==========
        ax3.set_title('起点周围局部放大 (±0.3m)', fontsize=12, fontweight='bold')
        
        if pose:
            # 计算放大区域
            center_x, center_y = pose.x, pose.y
            zoom_range = 0.3  # ±0.3m
            
            # 转换为像素范围
            cx_px = int(center_x / SLAM_RESOLUTION)
            cy_px = int(center_y / SLAM_RESOLUTION)
            zoom_px = int(zoom_range / SLAM_RESOLUTION)  # 30像素
            
            x0 = max(0, cx_px - zoom_px)
            x1 = min(occ_grid.shape[1], cx_px + zoom_px)
            y0 = max(0, cy_px - zoom_px)
            y1 = min(occ_grid.shape[0], cy_px + zoom_px)
            
            # 提取局部区域
            local_occ = occ_grid[y0:y1, x0:x1]
            
            # 创建彩色图像
            local_img = np.zeros(local_occ.shape, dtype=np.uint8)
            local_img[local_occ == 0] = 255  # Free
            local_img[local_occ == 1] = 0    # Obstacle
            local_img[local_occ == 2] = 128  # Unknown
            
            # 显示
            local_extent = [x0 * SLAM_RESOLUTION, x1 * SLAM_RESOLUTION,
                          y0 * SLAM_RESOLUTION, y1 * SLAM_RESOLUTION]
            ax3.imshow(local_img, cmap='gray', origin='lower', 
                      extent=local_extent, vmin=0, vmax=255)
            
            # 标记机器人和周围8个邻居
            ax3.plot(pose.x, pose.y, 'ro', markersize=15, label='机器人')
            
            # 8个邻居
            for di, dj in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                ni, nj = cy_px + di, cx_px + dj
                if 0 <= ni < occ_grid.shape[0] and 0 <= nj < occ_grid.shape[1]:
                    nx = nj * SLAM_RESOLUTION
                    ny = ni * SLAM_RESOLUTION
                    val = occ_grid[ni, nj]
                    color = 'g' if val == 0 else ('r' if val == 1 else 'y')
                    marker = 'o' if val == 0 else ('x' if val == 1 else 's')
                    ax3.plot(nx, ny, color+marker, markersize=8, alpha=0.7)
            
            # 网格线
            ax3.grid(True, alpha=0.5, linewidth=0.5)
            ax3.set_xlim(local_extent[0], local_extent[1])
            ax3.set_ylim(local_extent[2], local_extent[3])
            
            # 详细统计
            local_stats = f'局部统计 ({local_occ.shape[0]}×{local_occ.shape[1]}):\n'
            local_stats += f'自由: {np.sum(local_occ==0)} (绿○)\n'
            local_stats += f'障碍: {np.sum(local_occ==1)} (红×)\n'
            local_stats += f'未知: {np.sum(local_occ==2)} (黄□)'
            
            ax3.text(local_extent[0] + 0.01, local_extent[3] - 0.01, local_stats,
                    fontsize=9, bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
                    verticalalignment='top')
        else:
            ax3.text(0.5, 0.5, '无位姿信息', ha='center', va='center', fontsize=14)
        
        ax3.set_xlabel('X (m)', fontsize=10)
        ax3.set_ylabel('Y (m)', fontsize=10)
        ax3.legend(loc='lower right', fontsize=8)
        
        # ========== 右下：障碍物密度热图 ==========
        ax4.set_title('障碍物密度分析 (10×10像素块)', fontsize=12, fontweight='bold')
        
        # 计算每个10×10块的障碍物密度
        block_size = 10
        H, W = occ_grid.shape
        h_blocks = H // block_size
        w_blocks = W // block_size
        
        density_map = np.zeros((h_blocks, w_blocks), dtype=np.float32)
        
        for i in range(h_blocks):
            for j in range(w_blocks):
                y0 = i * block_size
                y1 = (i + 1) * block_size
                x0 = j * block_size
                x1 = (j + 1) * block_size
                
                block = occ_grid[y0:y1, x0:x1]
                # 障碍物密度 = 障碍物像素数 / 总像素数
                occ_in_block = np.sum(block == 1)
                density_map[i, j] = occ_in_block / (block_size * block_size)
        
        # 显示密度图
        im4 = ax4.imshow(density_map, cmap='hot', origin='lower',
                        extent=extent, vmin=0, vmax=1)
        plt.colorbar(im4, ax=ax4, label='障碍物密度')
        
        if pose:
            ax4.plot(pose.x, pose.y, 'co', markersize=10, 
                    markeredgecolor='white', markeredgewidth=2, label='机器人')
        
        ax4.set_xlim(0, WORLD_SIZE)
        ax4.set_ylim(0, WORLD_SIZE)
        ax4.set_xlabel('X (m)', fontsize=10)
        ax4.set_ylabel('Y (m)', fontsize=10)
        ax4.legend(loc='upper right', fontsize=8)
        ax4.grid(True, alpha=0.3, color='white', linewidth=0.5)
        
        # 添加说明文本
        desc_text = '调试说明:\n'
        desc_text += '• 左上：占用栅格（白=自由，黑=障碍，灰=未知）\n'
        desc_text += '• 右上：Log-odds热图（红=障碍，蓝=自由）\n'
        desc_text += '• 左下：起点局部放大（邻居状态）\n'
        desc_text += '• 右下：障碍物密度热图'
        
        fig_debug.text(0.5, 0.01, desc_text, ha='center', fontsize=9,
                      bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
        
        plt.tight_layout(rect=[0, 0.05, 1, 0.98])
        plt.show(block=True)  # 阻塞显示，等待用户关闭
        print("✅ 调试窗口已关闭")

    def _draw_target_reachable_region(self, region_rect: tuple):
        """绘制终点可达区域"""
        if region_rect is None:
            return
            
        # Rect format follows navigator: (xmin, xmax, ymin, ymax)
        x_min, x_max, y_min, y_max = region_rect
        
        # 绘制矩形区域
        import matplotlib.patches as patches
        
        # 在SLAM面板（右上图）绘制绿色矩形
        rect_slam = patches.Rectangle((x_min, y_min), x_max - x_min, y_max - y_min,
                                    linewidth=2, edgecolor='lime', facecolor='none',
                                    alpha=0.8, linestyle='--', label='Target Region')
        self._target_region_patch_M = rect_slam
        self.axM.add_patch(rect_slam)
        
        # 在规划面板（右下图）绘制橙色矩形
        rect_planning = patches.Rectangle((x_min, y_min), x_max - x_min, y_max - y_min,
                                        linewidth=2, edgecolor='orange', facecolor='none',
                                        alpha=0.8, linestyle=':', label='Target Region')
        self._target_region_patch_A = rect_planning
        self.axA.add_patch(rect_planning)
        
        # 添加文本标签
        center_x = (x_min + x_max) / 2
        center_y = (y_min + y_max) / 2
        
        # SLAM面板标签
        self.axM.text(center_x, center_y, 'TARGET\nREGION', 
                     ha='center', va='center', fontsize=8, 
                     color='lime', weight='bold', alpha=0.9)
        
        # 规划面板标签
        self.axA.text(center_x, center_y, 'TARGET\nREGION', 
                     ha='center', va='center', fontsize=8, 
                     color='orange', weight='bold', alpha=0.9)
    
    def close(self):
        try:
            plt.close(self.fig)
            print("✅ 主图形界面已关闭")
            print("\n" + "="*60)
            print("准备显示详细栅格地图诊断窗口...")
            print("="*60 + "\n")
        except Exception as e:
            print(f"⚠️ 关闭图形界面时出错: {e}")
