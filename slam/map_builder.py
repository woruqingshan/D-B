# ================================
# file: slam/map_builder.py
# ================================
"""
Pure Mapping Module - Decoupled from Localization

This module builds high-quality occupancy grid maps using ground truth poses
(in simulation) or estimated poses (in real mode). It does NOT perform localization.

Key Features:
- Log-odds occupancy grid mapping
- Bresenham ray tracing
- 3x3 obstacle splatting
- Inverse sensor model
"""
from __future__ import annotations
import math
import numpy as np
from typing import Tuple, Optional
from core import Pose2D, LaserScan, LIDAR_RANGE, LIDAR_MIN_RANGE
from core.config import (
    ROBOT_START_X, ROBOT_START_Y, ROBOT_RADIUS,
    NEIGHBOR_CONSIST_TOL_M, TAIL_GAP_CELLS, CONFIRMED_OCC_TH
)


class MapBuilder:
    """Pure mapping module using ground truth or reliable poses"""
    
    def __init__(self,
                 map_size_pixels: int,
                 map_size_meters: float,
                 resolution: float,
                 log_hit: float = 1.2,   # Increased from 0.85 to make obstacles more stable
                 log_miss: float = -0.3,  # Reduced from -0.4 to make FREE less aggressive
                 log_min: float = -2.5,
                 log_max: float = 3.5,
                 logger_func=None,
                 log_file=None) -> None:
        """
        Initialize map builder
        
        Args:
            map_size_pixels: Map size in pixels (N)
            map_size_meters: Map size in meters (L)
            resolution: Resolution in meters/pixel (r = L/N)
            log_hit: Log-odds increment for obstacle hits
            log_miss: Log-odds decrement for free space
            log_min: Minimum log-odds value
            log_max: Maximum log-odds value
        """
        # Map parameters
        self.N = int(map_size_pixels)
        self.L = float(map_size_meters)
        self.res = float(resolution)
        
        # Verify resolution consistency
        expected_res = self.L / self.N
        if abs(expected_res - self.res) > 1e-6:
            print(f"⚠️ Resolution mismatch: expected={expected_res:.6f}, got={self.res:.6f}")
        
        # Log-odds parameters
        self._log_hit = float(log_hit)
        self._log_miss = float(log_miss)
        self._log_min = float(log_min)
        self._log_max = float(log_max)
        
        # Log-odds grid (0 = neutral/unknown)
        self._lgrid = np.zeros((self.N, self.N), dtype=np.float32)
        
        # Statistics
        self._update_count = 0
        self._total_rays = 0
        self._total_hits = 0
        
        # Logger
        self.logger_func = logger_func
        self.log_file = log_file

        # --- Keep entrance and robot footprint free (simulation-friendly) ---
        # English: pre-compute entrance grid and clearing radii.
        self._entr_gx, self._entr_gy = self._world_to_grid(ROBOT_START_X, ROBOT_START_Y)
        # Use physical radius for footprint clearing (NOT planning radius which is 0)
        # Physical radius is a hardware constant describing actual robot size
        PHYSICAL_RADIUS = 0.10  # 10cm physical robot radius (hardware specification)
        self._foot_clear_cells = max(1, int(PHYSICAL_RADIUS / self.res))   # ~10 px
        # === Fix: Increase entrance protection radius to prevent false obstacle marking ===
        # English: Increased from 6px (0.06m) to 15px (0.15m) to protect entrance from
        # 3x3 splatting and distant scan endpoints that might hit near entrance
        self._entr_clear_cells = max(1, int(0.15 / self.res))  # 15 pixels ≈ 0.15m 

         # English: leak log-odds towards 0 to dissipate stale artifacts.
        self._leak_rate = 0.015          # ≈1.5% per update → ~8–12s half-life at 10 Hz
        # English: per-cell "last hit age" in updates; used to protect only recently-hit cells.
        self._hit_age = np.full((self.N, self.N), 65535, dtype=np.uint16)
        # English: remember last pose to detect "stationary" condition.
        self._last_pose = None
        
        # --- Known-latch: prevent known cells from reverting to UNKNOWN on export ---
        # 0=never seen, 1=seen FREE, 2=seen OCC (OCC wins over FREE when both happened)
        self._known_latch = np.zeros((self.N, self.N), dtype=np.uint8)
        self._last_known_cells = 0
        
        self._log_debug(f"MapBuilder初始化: N={self.N}, L={self.L:.2f}m, res={self.res:.4f}m/pixel")
    
    def update(self, pose: Pose2D, lidar_scan: LaserScan) -> None:
        """
        Update map using pose and lidar scan
        
        Args:
            pose: Robot pose (x, y, theta) - should be accurate (ground truth or good estimate)
            lidar_scan: Laser scan data
        """
        self._update_count += 1
        
        x0, y0, theta = pose.x, pose.y, pose.theta
        # English: detect stationary robot (accumulates artifacts if we keep adding hits)
        is_stationary = False
        if self._last_pose is not None:
            dx = pose.x - self._last_pose.x
            dy = pose.y - self._last_pose.y
            dth = abs((pose.theta - self._last_pose.theta + math.pi) % (2*math.pi) - math.pi)
            is_stationary = (dx*dx + dy*dy < (0.01*0.01)) and (dth < 0.03)
        self._last_pose = Pose2D(x0, y0, theta)
        
        # Debug: Check resolution on first update
        if self._update_count == 1:
            self._log_debug(f"[INIT] MapBuilder参数检查:")
            self._log_debug(f"  N={self.N}, L={self.L:.3f}m, res={self.res:.6f}m/pixel")
            self._log_debug(f"  log_hit={self._log_hit}, log_miss={self._log_miss}")
            self._log_debug(f"  log_min={self._log_min}, log_max={self._log_max}")
        
        # Check if pose is within reasonable bounds
        if not (-1.0 <= x0 <= self.L + 1.0 and -1.0 <= y0 <= self.L + 1.0):
            self._log_debug(f"⚠️ 位姿越界: ({x0:.3f}, {y0:.3f}), 跳过建图")
            return
        
        # Convert robot position to grid
        gx0, gy0 = self._world_to_grid(x0, y0)
        self._current_robot_gx = gx0   # English: record robot footprint for export
        self._current_robot_gy = gy0
        
        # Debug: Log coordinate conversion
        if self._update_count == 1:
            self._log_debug(f"[COORD] 起点转换: 世界({x0:.3f}, {y0:.3f}) → 网格({gx0}, {gy0})")
            self._log_debug(f"[COORD] 验证: {gx0} * {self.res:.6f} = {gx0 * self.res:.3f}")
        
        if not (0 <= gx0 < self.N and 0 <= gy0 < self.N):
            self._log_debug(f"⚠️ 起点网格坐标越界: ({gx0}, {gy0}), 跳过建图")
            return
        
        c0, s0 = math.cos(theta), math.sin(theta)
        
        valid_rays = 0
        hit_count = 0
        clipped_rays = 0  # Count rays clipped by map boundary
        
        # Debug: Log first few rays on first update
        log_ray_details = (self._update_count == 1)
        rays_logged = 0
        
        # 使用雷达消息自带的量程上/下限，防止常量与设备不一致
        rmin = max(LIDAR_MIN_RANGE, getattr(lidar_scan, "range_min", LIDAR_MIN_RANGE))
        rmax = min(LIDAR_RANGE,      getattr(lidar_scan, "range_max", LIDAR_RANGE))

        # Process each laser beam
        for i, range_val in enumerate(lidar_scan.ranges):
            # Filter invalid ranges
            if not np.isfinite(range_val):
                continue
            if range_val < rmin or range_val > rmax:
                continue
            
            # Compute beam angle in world frame
            beam_angle = lidar_scan.angle_min + i * lidar_scan.angle_increment
            ca, sa = math.cos(beam_angle), math.sin(beam_angle)
            
            # Ray direction in world frame
            ux = c0 * ca - s0 * sa
            uy = s0 * ca + c0 * sa
            
            # End point in world frame
            ex = x0 + range_val * ux
            ey = y0 + range_val * uy
            
            # Clip endpoint to map bounds instead of skipping
            ex_clipped = np.clip(ex, 0.0, self.L)
            ey_clipped = np.clip(ey, 0.0, self.L)
            
            # Track if ray was clipped
            was_clipped = (abs(ex - ex_clipped) > 1e-6) or (abs(ey - ey_clipped) > 1e-6)
            if was_clipped:
                clipped_rays += 1
            
            # Recalculate effective range after clipping
            clipped_dx = ex_clipped - x0
            clipped_dy = ey_clipped - y0
            effective_range = math.sqrt(clipped_dx**2 + clipped_dy**2)
            
            # Skip if clipped range is too small
            if effective_range < 0.05:  # Less than 5cm
                continue
            
            gx1, gy1 = self._world_to_grid(ex_clipped, ey_clipped)
            if not (0 <= gx1 < self.N and 0 <= gy1 < self.N):
                continue
            
            # Debug: Log first 3 rays on first update
            if log_ray_details and rays_logged < 3:
                self._log_debug(f"[RAY{i}] angle={beam_angle:.3f}, range={range_val:.3f}")
                self._log_debug(f"  dir=({ux:.3f}, {uy:.3f})")
                self._log_debug(f"  end: ({ex:.3f}, {ey:.3f}) → clipped({ex_clipped:.3f}, {ey_clipped:.3f})")
                self._log_debug(f"  grid: ({gx0}, {gy0}) → ({gx1}, {gy1})")
                self._log_debug(f"  clipped={was_clipped}")
                rays_logged += 1
            
            # === Engineering Standard Fix: Proper Ray Tracing with Obstacle Detection ===
            # English: Standard occupancy grid mapping practice (Probabilistic Robotics)
            # - Trace ray from robot to endpoint
            # - If ray passes through confirmed obstacle, stop FREE marking there
            # - Only mark FREE space up to first obstacle or endpoint
            
            valid_rays += 1
            
            # Get all cells along the ray
            ray_cells = list(self._bresenham(gx0, gy0, gx1, gy1))
            
            # === Key Fix: Find first confirmed obstacle along ray ===
            # English: If ray passes through a cell that's already confirmed as obstacle,
            # we should NOT mark it as FREE. This prevents "wall erosion".
            first_obstacle_idx = None
            for idx, (gx, gy) in enumerate(ray_cells):
                if not (0 <= gx < self.N and 0 <= gy < self.N):
                    break
                # Check if this cell is a confirmed obstacle (strong positive log-odds)
                ttl_updates = 25
                if self._lgrid[gy, gx] > 2.0 and self._hit_age[gy, gx] < ttl_updates:
                    first_obstacle_idx = idx
                    break
            
            # Determine safe FREE marking range
            if first_obstacle_idx is not None:
                # Ray hit confirmed obstacle - only mark FREE up to that point
                free_end_idx = first_obstacle_idx
                if log_ray_details and rays_logged < 5:
                    self._log_debug(f"  [RAY{i}] 遇到已知障碍@{ray_cells[first_obstacle_idx]}，只标记到此为FREE")
            elif was_clipped:
                # Ray was clipped - don't mark any FREE to avoid wall penetration
                free_end_idx = 0
                if log_ray_details and rays_logged < 5:
                    self._log_debug(f"  [RAY{i}] 被裁剪，跳过FREE标记")
            else:
                # Normal ray - mark all but endpoint as FREE
                free_end_idx = len(ray_cells) - 1
            
            # 在靠近终点预留一个尾距像素，抑制FREE把薄墙“擦穿”
            tail_gap = max(0, int(TAIL_GAP_CELLS))
            free_end_idx = max(0, free_end_idx - tail_gap)

            # Mark FREE space (up to safe endpoint)
            for idx in range(free_end_idx):
                gx, gy = ray_cells[idx]
                if 0 <= gx < self.N and 0 <= gy < self.N:
                    # === Additional Fix: Protect strong obstacles from FREE erosion ===
                    # English: If a cell is already strongly occupied, reduce FREE effect

                    # === Revised: protect ONLY recently-hit cells (age < ttl) ===
                    # English: allow clearing old artifacts; keep walls stable right after hits.
                    ttl_updates = 25  # ~2.5 s at 10 Hz
                    if self._lgrid[gy, gx] > 1.0 and self._hit_age[gy, gx] < ttl_updates:
                        self._lgrid[gy, gx] = max(self._log_min,
                                                  self._lgrid[gy, gx] + self._log_miss * 0.5)
                    else:
                        self._lgrid[gy, gx] = max(self._log_min,
                                                  self._lgrid[gy, gx] + self._log_miss)
            
            # Mark obstacle (if true hit, not max range, and not clipped)
            # Note: was_clipped already computed above
            
            if range_val < (rmax - 0.01) and not was_clipped:  # True hit, not clipped
                # Check if hit point is far enough from robot to avoid self-marking
                dist_to_robot = math.sqrt((gx1 - gx0)**2 + (gy1 - gy0)**2)
                min_pix = max(2, int(round(0.03 / self.res)))
                # === Fix: Protect entrance area from obstacle marking ===
                # English: Check if hit point is too close to entrance (within 15 pixels)
                # This prevents distant scans from marking entrance as obstacle
                dist_to_entrance = math.sqrt((gx1 - self._entr_gx)**2 + (gy1 - self._entr_gy)**2)
                if dist_to_entrance < 15:  # Within 15 pixels ≈ 0.15m of entrance
                    if self._update_count <= 5:
                        self._log_debug(f"  [RAY{i}] 命中点距入口{dist_to_entrance:.1f}px，跳过障碍标记")
                    continue  # Skip obstacle marking near entrance
                
                if (gx1 == gx0 and gy1 == gy0) or (gx1 == self._entr_gx and gy1 == self._entr_gy):
                    pass  # skip occupying entrance/robot cell
                elif dist_to_robot > min_pix:  # At least 3 pixels away (0.03m)
                    # 邻束一致性：两侧束都命中且与当前距离差 < NEIGHBOR_CONSIST_TOL_M
                    ok_neighbor = False
                    if 0 < i < len(lidar_scan.ranges) - 1:
                        d0, d1 = lidar_scan.ranges[i-1], lidar_scan.ranges[i+1]
                        if (np.isfinite(d0) and np.isfinite(d1)
                            and rmin < d0 < 0.99 * rmax and rmin < d1 < 0.99 * rmax
                            and abs(range_val - d0) < NEIGHBOR_CONSIST_TOL_M
                            and abs(range_val - d1) < NEIGHBOR_CONSIST_TOL_M):
                            ok_neighbor = True
                    # 不满足一致性 → 放弃本次建墙，避免原地卡住时把末端刷成墙
                    if not ok_neighbor:
                        continue

                    hit_count += 1
                    
                    # === Engineering Standard: Enhanced obstacle marking ===
                    # English: Use stronger log_hit for center cell, weaker for neighbors
                    # This creates more stable obstacles with clear boundaries
                    
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            gx_hit = gx1 + dx
                            gy_hit = gy1 + dy
                            
                            # Skip if hit cell is too close to robot position
                            dist_hit_to_robot = math.sqrt((gx_hit - gx0)**2 + (gy_hit - gy0)**2)
                            if dist_hit_to_robot < 2:  # Don't mark within 2 pixels of robot
                                continue
                            
                            # === Fix: Also check entrance distance for each splatting cell ===
                            # English: Ensure 3x3 splatting doesn't mark entrance area
                            dist_hit_to_entrance = math.sqrt((gx_hit - self._entr_gx)**2 + (gy_hit - self._entr_gy)**2)
                            if dist_hit_to_entrance < 15:  # Within entrance protection zone
                                continue
                            
                            if 0 <= gx_hit < self.N and 0 <= gy_hit < self.N:
                                # === Engineering Fix: Stronger hit for center, weaker for edges ===
                                # English: Center cell gets full log_hit, neighbors get 70%
                                # This creates clearer obstacle boundaries
                                if dx == 0 and dy == 0:
                                    # Center hit - full strength
                                    hit_strength = self._log_hit
                                else:
                                    # Neighbor cells - reduced strength
                                    hit_strength = self._log_hit * 0.7
                                
                                # English: if robot is stationary, weaken hit accumulation
                                hs = hit_strength * (0.25 if is_stationary else 1.0)
                                self._lgrid[gy_hit, gx_hit] = min(self._log_max,
                                                                   self._lgrid[gy_hit, gx_hit] + hs)
                                # update hit age (0 = just hit)
                                self._hit_age[gy_hit, gx_hit] = 0
        
        self._total_rays += valid_rays
        self._total_hits += hit_count

        # --- Global maintenance ---
        # English: age++ with saturation; leak log-odds towards 0 to dissipate stale errors.
        self._hit_age = np.minimum(self._hit_age + 1, np.iinfo(self._hit_age.dtype).max)
        if self._leak_rate > 0.0:
            self._lgrid *= (1.0 - self._leak_rate)
            # snap very small values to exactly 0 to reduce flicker
            self._lgrid[np.abs(self._lgrid) < 0.02] = 0.0

        self._cleanup_spurs()

        # English: ensure doorway and footprint never become occupied after ray updates.
        self._force_clear_disk(gx0, gy0, self._foot_clear_cells)
        self._force_clear_disk(self._entr_gx, self._entr_gy, self._entr_clear_cells)        
        
        # Enhanced logging for debugging
        if self._update_count <= 5 or self._update_count % 10 == 0:
            self._log_debug(f"建图更新#{self._update_count}:")
            self._log_debug(f"  位姿: ({x0:.3f}, {y0:.3f}, {theta:.3f}rad = {theta*180/3.14159:.1f}°)")
            self._log_debug(f"  起点网格: ({gx0}, {gy0})")
            self._log_debug(f"  扫描总束数: {len(lidar_scan.ranges)}")
            self._log_debug(f"  有效光束: {valid_rays} ({valid_rays*100.0/len(lidar_scan.ranges):.1f}%)")
            self._log_debug(f"  命中障碍: {hit_count}")
            self._log_debug(f"  边界裁剪: {clipped_rays}")
            self._log_debug(f"  跳过光束: {len(lidar_scan.ranges) - valid_rays}")
            
            # === Performance Optimization: Reduce statistics computation frequency ===
            # English: Only compute expensive statistics every 50 updates (reduces overhead)
            # These sum operations become slow as map grows (40000 cells each)
            if self._update_count % 50 == 1 or self._update_count <= 5:
                self._log_debug(f"  log-odds统计: min={self._lgrid.min():.2f}, max={self._lgrid.max():.2f}")
                free_count = np.sum(self._lgrid < -0.3)
                occ_count = np.sum(self._lgrid > 0.5)
                unknown_count = self.N * self.N - free_count - occ_count
                self._log_debug(f"  FREE网格(<-0.3): {free_count}, OCC网格(>0.5): {occ_count}, 未知: {unknown_count}")
    
    def _cleanup_spurs(self) -> None:
        """
        English: Post-update despeckle. Remove thin star-like accumulations by weakening
        occupied cells that have too few occupied 8-neighbors. This preserves thick walls.
        """
        occ = (self._lgrid > CONFIRMED_OCC_TH)
        if not occ.any():
            return
        # 8-neighbor count without SciPy: sum of shifted boolean arrays
        o = occ.astype(np.uint8)
        cnt = np.zeros_like(o, dtype=np.uint8)
        cnt[1:,  :] += o[:-1, :]   # up
        cnt[:-1, :] += o[1:,  :]   # down
        cnt[:, 1:] += o[:, :-1]    # left
        cnt[:, :-1] += o[:, 1:]    # right
        cnt[1:, 1:] += o[:-1, :-1] # up-left
        cnt[1:, :-1] += o[:-1, 1:] # up-right
        cnt[:-1,1:] += o[1:,  :-1] # down-left
        cnt[:-1,:-1] += o[1:,  1:] # down-right
        # Cells with ≤2 occupied neighbors are likely spurs (isolated or line-endpoints)
        weak = np.logical_and(occ, cnt <= 2)
        if weak.any():
            # Pull weak occupied cells toward FREE (use stronger "miss" to counter accumulation)
            self._lgrid[weak] = np.maximum(self._log_min, self._lgrid[weak] + 2.5 * self._log_miss)
    
    def get_occupancy_grid(self) -> np.ndarray:
        grid = np.full((self.N, self.N), 2, dtype=np.uint8)  # start as UNKNOWN

        # --- thresholds: strong/weak occupancy ---
        # Strong obstacles only if log-odds is clearly high.
        occ_th_strong = 1.2          # 原0.5 -> 1.2，避免单束命中即输出障碍
        free_threshold = -0.3

        # 先标记自由
        grid[self._lgrid < free_threshold] = 0  # FREE

        # 1) 强障碍：直接设为占用
        strong_occ = (self._lgrid > occ_th_strong)
        grid[strong_occ] = 1

        # 2) 弱障碍：采用3×3多数投票，抑制“薄障碍带/孤立点”
        #    weak cell will be set to occupied only if >=5 neighbors are weak/strong.
        weak_occ = (self._lgrid > 0.5) & (~strong_occ)
        if weak_occ.any():
            w = weak_occ.astype(np.uint8)
            # 计算八邻域计数（零拷贝切片实现）
            p = np.pad(w, 1, mode='constant', constant_values=0)
            neigh = (
                p[0:-2,0:-2] + p[0:-2,1:-1] + p[0:-2,2:] +
                p[1:-1,0:-2]                  + p[1:-1,2:] +
                p[2:,  0:-2] + p[2:,  1:-1] + p[2:,  2:]
            )
            majority = (neigh >= 5)  # English: majority filter
            grid[weak_occ & majority] = 1

        # --- keep robot footprint & entrance always free (unchanged) ---
        if hasattr(self, '_current_robot_gx') and hasattr(self, '_current_robot_gy'):
            if 0 <= self._current_robot_gx < self.N and 0 <= self._current_robot_gy < self.N:
                rx, ry = self._current_robot_gx, self._current_robot_gy
                r = max(2, self._foot_clear_cells)
                for di in range(-r, r+1):
                    for dj in range(-r, r+1):
                        if di*di + dj*dj <= r*r:
                            ii, jj = ry + di, rx + dj
                            if 0 <= ii < self.N and 0 <= jj < self.N:
                                grid[ii, jj] = 0

        if 0 <= self._entr_gx < self.N and 0 <= self._entr_gy < self.N:
            ex, ey = self._entr_gx, self._entr_gy
            r = max(1, self._entr_clear_cells)
            x0, x1 = max(0, ex - r), min(self.N - 1, ex + r)
            y0, y1 = max(0, ey - r), min(self.N - 1, ey + r)
            grid[y0:y1+1, x0:x1+1] = 0

        # --- Known-latch update & apply (read-only to callers) ----------------------
        # Update latch with current known states
        seen_free = (grid == 0)
        seen_occ  = (grid == 1)
        # OCC wins: set 2 where occupied now
        self._known_latch[seen_occ] = 2
        # Set FREE only where latch is 0 (never seen) and current is FREE
        self._known_latch[(self._known_latch == 0) & seen_free] = 1
        # Apply latch only where grid is UNKNOWN this frame
        unknown = (grid == 2)
        grid[unknown & (self._known_latch == 1)] = 0
        grid[unknown & (self._known_latch == 2)] = 1
        # Watchdog: known cells sudden drop (for logging/upper-layer fallback)
        known_now = int((grid != 2).sum())
        if known_now + int(0.05 * grid.size) < self._last_known_cells:
            self._log_debug("⚠️ Known cells dropped unexpectedly; possible ROI-side clear. Latch applied.")
        self._last_known_cells = known_now
        return grid

    def get_coverage_ratio(self) -> float:
        """
        Calculate global coverage ratio (known cells / total cells)
        Returns:
            Coverage ratio [0.0, 1.0] based on the whole map
        """
        occ_grid = self.get_occupancy_grid()
        known_cells = np.sum(occ_grid != 2)  # 0/1都算已知
        total_cells = occ_grid.size
        if total_cells == 0:
            return 0.0
        return float(known_cells) / float(total_cells)
    
    def export_map(self) -> dict:
        """
        Export map data for localization
        
        Returns:
            Dictionary containing map data and metadata
        """
        return {
            'occupancy_grid': self.get_occupancy_grid(),
            'log_odds_grid': self._lgrid.copy(),
            'metadata': {
                'resolution': self.res,
                'size': (self.N, self.N),
                'size_meters': self.L,
                'origin': (0.0, 0.0),
                'update_count': self._update_count,
                'total_rays': self._total_rays,
                'total_hits': self._total_hits,
                'coverage_ratio': self.get_coverage_ratio()
            }
        }
    
    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates"""
        gx = int(round(x / self.res))
        gy = int(round(y / self.res))
        return gx, gy
    
    def _bresenham(self, x0: int, y0: int, x1: int, y1: int):
        """Bresenham line algorithm (integer grid)"""
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        
        x, y = x0, y0
        
        while True:
            yield x, y
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
    
    def _log_debug(self, msg: str) -> None:
        """Log debug message"""
        if self.logger_func and self.log_file:
            try:
                self.logger_func(self.log_file, msg, "MapBuilder")
            except:
                print(f"[MapBuilder] {msg}")
        else:
            print(f"[MapBuilder] {msg}")

    def _force_clear_disk(self, cx: int, cy: int, r: int) -> None:
        """Set a small disk area to 'free' in log-odds.
        English: used to keep doorway and robot footprint free of accidental occupancy."""
        if r <= 0:
            return
        x0, x1 = max(0, cx - r), min(self.N - 1, cx + r)
        y0, y1 = max(0, cy - r), min(self.N - 1, cy + r)
        for gy in range(y0, y1 + 1):
            for gx in range(x0, x1 + 1):
                if (gx - cx) * (gx - cx) + (gy - cy) * (gy - cy) <= r * r:
                    # Push towards free but keep within bounds
                    self._lgrid[gy, gx] = min(self._lgrid[gy, gx], -1.2)