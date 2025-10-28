# ================================
# file: code/explore/frontier_explorer.py
# ================================
from __future__ import annotations
from typing import Optional, Tuple, List, Sequence, Dict
import numpy as np
import time
import math
import contextlib
import collections
from collections import deque
from core import MAP_SIZE, map_to_world, world_to_map
from planning import AStarPlanner
from core.types import Pose2D
from core.config import (
    EXPLORATION_MIN_TIME, EXPLORATION_MIN_COVERAGE_RATIO, EXPLORATION_MAX_COVERAGE_RATIO, 
    LIDAR_RANGE, SLAM_RESOLUTION, SAFE_BUFFER_M,
    USE_RING_PRIMARY, RING_RADIUS_M, RING_K, RING_AXIS_PREF_W, RING_MIN_FWD_DOT,
    FWD_SUPPRESS_CLEARANCE_M, FWD_SUPPRESS_CONE_DEG, SIDE_TURN_MIN_DELTA_DEG, SIDE_TURN_BONUS,
    BOUNDARY_PENALTY_ENABLED, BOUNDARY_SAFE_DISTANCE, BOUNDARY_PENALTY_SEVERE, 
    BOUNDARY_PENALTY_MEDIUM, BOUNDARY_PENALTY_LIGHT, WORLD_SIZE
)

# ------------------------ Ring-gain config (兜底参数) ------------------------
# 触发条件：usable=0 连续3次才允许环采样；raw=0 连续7次且 BFS≈1 则交给上层简便退避
RING_FAIL_USABLE_TH = 3
RETREAT_FAIL_RAW_TH = 7
BFS_SMALL_MAX = 1                   # "≈1"判定

# 采样参数
RING_RADIUS_M = 0.50                # 环半径（候选距离）
RING_K = 24                         # 角样本数
RING_SENSE_M = 1.10                 # 用于信息增益的统计半径（~雷达感知范围）

# 评分/约束
BACKWARD_DOT_MIN = 0.0              # 与来向单位向量点积 < 0 → 认为"回头"直接丢弃
GOAL_DIR_WEIGHT = 0.20              # 终点方向偏置权重（cos θ_goal）
TABOO_RADIUS_M = 0.25               # 禁忌带半径
TABOO_MAX_KEEP = 10                 # 记录最近访问的禁忌带数量

# 轴向量化：落点尽量横平竖直（对齐当前位姿的 x 或 y）
def _axis_snap(pose_xy: Tuple[float,float], pt_xy: Tuple[float,float]) -> Tuple[float,float]:
    px, py = pose_xy; x, y = pt_xy
    if abs(x - px) >= abs(y - py):
        # 水平优先：让 y 贴近机器人当前 y（并四舍五入到栅格）
        y = round(py / SLAM_RESOLUTION) * SLAM_RESOLUTION
    else:
        # 垂直优先：让 x 贴近机器人当前 x
        x = round(px / SLAM_RESOLUTION) * SLAM_RESOLUTION
    return (x, y)

@contextlib.contextmanager
def _readonly(arr: np.ndarray):
    """
    English: Make array read-only inside the context to catch illegal writes.
    This prevents any accidental modifications to the global occupancy grid.
    """
    old = arr.flags.writeable
    arr.flags.writeable = False
    try:
        yield
    finally:
        arr.flags.writeable = old


class FrontierExplorer:
    """Frontier-based exploration using occupancy grid."""
    def __init__(self, planner: AStarPlanner, slam_system=None, logger_func=None, log_file=None) -> None:
        self.planner = planner
        self.slam_system = slam_system  # SLAM系统引用
        self.logger_func = logger_func  # 日志函数
        self.log_file = log_file  # 日志文件对象
        self._exploration_start_time = None  # Track exploration start time
        self._min_exploration_time = EXPLORATION_MIN_TIME  # Minimum exploration time from config
        self._min_coverage_ratio = EXPLORATION_MIN_COVERAGE_RATIO  # Minimum coverage ratio from config
        self._max_coverage_ratio = EXPLORATION_MAX_COVERAGE_RATIO  # Maximum coverage ratio from config
        self.exit_pos: Optional[Tuple[float, float]] = None
        self.no_frontier_since: Optional[float] = None
        
        # 失败计数与拒绝统计
        self.fail_raw = 0        # 本次 ROI 内 raw 簇数量==0
        self.fail_usable = 0     # 通过 C-space+微调+A* 后 usable==0
        self.reject_stats = collections.Counter()
        
        # 禁忌带与来向
        self._taboo = deque(maxlen=TABOO_MAX_KEEP)   # [(x,y), ...]
        self._last_forward_u = None                  # 上一段有效运动的单位向量
        self._last_dispatched_goal = None
        
        # English: cache the latest binary C-space used during frontier selection,
        # so that navigator can plan on the SAME collision world and avoid double inflation.
        self._latest_cspace_bin = None
        self._last_cspace_ts = 0.0
        
        # === Performance optimization: throttle frontier extraction and work only in ROI ===
        # English: Only extract frontiers in robot's field of view + margin to reduce O(HW) operations
        from core.config import FRONTIER_MIN_INTERVAL, FRONTIER_ROI_MARGIN_M
        self._last_frontier_ts = 0.0
        self._last_pose = None
        self._roi_margin_m = FRONTIER_ROI_MARGIN_M
        self._frontier_min_interval = FRONTIER_MIN_INTERVAL
        self._last_frontiers = []
        # Known-latch on consumer side (extra safety for ROI-induced regressions)
        self._latch_map = None       # np.ndarray uint8 with values {0,1,2}
        self._last_known_cnt = 0
        self._roi_force_full = False
        
        # ---- Deadlock recovery: remembered frontiers (ring buffer) ----
        # 记忆最近若干个"已发现的前沿簇中心"，用于卡死时回退尝试
        # 元素: (gi, gj, wx, wy, ts)
        self._frontier_mem: deque[Tuple[int, int, float, float, float]] = deque(maxlen=40)
        
        # 环采样主策略开关与参数
        self._use_ring_primary = bool(USE_RING_PRIMARY)
        self._ring_R_m = float(RING_RADIUS_M)
        self._ring_K = int(RING_K)
        self._axis_pref_w = float(RING_AXIS_PREF_W)
        self._ring_min_fwd_dot = float(RING_MIN_FWD_DOT)
        
        # 到达带外扩参数（避免窄走廊"擦不到门线"）
        self.GOAL_BAND_OUTSET_M = 0.04  # 4cm outward tolerance for skinny corridors
        
        # ----- No-frontier statistics (large-component case) -----
        # 英文: count consecutive "no frontier but component is large" events, to trigger ring-sampling fallback.
        self._no_frontier_large_cnt: int = 0
        # 最近一次 C-space 连通域像素数（ROI坐标系）
        self._last_conn_count: int = 0

    def _log_debug(self, message: str) -> None:
        """Add debug message to log and integrate with main log_to_file system"""
        # 输出到控制台
        print(f"[EXPLORE_DEBUG] {message}")
        
        # 集成到main函数的log_to_file系统
        if self.logger_func and self.log_file:
            self.logger_func(self.log_file, message, "EXPLORE")
    
    # --- Stabilize occ grid against known→unknown rollback (read-only) -------------
    def _stabilize_occ(self, occ: np.ndarray) -> np.ndarray:
        if self._latch_map is None or self._latch_map.shape != occ.shape:
            self._latch_map = np.full_like(occ, 2, dtype=occ.dtype)  # 2=UNKNOWN
        known = (occ != 2)
        self._latch_map[known] = occ[known]          # remember current known
        out = occ.copy()
        rollback = (occ == 2) & (self._latch_map != 2)
        if rollback.any():
            out[rollback] = self._latch_map[rollback]  # fill back known
        # detect sudden known drop → disable ROI next call
        cur_known = int(known.sum())
        if cur_known + int(0.05 * occ.size) < self._last_known_cnt:
            self._log_debug("⚠️ Known drop detected; disable ROI for next frontier cycle.")
            self._roi_force_full = True
        self._last_known_cnt = cur_known
        return out

    def _slam_grid_to_world(self, grid_i: int, grid_j: int) -> Tuple[float, float]:
        """使用SLAM系统的坐标转换逻辑"""
        if self.slam_system:
            # Use actual SLAM resolution from system
            res = self.slam_system.res  # Get actual resolution (0.01m/pixel)
            world_x = grid_j * res  # j corresponds to x coordinate
            world_y = grid_i * res  # i corresponds to y coordinate
            return (world_x, world_y)
        else:
            # Fallback to default conversion
            return map_to_world(grid_i, grid_j)

    def _inflate_to_cspace(self, occ: np.ndarray, inflate_px: int) -> np.ndarray:
        """
        Create C-space using selective inflation: only inflate internal obstacles, exclude world boundaries.
        English:
          - Only inflate obstacles that are NOT on world boundaries
          - This prevents boundary inflation from blocking robot movement at maze edges
        
        Args:
            occ: Original occupancy grid (0=free, 1=obstacle, 2=unknown)
            inflate_px: Inflation radius in pixels (SAFE_BUFFER_M / resolution)
            
        Returns:
            Inflated C-space grid with same semantics (0/1/2)
        """
        import numpy as np
        from scipy.ndimage import maximum_filter1d

        # Masks
        obst = (occ == 1)   # obstacles to inflate
        unk  = (occ == 2)   # preserve unknown semantics

        # English: prevent world-border inflation
        # 清零世界边界，防止边界膨胀阻塞起点/终点
        obst[0, :]=0; obst[-1,:]=0; obst[:, 0]=0; obst[:, -1]=0

        # Rectangular (box) inflation for internal obstacles only
        if inflate_px > 0 and obst.any():
            dil = obst.astype(np.uint8)
            # English: 1D max-filter along each axis yields a 2D box dilation.
            dil = maximum_filter1d(dil, size=2*inflate_px + 1, axis=0, mode="nearest")
            dil = maximum_filter1d(dil, size=2*inflate_px + 1, axis=1, mode="nearest")
            inflated_obst = (dil > 0)
        else:
            inflated_obst = obst

        # Build C-space grid with original semantics
        out = np.zeros_like(occ, dtype=occ.dtype)
        out[inflated_obst] = 1    # inflated obstacles
        out[unk] = 2              # keep unknown as UNKNOWN
        # free cells remain 0
        return out
    
    # ----------------------- Memory helpers -----------------------
    def _remember_clusters(self, clusters: Sequence[Sequence[Tuple[int,int]]], res: float) -> None:
        """把本轮发现的前沿簇中心写入记忆队列（不做规划校验，只做备份）。"""
        if not clusters:
            return
        now = time.time()
        for cl in clusters:
            ci = int(sum(p[0] for p in cl) / len(cl))
            cj = int(sum(p[1] for p in cl) / len(cl))
            wx, wy = cj * res, ci * res
            self._frontier_mem.append((ci, cj, wx, wy, now))
        self._log_debug(f"📝 记忆前沿簇中心 {len(clusters)} 个（mem size={len(self._frontier_mem)}）")

    def _remember_point(self, gi: int, gj: int, res: float) -> None:
        """把最终选中的前沿点记进记忆队列。"""
        now = time.time()
        wx, wy = gj * res, gi * res
        self._frontier_mem.append((gi, gj, wx, wy, now))
        self._log_debug(f"📝 记忆最终选点: grid=({gi},{gj}) world=({wx:.2f},{wy:.2f})")

    def _fallback_from_memory(self, occ_grid: np.ndarray, pose: Pose2D) -> Optional[Tuple[float,float]]:
        """
        所有新候选失败后，从记忆队列回退：
        - 只尝试最近 40 个里的新近（<=120s）条目
        - 规划仍在同一张 C-space（若已有缓存）；否则用原图
        - 若目标格不是 C-space 自由格，允许朝向机器人回缩 1px
        """
        if not self._frontier_mem:
            return None

        # 构造与主流程一致的碰撞世界
        cspace_bin = self._last_cspace_bin
        if cspace_bin is None:
            # 没有缓存则临时构建一份
            from core.config import SLAM_RESOLUTION, SAFE_BUFFER_M
            res = self.slam_system.res if (self.slam_system and hasattr(self.slam_system, 'res')) else SLAM_RESOLUTION
            inflate_px = max(1, int(round(SAFE_BUFFER_M / res)))
            cspace = self._inflate_to_cspace(occ_grid, inflate_px)
            cspace_bin = (cspace != 0).astype(np.uint8)
        H, W = cspace_bin.shape

        res = self.slam_system.res if (self.slam_system and hasattr(self.slam_system, 'res')) else 0.01
        ri, rj = int(round(pose.y / res)), int(round(pose.x / res))

        now = time.time()
        tried = 0
        # 逆序遍历（最近的先试）
        for gi, gj, wx, wy, ts in list(self._frontier_mem)[::-1]:
            if now - ts > 120.0:
                continue
            if not (0 <= gi < H and 0 <= gj < W):
                continue
            tried += 1
            self._log_debug(f"↩️ 记忆回退尝试 #{tried}: grid=({gi},{gj}) world=({wx:.2f},{wy:.2f})")

            # 若目标格被占，用 1px 朝机器人方向的微调
            if cspace_bin[gi, gj] != 0:
                di = int(np.sign(ri - gi)) if ri != gi else 0
                dj = int(np.sign(rj - gj)) if rj != gj else 0
                gi2, gj2 = gi + di, gj + dj
                if 0 <= gi2 < H and 0 <= gj2 < W and cspace_bin[gi2, gj2] == 0:
                    gi, gj = gi2, gj2
                else:
                    self._log_debug("   回退点周围仍不可达，跳过")
                    continue

            # 直接在 C-space 上做一次 A* 可达性验证
            path = self.planner.plan((ri, rj), (gi, gj), cspace_bin, grid_is_cspace=True)
            if path:
                self._log_debug(f"   ✅ 记忆回退成功，使用旧点 world=({gj*res:.2f},{gi*res:.2f})")
                return (gj*res, gi*res)

        self._log_debug("↩️ 记忆回退全部失败")
        return None
    
    # ------------------------------------------------------------------
    # Ring-sampling information gain fallback (when no frontier found)
    # ------------------------------------------------------------------
    def _ring_sampling_fallback(self, occ_full: np.ndarray, pose: Pose2D) -> Optional[Tuple[Tuple[float,float], Tuple[float,float,float,float], dict]]:
        """
        在机器人周围半径 R 取 K 个角度样本，样本必须落在 C-space 自由格上；
        信息增益 = 样本点邻域(半径=传感器量程)内的未知像素数；评分 = 信息增益 / 路径代价，
        并加一个对齐轴向的轻微偏好。返回 (world_xy, goal_band)。
        """
        from core.config import SLAM_RESOLUTION, SAFE_BUFFER_M, LIDAR_RANGE
        res = self.slam_system.res if (self.slam_system and hasattr(self.slam_system,'res')) else SLAM_RESOLUTION
        inflate_px = max(1, int(round(SAFE_BUFFER_M / res)))
        # 构造与主流程一致的 C-space（二值）
        cspace = self._inflate_to_cspace(occ_full, inflate_px)
        cbin = (cspace != 0).astype(np.uint8)  # 0=free,1=blocked
        H, W = cbin.shape

        ri, rj = int(round(pose.y / res)), int(round(pose.x / res))
        if not (0 <= ri < H and 0 <= rj < W) or cbin[ri, rj] != 0:
            self._log_debug("  [RING] 机器人位置不在自由区，放弃兜底")
            return None

        # 预计算可达性：避免对明显不可达点做代价高的 A*
        def grid_ok(i, j) -> bool:
            return (0 <= i < H and 0 <= j < W and cbin[i, j] == 0)

        Rm = self._ring_R_m
        K  = self._ring_K
        sR = LIDAR_RANGE
        sR_px = int(round(sR / res))

        unk = (occ_full == 2)
        best = None  # (score, (wx,wy), (xmin,xmax,ymin,ymax))

        for k in range(K):
            th = 2.0 * math.pi * (k / K)
            wx = pose.x + Rm * math.cos(th)
            wy = pose.y + Rm * math.sin(th)
            gi, gj = int(round(wy / res)), int(round(wx / res))
            if not grid_ok(gi, gj):
                continue

            # 局部信息增益：统计圆邻域内未知像素
            i0 = max(0, gi - sR_px); i1 = min(H-1, gi + sR_px)
            j0 = max(0, gj - sR_px); j1 = min(W-1, gj + sR_px)
            sub = unk[i0:i1+1, j0:j1+1]
            # 圆掩膜
            yy, xx = np.ogrid[i0:i1+1, j0:j1+1]
            mask = (yy - gi)**2 + (xx - gj)**2 <= (sR_px*sR_px)
            gain = int(sub[mask].sum())
            if gain <= 0:
                continue

            # A* 代价（在同一份 C-space 上）
            path = self.planner.plan((ri, rj), (gi, gj), cbin, grid_is_cspace=True)
            if not path:
                continue
            length_m = (len(path) - 1) * res

            # 轴向偏好（越接近 0/90/180/270° 越好）
            d_axis = min(abs((th) % (math.pi/2)),
                         abs((th-math.pi/2) % (math.pi/2)))
            axis_pen = self._axis_pref_w * (d_axis / (math.pi/4))  # 归一化到 [0,1]

            score = gain / (length_m + 1e-3) - axis_pen

            # 为候选点生成 band（若失败就跳过）
            band = self._compute_goal_band_for_frontier((wx, wy), pose, occ_full)
            if band is None:
                continue

            # 到达带外扩（默认 4cm），避免窄走廊"擦不到门线"
            band_pad = getattr(self, "GOAL_BAND_OUTSET_M", 0.04)  # 可在 __init__ 里设置
            xmin, xmax, ymin, ymax = band
            band = (xmin - band_pad, xmax + band_pad, ymin - band_pad, ymax + band_pad)

            if (best is None) or (score > best[0]):
                best = (score, (wx, wy), band)

        if best is None:
            self._log_debug("  [RING] 无有效环采样候选")
            return None
        self._log_debug(f"  [RING] 选择分数最高的候选: score={best[0]:.2f}, world=({best[1][0]:.2f},{best[1][1]:.2f})")
        
        # 返回阻塞掩码，告知 planner 这是 C-space
        return (best[1], best[2], {
            "source": "ring_gain",
            "blocked_mask": cbin,  # bool HxW
        })
    
    # ---------------------------------------------------------------------
    # Goal-band 计算（与 Navigator 逻辑对齐的"条带目标"）
    # ---------------------------------------------------------------------
    def _compute_goal_band_for_frontier(
        self,
        frontier_world: Tuple[float, float],
        pose: Pose2D,
        occ_full: np.ndarray
    ) -> Optional[Tuple[float, float, float, float]]:
        """
        English:
          Build a rectangular goal-band around the frontier using the SAME
          collision world as verification (SAFE_BUFFER C-space). The band
          is constructed by scanning to the first occupied/unknown indices
          along axes from the frontier (snapped to nearest free if needed),
          then selecting horizontal/vertical strip whose orthogonal span
          best matches CORRIDOR_WIDTH_M and has positive area.
        Returns:
          (xmin, xmax, ymin, ymax) in meters, or None if invalid.
        """
        from core.config import SLAM_RESOLUTION, SAFE_BUFFER_M, CORRIDOR_GOAL_TOL_M, CORRIDOR_WIDTH_M
        from scipy.ndimage import distance_transform_edt

        res = self.slam_system.res if (self.slam_system and hasattr(self.slam_system, 'res')) else SLAM_RESOLUTION
        gx, gy = frontier_world  # world meters

        # —— 用与验证一致的 C-space（仅 SAFE_BUFFER 膨胀）——
        inflate_px = max(1, int(round(SAFE_BUFFER_M / res)))
        cspace = self._inflate_to_cspace(occ_full, inflate_px)     # 0/1/2
        cbin   = (cspace != 0).astype(np.uint8)                    # 0=free,1=blocked
        H, W = cbin.shape

        def w2ij(x: float, y: float) -> Tuple[int, int]:
            i = int(np.floor(y / res))
            j = int(np.floor(x / res))
            return max(0, min(H - 1, i)), max(0, min(W - 1, j))

        ig, jg = w2ij(gx, gy)
        # 若前沿点在 C-space 非自由，吸附最近自由格
        if cbin[ig, jg] != 0:
            occ_mask = (cbin != 0).astype(np.uint8)
            _, (iy, ix) = distance_transform_edt(occ_mask, return_distances=True, return_indices=True)
            ig, jg = int(iy[ig, jg]), int(ix[ig, jg])
            if cbin[ig, jg] != 0:
                return None

        # 轴向扫描（到第一处非自由）
        def scan_left(i, j):
            k = 0
            while j - k - 1 >= 0 and cbin[i, j - k - 1] == 0:
                k += 1
            return k
        def scan_right(i, j):
            k = 0
            while j + k + 1 < W and cbin[i, j + k + 1] == 0:
                k += 1
            return k
        def scan_up(i, j):
            k = 0
            while i - k - 1 >= 0 and cbin[i - k - 1, j] == 0:
                k += 1
            return k
        def scan_down(i, j):
            k = 0
            while i + k + 1 < H and cbin[i + k + 1, j] == 0:
                k += 1
            return k

        rL, rR = scan_left(ig, jg), scan_right(ig, jg)
        rU, rD = scan_up(ig, jg), scan_down(ig, jg)
        tol_pix = max(1, int(round(CORRIDOR_GOAL_TOL_M / res)))

        # 水平候选（竖直条带，x 轴为目标轴）
        j0 = jg - min(tol_pix, rL)
        j1 = jg + min(tol_pix, rR)
        # 纵向自由交集，保证整条竖条带可行
        i0_list, i1_list = [], []
        for jj in range(j0, j1 + 1):
            ii0 = ig
            while ii0 - 1 >= 0 and cbin[ii0 - 1, jj] == 0:
                ii0 -= 1
            ii1 = ig
            while ii1 + 1 < H and cbin[ii1 + 1, jj] == 0:
                ii1 += 1
            i0_list.append(ii0)
            i1_list.append(ii1)
        i0_h = int(max(i0_list)) if i0_list else ig
        i1_h = int(min(i1_list)) if i1_list else ig

        # 垂直候选（水平条带，y 轴为目标轴）
        i0v = ig - min(tol_pix, rU)
        i1v = ig + min(tol_pix, rD)
        j0_list, j1_list = [], []
        for ii in range(i0v, i1v + 1):
            jj0 = jg
            while jj0 - 1 >= 0 and cbin[ii, jj0 - 1] == 0:
                jj0 -= 1
            jj1 = jg
            while jj1 + 1 < W and cbin[ii, jj1 + 1] == 0:
                jj1 += 1
            j0_list.append(jj0)
            j1_list.append(jj1)
        j0_v = int(max(j0_list)) if j0_list else jg
        j1_v = int(min(j1_list)) if j1_list else jg

        # 像素转世界
        def px2x(j0, j1):
            if j0 > j1: return gx, gx
            return (j0 * res, (j1 + 1) * res)
        def px2y(i0, i1):
            if i0 > i1: return gy, gy
            return (i0 * res, (i1 + 1) * res)
        xL_h, xR_h = px2x(j0, j1);     yB_h, yT_h = px2y(i0_h, i1_h)
        xL_v, xR_v = px2x(j0_v, j1_v); yB_v, yT_v = px2y(i0v,  i1v)
        cand_h = (xL_h, xR_h, yB_h, yT_h)
        cand_v = (xL_v, xR_v, yB_v, yT_v)

        def area(r):
            xmin, xmax, ymin, ymax = r
            return max(0.0, xmax - xmin) * max(0.0, ymax - ymin)
        a_h, a_v = area(cand_h), area(cand_v)
        err_h = abs((yT_h - yB_h) - float(CORRIDOR_WIDTH_M))
        err_v = abs((xR_v - xL_v) - float(CORRIDOR_WIDTH_M))

        # 选择：优先面积>0，其次更贴近走廊宽
        if (a_h > 0 and a_v > 0 and err_h <= err_v) or (a_h > 0 and a_v <= 0):
            # defensive normalization
            xmin, xmax, ymin, ymax = cand_h
            xmin, xmax = min(xmin, xmax), max(xmin, xmax)
            ymin, ymax = min(ymin, ymax), max(ymin, ymax)
            return (xmin, xmax, ymin, ymax)
        if a_v > 0:
            xmin, xmax, ymin, ymax = cand_v
            xmin, xmax = min(xmin, xmax), max(xmin, xmax)
            ymin, ymax = min(ymin, ymax), max(ymin, ymax)
            return (xmin, xmax, ymin, ymax)
        return None
    

    def find_frontiers(self, occ: np.ndarray, robot_pose: Pose2D) -> Sequence[Sequence[Tuple[int,int]]]:
        """
        Industrial Standard: Find frontiers in C-space (inflated free space) with performance optimization.
        
        Key changes from original:
        1. Throttle frontier extraction to avoid excessive computation
        2. Work only in robot's field of view + margin (ROI) to reduce O(HW) operations
        3. Create C-space by inflating obstacles (SAFE_BUFFER_M)
        4. Compute connected component in C-space (not original map)
        5. Find frontiers where C-space free meets original unknown
        
        This ensures frontiers are strictly in safe navigable region.
        
        Args:
            occ: Original occupancy grid (0=free, 1=obstacle, 2=unknown)
            robot_pose: Current robot pose
            
        Returns:
            List of frontier clusters (each cluster is list of (i,j) coordinates)
        """
        # English: Make global occ temporarily read-only during frontier to catch illegal writes
        with _readonly(occ):
            occ_stable = self._stabilize_occ(occ)
            return self._find_frontiers_impl(occ_stable, robot_pose)
    
    def _find_frontiers_impl(self, occ: np.ndarray, robot_pose: Pose2D) -> Sequence[Sequence[Tuple[int,int]]]:
        """
        Internal implementation of frontier finding with read-only protection.
        """
        import time
        from collections import deque
        from core.config import SLAM_RESOLUTION, SAFE_BUFFER_M
        
        # === Performance optimization: throttle frontier extraction ===
        # English: Only extract frontiers every _frontier_min_interval seconds to avoid excessive computation
        now = time.time()
        if now - self._last_frontier_ts < self._frontier_min_interval:
            self._log_debug(f"前沿提取节流: 距离上次提取仅{now - self._last_frontier_ts:.2f}s < {self._frontier_min_interval}s")
            return getattr(self, "_last_frontiers", [])
        
        self._log_debug("========== find_frontiers 开始 ==========")
        self._log_debug(f"机器人世界坐标: ({robot_pose.x:.3f}, {robot_pose.y:.3f})")
        
        # 使用SLAM真实分辨率，避免与常量不一致
        res = self.slam_system.res if (self.slam_system and hasattr(self.slam_system, 'res')) else SLAM_RESOLUTION
        # Convert robot pose to grid coordinates
        robot_i = int(round(robot_pose.y / res))
        robot_j = int(round(robot_pose.x / res))
        self._log_debug(f"机器人网格坐标: ({robot_i}, {robot_j})")
        
        H, W = occ.shape
        self._log_debug(f"地图尺寸: ({H}, {W})")
        
        # === ROI cropping with fallback ============================================
        # English: Only work in robot's field of view + margin to reduce computation from O(HW) to O(πr²)
        # Calculate ROI based on sensor range + margin
        if hasattr(self.slam_system, 'get_sensor_max_range'):
            sensor_range = float(self.slam_system.get_sensor_max_range())
        else:
            sensor_range = LIDAR_RANGE
        
        # r: ROI radius in meters
        # Clamp by WORLD_SIZE to keep ROI consistent on 2.8×2.8m maps.
        from core.config import WORLD_SIZE
        r = min(sensor_range + self._roi_margin_m, WORLD_SIZE)  # English: avoid oversized ROI
        ri = int(r / res)
        
        if self._roi_force_full:
            i0, i1, j0, j1 = 0, H, 0, W
            self._roi_force_full = False
            self._log_debug("🔁 ROI disabled for this cycle (fallback to full map).")
        else:
            i0, i1 = max(0, robot_i - ri), min(H, robot_i + ri + 1)
            j0, j1 = max(0, robot_j - ri), min(W, robot_j + ri + 1)
        
        self._log_debug(f"ROI范围: ({i0}:{i1}, {j0}:{j1}), 半径={ri}px ({r:.2f}m)")
        
        # Extract ROI
        # IMPORTANT: copy to avoid accidental in-place writes on the global map
        # English: .copy() prevents any library function from writing back to the global map
        occ_roi = occ[i0:i1, j0:j1].copy()
        H_roi, W_roi = occ_roi.shape
        
        # Convert robot position to ROI coordinates
        robot_i_roi = robot_i - i0
        robot_j_roi = robot_j - j0
        
        # === [KEY CHANGE] Create C-space by inflating obstacles (on ROI) ===
        inflate_px = max(1, int(round(SAFE_BUFFER_M / res)))
        self._log_debug(f"创建C-space: 膨胀半径={inflate_px}px ({SAFE_BUFFER_M}m)")
        
        cspace_roi = self._inflate_to_cspace(occ_roi, inflate_px)
        # English: Ensure robot seed is free for connectivity. This prevents
        # false deadlock when robot is exactly on the buffer boundary.
        if 0 <= robot_i_roi < H_roi and 0 <= robot_j_roi < W_roi:
            cspace_roi[robot_i_roi, robot_j_roi] = 0
        free_mask_roi = (cspace_roi == 0)            # C-space free cells (safe to navigate)
        unknown_mask_roi = (occ_roi == 2)            # Unknown from original map

        from scipy.ndimage import binary_dilation, generate_binary_structure
        _struct = generate_binary_structure(2, 1)  # 4-connected base, dilation gives 8-neigh effect
        unk_band_roi = binary_dilation(unknown_mask_roi, structure=_struct, iterations=2)
        
        # Validate robot position (in ROI)
        if not (0 <= robot_i_roi < H_roi and 0 <= robot_j_roi < W_roi):
            self._log_debug(f"❌ 机器人位置越界: ({robot_i_roi},{robot_j_roi}) in ROI")
            return []
        
        # Check robot is in C-space free area (in ROI)
        if not free_mask_roi[robot_i_roi, robot_j_roi]:
            self._log_debug(f"⚠️ 机器人不在C-space自由区，尝试吸附到最近自由格")
            self._log_debug(f"   原始地图: occ_roi[{robot_i_roi},{robot_j_roi}] = {occ_roi[robot_i_roi, robot_j_roi]}")
            self._log_debug(f"   C-space: cspace_roi[{robot_i_roi},{robot_j_roi}] = {cspace_roi[robot_i_roi, robot_j_roi]}")
            
            # English: Snap BFS seed to nearest FREE cell within r<=3 if current cell is not free.
            # This handles localization errors or single-pixel noise that might mark robot position as occupied.
            snapped = False
            for r in range(1, 4):
                for di in range(-r, r + 1):
                    for dj in range(-r, r + 1):
                        ni, nj = robot_i_roi + di, robot_j_roi + dj
                        if 0 <= ni < H_roi and 0 <= nj < W_roi and free_mask_roi[ni, nj]:
                            self._log_debug(f"   ✅ 吸附到最近自由格: ({ni}, {nj}), 距离={r}px")
                            robot_i_roi, robot_j_roi = ni, nj
                            snapped = True
                            break
                    if snapped:
                        break
                if snapped:
                    break
            
            # If still not in free space after snapping, return empty
            if not free_mask_roi[robot_i_roi, robot_j_roi]:
                self._log_debug(f"   ❌ 无法找到附近的自由格，放弃前沿搜索")
                return []
        else:
            self._log_debug(f"✅ 机器人在C-space自由区")
        
        # === BFS in C-space to find connected free component (in ROI) ===
        conn_roi = np.zeros_like(free_mask_roi, dtype=bool)
        q = deque([(robot_i_roi, robot_j_roi)])
        conn_roi[robot_i_roi, robot_j_roi] = True
        
        while q:
            ci, cj = q.popleft()
            for di, dj in ((1,0), (-1,0), (0,1), (0,-1)):
                ni, nj = ci + di, cj + dj
                if 0 <= ni < H_roi and 0 <= nj < W_roi and (not conn_roi[ni, nj]) and free_mask_roi[ni, nj]:
                    conn_roi[ni, nj] = True
                    q.append((ni, nj))
        
        connected_free_count = int(conn_roi.sum())
        self._last_conn_count = connected_free_count
        self._log_debug(f"✅ C-space BFS完成，连通域大小: {connected_free_count}个自由格")
        
        conn_band_roi = binary_dilation(conn_roi, structure=_struct, iterations=1)

        # === Find frontiers: FREE in original map, reachable in C-space (in ROI) ===
        def is_frontier_on_conn(i: int, j: int) -> bool:
            # English:
            # - reachable: within 1px band of the C-space connected component
            # - safe: FREE in the original map
            # - frontier: lies inside a 2px unknown band (captures diagonal/offset edges)
            if not conn_band_roi[i, j]:
                return False
            if occ_roi[i, j] != 0:
                return False
            return bool(unk_band_roi[i, j])
        
        # === Step 3: Cluster frontiers (in ROI) ===
        visited_roi = np.zeros_like(occ_roi, dtype=bool)
        clusters_roi = []
        
        for i in range(H_roi):
            for j in range(W_roi):
                if visited_roi[i, j] or not is_frontier_on_conn(i, j):
                    continue
                # BFS to cluster adjacent frontiers
                q = deque([(i, j)])
                visited_roi[i, j] = True
                cl = []
                while q:
                    ci, cj = q.popleft()
                    cl.append((ci, cj))
                    # English: use 8-connectivity to merge diagonal frontier pixels
                    # This prevents fragmentation of frontier clusters on corridor edges
                    for di, dj in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                        ni, nj = ci + di, cj + dj
                        if (0 <= ni < H_roi and 0 <= nj < W_roi and
                            not visited_roi[ni, nj] and is_frontier_on_conn(ni, nj)):
                            visited_roi[ni, nj] = True
                            q.append((ni, nj))
                # Accept cluster if size >= 3
                if len(cl) >= 3:
                    clusters_roi.append(cl)
        
        # === Map clusters back to global coordinates ===
        # English: Convert ROI coordinates back to global map coordinates
        clusters = [[(i+i0, j+j0) for (i, j) in cl] for cl in clusters_roi]
        
        self._log_debug(f"找到 {len(clusters)} 个前沿点集群")
        
        # Cache results for throttling
        self._last_frontier_ts = now
        self._last_frontiers = clusters
        
        return clusters

    def choose_next_frontier(self, occ_grid: np.ndarray, pose: Pose2D, hint: Optional[Dict]=None) -> Optional[Dict]:
        """智能策略：
        1) 地图稀疏或本地自由少 → 优先环采样(带前进偏置且禁止回头)；
        2) 否则走传统前沿；环采样失败也回退到传统前沿。
        """
        # 保存occ_grid供中线计算使用
        self._last_occ_grid = occ_grid
        
        # 添加调试信息
        self._log_debug(f"开始选择前沿点，当前位置: ({pose.x:.2f}, {pose.y:.2f})")
        
        # 添加地图状态分析
        known_cells = (occ_grid != 2).sum()
        unknown_cells = (occ_grid == 2).sum()
        free_cells = (occ_grid == 0).sum()
        occupied_cells = (occ_grid == 1).sum()
        
        self._log_debug(f"地图状态分析:")
        self._log_debug(f"  - 已知区域: {known_cells} 网格 ({known_cells/occ_grid.size*100:.1f}%)")
        self._log_debug(f"  - 未知区域: {unknown_cells} 网格 ({unknown_cells/occ_grid.size*100:.1f}%)")
        self._log_debug(f"  - 自由空间: {free_cells} 网格 ({free_cells/occ_grid.size*100:.1f}%)")
        self._log_debug(f"  - 障碍物: {occupied_cells} 网格 ({occupied_cells/occ_grid.size*100:.1f}%)")
        
        occ_stable = self._stabilize_occ(occ_grid)
        cspace_bin = self._build_cspace(occ_stable)
        self._last_cspace_bin = cspace_bin  # 供可视化或回退使用

        # --- 策略门：根据全局和本地信息判断是否优先环采样 ---
        known_ratio = float((occ_stable != 2).sum()) / float(occ_stable.size)
        free_ratio  = float((occ_stable == 0).sum()) / float(occ_stable.size)
        local_free  = self._count_local_free(occ_stable, pose, radius_m=0.6)
        use_ring = self._use_ring_primary or (known_ratio < 0.30) or (free_ratio < 0.15) or (local_free < 80)

        if use_ring:
            self._log_debug("采用环采样作为主策略")
            cand = self._ring_select_forward_biased(occ_stable, cspace_bin, pose, hint)
            if cand is not None:
                (wx, wy), band = cand
                return {'frontier': (wx, wy), 'goal_band': band, 'source': 'ring_gain'}

        # 回退：传统前沿（保持原有实现）
        self._log_debug("回退到传统前沿探索")
        return self._choose_frontier_traditional(occ_stable, pose)

    # ---- 新增：本地自由格统计 ----
    def _count_local_free(self, occ: np.ndarray, pose: Pose2D, radius_m: float = 0.6) -> int:
        r_px = int(max(1, round(radius_m / SLAM_RESOLUTION)))
        i = int(round(pose.x / SLAM_RESOLUTION))
        j = int(round(pose.y / SLAM_RESOLUTION))
        h, w = occ.shape
        i0, i1 = max(0, i-r_px), min(w-1, i+r_px)
        j0, j1 = max(0, j-r_px), min(h-1, j+r_px)
        window = occ[j0:j1+1, i0:i1+1]
        return int((window == 0).sum())

    # ---- 新增：前进偏置的环采样选点（禁止回头）----
    def _ring_select_forward_biased(self, occ: np.ndarray, cspace: np.ndarray, pose: Pose2D, hint: Optional[Dict]) -> Optional[Tuple[Tuple[float,float], Tuple[float,float,float,float]]]:
        # 取当前朝向
        heading = 0.0
        if hint and 'current_heading' in hint:
            heading = float(hint['current_heading'])
        fwd = (math.cos(heading), math.sin(heading))  # 世界系前进方向

        # 1) 计算前向自由距离，决定是否压制前向候选
        clear_fwd = self._estimate_clearance_along(occ, pose, heading, max_m=0.6)
        suppress_forward = (clear_fwd < FWD_SUPPRESS_CLEARANCE_M)

        R = self._ring_R_m
        K = self._ring_K
        best = None
        best_score = -1e9

        for k in range(K):
            ang = 2.0 * math.pi * (k / K)
            ux, uy = math.cos(ang), math.sin(ang)
            # 前进性：与fwd的余弦
            fwd_dot = ux * fwd[0] + uy * fwd[1]
            # 严格禁止回头：低于阈值直接跳过
            if fwd_dot < self._ring_min_fwd_dot:
                continue
            
            # 计算角度差
            ang_diff = abs((ang - heading + math.pi) % (2*math.pi) - math.pi)
            
            # 2) 前向压制：前向扇区内的候选直接跳过
            if suppress_forward and ang_diff < math.radians(FWD_SUPPRESS_CONE_DEG):
                continue
                
            # 射线末点
            wx = pose.x + R * ux
            wy = pose.y + R * uy
            
            # 估计增益：沿射线统计"未知→自由"的转换数，越多越好；若碰撞则惩罚
            gain, blocked = self._ray_unknown_gain(occ, pose.x, pose.y, wx, wy, step_m=SLAM_RESOLUTION)
            if blocked:
                continue
            
            # 基础评分
            score = gain
            
            # 3) 侧向轻加分：促使在转角处选"左/右"而非继续直行
            if ang_diff > math.radians(SIDE_TURN_MIN_DELTA_DEG):
                score += SIDE_TURN_BONUS
            
            # 轻微朝向偏置，鼓励贴近纵横轴减少抖动
            axis_bias = self._axis_pref_w * (abs(ux) > 0.92 or abs(uy) > 0.92)
            score += axis_bias
            
            # 只有在不压制前向时才加前向偏向
            if not suppress_forward:
                score += 0.3 * fwd_dot
            
            # 边界距离惩罚：距离边界越近惩罚越大
            boundary_penalty = self._calculate_boundary_penalty(wx, wy)
            score += boundary_penalty
                
            if score > best_score:
                best_score = score
                best = (wx, wy)

        if best is None:
            return None

        # 目标band：以SAFE_BUFFER_M构造薄矩形门槛，垂直于前进向量
        wx, wy = best
        bx = max(0.02, min(0.12, SAFE_BUFFER_M))  # 宽度
        by = max(0.02, min(0.20, SAFE_BUFFER_M*1.8))  # 高度
        # 使band法向量近似与(ux,uy)一致：横向更窄，纵向更长
        xmin, xmax = wx - bx*0.5, wx + bx*0.5
        ymin, ymax = wy - by*0.5, wy + by*0.5
        return ( (wx, wy), (xmin, xmax, ymin, ymax) )

    def _estimate_clearance_along(self, occ: np.ndarray, pose: Pose2D, heading_rad: float, max_m: float = 0.6) -> float:
        """沿heading估算自由距离：逐像素ray-cast，遇到障碍即停。"""
        res = SLAM_RESOLUTION
        step_pix = 1
        max_pix = int(max_m / res)
        h, w = occ.shape
        
        # 世界->栅格
        gi = int(round(pose.x / res))
        gj = int(round(pose.y / res))
        
        di = math.cos(heading_rad)
        dj = math.sin(heading_rad)
        
        for t in range(1, max_pix+1):
            ii = int(round(gi + di * (t * step_pix)))
            jj = int(round(gj + dj * (t * step_pix)))
            
            if ii < 0 or jj < 0 or ii >= w or jj >= h:
                return t * res
            
            if occ[jj, ii] == 1:  # 障碍
                return t * res
        
        return max_m

    def _ray_unknown_gain(self, occ: np.ndarray, sx: float, sy: float, ex: float, ey: float, step_m: float) -> Tuple[float, bool]:
        """沿射线估计未知→自由的收益，若撞到障碍返回blocked=True。"""
        dx, dy = ex - sx, ey - sy
        L = max(1e-6, math.hypot(dx, dy))
        n = max(2, int(L / max(step_m, 1e-3)))
        gain = 0.0
        blocked = False
        last = 2  # 假设起点外圈未知
        h, w = occ.shape
        for t in range(1, n+1):
            x = sx + dx * (t / n)
            y = sy + dy * (t / n)
            i = int(round(x / SLAM_RESOLUTION))
            j = int(round(y / SLAM_RESOLUTION))
            if i < 0 or i >= w or j < 0 or j >= h:
                break
            v = int(occ[j, i])
            if v == 1:
                blocked = True
                break
            if last == 2 and v == 0:
                gain += 1.0
            last = v
        return gain, blocked

    def _calculate_boundary_penalty(self, wx: float, wy: float) -> float:
        """
        计算边界距离惩罚，距离边界越近惩罚越大
        
        Args:
            wx: 前沿点世界坐标 x (米)
            wy: 前沿点世界坐标 y (米)
            
        Returns:
            边界惩罚分数 (负值表示惩罚)
        """
        if not BOUNDARY_PENALTY_ENABLED:
            return 0.0
            
        # 计算到各边界的距离
        dist_to_left = wx                    # 到左边界 (x=0) 的距离
        dist_to_right = WORLD_SIZE - wx      # 到右边界 (x=2.8) 的距离  
        dist_to_bottom = wy                 # 到下边界 (y=0) 的距离
        dist_to_top = WORLD_SIZE - wy       # 到上边界 (y=2.8) 的距离
        
        # 取最小边界距离
        min_boundary_dist = min(dist_to_left, dist_to_right, dist_to_bottom, dist_to_top)
        
        # 边界惩罚：距离越近惩罚越大
        if min_boundary_dist <= 0.1:        # 距离边界 <= 10cm，严重惩罚
            penalty = BOUNDARY_PENALTY_SEVERE
            self._log_debug(f"边界严重惩罚: 点({wx:.2f},{wy:.2f}) 距离边界{min_boundary_dist:.2f}m 惩罚={penalty}")
        elif min_boundary_dist <= 0.2:      # 距离边界 <= 20cm，中等惩罚  
            penalty = BOUNDARY_PENALTY_MEDIUM
            self._log_debug(f"边界中等惩罚: 点({wx:.2f},{wy:.2f}) 距离边界{min_boundary_dist:.2f}m 惩罚={penalty}")
        elif min_boundary_dist <= BOUNDARY_SAFE_DISTANCE:  # 距离边界 <= 安全距离，轻微惩罚
            penalty = BOUNDARY_PENALTY_LIGHT
            self._log_debug(f"边界轻微惩罚: 点({wx:.2f},{wy:.2f}) 距离边界{min_boundary_dist:.2f}m 惩罚={penalty}")
        else:                               # 距离边界 > 安全距离，无惩罚
            penalty = 0.0
            
        return penalty

    # ---- 传统前沿选择封装（保持你原有实现入口名）----
    def _choose_frontier_traditional(self, occ_stable: np.ndarray, pose: Pose2D) -> Optional[Dict]:
        """传统前沿探索方法，保持原有逻辑"""
        frontiers = self.find_frontiers(occ_stable, pose)
        self._log_debug(f"找到 {len(frontiers)} 个前沿点集群")
        # 把本轮发现的前沿簇中心写入记忆（不影响本轮决策）
        res_for_mem = self.slam_system.res if (self.slam_system and hasattr(self.slam_system, 'res')) else 0.01
        self._remember_clusters(frontiers, res_for_mem)
        
        if not frontiers:
            # 统计raw前沿点数量
            self.fail_raw += 1
            self._log_debug(f"❌ 没有找到前沿点集群 (fail_raw={self.fail_raw})")
            
            # 检测连通域大小，如果为1则返回恢复信号
            if hasattr(self, '_last_conn_count') and self._last_conn_count == 1:
                self._log_debug("🚨 连通域大小为1，需要恢复模式")
                return {'recovery_needed': True, 'reason': 'connectivity_size_1'}
            
            # 尝试记忆回退
            fallback = self._fallback_from_memory(occ_stable, pose)
            if fallback is not None:
                return fallback
            return None
        
        # 处理找到的前沿点
        # 这里保持原有的前沿选择逻辑，但简化处理
        self._log_debug("处理找到的前沿点")
        
        # 简单的前沿点选择：选择距离最近的前沿点
        candidates = []
        for cluster in frontiers:
            center_i = sum(point[0] for point in cluster) / len(cluster)
            center_j = sum(point[1] for point in cluster) / len(cluster)
            center_grid = (int(center_i), int(center_j))
            world_pos = self._slam_grid_to_world(center_grid[0], center_grid[1])
            
            distance = pose.distance_to(Pose2D(world_pos[0], world_pos[1], 0.0))
            from core.config import MIN_FRONTIER_DISTANCE, MAX_FRONTIER_DISTANCE
            if MIN_FRONTIER_DISTANCE <= distance <= MAX_FRONTIER_DISTANCE:
                candidates.append((distance, world_pos, len(cluster)))
        
        if not candidates:
            return None
        
        # 选择距离最近的前沿点
        candidates.sort(key=lambda x: x[0])
        distance, world_pos, cluster_size = candidates[0]
        
        # 计算目标band
        band_rect = self._compute_goal_band_for_frontier(world_pos, pose, occ_stable)
        if band_rect is not None:
            return {'frontier': world_pos, 'goal_band': band_rect, 'source': 'traditional'}
        
        return None


    def get_latest_cspace_bin(self) -> Optional[np.ndarray]:
        """
        Get the latest cached binary C-space snapshot.
        
        English: Expose the latest binary C-space snapshot to the navigator,
        so planning runs on the SAME collision model as frontier verification.
        This avoids double inflation (verification on C-space, planning on original map).
        
        Returns:
            Binary C-space (0=free, 1=blocked) or None if not cached yet
        """
        return self._last_cspace_bin

    def get_latest_cspace_shape(self) -> Optional[Tuple[int, int]]:
        """
        Get the shape of the latest cached C-space.
        
        English: Expose cached C-space shape for sanity check.
        This helps navigator verify that C-space dimensions match the SLAM grid.
        
        Returns:
            (height, width) tuple or None if not cached yet
        """
        if self._last_cspace_bin is None:
            return None
        return self._last_cspace_bin.shape


    # Navigator 在"真正下发目标"时调用：记录来向与禁忌带
    def note_committed_goal(self, pose_xy: Tuple[float,float], goal_xy: Tuple[float,float], band=None):
        px, py = pose_xy; gx, gy = goal_xy
        vx, vy = (gx - px, gy - py)
        n = (vx*vx + vy*vy) ** 0.5
        if n > 1e-6:
            self._last_forward_u = (vx/n, vy/n)
        self._last_dispatched_goal = goal_xy
        self._taboo.append(goal_xy)   # 访问过的位置加入禁忌带
        # 真正下发才清零失败计数
        self.fail_raw = 0
        self.fail_usable = 0

    # Navigator 在"到达/清空目标"时调用：用最终位姿再确认来向
    def note_arrival(self, pose_xy: Tuple[float,float]):
        if self._last_dispatched_goal is None:
            return
        gx, gy = self._last_dispatched_goal
        px, py = pose_xy
        vx, vy = (px - gx, py - gy)
        n = (vx*vx + vy*vy) ** 0.5
        if n > 1e-6:
            self._last_forward_u = (vx/n, vy/n)
        self._taboo.append((gx, gy))
        self._last_dispatched_goal = None

    def _build_cspace(self, occ: np.ndarray) -> np.ndarray:
        """只膨胀一次：已知障碍(=1)按 SAFE_BUFFER_M 膨胀，并与 unknown(=2) OR。返回 0/1。"""
        from scipy.ndimage import maximum_filter1d
        obst = (occ == 1).astype(np.uint8)
        unk  = (occ == 2).astype(np.uint8)
        # English: prevent world-border inflation
        # 清零世界边界，防止边界膨胀阻塞起点/终点
        obst[0, :]=0; obst[-1,:]=0; obst[:, 0]=0; obst[:, -1]=0
        inflate_cells = int(math.ceil(SAFE_BUFFER_M / SLAM_RESOLUTION))
        if inflate_cells > 0:
            k = 2*inflate_cells + 1
            tmp = maximum_filter1d(obst, size=k, axis=0, mode="nearest")
            inflated = maximum_filter1d(tmp,  size=k, axis=1, mode="nearest")
        else:
            inflated = obst
        cspace = ((inflated > 0) | (unk > 0)).astype(np.uint8)
        self._latest_cspace_bin = cspace  # cache for Navigator
        self._last_cspace_bin   = cspace  # English: keep alias consistent for getter
        return cspace

    # 兜底：环采样信息增益
    def _ring_gain_select(self, pose_xy: Tuple[float,float], occ: np.ndarray, cspace: np.ndarray,
                          hint: Optional[Dict]) -> Optional[Dict]:
        px, py = pose_xy
        # 准备方向/终点信息
        last_u = None
        if hint and ('last_dir' in hint) and hint['last_dir'] is not None:
            last_u = hint['last_dir']
        elif self._last_forward_u is not None:
            last_u = self._last_forward_u
        goal_xy = None
        if hint and ('exit_xy' in hint) and hint['exit_xy'] is not None:
            goal_xy = hint['exit_xy']

        # 采样角度
        best = None; best_score = -1e18
        ang_step = 2*math.pi / max(4, RING_K)
        # English: use actual SLAM resolution to keep ring window consistent with planner
        res = self.slam_system.res if (self.slam_system and hasattr(self.slam_system, 'res')) else SLAM_RESOLUTION
        sense_r_pix = int(round(RING_SENSE_M / res))
        taboo_r2 = (TABOO_RADIUS_M ** 2)

        H, W = occ.shape
        def in_bounds(i, j): return (0 <= i < H and 0 <= j < W)

        for k in range(RING_K):
            th = k * ang_step
            cx = px + RING_RADIUS_M * math.cos(th)
            cy = py + RING_RADIUS_M * math.sin(th)
            # 轴向量化
            cx, cy = _axis_snap((px, py), (cx, cy))
            # grid
            gi = int(round(cy / res))
            gj = int(round(cx / res))
            if (not in_bounds(gi, gj)) or (cspace[gi, gj] != 0):
                self.reject_stats['ring_out_of_cspace'] += 1
                continue

            # 来向抑制：不回头
            if last_u is not None:
                ux = cx - px; uy = cy - py
                n = (ux*ux + uy*uy) ** 0.5
                if n < 1e-6:
                    continue
                dot = (ux/n) * last_u[0] + (uy/n) * last_u[1]
                if dot < BACKWARD_DOT_MIN:
                    self.reject_stats['backward_sector'] += 1
                    continue

            # 禁忌带抑制
            taboo_pen = 0.0
            for tx, ty in self._taboo:
                if (cx-tx)**2 + (cy-ty)**2 <= taboo_r2:
                    taboo_pen = 1.0
                    break

            # 信息增益：统计未知像素
            gi0 = max(0, gi - sense_r_pix); gi1 = min(H-1, gi + sense_r_pix)
            gj0 = max(0, gj - sense_r_pix); gj1 = min(W-1, gj + sense_r_pix)
            unknown_cnt = int((occ[gi0:gi1+1, gj0:gj1+1] == 2).sum())
            if unknown_cnt == 0:
                self.reject_stats['zero_gain'] += 1
                continue

            # 终点方向偏置
            goal_bias = 0.0
            if goal_xy is not None:
                gx, gy = goal_xy
                vg = (gx - px, gy - py)
                ng = (vg[0]**2 + vg[1]**2) ** 0.5
                if ng > 1e-6:
                    ug = (vg[0]/ng, vg[1]/ng)
                    ux = cx - px; uy = cy - py
                    n = (ux*ux + uy*uy) ** 0.5
                    ugain = 0.0 if n < 1e-6 else (ux/n)*ug[0] + (uy/n)*ug[1]
                    goal_bias = max(0.0, ugain) * GOAL_DIR_WEIGHT

            # 近似代价：曼哈顿步长（与 A* 一致的 4-邻域）
            cost = abs(gi - int(round(py / res))) + abs(gj - int(round(px / res)))
            score = (unknown_cnt / max(1, cost)) + goal_bias - 1000.0 * taboo_pen
            if score > best_score:
                best_score = score
                best = (cx, cy)

        if best is None:
            return None

        # 输出 band（小矩形即可，Planner 会在 C-space 内验证）
        bx, by = best
        half = max(2*res, 0.05)   # English: ~2 pixels in current map resolution
        band = (bx - half, bx + half, by - half, by + half)
        # English: expose blocked_mask so Navigator/A* uses this C-space as-is (no re-inflation)
        return {"frontier": (bx, by), "goal_band": band,
                "meta": {"source": "ring_gain", "blocked_mask": cspace}}

    # 你现有日志系统里，加一段聚合输出（可放在函数尾或定期打印）
    def _log_reject_summary(self):
        if not self.reject_stats:
            return
        msg = "拒绝统计: " + ", ".join([f"{k}={v}" for k,v in self.reject_stats.items()])
        if self.logger_func and self.log_file: 
            self.logger_func(self.log_file, msg, "EXPLORE")
        else: 
            print("[EXPLORE]", msg)