# ================================
# file: code/planning/global_planner.py·
# ================================
from __future__ import annotations
from typing import Tuple, Sequence, Optional, Dict, List
import math, heapq
import numpy as np
from core.config import MAP_RES, SLAM_RESOLUTION
from core import map_to_world

class AStarPlanner:
    """Grid-based 4-neighbor A* with obstacle inflation and minimum turns optimization.
    Inputs are occupancy grid with values {0=free,1=occ,2=unknown}.
    Unknown is treated as obstacle for safety.
    Optimizes for minimum number of turns while maintaining Manhattan distance paths.
    """
    def __init__(self, logger_func=None, log_file=None) -> None:
        self.logger_func = logger_func
        self.log_file = log_file
        # 转弯惩罚参数：用于优化最小拐角数量
        self.TURN_PENALTY = 1.5  # 转弯成本，略大于单步成本
        self._BIG_TURN = 1000  # 大M惩罚，确保转弯优先于步长
    
    def _log_debug(self, message: str) -> None:
        """Add debug message to log and integrate with main log_to_file system"""
        # 输出到控制台
        print(f"[A*_DEBUG] {message}")
        
        # 集成到main函数的log_to_file系统
        if self.logger_func and self.log_file:
            self.logger_func(self.log_file, message, "A*")

    def _inflate(self, grid: np.ndarray, inflate_cells: int) -> np.ndarray:
        """矩形膨胀（轴向可分）：仅对已知障碍(=1)进行，未知(=2)不参与膨胀。返回 0/1 二值 C-space。"""
        from scipy.ndimage import maximum_filter1d
        obst = (grid == 1).astype(np.uint8)
        if inflate_cells <= 0:
            return obst
        k = 2 * inflate_cells + 1
        tmp = maximum_filter1d(obst, size=k, axis=0, mode="nearest")
        inflated = maximum_filter1d(tmp,  size=k, axis=1, mode="nearest")
        return (inflated > 0).astype(np.uint8)

    def _carve_gate_at_position(self, occ: np.ndarray, world_xy: tuple) -> np.ndarray:
        """在指定世界坐标位置开辟30×30cm的圆形闸门"""
        from core.config import SLAM_RESOLUTION, GATE_RADIUS_M
        import numpy as np
        
        if world_xy is None:
            return occ
            
        gate_radius_px = int(round(GATE_RADIUS_M / SLAM_RESOLUTION))  # 15像素
        gx, gy = world_xy
        gj = int(round(gx / SLAM_RESOLUTION))  # 目标网格X
        gi = int(round(gy / SLAM_RESOLUTION))  # 目标网格Y
        
        H, W = occ.shape
        
        # 创建圆形闸门 - 强制设为自由空间
        for di in range(-gate_radius_px, gate_radius_px + 1):
            for dj in range(-gate_radius_px, gate_radius_px + 1):
                if di*di + dj*dj <= gate_radius_px * gate_radius_px:  # 圆形区域
                    ni, nj = gi + di, gj + dj
                    if 0 <= ni < H and 0 <= nj < W:
                        occ[ni, nj] = 0  # 强制设为自由空间
        
        return occ

    def _carve_gates(self, occ: np.ndarray, start_world_xy: tuple, goal_world_xy: tuple) -> np.ndarray:
        """同时开辟起点和终点闸门"""
        occ = self._carve_gate_at_position(occ, start_world_xy)  # 起点闸门
        occ = self._carve_gate_at_position(occ, goal_world_xy)   # 终点闸门
        return occ


    # ----------------- 工具函数：走廊检测 -----------------
    def _axis_clear(self, occ: np.ndarray, i0: int, j0: int, i1: int, j1: int) -> bool:
        """检查一条**严格水平或严格垂直**的网格线段是否在 C-space 中全通"""
        if i0 == i1:
            step = 1 if j1 >= j0 else -1
            for j in range(j0, j1 + step, step):
                if occ[i0, j] != 0:
                    return False
            return True
        if j0 == j1:
            step = 1 if i1 >= i0 else -1
            for i in range(i0, i1 + step, step):
                if occ[i, j0] != 0:
                    return False
            return True
        return False  # 只允许轴对齐
    
    def _l_candidate(self, si, sj, i, j, occ, first='H'):
        """Build an L path in a specified order. first in {'H','V'}."""
        if first=='H':
            # (si,sj)->(si,j)->(i,j)
            if self._axis_clear(occ, si, sj, si, j) and self._axis_clear(occ, si, j, i, j):
                return [(si,sj),(si,j),(i,j)]
            # try opposite if preferred fails
            if self._axis_clear(occ, si, sj, i, sj) and self._axis_clear(occ, i, sj, i, j):
                return [(si,sj),(i,sj),(i,j)]
        else:
            # (si,sj)->(i,sj)->(i,j)
            if self._axis_clear(occ, si, sj, i, sj) and self._axis_clear(occ, i, sj, i, j):
                return [(si,sj),(i,sj),(i,j)]
            if self._axis_clear(occ, si, sj, si, j) and self._axis_clear(occ, si, j, i, j):
                return [(si,sj),(si,j),(i,j)]
        return None


    # ----------------- 工具函数：共线压缩 -----------------
    def _compress_collinear(self, cells: Sequence[Tuple[int,int]]) -> Sequence[Tuple[int,int]]:
        """把连续共线点合并掉，保证每段严格水平或垂直且拐角最少。"""
        if len(cells) <= 2:
            return list(cells)
        out = [cells[0]]
        for p in cells[1:]:
            if len(out) >= 2:
                a, b, c = out[-2], out[-1], p
                if (a[0] == b[0] == c[0]) or (a[1] == b[1] == c[1]):
                    out[-1] = c  # 继续延长该直线段
                    continue
            out.append(p)
        return out

    def plan_to_band(self, start_ij: Tuple[int,int], band_world: Tuple[float,float,float,float],
                     grid: np.ndarray, safe_buffer_m: float = None, grid_is_cspace: bool = False,
                     blocked_mask=None  # 如果给了，就直接用，不再构造/膨胀
                     ) -> Sequence[Tuple[float,float]]:
        """Plan to rectangular arrival band (xmin,xmax,ymin,ymax) in WORLD meters.
        Strategy: straight line → L-shape → fallback A* to band center. Manhattan only."""
        H, W = grid.shape
        # 1) 构造阻塞掩码（严格 C-space），或直接使用调用方提供的 blocked_mask
        if blocked_mask is None:
            # Prepare C-space: SAFE_BUFFER only. If already C-space, use as-is.
            if grid_is_cspace:
                occ = (grid != 0).astype(np.uint8)  # 已是 0/1
            else:
                from core.config import SAFE_BUFFER_M, SLAM_RESOLUTION
                buf = float(SAFE_BUFFER_M if safe_buffer_m is None else safe_buffer_m)
                inflate_cells = int(math.ceil(buf / SLAM_RESOLUTION))
                # 只膨胀已知障碍，再与 unknown(=2) 合并
                occ_infl = self._inflate(grid, inflate_cells)      # 0/1
                occ = np.maximum(occ_infl, (grid == 2).astype(np.uint8))
        else:
            # 调用方已给出"哪些格子不可走"，直接用
            occ = blocked_mask.astype(np.uint8)
        
        # Rect → grid window
        from core.config import SLAM_RESOLUTION
        
        # 开辟起点闸门（plan_to_band只需要起点闸门）
        si, sj = start_ij
        start_world_xy = (sj * SLAM_RESOLUTION, si * SLAM_RESOLUTION)
        occ = self._carve_gate_at_position(occ, start_world_xy)
        # Defensive normalization: ensure callers' band_world follows (xmin,xmax,ymin,ymax)
        try:
            xmin,xmax,ymin,ymax = band_world
        except Exception:
            # Fallback: if someone passed (xmin,ymin,xmax,ymax), try to detect and reorder
            if len(band_world) == 4:
                a,b,c,d = band_world
                # Heuristic: if second < first it's likely (xmin,ymin,xmax,ymax)
                if b < a and c < d:
                    xmin, ymin, xmax, ymax = a,b,c,d
                    xmin, xmax = min(xmin, xmax), max(xmin, xmax)
                    ymin, ymax = min(ymin, ymax), max(ymin, ymax)
                else:
                    xmin,xmax,ymin,ymax = a,b,c,d
            else:
                raise
        # ensure ordering
        xmin, xmax = min(xmin, xmax), max(xmin, xmax)
        ymin, ymax = min(ymin, ymax), max(ymin, ymax)
        j0 = max(0, int(math.floor(xmin / SLAM_RESOLUTION)))
        j1 = min(W-1, int(math.ceil (xmax / SLAM_RESOLUTION)))
        i0 = max(0, int(math.floor(ymin / SLAM_RESOLUTION)))
        i1 = min(H-1, int(math.ceil (ymax / SLAM_RESOLUTION)))
        si, sj = start_ij
        cand = [(i,j) for i in range(i0,i1+1) for j in range(j0,j1+1) if occ[i,j]==0]
        # 1) straight line on same row or same column
        best=None; best_cost=1e18
        for (i,j) in cand:
            if i==si and self._axis_clear(occ, si, sj, i, j):
                c=abs(j-sj); 
                if c<best_cost: best_cost=c; best=[(si,sj),(i,j)]
        for (i,j) in cand:
            if j==sj and self._axis_clear(occ, si, sj, i, j):
                c=abs(i-si);
                if c<best_cost: best_cost=c; best=[(si,sj),(i,j)]
        if best is not None:
            cells=self._compress_collinear(best)
            return [(j*SLAM_RESOLUTION, i*SLAM_RESOLUTION) for (i,j) in cells]

        # 2) L-shape with forced order (reduce early turns)
        # English: choose first leg along the dominant axis from start to band center.
        cx=0.5*(xmin+xmax); cy=0.5*(ymin+ymax)
        dx=abs(cx - sj*SLAM_RESOLUTION); dy=abs(cy - si*SLAM_RESOLUTION)
        first = 'H' if dx >= dy else 'V'
        ordered = sorted(cand, key=lambda t: abs(t[0]-si)+abs(t[1]-sj))
        # Try preferred order first
        for (i,j) in ordered:
            L=self._l_candidate(si, sj, i, j, occ, first=first)
            if L is not None:
                L=self._compress_collinear(L)
                return [(jj*SLAM_RESOLUTION, ii*SLAM_RESOLUTION) for (ii,jj) in L]
        # Fallback: opposite order
        other = 'V' if first=='H' else 'H'
        for (i,j) in ordered:
            L=self._l_candidate(si, sj, i, j, occ, first=other)
            if L is not None:
                L=self._compress_collinear(L)
                return [(jj*SLAM_RESOLUTION, ii*SLAM_RESOLUTION) for (ii,jj) in L]

        # 3) fallback: A* to band center
        cx=0.5*(xmin+xmax); cy=0.5*(ymin+ymax)
        gi=int(round(cy/SLAM_RESOLUTION)); gj=int(round(cx/SLAM_RESOLUTION))
        return self.plan((si,sj),(gi,gj), grid, safe_buffer_m=safe_buffer_m, grid_is_cspace=grid_is_cspace, blocked_mask=blocked_mask)

    # ----------------- 主函数：最少拐角 A* -----------------
    def plan(self, start_ij: Tuple[int,int], goal_ij: Tuple[int,int],
             grid: np.ndarray, safe_buffer_m: float = None, grid_is_cspace: bool = False, blocked_mask=None) -> Sequence[Tuple[float,float]]:
        self._log_debug("========== 开始路径规划(最少拐角) ==========")
        H, W = grid.shape
        si, sj = start_ij
        gi, gj = goal_ij

        # 1) 构造阻塞掩码（严格 C-space），或直接使用调用方提供的 blocked_mask
        if blocked_mask is None:
            if grid_is_cspace:
                occ = (grid != 0).astype(np.uint8)  # 强制 0/1
            else:
                if safe_buffer_m is None:
                    from core.config import SAFE_BUFFER_M
                    safe_buffer_m = SAFE_BUFFER_M
                # English: Use SAFE_BUFFER_M directly since ROBOT_RADIUS is now 0 (point mass model)
                # Physical radius is already included in SAFE_BUFFER_M
                eff = float(safe_buffer_m)  # No need to add ROBOT_RADIUS (it's 0)
                inflate_cells = int(math.ceil(eff / SLAM_RESOLUTION))
                # 只膨胀已知障碍，再与 unknown(=2) 合并
                occ_infl = self._inflate(grid, inflate_cells)      # 0/1
                occ = np.maximum(occ_infl, (grid == 2).astype(np.uint8))
        else:
            # 调用方已给出"哪些格子不可走"，直接用
            occ = blocked_mask.astype(np.uint8)

        # 开辟起点和终点闸门
        start_world_xy = (sj * SLAM_RESOLUTION, si * SLAM_RESOLUTION)
        goal_world_xy = (gj * SLAM_RESOLUTION, gi * SLAM_RESOLUTION)
        occ = self._carve_gates(occ, start_world_xy, goal_world_xy)

        # 2) 先尝试**L 型候选**（可直接得到 1 次转弯的最优解） --------------
        l_cells = self._l_candidate(si, sj, gi, gj, occ)
        if l_cells is not None:
            l_cells = self._compress_collinear(l_cells)
            pts = [(j*SLAM_RESOLUTION, i*SLAM_RESOLUTION) for (i,j) in l_cells]
            self._log_debug("✅ 使用 L 型候选路径（1 次拐角）")
            return pts

        # 3) 否则执行**方向状态 A\***（少拐角 → 少步长） --------------------
        dirs = [(1,0),(0,1),(-1,0),(0,-1)]  # 下、右、上、左（4-邻域）
        NONE = 4  # 起点无方向
        # g 里存"(步数, 拐角数)"，键为 (i,j,dir_idx)
        g: Dict[Tuple[int,int,int], Tuple[int,int]] = {(si,sj,NONE):(0,0)}
        parent: Dict[Tuple[int,int,int], Tuple[int,int,int]] = {}
        # open: (f, steps, (i,j,dir))
        h0 = abs(gi - si) + abs(gj - sj)
        openq = [(0, 0, (si, sj, NONE))]

        def push(steps: int, turns: int, state: Tuple[int,int,int]):
            i,j,d = state
            h = abs(gi - i) + abs(gj - j)
            # f = "拐角优先"的大 M 代价 + 步长 + 启发
            f = turns * self._BIG_TURN + steps + h
            heapq.heappush(openq, (f, steps, state))

        while openq:
            _, steps, (ci, cj, cd) = heapq.heappop(openq)
            # 若到达目标网格（忽略朝向），结束：从所有朝向里挑最少拐角的
            if (ci, cj) == (gi, gj):
                goal_states = [(ci, cj, k) for k in range(4) if (ci, cj, k) in g] or [(ci,cj,cd)]
                best = min(goal_states, key=lambda s: (g[s][1], g[s][0]))  # (turns, steps)
                # 回溯
                path_cells = [ (gi, gj) ]
                cur = best
                while cur in parent:
                    cur = parent[cur]
                    path_cells.append((cur[0], cur[1]))
                path_cells.reverse()
                path_cells = self._compress_collinear(path_cells)
                pts = [(j*SLAM_RESOLUTION, i*SLAM_RESOLUTION) for (i,j) in path_cells]
                self._log_debug(f"✅ A* 完成：拐角{min(g[s][1] for s in goal_states)} 次, 步长{min(g[s][0] for s in goal_states)}")
                return pts

            for k,(di,dj) in enumerate(dirs):
                ni, nj = ci+di, cj+dj
                if not (0 <= ni < H and 0 <= nj < W): 
                    continue
                if occ[ni, nj] != 0: 
                    continue
                n_steps = g[(ci,cj,cd)][0] + 1
                n_turns = g[(ci,cj,cd)][1] + (0 if (cd in (k, NONE)) else 1)
                key = (ni, nj, k)
                if (key not in g) or ((n_turns, n_steps) < (g[key][1], g[key][0])):
                    g[key] = (n_steps, n_turns)
                    parent[key] = (ci, cj, cd)
                    push(n_steps, n_turns, key)

        self._log_debug("❌ 无法找到可行路径")
        return []

    # === NEW: axis-aligned motion primitives for corridor band ===================
    def plan_axis_primitives(
        self,
        pose,  # Pose2D
        band_rect: Tuple[float, float, float, float],
    ) -> List[Dict]:
        """
        Return a short queue of primitives to reach a corridor band.
        band_rect: (xmin,xmax,ymin,ymax) in meters.
        Primitive dicts:
          - TURN: {"type":"TURN","heading_rad":float}
          - MOVE: {"type":"MOVE","axis":"x"|"y","distance":float, "dir":+1|-1}
          - GATE: {"type":"GATE","axis":"x"|"y","gate":float, "dir":+1|-1 (optional)}
        """
        gx_min, gx_max, gy_min, gy_max = band_rect
        x0, y0, th = pose.x, pose.y, pose.theta
        width_x = gx_max - gx_min
        height_y = gy_max - gy_min
        prims: List[Dict] = []

        def _heading(rad: float):  # English: store radians for controller
            prims.append({"type": "TURN", "heading_rad": float(rad)})
        def _move(axis: str, dist: float, sgn: int):
            prims.append({"type": "MOVE", "axis": axis, "distance": float(abs(dist)), "dir": +1 if sgn >= 0 else -1})
        def _gate(axis: str, val: float):
            prims.append({"type": "GATE", "axis": axis, "gate": float(val)})

        # centerlines (for gentle centering before heading to the gate)
        y_mid = 0.5 * (gy_min + gy_max)
        x_mid = 0.5 * (gx_min + gx_max)

        # Horizontal corridor → gate at LEFT vertical line x=gx_min
        if width_x >= height_y:
            # 1) center in Y if needed
            if not (gy_min <= y0 <= gy_max):
                dy = y_mid - y0
                _heading((math.pi/2) if dy >= 0.0 else -(math.pi/2))
                _move("y", dy, +1 if dy >= 0.0 else -1)
            # 2) head along +X or -X then cross gate
            need_right = (x_mid >= x0)
            _heading(0.0 if need_right else math.pi)
            adv = (gx_min - x0) if need_right else (x0 - gx_min)
            if adv > 0.0:
                _move("x", adv, +1 if need_right else -1)
            _gate("x", gx_min)
        # Vertical corridor → gate at BOTTOM horizontal line y=gy_min
        else:
            if not (gx_min <= x0 <= gx_max):
                dx = x_mid - x0
                _heading(0.0 if dx >= 0.0 else math.pi)
                _move("x", dx, +1 if dx >= 0.0 else -1)
            need_up = (y_mid >= y0)
            _heading((math.pi/2) if need_up else -(math.pi/2))
            adv = (gy_min - y0) if need_up else (y0 - gy_min)
            if adv > 0.0:
                _move("y", adv, +1 if need_up else -1)
            _gate("y", gy_min)

        return prims