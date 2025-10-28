# ================================
# file: nav/nav_adapters.py
# ================================
"""
Navigation adapters for integrating with existing navigation modules
Bridge functions connecting frontier exploration, path planning, and command generation
"""

from __future__ import annotations
from typing import Tuple, List, Dict, Optional
import math

# Dynamic imports to handle optional dependencies
try:
    from explore.frontier_explorer import FrontierExplorer
    from planning.global_planner import AStarPlanner  
    from nav.navigator import NavigatorFSM
    from core.types import Pose2D
    from core.config import SLAM_RESOLUTION, SAFE_BUFFER_M
    NAV_MODULES_AVAILABLE = True
except ImportError:
    FrontierExplorer = None
    AStarPlanner = None
    NavigatorFSM = None
    Pose2D = None
    NAV_MODULES_AVAILABLE = False


def wrap_pi(a: float) -> float:
    """Wrap angle to [-pi, pi]"""
    while a > math.pi: 
        a -= 2.0 * math.pi
    while a < -math.pi: 
        a += 2.0 * math.pi
    return a


def reached_band(pose_xy: Tuple[float,float], band: Tuple[float,float,float,float], tol_m: float=0.05) -> bool:
    """检查机器人是否到达目标区域"""
    x, y = pose_xy
    xmin, xmax, ymin, ymax = band
    return (x >= xmin - tol_m and x <= xmax + tol_m and
            y >= ymin - tol_m and y <= ymax + tol_m)


def compute_frontier_and_band(explorer, occ_grid, pose_xyth: Tuple[float, float, float]):
    """Adapter to your FrontierExplorer API.
    Returns (goal_xy, band_rect) where band_rect=(xmin,xmax,ymin,ymax) in meters.
    """
    if not NAV_MODULES_AVAILABLE or explorer is None:
        return None, None
        
    try:
        print(f"[FRONTIER] 开始前沿点探索:")
        print(f"  - 当前位姿: ({pose_xyth[0]:.3f}, {pose_xyth[1]:.3f}, {math.degrees(pose_xyth[2]):.1f}°)")
        
        # 转换为Pose2D对象
        if Pose2D is not None:
            pose_obj = Pose2D(pose_xyth[0], pose_xyth[1], pose_xyth[2])
        else:
            # Fallback if Pose2D not available
            pose_obj = {
                'x': pose_xyth[0],
                'y': pose_xyth[1], 
                'theta': pose_xyth[2]
            }
        
        front = explorer.choose_next_frontier(occ_grid, pose_obj, hint={'current_heading': pose_xyth[2]})
        if front and isinstance(front, dict):
            # 检测恢复信号
            if front.get('recovery_needed'):
                print(f"[FRONTIER] 🚨 检测到恢复需求: {front.get('reason', 'unknown')}")
                return 'RECOVERY_NEEDED', None
            
            goal_xy = front.get('frontier')
            band_rect = front.get('goal_band')
            
            # 输出前沿点详细信息
            if goal_xy is not None:
                print(f"[FRONTIER] ✅ 找到前沿点:")
                print(f"  - 目标坐标: ({goal_xy[0]:.3f}, {goal_xy[1]:.3f})")
                distance_to_frontier = math.sqrt((goal_xy[0] - pose_xyth[0])**2 + (goal_xy[1] - pose_xyth[1])**2)
                print(f"  - 距离目标: {distance_to_frontier:.3f}m")
            
            if band_rect is not None:
                print(f"[FRONTIER] 📊 目标区域 (band):")
                print(f"  - 区域范围: x[{band_rect[0]:.3f}, {band_rect[1]:.3f}], y[{band_rect[2]:.3f}, {band_rect[3]:.3f}]")
                band_width = band_rect[1] - band_rect[0]
                band_height = band_rect[3] - band_rect[2]
                print(f"  - 区域尺寸: {band_width:.3f}m × {band_height:.3f}m")
            else:
                print(f"[FRONTIER] ⚠️  未找到目标区域")
            
            # 输出完整frontier信息
            print(f"[FRONTIER] 📋 完整前沿信息: {list(front.keys())}")
            for key, value in front.items():
                if key not in ['frontier', 'goal_band']:
                    print(f"  - {key}: {value}")
            
            return goal_xy, band_rect
        else:
            print(f"[FRONTIER] ❌ 未找到有效前沿点")
            if front is not None:
                print(f"  - 返回类型: {type(front)}")
                print(f"  - 返回内容: {front}")
        return None, None
    except Exception as e:
        print(f"[ERROR] Frontier exploration failed: {e}")
        import traceback
        traceback.print_exc()
        return None, None


def compute_frontier_and_band_with_viz(explorer, occ_grid, pose_xyth: Tuple[float, float, float]):
    """新接口：返回可视化所需的完整前沿探索信息
    Returns dict with visualization-friendly data while maintaining compatibility
    """
    goal_xy, band_rect = compute_frontier_and_band(explorer, occ_grid, pose_xyth)
    
    # 尝试获取候选前沿点（如果explorer支持）
    candidates = []
    try:
        if hasattr(explorer, 'last_candidates') and explorer.last_candidates:
            candidates = explorer.last_candidates
        elif hasattr(explorer, '_frontier_candidates') and explorer._frontier_candidates:
            candidates = explorer._frontier_candidates
    except Exception as e:
        print(f"[FRONTIER_VIZ] 获取候选前沿点失败: {e}")
    
    return {
        'goal_xy': goal_xy,
        'band': band_rect, 
        'candidates': candidates
    }


def plan_manhattan_path(planner, occ_grid, start_xy: Tuple[float, float], 
                       band_rect: Tuple[float, float, float, float]) -> List[Tuple[float, float]]:
    """Plan a path in meters from start to goal band with minimal turns (prefer L/straight)."""
    if not NAV_MODULES_AVAILABLE or planner is None:
        return []
        
    print(f"[PATH] 开始路径规划:")
    print(f"  - 起点: ({start_xy[0]:.3f}, {start_xy[1]:.3f})")
    print(f"  - 目标区域: x[{band_rect[0]:.3f}, {band_rect[1]:.3f}], y[{band_rect[2]:.3f}, {band_rect[3]:.3f}]")
    
    # Convert to grid indices (i=row=y, j=col=x)
    si = int(round(start_xy[1] / SLAM_RESOLUTION))
    sj = int(round(start_xy[0] / SLAM_RESOLUTION))
    print(f"  - 网格起点: (i={si}, j={sj})")
    
    try:
        path_xy = planner.plan_to_band((si, sj), band_rect, occ_grid,
                                     safe_buffer_m=SAFE_BUFFER_M, grid_is_cspace=False, blocked_mask=None)
        
        if path_xy:
            print(f"[PATH] ✅ 路径规划成功:")
            print(f"  - 路径点数: {len(path_xy)}")
            print(f"  - 路径坐标:")
            for i, (x, y) in enumerate(path_xy[:5]):  # 显示前5个点
                print(f"    [{i}]: ({x:.3f}, {y:.3f})")
            if len(path_xy) > 5:
                print(f"    ... (还有 {len(path_xy)-5} 个点)")
            
            # 计算路径总长度
            total_distance = 0.0
            for i in range(1, len(path_xy)):
                dx = path_xy[i][0] - path_xy[i-1][0]
                dy = path_xy[i][1] - path_xy[i-1][1]
                total_distance += math.sqrt(dx*dx + dy*dy)
            print(f"  - 路径总长度: {total_distance:.3f}m")
            
            return path_xy
        else:
            print(f"[PATH] ❌ 路径规划失败: 无法找到有效路径")
            return []
            
    except Exception as e:
        print(f"[PATH] ❌ 路径规划异常: {e}")
        import traceback
        traceback.print_exc()
        return []


def decompose_to_primitives(nfsm, path_xy: List[Tuple[float, float]], theta0: float) -> List[Dict]:
    """Use NavigatorFSM to decompose to TURN/MOVE primitives. Fall back to naive L-path."""
    if not path_xy:
        print(f"[PRIMITIVES] ❌ 空路径，无法分解原语")
        return []
    
    print(f"[PRIMITIVES] 开始原语分解:")
    print(f"  - 初始朝向: {math.degrees(theta0):.1f}°")
    print(f"  - 路径点数: {len(path_xy)}")
    
    if NAV_MODULES_AVAILABLE and nfsm is not None:
        try:
            cleaned = nfsm._clean_primitive_path(path_xy)
            prims = nfsm._decompose_path_to_primitives(cleaned, pose_theta=theta0)
            print(f"[PRIMITIVES] ✅ NavigatorFSM分解成功，原语数: {len(prims)}")
        except Exception as e:
            print(f"[PRIMITIVES] ⚠️  NavigatorFSM分解失败，使用备用方法: {e}")
            prims = _fallback_decompose_to_primitives(path_xy, theta0)
    else:
        prims = _fallback_decompose_to_primitives(path_xy, theta0)
    
    # 输出原语详细信息
    if prims:
        print(f"[PRIMITIVES] 📋 原语序列:")
        for i, prim in enumerate(prims):
            if prim["type"] == "TURN":
                target_deg = math.degrees(prim.get("heading_rad", 0))
                print(f"  [{i}] 转向: 目标朝向 {target_deg:.1f}°")
            elif prim["type"] == "MOVE":
                dist = prim.get("distance", 0)
                axis = prim.get("axis", "unknown")
                direction = prim.get("dir", 1)
                print(f"  [{i}] 移动: 沿{axis}轴移动 {dist:.3f}m (方向: {direction})")
            else:
                print(f"  [{i}] 未知原语: {prim}")
    else:
        print(f"[PRIMITIVES] ❌ 未生成任何原语")
    
    return prims


def _fallback_decompose_to_primitives(path_xy: List[Tuple[float, float]], theta0: float) -> List[Dict]:
    """Fallback primitive decomposition method"""
    prims = []
    th = theta0
    for k in range(1, len(path_xy)):
        x0, y0 = path_xy[k-1]
        x1, y1 = path_xy[k]
        dx, dy = x1-x0, y1-y0
        if abs(dx) > abs(dy):
            heading = 0.0 if dx >= 0 else math.pi
            d = abs(dx)
        else:
            heading = math.pi/2 if dy >= 0 else -math.pi/2
            d = abs(dy)
        dth = wrap_pi(heading - th)
        if abs(dth) > 1e-3:
            prims.append({"type": "TURN", "heading_rad": heading})
        if d > 1e-3:
            prims.append({"type": "MOVE", "distance": d, 
                         "axis": "x" if abs(dx) > abs(dy) else "y", "dir": 1})
        th = heading
    
    print(f"[PRIMITIVES] ✅ 备用方法分解完成，原语数: {len(prims)}")
    return prims


def primitives_to_ble_commands(prims: List[Dict], theta0: float) -> List[str]:
    """Collapse TURN+MOVE to '(M,<deg>,<mm>)'. Heading is absolute target in TURN primitive."""
    from core.config import MAX_TURN_CHUNK_DEG as _MAX_TURN_CHUNK_DEG
    
    print(f"[COMMANDS] 开始生成BLE指令:")
    print(f"  - 输入原语数: {len(prims)}")
    print(f"  - 初始朝向: {math.degrees(theta0):.1f}°")
    
    cmds = []
    th = theta0
    i = 0
    cmd_count = 0
    
    def _emit_turn_chunks(deg_rel: float):
        """Emit turn chunks so each command's rotation ≤90°, avoid single 180°"""
        nonlocal th, cmd_count, cmds
        remaining = deg_rel
        # split into ≤ _MAX_TURN_CHUNK_DEG steps
        while abs(remaining) > _MAX_TURN_CHUNK_DEG + 1e-6:
            step = math.copysign(_MAX_TURN_CHUNK_DEG, remaining)
            cmd = f"(M,{step:.1f},0)"
            cmds.append(cmd)
            # English: update internal heading; BLE positive=clockwise -> negative radians
            th = wrap_pi(th - math.radians(step))
            cmd_count += 1
            print(f"  [{cmd_count-1}] 分块转向: {cmd}")
            remaining -= step
        return remaining  # the small leftover (|≤90|)
    
    while i < len(prims):
        p = prims[i]
        if p["type"] == "TURN":
            target = float(p.get("heading_rad", th))
            dth = wrap_pi(target - th)
            deg = round(math.degrees(dth), 1)
            # 反转符号以符合BLE约定：正数=顺时针转动
            deg = -deg

            # 1) 先把大角度分块成若干≤90°的TURN-only
            leftover = _emit_turn_chunks(deg)

            # 2) 若下一个是MOVE且剩余角度|≤90°，再合并成一条
            if i+1 < len(prims) and prims[i+1]["type"] == "MOVE" and abs(leftover) > 1e-3:
                dist_m = float(prims[i+1].get("distance", 0.0))
                cmd = f"(M,{leftover:.1f},{int(round(abs(dist_m)*1000))})"
                cmds.append(cmd)
                print(f"  [{cmd_count}] 复合指令: {cmd}")
                # English: apply the leftover rotation then move
                th = wrap_pi(th - math.radians(leftover))
                cmd_count += 1
                i += 2
            else:
                # 如果没有MOVE相随或没有剩余角度，就只产生前面的TURN分块
                i += 1
            continue
            
        elif p["type"] == "MOVE":
            dist_m = float(p.get("distance", 0.0))
            cmd = f"(M,0,{int(round(abs(dist_m)*1000))})"
            cmds.append(cmd)
            print(f"  [{cmd_count}] 移动指令: {cmd} (移动 {dist_m:.3f}m = {int(round(abs(dist_m)*1000))}mm)")
            i += 1
            cmd_count += 1
        else:
            print(f"  [{cmd_count}] 跳过未知原语: {p}")
            i += 1
    
    print(f"[COMMANDS] ✅ BLE指令生成完成:")
    print(f"  - 总指令数: {len(cmds)}")
    print(f"  - 指令序列: {cmds}")
    
    return cmds


# --- NEW: primitive slicers ---
def split_long_moves(prims, max_run_m: float = 0.35):
    """Split long MOVE primitives into smaller chunks"""
    out = []
    for p in prims:
        if p.get('type') != 'MOVE':
            out.append(p)
            continue
        d = float(p['distance'])
        axis = p['axis']
        sgn = int(p['dir'])
        while d > 1e-6:
            step = min(max_run_m, d)
            out.append({'type': 'MOVE', 'axis': axis, 'dir': sgn, 'distance': step})
            d -= step
    return out


def split_large_turns(prims, max_deg: float = 90.0):
    """No-op: splitting is handled in primitives_to_ble_commands with real heading."""
    return prims
