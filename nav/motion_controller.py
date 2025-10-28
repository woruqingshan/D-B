# ================================
# file: code/nav/motion_controller.py
# ================================
from __future__ import annotations
from typing import Sequence, Tuple, Optional
import math
from core import Pose2D
from core.config import SLAM_RESOLUTION, V_MAX, W_MAX, ROBOT_RADIUS, SAFE_BUFFER_M, CARDINAL_ONLY, MOTION_CONTROLLER_V_MAX, MOTION_CONTROLLER_W_SPIN

# Config with safe fallbacks (prefer core.config)
try:
    from core.config import (
        PRIM_TURN_TOL_DEG, PRIM_MOVE_YAW_TOL_DEG,
        PRIM_TURN_KP, PRIM_MOVE_KP,
        PRIM_MOVE_V_FAST, PRIM_MOVE_V_SLOW,
        PRIM_GATE_V_FAST, PRIM_GATE_V_SLOW,
        PRIM_GATE_EPS_M, PRIM_GATE_YAW_TOL_DEG,
        PRIM_TURN_W_FIXED,
    )
except Exception:
    # Fallback defaults if config not found
    PRIM_TURN_TOL_DEG = 4.0
    PRIM_MOVE_YAW_TOL_DEG = 6.0
    PRIM_GATE_YAW_TOL_DEG = PRIM_MOVE_YAW_TOL_DEG
    PRIM_TURN_KP = 1.0
    PRIM_MOVE_KP = 0.8
    PRIM_TURN_W_FIXED = 0.6
    PRIM_MOVE_V_FAST = 0.25
    PRIM_MOVE_V_SLOW = 0.12
    PRIM_GATE_V_FAST = 0.22
    PRIM_GATE_V_SLOW = 0.10
    PRIM_GATE_EPS_M = 0.03
    PRIM_TURN_W_FIXED = 0.6

class MotionController:
    """
    Strict rotate-then-translate controller for Manhattan paths.

    - Waypoints MUST be axis-aligned (collinear-compressed path)
    - At a corner:
        * rotate in place until |yaw_err| <= yaw_exit
        * then translate straight with w=0
    """
    def __init__(self,
                 yaw_enter: float = math.radians(2.5),   # 进入旋转阈值
                 yaw_exit: float  = math.radians(1.0),   # 退出旋转阈值
                 w_spin: float    = MOTION_CONTROLLER_W_SPIN,  # 使用config中的旋转角速度
                 v_max: float     = MOTION_CONTROLLER_V_MAX,   # 使用config中的直线速度
                 wp_tol: float    = 1.5*SLAM_RESOLUTION  # 局部点到达阈值
                 ) -> None:
        self.yaw_enter = yaw_enter
        self.yaw_exit  = yaw_exit
        self.w_spin    = abs(w_spin)
        self.v_max     = v_max
        self.wp_tol    = wp_tol

        self._wps: Sequence[Tuple[float,float]] = []
        self._i: int = 0
        self._spinning: bool = False
        self._done: bool = True
        
        # Segment axis state provided by Navigator: ('x' or 'y', dir=+1/-1)
        # This enables heading snapping to the 4 cardinal directions.
        self._axis: Optional[str] = None
        self._dir: int = +1

    # --------- 公共接口 ---------
    def set_segment_axis(self, axis: str, direction: int) -> None:
        """axis in {'x','y'}, direction in {+1,-1}."""
        self._axis = axis
        self._dir = +1 if direction >= 0 else -1

    def _cardinal_heading(self) -> Optional[float]:
        """Return snapped heading target if CARDINAL_ONLY and axis known.
        x,+ -> 0 rad; x,- -> pi; y,+ -> +pi/2; y,- -> -pi/2"""
        if not CARDINAL_ONLY or self._axis is None:
            return None
        if self._axis == 'x':
            return 0.0 if self._dir > 0 else math.pi
        # axis == 'y'
        return +math.pi/2 if self._dir > 0 else -math.pi/2

    def set_path(self, pts_world: Sequence[Tuple[float,float]]) -> None:
        """传入**已轴对齐并做共线压缩**的路径点序列（含起点与终点）。"""
        self._wps = list(self._compress_collinear(pts_world))
        self._i = 0
        self._spinning = False
        self._done = len(self._wps) <= 1

    def is_done(self) -> bool:
        return self._done or self._i >= len(self._wps)

    def current_waypoint(self) -> Optional[Tuple[float,float]]:
        if self.is_done():
            return None
        return self._wps[self._i]


    def step(self, pose: Pose2D) -> Tuple[float,float]:
        """
        返回 (v,w)。
        English:
          - Spin phase: v = 0, w = ±w_spin until heading aligns with current segment axis.
          - Drive phase: w = 0, v > 0 along the locked axis, stop at the segment 'gate' (waypoint).
        """
        # No path or already done
        if self.is_done() or not self._wps:
            self._done = True
            return (0.0, 0.0)

        # Current segment target (axis-aligned waypoint)
        tx, ty = self._wps[self._i]

        # --- Heading target = one of {0, ±pi/2, pi} by segment direction (cardinal lock) ---
        # 优先按段轴线做"四向锁定"，避免 dx/dy 噪声来回切换导致一直旋转
        dx = tx - pose.x
        dy = ty - pose.y
        snap = self._cardinal_heading()
        AXIS_TOLERANCE = 0.02  # 2cm 容差抑制抖动
        if snap is not None:
            # 已知段轴：按轴向锁定航向，并用对应轴向误差做"闸门"推进
            tgt_yaw = snap
            if self._axis == 'x':
                along_err = dx
            else:
                along_err = dy
        else:
            # 未显式设置轴：根据剩余位移选择主轴（带容差），避免反复切轴
            if abs(dx) >= abs(dy) + AXIS_TOLERANCE:
                tgt_yaw = 0.0 if dx >= 0.0 else math.pi
                along_err = dx
            elif abs(dy) >= abs(dx) + AXIS_TOLERANCE:
                tgt_yaw = math.pi/2 if dy >= 0.0 else -math.pi/2
                along_err = dy
            else:
                tgt_yaw = pose.theta
                along_err = math.hypot(dx, dy)

        # normalize yaw error to [-pi,pi]
        yaw_err = math.atan2(math.sin(tgt_yaw - pose.theta), math.cos(tgt_yaw - pose.theta))

        # 添加调试信息
        if abs(yaw_err) > 0.01:  # 只在有明显角度误差时输出
            print(f"[MC_DEBUG] pose=({pose.x:.3f},{pose.y:.3f},{pose.theta:.3f}) "
                  f"target=({tx:.3f},{ty:.3f}) dx={dx:.3f} dy={dy:.3f} "
                  f"tgt_yaw={tgt_yaw:.3f} yaw_err={yaw_err:.3f}")

        # --- Phase selection ---
        # Enter spin if large yaw error OR we were already spinning and not yet within exit band
        need_spin = (abs(yaw_err) > self.yaw_enter) or (self._spinning and abs(yaw_err) > self.yaw_exit)
        if need_spin:
            self._spinning = True
            w = self.w_spin if yaw_err > 0.0 else -self.w_spin
            print(f"[MC_DEBUG] SPINNING: yaw_err={abs(yaw_err):.3f} > {self.yaw_enter:.3f}")
            return (0.0, w)

        # Exiting spin → drive straight, keep w=0
        self._spinning = False

        # Reached waypoint gate? use sign-aware threshold
        if abs(along_err) <= self.wp_tol:
            # advance to next leg or finish
            self._i += 1
            if self._i >= len(self._wps):
                self._done = True
                return (0.0, 0.0)
            # next step will spin to align with new leg
            self._spinning = True
            return (0.0, 0.0)

        # Drive forward along axis with zero curvature
        v = self.v_max
        return (v, 0.0)

    # --------- 工具：确保路径只有角点 ---------
    @staticmethod
    def _compress_collinear(pts: Sequence[Tuple[float,float]]) -> Sequence[Tuple[float,float]]:
        if len(pts) <= 2:
            return list(pts)
        out = [pts[0]]
        for p in pts[1:]:
            if len(out) >= 2:
                ax, ay = out[-2]
                bx, by = out[-1]
                cx, cy = p
                # 三点共线（严格水平/垂直）
                if (abs(ay-by) < 1e-9 and abs(by-cy) < 1e-9) or (abs(ax-bx) < 1e-9 and abs(bx-cx) < 1e-9):
                    out[-1] = p
                    continue
            out.append(p)
        return out

    # expose idx for logging compatibility
    @property
    def idx(self) -> int:
        return self._i

    # === NEW: axis-aligned primitive follower ====================================
    def follow_primitive(self, pose, primitive, remaining_dist: float = None):
        """
        Execute a primitive and return (v, w, done).
        Primitive format is defined in GlobalPlanner.plan_axis_primitives().
        """
        typ = primitive.get("type")
        if typ == "TURN":
            return self._cmd_turn(pose, float(primitive["heading_rad"]))
        if typ == "MOVE":
            d = float(primitive["distance"]) if remaining_dist is None else float(remaining_dist)
            return self._cmd_move_axis(pose, str(primitive["axis"]), d, int(primitive["dir"]))
        if typ == "GATE":
            return self._cmd_move_till_gate(
                pose,
                str(primitive["axis"]),
                int(primitive.get("dir", +1)),
                float(primitive["gate"])
            )
        return 0.0, 0.0, True

    def _cmd_turn(self, pose, heading_rad: float, tol_deg: float = PRIM_TURN_TOL_DEG):
        """Rotate in place to desired heading (radians) with fixed angular velocity."""
        cur = pose.theta
        err = (heading_rad - cur + math.pi) % (2 * math.pi) - math.pi
        if abs(err) <= math.radians(tol_deg):
            return 0.0, 0.0, True
        
        # Fixed angular velocity for fast and consistent rotation
        # English: use fixed max speed instead of proportional control
        w = math.copysign(PRIM_TURN_W_FIXED, err)
        
        return 0.0, w, False

    def _cmd_move_axis(self, pose, axis: str, distance_m: float, dir_sign: int,
                       yaw_tol_deg: float = PRIM_MOVE_YAW_TOL_DEG):
        """
        Move along fixed axis with small heading correction.
        distance_m is the remaining distance (Navigator updates each tick).
        """
        # desired heading depends on axis AND dir_sign
        # English: +x→0, -x→π; +y→+π/2, -y→-π/2
        desired = (0.0 if dir_sign > 0 else math.pi) if axis == "x" \
                  else ((math.pi/2) if dir_sign > 0 else (-math.pi/2))
        cur = pose.theta
        err_h = (desired - cur + math.pi) % (2 * math.pi) - math.pi
        w = max(min(PRIM_MOVE_KP * err_h, 1.5), -1.5)  # English: steering P, clamped - increased limit
        v = (PRIM_MOVE_V_FAST if abs(err_h) < math.radians(yaw_tol_deg) else PRIM_MOVE_V_SLOW)
        # English: keep v >= 0; direction is encoded by desired heading, NOT by the sign of v
        # v *= dir_sign   # ← remove this line
        if distance_m <= 0.5 * SLAM_RESOLUTION:
            return 0.0, 0.0, True
        return v, w, False

    def _cmd_move_till_gate(self, pose, axis: str, dir_sign: int, gate_val: float,
                            eps: float = PRIM_GATE_EPS_M,
                            yaw_tol_deg: float = PRIM_GATE_YAW_TOL_DEG):
        """Keep moving along the axis until the gate coordinate is crossed **in the travel direction**.
        Use the same yaw tolerance family as MOVE (configurable).
        """
        # English: align to travel direction while approaching the gate
        desired = (0.0 if dir_sign > 0 else math.pi) if axis == "x" \
                  else ((math.pi/2) if dir_sign > 0 else (-math.pi/2))
        cur = pose.theta
        err_h = (desired - cur + math.pi) % (2 * math.pi) - math.pi
        w = max(min(PRIM_MOVE_KP * err_h, 1.5), -1.5)  # increased limit for faster steering
        # 统一角度门限：|yaw_err| < yaw_tol_deg 用快速速度，否则慢速
        v = (PRIM_GATE_V_FAST if abs(err_h) < math.radians(yaw_tol_deg) else PRIM_GATE_V_SLOW)
        # English: forward-only speed; crossing test uses dir_sign
        # v *= dir_sign   # ← remove this line
        # 按前进方向判断是否穿越闸门（避免反向时永远不触发）
        if axis == "x":
            crossed = (pose.x >= gate_val - eps) if dir_sign > 0 else (pose.x <= gate_val + eps)
        else:
            crossed = (pose.y >= gate_val - eps) if dir_sign > 0 else (pose.y <= gate_val + eps)
        if crossed:
            return 0.0, 0.0, True
        return v, w, False

    # 兼容历史调用：把 compute 当成 step 使用，避免旧代码误调用导致卡住
    def compute(self, pose: Pose2D, *_):
        return self.step(pose)
