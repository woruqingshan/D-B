# ================================
# file: gui/nav_visualizer.py
# ================================
"""
Navigation-specific visualizer for real-time mapping and robot state display
Minimal visualizer focused on navigation and mapping tasks
"""

from __future__ import annotations
from typing import Tuple, Optional
import math
import numpy as np

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

from core.config import SLAM_RESOLUTION, WORLD_SIZE


class NavVisualizer:
    """Minimal occupancy visualizer for navigation, origin at lower-left, extent in meters."""
    
    def __init__(self, size_m: float = WORLD_SIZE, res: float = SLAM_RESOLUTION):
        if plt is None:
            raise RuntimeError("matplotlib not available")
            
        self.size_m, self.res = size_m, res
        self.N = int(round(size_m / res))
        
        # Create figure and axes
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.im = self.ax.imshow(
            np.zeros((self.N, self.N), dtype=np.uint8),
            cmap='gray_r', vmin=0, vmax=2, origin='lower',
            extent=(0, size_m, 0, size_m)
        )
        
        # Robot pose visualization
        self.rob, = self.ax.plot([], [], 'r^', markersize=8)
        self.head = self.ax.quiver([], [], [], [], angles='xy', scale_units='xy', scale=1.0, color='r')
        
        # Navigation overlay elements
        self._frontiers_sc = None
        self._goal_band_patch = None
        self._exit_band_patch = None  # 终点区域
        self._home_band_patch = None  # 起点区域
        self._path_ln, = self.ax.plot([], [], 'o-', lw=2, color='#ff8c00', alpha=0.9, markersize=3)  # orange path
        
        # Setup plot
        self.ax.set_title("Real-time Navigation Mapping")
        self.ax.set_xlim(0, size_m)
        self.ax.set_ylim(0, size_m)
        self.fig.tight_layout()
        plt.ion()
        plt.show(block=False)

    def update(self, occ_grid: np.ndarray, pose: Tuple[float, float, float]):
        """Update image and pose arrow."""
        try:
            if occ_grid is not None:
                self.im.set_data(occ_grid)
            
            x, y, th = pose
            self.rob.set_data([x], [y])
            
            # Heading arrow (0.15m)
            dx, dy = 0.15 * math.cos(th), 0.15 * math.sin(th)
            
            # Remove old quiver and create new one
            for coll in self.ax.collections[:]:
                if isinstance(coll, type(self.head)):
                    try:
                        self.ax.collections.remove(coll)
                    except Exception:
                        pass
            
            self.head = self.ax.quiver([x], [y], [dx], [dy], 
                                     angles='xy', scale_units='xy', scale=1.0, color='r')
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            print(f"[GUI_ERROR] GUI更新失败: {e}")
            # 不要因为GUI错误而退出程序

    def update_mapping_realtime(self, occ_grid: np.ndarray, pose: Tuple[float, float, float], maybe_point=None):
        """增量绘制：建图循环里每处理若干点就调用一次
        maybe_point: (x,y) 世界坐标，可选，仅用于动画反馈
        """
        try:
            # 复用现有update逻辑，保证一致性
            self.update(occ_grid, pose)
            
            # 可选：瞬时高亮一个点（不保留历史，避免大量 Artist）
            if maybe_point is not None:
                px, py = maybe_point
                self.ax.plot([px], [py], '.', ms=2, color='cyan', alpha=0.8)
                
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            print(f"[GUI_ERROR] 实时GUI更新失败: {e}")

    def clear_navigation_overlay(self):
        """清空前沿/路径覆盖层"""
        try:
            if self._frontiers_sc is not None:
                self._frontiers_sc.remove()
                self._frontiers_sc = None
            if self._goal_band_patch is not None:
                self._goal_band_patch.remove()
                self._goal_band_patch = None
            if self._exit_band_patch is not None:
                self._exit_band_patch.remove()
                self._exit_band_patch = None
            if self._home_band_patch is not None:
                self._home_band_patch.remove()
                self._home_band_patch = None
            self._path_ln.set_data([], [])
            self.fig.canvas.draw_idle()
        except Exception as e:
            print(f"[GUI_ERROR] 清除导航覆盖层失败: {e}")

    def show_navigation_elements(self, frontier_xy=None, goal_band=None, path_points=None, frontier_candidates=None, exit_xy=None, exit_band=None, home_band=None, primitives_info=None):
        """在图上显示前沿点、可达门槛区域、路径、终点、起点
        - frontier_xy: (fx,fy)
        - goal_band: (xmin,xmax,ymin,ymax)
        - path_points: [(x,y), ...]
        - frontier_candidates: [(x,y), ...]  可选，用浅绿作参考
        - exit_xy: (ex,ey) 终点坐标
        - exit_band: (xmin,xmax,ymin,ymax) 终点区域
        - home_band: (xmin,xmax,ymin,ymax) 起点区域
        - primitives_info: 原语信息字符串
        """
        try:
            import matplotlib.patches as patches
            
            # 候选前沿点
            if frontier_candidates and len(frontier_candidates) > 0:
                xs, ys = zip(*frontier_candidates)
                if self._frontiers_sc is not None:
                    self._frontiers_sc.remove()
                self._frontiers_sc = self.ax.scatter(xs, ys, s=16, c='#55dd55', alpha=0.7, label='frontiers')
            
            # 目标前沿点
            if frontier_xy is not None:
                self.ax.scatter([frontier_xy[0]], [frontier_xy[1]], s=36, c='g', marker='o', 
                               edgecolor='k', zorder=5, label='target frontier')
            
            # 可达区域
            if goal_band is not None and len(goal_band) == 4:
                xmin, xmax, ymin, ymax = goal_band
                if self._goal_band_patch is not None:
                    self._goal_band_patch.remove()
                self._goal_band_patch = patches.Rectangle((xmin, ymin), xmax-xmin, ymax-ymin,
                                                        fill=False, ec='#3399ff', lw=2, 
                                                        label='goal band')
                self.ax.add_patch(self._goal_band_patch)
            
            # 路径
            if path_points and len(path_points) > 0:
                xs = [p[0] for p in path_points]
                ys = [p[1] for p in path_points]
                self._path_ln.set_data(xs, ys)
            
            # 终点区域（优先显示区域，如果只有坐标点则显示点）
            if exit_band is not None and len(exit_band) == 4:
                xmin, xmax, ymin, ymax = exit_band
                if self._exit_band_patch is not None:
                    self._exit_band_patch.remove()
                self._exit_band_patch = patches.Rectangle((xmin, ymin), xmax-xmin, ymax-ymin,
                                                        fill=True, fc='red', alpha=0.3, ec='red', lw=2,
                                                        label='Exit Band')
                self.ax.add_patch(self._exit_band_patch)
                print(f"[GUI] 显示终点区域: ({xmin:.3f}, {xmax:.3f}, {ymin:.3f}, {ymax:.3f})")
            elif exit_xy is not None:
                # 如果没有区域信息，只显示坐标点
                self.ax.scatter([exit_xy[0]], [exit_xy[1]], s=50, c='red', 
                               marker='X', zorder=10, label='Exit')
            
            # 起点区域
            if home_band is not None and len(home_band) == 4:
                xmin, xmax, ymin, ymax = home_band
                if self._home_band_patch is not None:
                    self._home_band_patch.remove()
                self._home_band_patch = patches.Rectangle((xmin, ymin), xmax-xmin, ymax-ymin,
                                                        fill=True, fc='blue', alpha=0.3, ec='blue', lw=2,
                                                        label='Home Band')
                self.ax.add_patch(self._home_band_patch)
                print(f"[GUI] 显示起点区域: ({xmin:.3f}, {xmax:.3f}, {ymin:.3f}, {ymax:.3f})")
            
            # 原语信息显示
            if primitives_info is not None:
                # 在图上添加原语信息文本
                self.ax.text(0.02, 0.98, primitives_info, transform=self.ax.transAxes, 
                            fontsize=8, verticalalignment='top', 
                            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
                print(f"[GUI] 显示原语信息: {primitives_info}")
            
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            print(f"[GUI_ERROR] 显示导航元素失败: {e}")

    def close(self):
        """Close the visualizer"""
        try:
            if self.fig:
                plt.close(self.fig)
        except Exception:
            pass
