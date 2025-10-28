#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiDAR Map Visualization Tool

Usage:
    python visualize_lidar_map.py --npz logs/lidar_mapping/slam_map_20251012_233645.npz
    python visualize_lidar_map.py --json logs/lidar_mapping/lidar_odom_snapshot_20251012_233136.json
    python visualize_lidar_map.py --npz <file> --json <file>  # Show both
"""

import os
import sys
import json
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from typing import Optional, Dict, List, Tuple
import math
import locale
from sparse_lidar_processor import SparseLidarProcessor

# Configure Chinese font support - COMPLETE FIX
import warnings
warnings.filterwarnings('ignore', category=UserWarning, module='matplotlib')

# Try multiple font configurations
try:
    # Method 1: Use system default fonts
    plt.rcParams['font.family'] = ['DejaVu Sans', 'Arial', 'sans-serif']
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Liberation Sans', 'sans-serif']
    plt.rcParams['axes.unicode_minus'] = False
    
    # Test if Chinese fonts are available
    import matplotlib.font_manager as fm
    available_fonts = [f.name for f in fm.fontManager.ttflist]
    
    # Add Chinese fonts if available
    chinese_fonts = []
    for font in ['Microsoft YaHei', 'SimHei', 'SimSun', 'KaiTi', 'FangSong']:
        if font in available_fonts:
            chinese_fonts.append(font)
    
    if chinese_fonts:
        plt.rcParams['font.sans-serif'] = chinese_fonts + ['DejaVu Sans', 'Arial', 'sans-serif']
        print(f"[FONT] Using Chinese fonts: {chinese_fonts}")
    else:
        print("[FONT] No Chinese fonts found, using English only")
        
except Exception as e:
    print(f"[FONT] Font configuration failed: {e}")
    # Fallback to basic configuration
    plt.rcParams['font.family'] = ['DejaVu Sans', 'Arial']
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial']
    plt.rcParams['axes.unicode_minus'] = False

# Force UTF-8 encoding to fix UnicodeDecodeError on Windows
try:
    locale.setlocale(locale.LC_ALL, 'C.UTF-8')
except locale.Error:
    print("[WARN] 无法设置 locale 为 C.UTF-8，可能在非 Unix 系统上。")
    try:
        locale.setlocale(locale.LC_ALL, 'zh_CN.UTF-8')
    except locale.Error:
        print("[WARN] 无法设置 locale 为 zh_CN.UTF-8。")
        os.environ['PYTHONIOENCODING'] = 'utf-8'
        sys.stdout.reconfigure(encoding='utf-8')
        sys.stderr.reconfigure(encoding='utf-8')


class MapVisualizer:
    """Visualize SLAM maps and LiDAR data"""
    
    def __init__(self):
        self.fig = None
        self.axes = []
        
    def load_npz_map(self, npz_path: str) -> Optional[Dict]:
        """Load map from NPZ file"""
        if not os.path.exists(npz_path):
            print(f"❌ NPZ文件不存在: {npz_path}")
            return None
        
        try:
            # Allow pickle to load metadata dict
            data = np.load(npz_path, allow_pickle=True)
            
            # Debug: Print available keys
            print(f"[DEBUG] NPZ文件包含的键: {list(data.keys())}")
            
            # Try different possible key names
            occ_grid = None
            if 'occupancy_grid' in data:
                occ_grid = data['occupancy_grid']
            elif 'occ_grid' in data:
                occ_grid = data['occ_grid']
            else:
                print(f"❌ NPZ文件中没有找到occupancy_grid或occ_grid键")
                print(f"   可用的键: {list(data.keys())}")
                return None
            
            # Try to get resolution and map size
            resolution = None
            map_size_meters = None
            
            if 'metadata' in data:
                metadata = data['metadata'].item()  # Convert numpy scalar to dict
                resolution = metadata.get('resolution', None)
                map_size_meters = metadata.get('size_meters', None)
            
            # Fallback: try direct keys
            if resolution is None and 'resolution' in data:
                resolution = float(data['resolution'])
            if map_size_meters is None and 'map_size_meters' in data:
                map_size_meters = float(data['map_size_meters'])
            
            # Calculate from shape if still not available
            if resolution is None or map_size_meters is None:
                H, W = occ_grid.shape
                if map_size_meters is None:
                    map_size_meters = 2.0  # Default
                if resolution is None:
                    resolution = map_size_meters / H
            
            # Get log-odds grid if available
            log_odds_grid = None
            if 'log_odds_grid' in data:
                log_odds_grid = data['log_odds_grid']
            
            map_info = {
                'occ_grid': occ_grid,
                'resolution': float(resolution),
                'map_size_meters': float(map_size_meters),
                'log_odds_grid': log_odds_grid
            }
            
            print(f"\n✅ 成功加载NPZ地图:")
            print(f"   - 文件: {npz_path}")
            print(f"   - 地图尺寸: {map_info['occ_grid'].shape}")
            print(f"   - 分辨率: {map_info['resolution']:.4f} m/pixel")
            print(f"   - 物理尺寸: {map_info['map_size_meters']:.2f} m")
            print(f"   - 自由空间: {np.sum(map_info['occ_grid'] == 0)} pixels")
            print(f"   - 障碍物: {np.sum(map_info['occ_grid'] == 1)} pixels")
            print(f"   - 未知区域: {np.sum(map_info['occ_grid'] == 2)} pixels")
            
            return map_info
        except Exception as e:
            print(f"❌ 加载NPZ文件失败: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def load_json_data(self, json_path: str) -> Optional[Dict]:
        """Load LiDAR data from JSON snapshot"""
        if not os.path.exists(json_path):
            print(f"❌ JSON文件不存在: {json_path}")
            return None
        
        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            print(f"\n✅ 成功加载JSON数据:")
            print(f"   - 文件: {json_path}")
            print(f"   - 采集时间: {data.get('time', 'N/A')}")
            
            if 'statistics' in data:
                stats = data['statistics']
                print(f"   - 原始行数: {stats.get('total_raw_lines', 0)}")
                print(f"   - 雷达点数: {stats.get('lidar_points_parsed', 0)}")
                print(f"   - 里程计记录: {stats.get('odom_messages_parsed', 0)}")
                print(f"   - 稀疏点总数: {stats.get('sparse_points_collected', 0)}")
            
            return data
        except Exception as e:
            print(f"❌ 加载JSON文件失败: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def visualize_npz_map(self, map_info: Dict, ax=None):
        """Visualize occupancy grid from NPZ"""
        if ax is None:
            self.fig, ax = plt.subplots(1, 1, figsize=(10, 10))
        
        occ_grid = map_info['occ_grid']
        resolution = map_info['resolution']
        map_size = map_info['map_size_meters']
        
        # Create color image: 0=white(free), 1=black(obstacle), 2=gray(unknown)
        img = np.zeros(occ_grid.shape, dtype=np.uint8)
        img[occ_grid == 0] = 255  # Free = white
        img[occ_grid == 1] = 0    # Obstacle = black
        img[occ_grid == 2] = 128  # Unknown = gray
        
        # Display
        extent = [0.0, map_size, 0.0, map_size]
        ax.imshow(img, cmap='gray', origin='lower', extent=extent, vmin=0, vmax=255)
        
        ax.set_title('Occupancy Grid Map', fontsize=14, fontweight='bold')
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.grid(True, alpha=0.3)
        
        # Add statistics text
        free_count = np.sum(occ_grid == 0)
        occ_count = np.sum(occ_grid == 1)
        unknown_count = np.sum(occ_grid == 2)
        total = occ_grid.size
        
        stats_text = f'Statistics:\n'
        stats_text += f'Free Space: {free_count} ({free_count/total*100:.1f}%)\n'
        stats_text += f'Obstacles: {occ_count} ({occ_count/total*100:.1f}%)\n'
        stats_text += f'Unknown: {unknown_count} ({unknown_count/total*100:.1f}%)\n'
        stats_text += f'Resolution: {resolution:.4f} m/px'
        
        ax.text(0.02, map_size-0.02, stats_text, fontsize=10,
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.9),
                verticalalignment='top', family='monospace')
        
        return ax
    
    def visualize_log_odds(self, map_info: Dict, ax=None):
        """Visualize log-odds heatmap"""
        if ax is None:
            self.fig, ax = plt.subplots(1, 1, figsize=(10, 10))
        
        if map_info.get('log_odds_grid') is None:
            ax.text(0.5, 0.5, '无Log-odds数据', ha='center', va='center', fontsize=16)
            ax.set_title('Log-odds 热图', fontsize=14, fontweight='bold')
            return ax
        
        log_odds = map_info['log_odds_grid']
        map_size = map_info['map_size_meters']
        extent = [0.0, map_size, 0.0, map_size]
        
        # Display log-odds heatmap
        im = ax.imshow(log_odds, cmap='RdBu_r', origin='lower', 
                      extent=extent, vmin=-2.5, vmax=3.5)
        plt.colorbar(im, ax=ax, label='Log-odds Value')
        
        ax.set_title('Log-odds Heatmap (Red=Obstacle, Blue=Free)', fontsize=14, fontweight='bold')
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.grid(True, alpha=0.3, color='white', linewidth=0.5)
        
        # Statistics
        lo_stats = f'Log-odds Stats:\n'
        lo_stats += f'Min: {log_odds.min():.2f}\n'
        lo_stats += f'Max: {log_odds.max():.2f}\n'
        lo_stats += f'Mean: {log_odds.mean():.2f}\n'
        lo_stats += f'Std: {log_odds.std():.2f}'
        
        ax.text(0.02, map_size-0.02, lo_stats, fontsize=10,
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.9),
                verticalalignment='top', family='monospace')
        
        return ax
    
    def visualize_lidar_points(self, json_data: Dict, ax=None):
        """Visualize sparse LiDAR points"""
        if ax is None:
            self.fig, ax = plt.subplots(1, 1, figsize=(10, 10))
        
        # Get sparse points - try multiple methods
        sparse_points = json_data.get('sparse_lidar_points', [])
        
        # If sparse_lidar_points is empty, try extracting from raw lines
        if not sparse_points:
            from sparse_lidar_processor import SparseLidarProcessor
            processor = SparseLidarProcessor()
            sparse_points = processor.extract_sparse_points_from_json(json_data)
        
        if not sparse_points:
            ax.text(0.5, 0.5, '无雷达数据', ha='center', va='center', fontsize=16)
            ax.set_title('雷达扫描点', fontsize=14, fontweight='bold')
            return ax
        
        # Robot pose and map parameters - use config.py values
        from core.config import ROBOT_START_X, ROBOT_START_Y, ROBOT_START_THETA, SLAM_MAP_SIZE_METERS, LIDAR_RANGE
        robot_x, robot_y = ROBOT_START_X, ROBOT_START_Y
        robot_theta = ROBOT_START_THETA  # 0 degrees (facing east)
        map_size = SLAM_MAP_SIZE_METERS  # 2.8m
        lidar_range = LIDAR_RANGE  # 雷达最大距离
        
        # Convert LiDAR points to world coordinates
        world_points = []
        for pt in sparse_points:
            angle_deg = pt['angle_deg']
            distance_m = pt['distance_m']
            quality = pt.get('quality', 0)
            
            # 修正雷达角度方向：硬件顺时针为正 → 数学逆时针为正
            # 将雷达角度从顺时针转为逆时针（取负值）
            corrected_angle_deg = -angle_deg
            # 雷达角度 + 小车朝向 = 世界坐标系角度
            world_angle_deg = corrected_angle_deg + math.degrees(robot_theta)
            world_angle_rad = math.radians(world_angle_deg)
            
            # 转换到世界坐标系
            x_world = distance_m * math.cos(world_angle_rad) + robot_x
            y_world = distance_m * math.sin(world_angle_rad) + robot_y
            
            world_points.append((x_world, y_world, quality))
        
        # Plot points
        if world_points:
            xs, ys, qs = zip(*world_points)
            scatter = ax.scatter(xs, ys, c=qs, cmap='viridis', s=50, 
                               alpha=0.7, edgecolors='black', linewidth=0.5)
            plt.colorbar(scatter, ax=ax, label='质量 (Quality)')
        
        # Plot robot
        ax.plot(robot_x, robot_y, 'ro', markersize=15, label='机器人位置')
        ax.arrow(robot_x, robot_y,
                0.2*math.cos(robot_theta), 0.2*math.sin(robot_theta),
                head_width=0.1, head_length=0.1, fc='red', ec='red')
        
        # Draw LiDAR range circle - use config.py parameter
        circle = plt.Circle((robot_x, robot_y), lidar_range, color='blue', 
                          fill=False, linestyle='--', linewidth=2, alpha=0.5)
        ax.add_patch(circle)
        
        # Set axis limits to map size - use config.py parameter
        margin = 0.1  # Small margin around the map
        ax.set_xlim(-margin, map_size + margin)
        ax.set_ylim(-margin, map_size + margin)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=10)
        
        ax.set_title(f'LiDAR Scan Points (Total: {len(sparse_points)})', 
                    fontsize=14, fontweight='bold')
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        
        # Add statistics
        if world_points:
            distances = [pt['distance_m'] for pt in sparse_points]
            angles = [pt['angle_deg'] for pt in sparse_points]
            qualities = [pt.get('quality', 0) for pt in sparse_points]
            
            stats_text = f'LiDAR Stats:\n'
            stats_text += f'Points: {len(sparse_points)}\n'
            stats_text += f'Distance: {min(distances):.2f} ~ {max(distances):.2f} m\n'
            stats_text += f'Angle: {min(angles):.1f}° ~ {max(angles):.1f}°\n'
            stats_text += f'Quality: {min(qualities)} ~ {max(qualities)}'
            
            ax.text(0.02, map_size - 0.02, stats_text, fontsize=10,
                   bbox=dict(boxstyle='round', facecolor='white', alpha=0.9),
                   verticalalignment='top', family='monospace')
        
        return ax
    
    def visualize_polar_plot(self, json_data: Dict, ax=None):
        """Visualize LiDAR data in polar plot"""
        if ax is None:
            self.fig = plt.figure(figsize=(10, 10))
            ax = self.fig.add_subplot(111, projection='polar')
        
        # Get sparse points - try multiple methods
        sparse_points = json_data.get('sparse_lidar_points', [])
        
        # If sparse_lidar_points is empty, try extracting from raw lines
        if not sparse_points:
            from sparse_lidar_processor import SparseLidarProcessor
            processor = SparseLidarProcessor()
            sparse_points = processor.extract_sparse_points_from_json(json_data)
        
        if not sparse_points:
            return ax
        
        # Extract data
        angles = [math.radians(pt['angle_deg']) for pt in sparse_points]
        distances = [pt['distance_m'] for pt in sparse_points]
        qualities = [pt.get('quality', 0) for pt in sparse_points]
        
        # Plot
        scatter = ax.scatter(angles, distances, c=qualities, cmap='viridis',
                           s=100, alpha=0.7, edgecolors='black', linewidth=0.5)
        
        ax.set_theta_zero_location('N')  # 0° at top
        ax.set_theta_direction(-1)  # Clockwise
        ax.set_ylim(0, 6.0)
        ax.set_title(f'Polar LiDAR Plot (Points: {len(sparse_points)})', 
                    fontsize=14, fontweight='bold', pad=20)
        
        # Add range rings
        ax.set_rticks([0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5])
        ax.grid(True, alpha=0.3)
        
        plt.colorbar(scatter, ax=ax, label='Quality', pad=0.1)
        
        return ax
    
    def visualize_realtime_map(self, map_info: Dict, ax=None):
        """可视化实时建图结果（类似GUI界面右上图）"""
        if ax is None:
            self.fig, ax = plt.subplots(1, 1, figsize=(10, 10))
        
        occ_grid = map_info['occupancy_grid']
        resolution = map_info['resolution']
        map_size = map_info['map_size_meters']
        robot_pose = map_info['robot_pose']
        stats = map_info['statistics']
        
        # 创建颜色图像：0=白色(自由), 1=黑色(障碍), 2=灰色(未知)
        img = np.zeros(occ_grid.shape, dtype=np.uint8)
        img[occ_grid == 0] = 255  # 自由 = 白色
        img[occ_grid == 1] = 0    # 障碍 = 黑色
        img[occ_grid == 2] = 128  # 未知 = 灰色
        
        # 显示地图
        extent = [0.0, map_size, 0.0, map_size]
        ax.imshow(img, cmap='gray', origin='lower', extent=extent, vmin=0, vmax=255)
        
        # 显示机器人位置
        rx, ry, rtheta = robot_pose
        ax.plot(rx, ry, 'ro', markersize=12, label='机器人位置')
        ax.arrow(rx, ry, 0.2*math.cos(rtheta), 0.2*math.sin(rtheta),
                 head_width=0.05, head_length=0.05, fc='red', ec='red')
        
        # 设置标题和范围
        ax.set_title(f'实时稀疏建图 - 2.8m x 2.8m (点数: {stats["total_points"]})', 
                    fontsize=14, fontweight='bold')
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        
        # 设置坐标轴范围
        margin = 0.1
        ax.set_xlim(-margin, map_size + margin)
        ax.set_ylim(-margin, map_size + margin)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=10)
        
        # 添加统计信息（简化显示，避免中文乱码）
        stats_text = f'Real-time Mapping Stats:\n'
        stats_text += f'Points: {stats["total_points"]}\n'
        stats_text += f'Free: {stats["free_cells"]}\n'
        stats_text += f'Obstacles: {stats["occupied_cells"]}\n'
        stats_text += f'Unknown: {stats["unknown_cells"]}\n'
        stats_text += f'Coverage: {stats["coverage_ratio"]:.1%}'
        
        ax.text(0.02, map_size - 0.02, stats_text, fontsize=10,
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.9),
                verticalalignment='top', family='monospace')
        
        return ax
    
    def create_map_from_sparse_points(self, sparse_points: List[Dict], 
                                    robot_pose: Tuple[float, float, float] = None) -> Dict:
        """从稀疏雷达点创建地图"""
        print(f"[VISUALIZER] 从 {len(sparse_points)} 个稀疏点创建地图")
        
        # 创建稀疏雷达处理器 - 使用config.py默认参数
        self.sparse_processor = SparseLidarProcessor()
        
        # 处理稀疏点
        if robot_pose is None:
            from core.config import ROBOT_START_X, ROBOT_START_Y, ROBOT_START_THETA
            robot_pose = (ROBOT_START_X, ROBOT_START_Y, ROBOT_START_THETA)  # 使用config.py默认位姿
        
        map_data = self.sparse_processor.process_sparse_points(sparse_points, robot_pose)
        
        # 创建地图信息
        map_info = self.sparse_processor.create_map_info_for_visualization()
        
        print(f"[VISUALIZER] 地图创建完成: {map_info['occ_grid'].shape}")
        return map_info
    
    
    
    def show_complete_analysis(self, npz_path: Optional[str] = None, 
                              json_path: Optional[str] = None, use_sparse_mapping: bool = False):
        """Show complete analysis with multiple panels"""
        
        # Load data
        map_info = None
        json_data = None
        
        if npz_path:
            map_info = self.load_npz_map(npz_path)
        
        if json_path:
            json_data = self.load_json_data(json_path)
            
            # 如果启用稀疏建图且JSON中有稀疏点数据，创建地图
            if use_sparse_mapping and json_data:
                # 使用处理器提取稀疏点
                from sparse_lidar_processor import SparseLidarProcessor
                processor = SparseLidarProcessor()
                sparse_points = processor.extract_sparse_points_from_json(json_data)
                
                if sparse_points:
                    print(f"[VISUALIZER] 从JSON中的 {len(sparse_points)} 个稀疏点创建地图")
                    sparse_map_info = self.create_map_from_sparse_points(sparse_points)
                    # 如果原来没有地图，使用稀疏建图结果
                    if map_info is None:
                        map_info = sparse_map_info
                    else:
                        print("[VISUALIZER] 已有NPZ地图，跳过稀疏建图")
                else:
                    print("[VISUALIZER] JSON中没有稀疏点数据，无法建图")
        
        if map_info is None and json_data is None:
            print("❌ 没有可用的数据")
            return
        
        # Determine layout
        if map_info and json_data:
            # Show everything: 2x3 layout
            self.fig, axes = plt.subplots(2, 3, figsize=(20, 13))
            axes = axes.flatten()
        elif map_info:
            # Show map only: 1x2 layout
            self.fig, axes = plt.subplots(1, 2, figsize=(18, 8))
        elif json_data:
            # Show LiDAR only: 1x3 layout
            self.fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        
        self.fig.suptitle('LiDAR 建图分析工具', fontsize=18, fontweight='bold')
        
        # Plot panels
        idx = 0
        
        if map_info:
            self.visualize_npz_map(map_info, axes[idx])
            idx += 1
            
            self.visualize_log_odds(map_info, axes[idx])
            idx += 1
        
        if json_data:
            self.visualize_lidar_points(json_data, axes[idx])
            idx += 1
            
            # Polar plot needs special handling
            if map_info and json_data:
                # Remove the axis and create polar subplot
                self.fig.delaxes(axes[idx])
                ax_polar = self.fig.add_subplot(2, 3, idx+1, projection='polar')
                self.visualize_polar_plot(json_data, ax_polar)
                idx += 1
        
        # Hide unused axes
        for i in range(idx, len(axes)):
            axes[i].axis('off')
        
        plt.tight_layout()
        plt.show()
        
        print("\n✅ 可视化完成！")


def main():
    parser = argparse.ArgumentParser(
        description='LiDAR建图可视化工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 可视化NPZ地图文件
  python visualize_lidar_map.py --npz logs/lidar_mapping/slam_map_20251012_233645.npz
  
  # 可视化JSON雷达数据
  python visualize_lidar_map.py --json logs/lidar_mapping/lidar_odom_snapshot_20251012_233136.json
  
  # 同时显示地图和雷达数据
  python visualize_lidar_map.py --npz slam_map.npz --json lidar_snapshot.json
        """
    )
    
    parser.add_argument('--npz', type=str, help='NPZ地图文件路径')
    parser.add_argument('--json', type=str, help='JSON雷达数据文件路径')
    parser.add_argument('--sparse', action='store_true', help='从JSON稀疏点建图')
    
    args = parser.parse_args()
    
    # Create visualizer
    viz = MapVisualizer()
    
    
    if not args.npz and not args.json:
        parser.print_help()
        print("\n❌ 错误: 请至少指定一个文件 (--npz 或 --json)")
        sys.exit(1)
    
    # Show analysis with sparse mapping option
    viz.show_complete_analysis(args.npz, args.json, use_sparse_mapping=args.sparse)


if __name__ == '__main__':
    main()

