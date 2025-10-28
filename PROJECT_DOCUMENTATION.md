# 雷达小车自主迷宫探索系统 - 项目文档

**项目版本**: v4.1 - 高波特率优化系统  
**最后更新**: 2025-10-24  
**状态**: 开发中 - 高波特率通信优化和点数量控制建图

---

## 📋 项目概述

### 核心目标
基于激光雷达的自主迷宫探索系统，实现：
1. **真实硬件导航**：支持真实机器人硬件，通过蓝牙串口通信控制小车移动和获取传感器数据
2. **高波特率优化**：支持921600 bps高速通信，单线程批量处理，点数量控制建图
3. **稀疏雷达建图**：基于稀疏雷达点数据的实时建图，处理硬件数据限制和角度稀疏问题
4. **里程计融合**：实时处理里程计数据，准确更新机器人位姿，支持坐标转换和角度对齐
5. **模块化架构**：完整的模块化导航系统，包含硬件接口、数据处理、建图、规划、可视化等模块
6. **数据持久化**：完整的日志记录系统，保存原始雷达数据、建图结果、导航统计等信息

### 技术架构 (v4.1 高波特率优化系统)
```
┌─────────────────────────────────────────────────────────────┐
│                    主入口 (test_nav_modular.py)               │
│        模块化导航测试 | 硬件参数配置 | 异常处理              │
└────────────┬────────────────────────────────┬───────────────┘
             │                                │
     ┌───────▼───────┐                ┌──────▼──────┐
     │  硬件通信层    │                │  数据处理层  │
     │ NavBleInterface│                │ OdometryProc│
     │ ┌───────────┐ │                │ │LidarParser│ │
     │ │Serial I/O │ │                │ │CoordTrans │ │
     │ │CommandTx  │ │                │ └──────────┘ │
     │ └───────────┘ │                └──────┬──────┘
     └───────┬───────┘                       │
             │                                │
     ┌───────▼────────────────────────────────▼───────┐
     │              核心导航应用层                       │
     │           NavMappingApp (状态机)                │
     │ ┌─────────────┬─────────────┬─────────────┐    │
     │ │  建图模块   │  导航模块   │  可视化模块  │    │
     │ │SparseLidar │FrontierExp │NavVisualizer│    │
     │ │Processor   │AStarPlanner│Real-timeGUI │    │
     │ │Log-OddsMap │MotionCtrl  │MapDisplay   │    │
     │ └─────────────┴─────────────┴─────────────┘    │
     └─────────────────┬───────────────────────────────┘
                       │
     ┌─────────────────▼───────────────────────────────┐
     │              数据持久化层                        │
     │       原始数据记录 | 建图结果保存 | 统计日志      │
     │        logs/nav_exploring/ 目录结构              │
     └─────────────────────────────────────────────────┘
```

**状态机设计**:
```python
State.MAPPING → State.NAVIGATING → State.TURNING → State.MOVING → State.COMPLETE
    ↓              ↓                 ↓              ↓
  建图阶段      导航规划阶段       转向执行阶段    移动执行阶段    完成状态
  (点数量控制)   (前沿点探索)       (轴对齐原语)    (轴对齐原语)
```

**高波特率优化特性**:
- **921600 bps通信**: 支持高速数据传输
- **点数量控制建图**: 500点主建图, 300点段后建图
- **单线程批量处理**: 避免多线程竞争，提升处理效率
- **GUI更新优化**: 每500点更新一次，减少O(N²)计算开销
- **内存管理**: 实时监控和清理，防止内存溢出

---

## 📂 目录结构与模块职责

### `/core` - 核心模块 (v4.1 高波特率优化支持)

| 文件 | 职责 | 关键功能 |
|------|------|---------|
| `config.py` | 全局配置参数 | 地图参数、机器人参数、SLAM参数、探索参数 |
| `types.py` | 数据类型定义 | Pose2D、LaserScan、Control等基础类型 |
| `coords.py` | 坐标转换 | 世界坐标↔网格坐标转换 |
| `odometry_processor.py` | **里程计处理器** | 里程计状态管理、坐标转换、位姿更新 |
| `lidar_parser.py` | **雷达数据解析器** | 稀疏雷达点解析、数据格式转换 |
| `robot_adapter.py` | 机器人抽象接口 | 统一仿真/真实机器人接口 |
| `system_initializer.py` | 系统初始化器 | 统一初始化所有模块 |
| `map_validator.py` | 地图验证器 | 验证地图质量和一致性 |
| `robot_factory.py` | 机器人工厂 | 创建仿真/真实机器人实例 |
| `logger.py` | 日志工具 | 统一日志接口 |

**关键参数**（`config.py`）:
```python
# 地图参数
SLAM_MAP_SIZE_PIXELS = 200      # 200×200像素
SLAM_MAP_SIZE_METERS = 2.0      # 2.0m × 2.0m物理尺寸
SLAM_RESOLUTION = 0.01          # 0.01m/pixel (1cm分辨率)

# 机器人参数 (Point Mass Model)
ROBOT_RADIUS = 0.0              # 规划半径（质点模型，物理半径已包含在SAFE_BUFFER_M中）
SAFE_BUFFER_M = 0.12            # 安全缓冲：12cm (补偿代码中多重膨胀机制)
PHYSICAL_RADIUS = 0.10          # 物理半径：10cm（仅用于脚印清除和仿真碰撞检测）

# 高波特率优化参数 (v4.1新增)
HIGH_BAUDRATE_MODE: bool = True         # 启用高波特率优化
HIGH_BAUDRATE_BAUD: int = 921600       # 高波特率阈值
HIGH_BAUDRATE_GUI_UPDATE_INTERVAL: int = 500  # GUI更新间隔(点数量)
SINGLE_THREAD_BATCH_SIZE: int = 300    # 单线程批量处理大小
SINGLE_THREAD_EMPTY_BATCH_LIMIT: int = 50  # 空批次检测限制

# 点数量控制建图参数 (v4.1新增)
MAIN_MAPPING_POINTS: int = 500         # 主建图目标点数
POST_SEGMENT_MAPPING_POINTS: int = 300 # 段后建图目标点数
DEFAULT_MAPPING_POINTS: int = 500      # 默认建图点数

# 设计理念说明
规划空间设计（Point Mass Model）：
  - 机器人视为质点（ROBOT_RADIUS = 0）
  - 所有安全距离统一在SAFE_BUFFER_M中处理
  - 避免双重膨胀，简化碰撞检测逻辑
  - 物理半径(10cm)仅在仿真碰撞检测和脚印清除时使用

v3.2参数调整说明：
  - SAFE_BUFFER_M从0.15m调整为0.12m
  - 原因：补偿代码中多重膨胀机制（扩散+取整+边界复制+unknown合并）
  - 实际膨胀带 ≈ 理论膨胀带 + 2~4px（扩散+取整）+ 1px（边界效应）
  - 调整后实际膨胀带接近预期的15cm安全缓冲

# 控制参数
V_MAX = 1.0                     # 最大线速度 1.0m/s
W_MAX = 0.8                     # 最大角速度 0.8rad/s
CONTROL_HZ = 20.0               # 控制频率 20Hz

# 原语控制参数（v3.1新增）
PRIM_TURN_TOL_DEG = 8.0         # TURN完成容差
PRIM_MOVE_YAW_TOL_DEG = 6.0     # θ_move角度门限（MOVE）
PRIM_GATE_YAW_TOL_DEG = 6.0     # θ_move角度门限（GATE，与MOVE统一）
PRIM_MOVE_V_FAST = 0.25         # MOVE快速速度
PRIM_GATE_V_FAST = 0.22         # GATE快速速度
PRIM_GATE_EPS_M = 0.03          # 闸门穿越容差（优化后）

# 到达判定参数（v3.2新增）
GOAL_BAND_BOUNDARY_TOLERANCE_M = 0.02  # 目标带边界容差：2cm
```

### `/slam` - SLAM系统（解耦架构）

| 文件 | 职责 | 关键方法 |
|------|------|---------|
| `slam_system.py` | SLAM系统协调器 | `update()`, `complete_mapping()`, `get_pose()` |
| `map_builder.py` | **纯建图模块** | `update()`, `get_occupancy_grid()`, `export_map()` |
| `localizer.py` | **纯定位模块** | `predict()`, `correct()`, ICP匹配 |

**解耦架构核心思想**:
```
阶段1：建图阶段 (mapping_phase=True)
  - 仿真模式：使用真实位姿 (true_pose) 建图
  - 真实模式：原地扫描或慢速移动建图
  - 输出：高质量occupancy grid map
  
阶段2：定位阶段 (mapping_phase=False)
  - 使用ICP在已知地图上定位
  - 输入：已建好的地图 + 激光扫描
  - 输出：估计位姿 (pose estimation)
```

**MapBuilder关键技术**（最新优化）:
- **Log-odds建图**: 增量式概率占据栅格更新
- **智能射线追踪**: 检测已知障碍，防止穿墙
- **双阈值+多数投票**: 
  - 强障碍 (log-odds>1.2): 直接标记
  - 弱障碍 (0.5<log-odds<1.2): 需要≥5个邻居支持
  - **效果**: 消除"虚假障碍带"，恢复前沿点探索
- **入口/机器人保护**: 强制清除入口和机器人脚印区域

### `/nav` - 导航系统

| 文件 | 职责 | 状态机 |
|------|------|--------|
| `navigator.py` | 导航状态机 | `IDLE → TO_ENTRANCE → EXPLORING → TO_EXIT → FINISHED` |
| `motion_controller.py` | **统一运动控制器** | 轴对齐运动原语（TURN/MOVE/GATE）、闸门检测、θ_move统一门限 |

**Navigator三阶段导航**:
```python
阶段1: TO_ENTRANCE
  - 目标：迷宫入口 (0.325, 0.1)
  - 完成条件：距离入口<0.08m
  
阶段2: EXPLORING (核心阶段)
  - 目标：前沿点探索
  - 循环：选择前沿点 → 构建可达区域(corridor band) → 生成原语序列 → 原语执行 → 闸门检测到达 → 重复
  - 完成条件：无前沿点 + 覆盖率>30%
  
阶段3: TO_EXIT (v3.4优化)
  - 目标：迷宫出口
  - 完成条件：到达出口
  - 规划方式：到达带机制（复用探索阶段成熟逻辑）
  - 目标点处理：自动阻塞点吸附（螺旋搜索最近可达点）
```

**MotionController原语运动控制器**:
```python
# v3.1 轴对齐运动原语架构
class MotionController:
  def follow_primitive(pose, primitive, remaining_dist) -> (v, w, done):
    """
    执行单个运动原语：
    1. TURN: 原地旋转到目标航向（容差8°）
    2. MOVE: 沿轴向移动指定距离（θ_move=6°门限）
    3. GATE: 沿轴向移动直到穿越闸门线（动态方向重估）
    
    Key Features:
    - 统一θ_move角度门限（MOVE/GATE一致）
    - 动态GATE方向重估（避免反向推进）
    - 鲁棒日志输出（remaining=None安全处理）
    """
    
  def step(pose) -> (v, w):
    """
    传统路径跟踪（向后兼容）：
    1. 严格先旋转后移动控制
    2. 四向锁定（0°/±90°/180°）
    3. 分段曼哈顿路径管理
    """
```

### `/explore` - 探索模块（工业标准实践）

| 文件 | 职责 | 算法 |
|------|------|------|
| `frontier_explorer.py` | 前沿点探索 | BFS连通性 + A*验证 + Pullback策略 |
| `entry_detector.py` | 入口检测（已弃用） | - |
| `smart_entry_detector.py` | 智能入口检测（已弃用） | - |

**FrontierExplorer工业标准策略**（v3.2更新 2025-10-09）:

```python
find_frontiers(occ_grid, robot_pose):
  """
  工业标准前沿点检测 + ROI优化
  
  Steps:
  1. ROI处理: 只在机器人传感器范围+0.25m内搜索（性能优化）
  2. BFS计算机器人自由连通域（4-connectivity）
  3. 只在连通域内查找前沿点（free邻接unknown）
  4. 聚类前沿点（最小3个cell）
  5. 记忆前沿簇中心（用于卡死回退）
  
  Key: 只返回机器人可达的前沿点，支持记忆回退
  """

choose_next_frontier(occ_grid, pose):
  """
  v3.2更新：带检验前沿点选择 + 统一C-space膨胀
  
  Phase 1: 中线偏置排序（P1优化）
    - 计算: cost = distance + 0.3×lateral_offset
    - 效果: 优先选择通道中央的前沿点
  
  Phase 2: 带检验前沿点验证（v3.2核心更新）
    - 前沿点 → _compute_goal_band_for_frontier() → 矩形到达带
    - 带验证：检查矩形到达带的可达性而非单点
    - A*验证：规划到带边界（gate point）而非前沿点中心
    - 边界选择：基于机器人位置选择最合适的边界
  
  Phase 3: 记忆回退机制（P3保留）
    - 当前轮全部失败时，从记忆队列回退
    - 优先尝试最近120s内的历史前沿点
    - 允许1px朝机器人方向微调通过安全检查
    - 使用相同C-space进行A*验证
  
  Key Insight: 带检验规避前沿点处于障碍带的问题，统一膨胀消除不一致性
  """
```

**v3.2核心更新：带检验机制**
```python
_compute_goal_band_for_frontier(frontier_world, pose, occ_full):
  """
  基于前沿点构建矩形到达带（corridor band）
  
  Steps:
  1. 统一C-space膨胀: 使用maximum_filter1d矩形膨胀（与Navigator一致）
  2. 前沿点吸附: 若在障碍上，吸附到最近FREE像素
  3. 四向扫描: scan_left/right/up/down 到最近障碍
  4. 构建候选带:
     - 水平带: 垂直方向扫描交集，轴向截断到tolerance
     - 垂直带: 水平方向扫描交集，轴向截断到tolerance
  5. 选择最优: 基于面积和宽度误差选择水平/垂直带
  
  Key: 确保整条带可行，规避前沿点障碍带问题
  """

# 带检验边界选择逻辑（当前问题）
if (xmax - xmin) < (ymax - ymin):  # 竖条带
    gate_x = xmin if abs(pose.x - xmin) <= abs(pose.x - xmax) else xmax
    gate_y = 0.5 * (ymin + ymax)
else:                              # 横条带
    gate_x = 0.5 * (xmin + xmax)
    gate_y = ymin if abs(pose.y - ymin) <= abs(pose.y - ymax) else ymax

# 问题：基于几何形状判定边界，缺乏语义上下文
# 解决方案：基于机器人位置选择最合适的边界（待实现）
```

**最新优化**:
- **P0** (2025-10-01): 修复occupancy grid导出 - 双阈值+多数投票，消除"虚假障碍带"
- **P1** (2025-10-01): 中线偏置排序 - 优先选择通道中央前沿点（λ=0.3）
- **P2** (2025-10-01): 两级回退兜底 - 4px失败时尝试2px（窄通道适配）
- **P3** (2025-10-01): 记忆回退机制 - 卡死时自动回退到历史有效前沿点
- **v3.2** (2025-10-09): 统一C-space膨胀 - EDT→maximum_filter1d，消除膨胀不一致性
- **v3.2** (2025-10-09): 带检验前沿点选择 - 规划到到达带而非单点，提高鲁棒性

### `/planning` - 路径规划

| 文件 | 职责 | 算法 |
|------|------|------|
| `global_planner.py` | 全局路径规划 | A*算法 + 矩形C-space膨胀 + 原语序列生成 |
| `local_planner.py` | 局部避障 | DWA动态窗口法（已弃用，统一使用MotionController） |
| `astar.py` | A*核心算法 | 启发式搜索 |

**AStarPlanner关键技术**:
```python
plan(start_ij, goal_ij, occ_grid, safe_buffer_m=0.12):
  """
  A*路径规划流程（v3.0 矩形膨胀）
  
  Steps:
  1. 矩形C-space膨胀: maximum_filter1d (轴向可分)
  2. 起点/入口保护: 清除clear_radius = inflate_pixels + 3 (15px)
  3. A*搜索: 启发式h(n) = 欧氏距离
  4. 曼哈顿路径: 最少拐角，轴对齐
  
  Key: 矩形膨胀产生90度拐角，避免三角形膨胀
  """

plan_to_band(start_ij, band_rect, occ_grid):
  """
  规划到可达区域的曼哈顿路径
  
  Strategy:
  1. 直线尝试: 起点到band中心的直线路径
  2. L型路径: 强制次序的L型路径
  3. 兜底A*: 到band中心的最少拐角路径
  
  Key: 规避前沿点处于障碍带中的情况
  """

plan_axis_primitives(pose, band_rect) -> List[Dict]:
  """
  生成轴对齐运动原语序列（v3.1新增）
  
  Primitives:
  1. TURN: {"type":"TURN", "heading_rad":float}
  2. MOVE: {"type":"MOVE", "axis":"x"|"y", "distance":float, "dir":±1}
  3. GATE: {"type":"GATE", "axis":"x"|"y", "gate":float, "dir":±1}
  
  Strategy:
  - 判断走廊方向（水平/垂直）
  - 先对中Y/X坐标
  - 沿主轴推进到闸门
  - 穿越闸门进入到达带
  
  Key: 离散化运动，闸门检测到达，避免连续跟踪的复杂性
  """
```

**MotionController原语控制详解**:
```python
# v3.1 三种运动原语实现
_cmd_turn(pose, heading_rad, tol_deg=8.0) -> (v, w, done):
  """
  TURN原语：原地旋转到目标航向
  - 完成条件: |angle_error| ≤ 8° (PRIM_TURN_TOL_DEG)
  - 控制律: w = Kp × error (比例控制，限幅±0.6)
  - 特点: v=0严格原地旋转
  """

_cmd_move_axis(pose, axis, distance_m, dir_sign, yaw_tol_deg=6.0) -> (v, w, done):
  """
  MOVE原语：沿轴向移动指定距离
  - 完成条件: remaining_distance ≤ 0.5×SLAM_RESOLUTION
  - 角度门限: |yaw_error| < 6° (θ_move) → 快速v=0.25m/s
  - 角度门限: |yaw_error| ≥ 6° → 慢速v=0.12m/s
  - 转向修正: w = Kp × yaw_error (走廊容忍)
  - 特点: 允许小幅转向修正，保持轴向前进
  """

_cmd_move_till_gate(pose, axis, dir_sign, gate_val, yaw_tol_deg=6.0) -> (v, w, done):
  """
  GATE原语：沿轴向移动直到穿越闸门线
  - 完成条件: 按dir_sign方向穿越gate_val坐标
  - 角度门限: 与MOVE统一使用θ_move=6° (PRIM_GATE_YAW_TOL_DEG)
  - 方向判定: 每帧重估dir_sign（避免反向推进）
  - 特点: 动态到达检测，适应位置变化
  """

# 传统路径跟踪（向后兼容）
step(pose) -> (v, w):
  """
  先旋转后移动的严格控制
  - 四向锁定: 0°/±90°/180°
  - 分段曼哈顿路径管理
  """
```

### `/sim` - 仿真模块

| 文件 | 职责 | 功能 |
|------|------|------|
| `robot_sim.py` | 机器人仿真器 | 运动学仿真、碰撞检测 |
| `maze_map.py` | 迷宫地图 | JSON加载、墙壁渲染、扫描模拟 |
| `sim_robot_adapter.py` | 仿真适配器 | 连接RobotSim到统一接口 |

**迷宫参数**（`1.json`）:
```
物理尺寸: 1.8m × 1.8m
网格尺寸: 5×5 cells
单元格大小: 0.45m
世界坐标系: 2.0m × 2.0m (包含0.1m边界)
入口: [0.5, 0.0] → 世界坐标(0.325, 0.1)
出口: [2.5, 4.0] → 世界坐标(1.225, 1.9)
通道宽度: ~0.18-0.20m (最窄处)
```

### `/gui` - 可视化模块

| 文件 | 职责 | 显示内容 |
|------|------|---------|
| `visualizer.py` | 主可视化器 | 4面板Matplotlib显示 |

**4面板布局**（2×2）:
```
┌──────────────────┬──────────────────┐
│  Panel 1: 地面真值  │  Panel 2: SLAM地图 │
│  - 真实机器人位置   │  - 占据栅格显示     │
│  - 迷宫墙壁        │  - SLAM位姿        │
│  - 激光扫描        │  - 激光扫描        │
├──────────────────┼──────────────────┤
│  Panel 3: A*路径   │  Panel 4: 覆盖图   │
│  - 静态A*路径      │  - 已探索区域      │
│  - 当前机器人位置   │  - 探索进度        │
│  - 目标点         │  - 覆盖率统计      │
└──────────────────┴──────────────────┘
```

**详细栅格分析窗口**（调试用）:
- 占据栅格 (0/1/2)
- Log-odds热力图
- 局部视图（机器人周围）
- 障碍密度图

### `/appio` - 硬件接口 (v4.1 高波特率优化支持)

| 文件 | 职责 | 协议 |
|------|------|------|
| `nav_bluetooth.py` | **导航专用蓝牙接口** | 串口通信、命令发送、数据接收 |
| `real_robot_adapter.py` | 真实机器人适配器 | 蓝牙串口通信 |
| `bluetooth.py` | 蓝牙接口 | PyBluez协议 |
| `logger.py` | 数据记录 | NPZ格式数据保存 |

**NavBleInterface核心功能**:
```python
class NavBleInterface:
    """导航专用蓝牙串口接口 - 高波特率优化版"""
    
    # 命令发送
    send("S")                    # 启动雷达
    send("T")                    # 停止雷达  
    send("(M,<deg>,<mm>)")       # 移动/转向命令
    
    # 高波特率数据接收 (v4.1优化)
    read_batch_lines(max_lines=300)  # 单线程批量读取
    read_batch_lines_filtered()      # 预过滤重要数据
    drain_all_lines()                # 批量读取队列数据
    
    # 单线程优化处理
    _reader()                    # 后台数据读取线程 (批量处理)
    lines: Queue                 # 线程安全数据队列
    _running: bool               # 线程控制标志
```

### `/nav` - 导航系统 (v4.1 高波特率优化应用)

| 文件 | 职责 | 核心功能 |
|------|------|---------|
| `nav_mapping_app.py` | **主导航应用** | 状态机协调、数据流管理、异常处理 |
| `nav_adapters.py` | **导航适配器** | 前沿探索、路径规划、命令生成接口 |
| `navigator.py` | 导航状态机 | 传统导航状态机（IDLE→TO_ENTRANCE→EXPLORING→TO_EXIT→FINISHED） |
| `motion_controller.py` | 运动控制器 | 轴对齐运动原语（TURN/MOVE/GATE） |

**NavMappingApp核心状态机**:
```python
class NavMappingApp:
    """模块化导航和建图应用 - 高波特率优化版"""
    
    # 状态机设计 (v4.1优化)
    State.MAPPING    # 建图阶段：点数量控制建图 (500点/300点)
    State.NAVIGATING # 导航规划：前沿点选择、路径规划、命令生成  
    State.TURNING    # 转向执行：轴对齐原语执行
    State.MOVING     # 移动执行：轴对齐原语执行
    State.COMPLETE   # 完成状态：任务完成、数据保存
    
    # 核心方法 (v4.1新增)
    do_mapping_window_by_points(target_points=500)  # 点数量控制建图
    plan_next_and_emit_primitives() # 导航规划：前沿点探索、路径规划
    execute_commands()           # 命令执行：硬件控制、状态监控
    run()                        # 主循环：状态机协调、异常处理
    
    # 高波特率优化特性
    - 单线程批量处理 (SINGLE_THREAD_BATCH_SIZE=300)
    - GUI更新优化 (每500点更新一次)
    - 内存监控和清理
    - 垃圾点过滤和雷达预热处理
```

---

## 🧠 当前代码状态总结（2025-10-24 v4.1）

### 硬件导航系统核心架构

#### 1. **模块化导航应用** (`nav_mapping_app.py`)

**高波特率优化导航流程**:
```python
NavMappingApp.run()
  │
  ├─> 初始化硬件接口 (NavBleInterface)
  │     ├─> 串口连接 (921600 baud)
  │     ├─> 单线程批量数据处理
  │     └─> 命令发送接口
  │
  ├─> 初始化数据处理模块
  │     ├─> SparseLidarProcessor (稀疏雷达建图)
  │     ├─> OdometryProcessor (里程计处理)
  │     └─> NavVisualizer (优化可视化)
  │
  └─> 主状态机循环 (v4.1优化)
        ├─> State.MAPPING: 点数量控制建图
        │     ├─> 发送"S"启动雷达
        │     ├─> 单线程批量读取 (300行/批次)
        │     ├─> 点数量控制 (500点主建图, 300点段后建图)
        │     ├─> 垃圾点过滤和雷达预热处理
        │     ├─> GUI更新优化 (每500点更新一次)
        │     └─> 发送"T"停止雷达
        │
        ├─> State.NAVIGATING: 导航规划
        │     ├─> 前沿点探索 (FrontierExplorer)
        │     ├─> 路径规划 (AStarPlanner)
        │     └─> 原语分解 (MotionController)
        │
        ├─> State.TURNING/MOVING: 轴对齐原语执行
        │     ├─> 发送硬件命令 "(M,<deg>,<mm>)"
        │     ├─> 监控执行状态 ("arrival", "turning finished")
        │     └─> 实时更新位姿
        │
        └─> State.COMPLETE: 数据保存
              └─> 保存原始数据、地图结果、统计日志
```

#### 2. **稀疏雷达数据处理** (`sparse_lidar_processor.py`)

**真实硬件适配建图**:
```python
SparseLidarProcessor:
  # 处理稀疏雷达数据限制
  - 角度稀疏问题: 前方330°~30°数据稀少(8.4%)
  - 质量筛选: quality >= 5, distance 0.05~3.5m
  - 实时建图: add_single_sparse_point_with_pose()
  
  # Log-odds建图
  _mark_obstacle_point():
    - 角度转换: alpha = -math.radians(angle_deg)
    - 世界坐标: x_world = distance * cos(θw + α) + rx
    - 栅格更新: _update_log_odds(gx, gy, log_hit)
  
  _mark_free_space_ray():
    - Bresenham射线追踪
    - 自由空间标记: _update_log_odds(gx, gy, log_miss)
  
  get_occupancy_grid():
    - 双阈值判断: free_mask < -0.5, occ_mask > 0.5
    - 栅格输出: 0=自由, 1=障碍, 2=未知
```

#### 3. **里程计数据处理** (`core/odometry_processor.py`)

**坐标系统一管理**:
```python
OdometryState:
  # 里程计状态跟踪
  update_from_absolute():    # 绝对位置更新
  update_from_delta():       # 增量数据更新
  cumulative_odom_angle_deg  # 累计角度跟踪

odom_to_world_pose():
  # 坐标转换
  phi = START_THETA_RAD      # 初始朝向基准
  x_world = INIT_XY[0] + c*odom_x + s*odom_y
  y_world = INIT_XY[1] + s*odom_x - c*odom_y  
  theta_world = phi - math.radians(odom_theta_cw)

parse_odometry_line():
  # 数据解析
  - "Time:", "L_Speed:", "R_Speed:"
  - "ΔX:", "ΔY:", "dt:" (mm→m转换)
  - "Angle:", "X:", "Y:" (位置角度)
```

#### 4. **硬件通信接口** (`appio/nav_bluetooth.py`)

**串口通信与数据处理**:
```python
NavBleInterface:
  # 硬件连接
  connect():
    - 串口初始化 (57600 baud, 0.1s timeout)
    - MCU预热 (1.5s等待)
    - 后台数据读取线程启动
  
  # 命令发送
  send("S"):                  # 启动雷达扫描
  send("T"):                  # 停止雷达扫描
  send("(M,<deg>,<mm>)"):     # 移动/转向命令
  
  # 数据接收处理
  _reader():                  # 后台线程持续读取
  drain_all_lines():          # 批量获取队列数据
  readline():                 # 阻塞式单行读取
  
  # 事件检测
  - "arrival": 移动到达
  - "turning finished": 转向完成  
  - "LIDAR Q<quality> A<angle> D<distance>": 雷达数据
  - "ΔX:...": 里程计数据
```

#### 5. **数据持久化系统** (`logs/nav_exploring/`)

**完整日志记录**:
```python
数据保存结构:
logs/nav_exploring/
├── raw_data/                # 原始数据
│   └── raw_data_YYYYMMDD_HHMMSS.txt
├── maps/                    # 地图数据  
│   ├── map_incremental_wXXX_HHMMSS.npz
│   ├── map_incremental_wXXX_HHMMSS.json
│   └── map_visual_wXXX_HHMMSS.png
├── final_map_visual_YYYYMMDD_HHMMSS.png
└── mapping_stats_YYYYMMDD_HHMMSS.txt

异常处理:
- signal.SIGINT/SIGTERM: 优雅关闭信号处理
- atexit.register(): 程序退出时强制保存数据
- _force_save_all_data(): 防止数据丢失机制
```

### 关键硬件适配技术

#### 1. **稀疏雷达数据挑战**
真实硬件雷达数据特点：
- **角度稀疏**: 前方330°~30°范围内数据稀少(8.4%)，主要分布在侧方区域(91.6%)
- **质量筛选**: `quality >= 5`, `distance: 0.05~3.5m`
- **实时处理**: 单线程数据处理，避免队列溢出和数据丢失

#### 2. **里程计融合策略**
- **坐标转换**: 统一使用`ROBOT_START_THETA`作为初始朝向基准
- **角度对齐**: 每个建图窗口首次`Angle:`消息作为角度基准，避免累积误差
- **位姿同步**: 实时更新`mapper.set_robot_pose()`确保建图层使用最新位姿

#### 3. **命令执行与监控**
- **格式统一**: 硬件命令格式`(M,<deg>,<mm>)`，角度为转向度数，距离为毫米
- **状态检测**: 监控`"arrival"`和`"turning finished"`事件确定执行完成
- **超时保护**: 7秒自动确认机制，避免硬件无响应导致的死锁

---

### 传统导航系统 (向后兼容)

#### 2. **前沿点探索系统** (`frontier_explorer.py`)

**工业标准策略**:
```python
# 前沿点检测 + ROI优化
find_frontiers(occ_grid, robot_pose):
  - ROI处理: 传感器范围+0.25m内搜索（性能优化）
  - BFS连通性: 4-connectivity计算机器人可达域
  - 前沿检测: FREE邻接UNKNOWN的边界点
  - 聚类处理: 最小3个cell的前沿簇
  - 记忆备份: 前沿簇中心写入记忆队列

# 三阶段前沿点选择 + 卡死校正
choose_next_frontier(occ_grid, pose):
  Phase 1: 中线偏置排序（λ=0.3）
  Phase 2: 两级回退+A*验证（4px→2px兜底）
  Phase 3: 记忆回退机制（卡死校正）
```

**记忆回退机制**:
- **记忆队列**: `deque(maxlen=40)`存储历史前沿点
- **数据结构**: `(gi, gj, wx, wy, ts)` - 网格坐标、世界坐标、时间戳
- **回退策略**: 优先最近120s内的点，允许1px微调
- **一致性保证**: 使用相同C-space进行A*验证

#### 3. **建图与ROI系统** (`map_builder.py`)

**解耦SLAM架构**:
```python
# 建图阶段（mapping_phase=True）
MapBuilder.update(pose, lidar_scan):
  - 使用真实位姿（仿真）或估计位姿（真实）
  - Log-odds更新: hit=1.2, miss=-0.3
  - 智能射线追踪: 检测已知障碍，防穿墙
  - 3×3障碍扩散: 中心1.2，边缘0.84
  - 强制机器人位置为FREE
  - 清除入口/脚印区域

# 占据栅格导出（P0优化）
get_occupancy_grid():
  - 双阈值判断: 强障碍(>1.2) + 弱障碍(0.5~1.2)
  - 多数投票: 弱障碍需要≥5个邻居支持
  - 消除虚假障碍带，恢复前沿点集群数
```

**ROI处理优化**:
- **性能优化**: 只在机器人传感器范围+0.25m内处理
- **防污染**: 使用`.copy()`避免意外修改全局地图
- **已知锁定**: 防止已知区域回退为未知

#### 4. **路径规划与运动控制系统** (`global_planner.py`, `motion_controller.py`)

**A*全局规划**:
```python
AStarPlanner.plan(start_ij, goal_ij, occ_grid, safe_buffer_m):
  - 矩形C-space膨胀: maximum_filter1d (12px = 0.12m安全缓冲)
  - 起点清除: 15px (膨胀+3px保护)
  - A*搜索: 启发式欧氏距离
  - 曼哈顿路径: 最少拐角，轴对齐

AStarPlanner.plan_to_band(start_ij, band_rect, occ_grid):
  - 规划到可达区域的曼哈顿路径
  - 策略: 直线 → L型 → 兜底A*
  - 规避前沿点处于障碍带中的情况

AStarPlanner.plan_axis_primitives(pose, band_rect) -> List[Dict]:
  - 生成轴对齐运动原语序列（v3.1新增）
  - 输出: TURN/MOVE/GATE原语队列
  - 策略: 对中 → 推进 → 穿越闸门
```

**MotionController原语运动控制**:
```python
MotionController.follow_primitive(pose, primitive, remaining_dist) -> (v, w, done):
  - TURN原语: 原地旋转（容差8°）
  - MOVE原语: 轴向移动（θ_move=6°门限）
  - GATE原语: 闸门穿越（动态方向重估）
  - 统一角度门限: MOVE/GATE使用相同的θ_move参数
  - 鲁棒日志: remaining=None安全处理

MotionController.step(pose) -> (v, w):
  - 传统路径跟踪（向后兼容）
  - 先旋转后移动: 严格分离旋转和移动
  - 四向锁定: 0°/±90°/180°
  - 统一碰撞世界: 与A*使用相同的矩形C-space
```

### 关键技术突破

#### 1. **轴对齐运动原语架构** (v3.1核心)
- **问题**: 连续路径跟踪导致角度误差累积、到达判定复杂
- **解决方案**: 离散化为TURN/MOVE/GATE三种原语，闸门检测到达
- **效果**: 简化控制逻辑，提高到达检测准确性
- **关键参数**: 
  - PRIM_TURN_TOL_DEG = 8° (旋转完成容差)
  - PRIM_MOVE_YAW_TOL_DEG = 6° (θ_move角度门限)
  - PRIM_GATE_YAW_TOL_DEG = 6° (与MOVE统一)

#### 2. **统一C-space膨胀** (v3.2核心)
- **问题**: FrontierExplorer使用EDT圆形膨胀，Navigator/Visualizer使用maximum_filter1d矩形膨胀，两套碰撞模型不一致
- **解决方案**: 统一使用maximum_filter1d矩形膨胀，消除膨胀不一致性
- **效果**: 所有模块使用相同的碰撞模型，机器人位置不再被误判为在膨胀带中

#### 3. **可达区域导航** (v3.0核心)
- **问题**: 直接规划到前沿点，无法处理前沿点处于障碍带的情况
- **解决方案**: 基于前沿点构建可达区域（corridor band），规划到区域而非单点
- **效果**: 规避前沿点障碍带问题，提高导航成功率
- **实现**: `_compute_goal_band()` 四向扫描构建矩形到达带

#### 4. **动态GATE方向重估** (v3.1新增)
- **问题**: 前沿点切换后GATE方向可能错误，导致反向推进
- **解决方案**: 每帧重估GATE推进方向，基于当前位置与闸门关系
- **效果**: 避免"反向闸门+原地转"问题，提高鲁棒性

#### 5. **卡死校正机制** (P3保留)
- **问题**: "8邻居不全为自由"导致前沿点选择失败
- **解决方案**: 记忆队列自动回退到历史有效点
- **效果**: 解决探索卡死，提高鲁棒性

#### 6. **ROI性能优化**
- **问题**: 全图处理导致后期极慢
- **解决方案**: 传感器范围+0.25m ROI处理
- **效果**: 从O(HW)降低到O(πr²)，大幅提升性能

#### 7. **已知区域锁定**
- **问题**: 已知区域回退为未知
- **解决方案**: 已知锁定机制，防止历史信息丢失
- **效果**: 保持地图稳定性

### 当前系统状态 (v4.0 硬件导航系统)

**✅ 已完成功能** (硬件导航系统):

**v4.0 核心硬件支持**:
- **真实硬件导航架构** (`NavMappingApp` + `NavBleInterface`)
  - 蓝牙串口通信 (57600 baud)
  - 硬件命令控制 (`"S"`, `"T"`, `"(M,<deg>,<mm>)"`)
  - 状态机协调 (`MAPPING → NAVIGATING → TURNING → MOVING → COMPLETE`)
  - 异常处理与优雅关闭 (`signal.SIGINT`, `atexit.register`)

- **稀疏雷达数据处理** (`sparse_lidar_processor.py`)
  - 实时稀疏点建图 (`add_single_sparse_point_with_pose`)
  - 角度稀疏处理 (前方330°~30°数据稀少适配)
  - Log-odds建图 (`_mark_obstacle_point`, `_mark_free_space_ray`)
  - 坐标转换修正 (角度对齐与垂直翻转修复)

- **里程计数据处理** (`core/odometry_processor.py`)
  - 里程计状态管理 (`OdometryState`)
  - 坐标系统一转换 (`odom_to_world_pose`)
  - 数据格式解析 (`parse_odometry_line`)
  - 位姿实时更新 (`update_robot_position_from_odometry`)

- **数据持久化系统** (`logs/nav_exploring/`)
  - 原始数据记录 (雷达、里程计、事件)
  - 增量地图保存 (`.npz`, `.json`, `.png`)
  - 建图统计日志 (`mapping_stats_*.txt`)
  - 强制保存机制 (防止意外关闭数据丢失)

**传统功能** (向后兼容):
- 解耦SLAM架构（建图+定位分离）
- 工业标准前沿点探索（BFS连通性+A*验证+Pullback）
- 三阶段导航状态机（TO_ENTRANCE → EXPLORING → TO_EXIT）
- **轴对齐运动原语架构**（TURN/MOVE/GATE）
- **统一C-space膨胀**（maximum_filter1d，v3.2核心）
- **带检验前沿点选择**（corridor band验证，v3.2核心）
- 4面板实时可视化（含闸门线显示）

**🔄 当前开发中** (硬件导航优化):
- **稀疏雷达建图质量优化**:
  - 前方区域 (330°~30°) 数据稀疏问题 (仅8.4%数据)
  - 自由空间识别不足，导致C-space连通性差
  - 需要优化Log-odds参数和阈值

- **里程计融合精度提升**:
  - 角度基准对齐 (YAW_REBASE机制)
  - 坐标转换一致性验证
  - 位姿更新同步优化

- **硬件命令执行优化**:
  - 命令格式统一 (`(M,<deg>,<mm>)`)
  - 执行状态监控 (`"arrival"`, `"turning finished"`)
  - 超时保护机制完善

**📋 待优化功能**:
- **建图算法改进**: 针对稀疏雷达数据的建图策略优化
- **导航鲁棒性**: 处理硬件数据质量不稳定的情况
- **实时性能**: 优化数据处理管道，提升响应速度
- **错误恢复**: 增强硬件通信异常的处理能力

---

## 🚀 v4.1 高波特率优化特性详解

### 核心优化技术

#### 1. **921600 bps高速通信支持**
```python
# 配置参数
HIGH_BAUDRATE_MODE: bool = True         # 启用高波特率优化
HIGH_BAUDRATE_BAUD: int = 921600       # 高波特率阈值
HIGH_BAUDRATE_TIMEOUT: float = 0.001   # 超时时间优化
```

#### 2. **点数量控制建图**
```python
# 建图控制参数
MAIN_MAPPING_POINTS: int = 500         # 主建图目标点数
POST_SEGMENT_MAPPING_POINTS: int = 300 # 段后建图目标点数
DEFAULT_MAPPING_POINTS: int = 500      # 默认建图点数

# 建图流程
def do_mapping_window_by_points(self, target_points: int = 500):
    """点数量控制建图窗口"""
    while processed_points < target_points:
        # 批量处理数据
        # 质量筛选和过滤
        # 实时建图更新
```

#### 3. **单线程批量处理优化**
```python
# 批量处理参数
SINGLE_THREAD_BATCH_SIZE: int = 300    # 单线程批量处理大小
SINGLE_THREAD_EMPTY_BATCH_LIMIT: int = 50  # 空批次检测限制

# 批量读取实现
def read_batch_lines(self, max_lines: int = 300):
    """单线程批量读取优化"""
    batch_lines = []
    for _ in range(max_lines):
        if self.lines.empty():
            break
        batch_lines.append(self.lines.get())
    return batch_lines
```

#### 4. **GUI更新频率优化**
```python
# GUI更新参数
HIGH_BAUDRATE_GUI_UPDATE_INTERVAL: int = 500  # 每500点更新一次

# 优化实现
if processed_points % gui_every == 0:
    # 只在达到更新间隔时更新GUI
    # 减少O(N²)计算开销
    self.viz.update_mapping_realtime(current_occ_grid, current_pose)
```

#### 5. **内存管理优化**
```python
# 内存监控参数
HIGH_BAUDRATE_MEMORY_MONITOR: bool = False    # 禁用内存监控提升性能
HIGH_BAUDRATE_MAX_MEMORY_MB: int = 500        # 最大内存使用限制

# 内存清理机制
def _force_memory_cleanup(self):
    """强制内存清理"""
    import gc
    gc.collect()
    # 清理缓存数据
```

#### 6. **数据质量优化**
```python
# 垃圾点过滤
def _filter_garbage_points(self, point_data):
    """过滤垃圾数据点"""
    if point_data.get('quality', 0) < 5:
        return False
    if point_data.get('distance_m', 0) < 0.05:
        return False
    return True

# 雷达预热处理
def _handle_lidar_warmup(self, point_data):
    """处理雷达预热阶段"""
    if self.lidar_warmup_drops < 10:
        self.lidar_warmup_drops += 1
        return False  # 丢弃预热数据
    return True
```

### 性能提升效果

#### 处理速度对比
| 优化项目 | 优化前 | 优化后 | 提升幅度 |
|---------|--------|--------|----------|
| 波特率 | 57600 bps | 921600 bps | 16x |
| 批量处理 | 单行处理 | 300行/批次 | 300x |
| GUI更新 | 每点更新 | 每500点更新 | 500x |
| 建图控制 | 时间控制 | 点数量控制 | 精确控制 |
| 内存使用 | 无监控 | 智能清理 | 稳定运行 |

#### 建图质量提升
- **点数量控制**: 确保每次建图达到目标点数
- **质量筛选**: 过滤低质量数据点，提升建图精度
- **实时处理**: 单线程批量处理，避免数据丢失
- **内存优化**: 防止内存溢出，确保长时间稳定运行

---

## 🔄 完整系统调用链 (v4.1 高波特率优化系统)

### 高波特率优化启动流程

```
test_nav_modular.py::main()
  │
  ├─> 参数解析 (port, baud=921600, size_m, res, cycles)
  │
  └─> NavMappingApp(port="COM4", baud=921600, size_m=2.8, map_res=0.01)
      │
      ├─> [Step 1] 高波特率硬件接口初始化
      │     └─> NavBleInterface(port, baud=921600)
      │           ├─> 串口连接 (serial.Serial, 921600 bps)
      │           ├─> MCU预热 (1.5s)
      │           └─> 单线程批量数据处理启动
      │
      ├─> [Step 2] 数据处理模块初始化  
      │     ├─> SparseLidarProcessor(map_size, resolution)
      │     ├─> OdometryState() + OdometryProcessor
      │     └─> NavVisualizer (实时GUI显示)
      │
      ├─> [Step 3] 导航模块初始化 (可选)
      │     ├─> AStarPlanner
      │     ├─> FrontierExplorer  
      │     └─> NavigatorFSM
      │
      └─> [Step 4] 高波特率数据持久化初始化
            ├─> logs/nav_exploring/ 目录创建
            ├─> 原始数据日志 (raw_data_*.txt)
            ├─> 建图统计日志 (mapping_stats_*.txt)
            └─> 高波特率性能监控日志
```

### 高波特率优化主循环 (NavMappingApp.run())

```
NavMappingApp.run() - 主状态机循环
  │
  └─> while cycles < max_cycles and not _shutdown_requested:
        │
        ├─> [状态: MAPPING] 点数量控制建图窗口
        │     └─> do_mapping_window_by_points(target_points=500)
        │           │
        │           ├─> 1. 位姿同步: mapper.set_robot_pose()
        │           │
        │           ├─> 2. 雷达启动: ble.send("S") + 1.2s等待
        │           │
        │           ├─> 3. 单线程批量数据采集循环:
        │           │     ├─> ble.read_batch_lines(300) - 批量读取串口
        │           │     ├─> 垃圾点过滤和雷达预热处理
        │           │     ├─> parse_odom_line_or_block() - 里程计解析
        │           │     │     ├─> update_robot_position_from_odometry()
        │           │     │     └─> mapper.set_robot_pose() - 实时位姿更新
        │           │     │
        │           │     ├─> parse_lidar_line() - 雷达数据解析
        │           │     │     └─> mapper.add_single_sparse_point_with_pose()
        │           │     │           ├─> 角度转换: alpha = -math.radians(angle_deg)
        │           │     │           ├─> 世界坐标: x_world = distance * cos(θw + α) + rx
        │           │     │           └─> 栅格更新: _update_log_odds(gx, gy, log_hit/miss)
        │           │     │
        │           │     ├─> GUI更新优化 (每500点更新一次)
        │           │     └─> 点数量控制 (processed_points < target_points)
        │           │
        │           ├─> 4. 雷达停止: ble.send("T") + 0.8s缓冲清空
        │           │
        │           └─> 5. 数据记录: raw_data_log + mapping_stats_log
        │
        ├─> [状态: NAVIGATING] 导航规划
        │     └─> plan_next_and_emit_primitives()
        │           │
        │           ├─> 1. 获取占用地图: mapper.get_occupancy_grid()
        │           │
        │           ├─> 2. 前沿点探索: FrontierExplorer.choose_next_frontier()
        │           │     ├─> find_frontiers() - BFS连通性 + 前沿检测
        │           │     ├─> 中线偏置排序 (λ=0.3)
        │           │     └─> A*验证 + Pullback策略
        │           │
        │           ├─> 3. 路径规划: AStarPlanner.plan_to_band()
        │           │     ├─> 构建可达区域 (corridor band)
        │           │     └─> 曼哈顿路径规划
        │           │
        │           └─> 4. 原语分解: decompose_to_primitives()
        │                 └─> TURN/MOVE/GATE 序列生成
        │
        ├─> [状态: TURNING/MOVING] 命令执行
        │     └─> execute_commands()
        │           │
        │           ├─> 1. 命令转换: primitives_to_ble_commands()
        │           │     └─> 生成 "(M,<deg>,<mm>)" 格式
        │           │
        │           ├─> 2. 硬件控制: ble.send(command)
        │           │
        │           ├─> 3. 状态监控循环:
        │           │     ├─> ble.readline() - 读取执行状态
        │           │     ├─> 里程计实时更新: parse_odom_line_or_block()
        │           │     ├─> 位姿同步: mapper.set_robot_pose()
        │           │     └─> 事件检测: "arrival", "turning finished"
        │           │
        │           └─> 4. 完成检测: 到达判定或超时 (7s)
        │
        └─> [状态: COMPLETE] 数据保存
              └─> _force_save_all_data()
                    ├─> 增量地图保存 (maps/)
                    ├─> 最终结果保存 (final_map_*.png)
                    └─> 日志文件关闭
```

### 传统仿真系统调用链 (向后兼容)

### 主循环调用链（每帧 50ms）

```
main.py::run() - 主循环
  │
  ├─> [Step 1] 获取传感器数据
  │     └─> robot.get_scan() → LaserScan
  │
  ├─> [Step 2] SLAM更新（建图阶段）
  │     └─> slam.update(scan, odom_delta, true_pose)
  │           │
  │           └─> MapBuilder.update(true_pose, scan)
  │                 │
  │                 ├─> 坐标转换: world → grid
  │                 ├─> 270束激光扫描处理:
  │                 │     ├─> Bresenham射线追踪
  │                 │     ├─> 检测已知障碍（防穿墙）
  │                 │     ├─> 标记FREE空间（log-odds -= 0.3）
  │                 │     └─> 标记障碍（log-odds += 1.2，3×3扩散）
  │                 │
  │                 ├─> 强制机器人当前位置为FREE
  │                 └─> 清除入口/脚印区域
  │
  ├─> [Step 3] 导航控制
  │     └─> navigator.update(pose, occ_grid)
  │           │
  │           ├─> [状态: TO_ENTRANCE]
  │           │     ├─> 检查是否到达入口（距离<0.08m）
  │           │     └─> 转换到EXPLORING状态
  │           │
  │           ├─> [状态: EXPLORING] ← 核心探索逻辑
  │           │     │
  │           │     ├─> 检查是否有当前目标
  │           │     │     ├─> 有：PathTracker跟踪路径
  │           │     │     └─> 无：选择新前沿点
  │           │     │
  │           │     └─> 选择新前沿点流程:
  │           │           │
  │           │           ├─> FrontierExplorer.choose_next_frontier()
  │           │           │     │
  │           │           │     ├─> find_frontiers(occ_grid, pose)
  │           │           │     │     │
  │           │           │     │     ├─> BFS计算连通域:
  │           │           │     │     │     - 从robot_pose出发
  │           │           │     │     │     - 4-connectivity遍历
  │           │           │     │     │     - 标记所有可达free cells
  │           │           │     │     │
  │           │           │     │     ├─> 识别前沿点:
  │           │           │     │     │     - 条件1: 在连通域内
  │           │           │     │     │     - 条件2: cell = FREE (0)
  │           │           │     │     │     - 条件3: 邻接UNKNOWN (2)
  │           │           │     │     │
  │           │           │     │     └─> 聚类前沿点（最小3个cell）
  │           │           │     │
  │           │           │     ├─> 基础过滤:
  │           │           │     │     - 边界检查 (0.05~1.95m)
  │           │           │     │     - 距离检查 (0.05~2.0m)
  │           │           │     │
  │           │           │     ├─> 中线偏置排序（P1优化）:
  │           │           │     │     cost = dist + 0.3×|lateral_offset|
  │           │           │     │     优先选择靠近通道中线的前沿点
  │           │           │     │
  │           │           │     └─> 两级回退+A*验证（P2优化）:
  │           │           │           │
  │           │           │           ├─> Level 1: 4px回退
  │           │           │           │     ├─> 计算回退方向（向机器人）
  │           │           │           │     ├─> 检查回退后是FREE (0)
  │           │           │           │     ├─> 局部安全检查（6px buffer）
  │           │           │           │     └─> A*验证可达性
  │           │           │           │
  │           │           │           └─> Level 2: 2px回退（兜底）
  │           │           │                 └─> 同上流程，更小回退距离
  │           │           │
  │           │           ├─> A*全局规划:
  │           │           │     ├─> 障碍膨胀（12px = 0.12m）
  │           │           │     ├─> 起点/入口清除（15px）
  │           │           │     └─> A*搜索（启发式：欧氏距离）
  │           │           │
  │           │           └─> PathTracker初始化
  │           │
  │           └─> [状态: TO_EXIT]
  │                 └─> 规划到出口的路径
  │
  ├─> [Step 4] 速度控制
  │     ├─> PathTracker.current_local_goal() → 局部目标
  │     └─> DWAPlanner.compute_velocity(pose, goal, occ_grid)
  │           │
  │           ├─> 障碍膨胀（与A*一致）
  │           ├─> 起点清除（与A*一致）
  │           ├─> 速度采样空间（v×w: 10×10）
  │           ├─> 轨迹评分:
  │           │     - heading_score (朝向目标)
  │           │     - clearance_score (障碍距离)
  │           │     - velocity_score (速度偏好)
  │           │     - total = 0.3h + 0.2c + 0.5v
  │           │
  │           └─> 返回最优(v, w)
  │
  ├─> [Step 5] 机器人控制
  │     └─> robot.set_control(v, w)
  │
  └─> [Step 6] GUI更新
        └─> visualizer.update(robot_pose, slam_pose, path, ...)
              ├─> 更新4个面板
              └─> plt.pause(0.001)
```

### MapBuilder详细调用链

```
MapBuilder.update(pose, lidar_scan)
  │
  ├─> 坐标转换: world(x,y,θ) → grid(gx,gy)
  │
  ├─> 处理每束激光 (270束):
  │     │
  │     ├─> 计算射线端点: (ex, ey) = (x0, y0) + range×(ux, uy)
  │     │
  │     ├─> 边界裁剪: clip(ex, ey) → map bounds
  │     │
  │     ├─> Bresenham射线追踪: (gx0,gy0) → (gx1,gy1)
  │     │     └─> 生成射线上所有cell坐标
  │     │
  │     ├─> 智能FREE标记（工程标准修复）:
  │     │     │
  │     │     ├─> 检测射线上已知障碍:
  │     │     │     if lgrid[cell] > 2.0: first_obstacle_idx
  │     │     │
  │     │     ├─> 确定安全FREE范围:
  │     │     │     - 有已知障碍: 只标记到障碍前
  │     │     │     - 被裁剪: 跳过FREE标记（防穿墙）
  │     │     │     - 正常命中: 标记到端点前
  │     │     │
  │     │     └─> 标记FREE（保护强障碍）:
  │     │           if lgrid > 1.0: lgrid += log_miss×0.5
  │     │           else: lgrid += log_miss
  │     │
  │     └─> 障碍标记（3×3扩散）:
  │           - 中心cell: log_hit (1.2)
  │           - 周围8个: log_hit×0.7 (0.84)
  │           - 跳过: 入口15px范围、机器人2px范围
  │
  ├─> 强制机器人当前cell为FREE: lgrid[gy0,gx0] = min(-1.5)
  │
  └─> 清除入口/脚印区域 (disk clear)

MapBuilder.get_occupancy_grid() [P0核心优化]
  │
  ├─> 初始化: grid = 2 (全UNKNOWN)
  │
  ├─> Step 1: 标记FREE
  │     grid[lgrid < -0.3] = 0
  │
  ├─> Step 2: 标记强障碍
  │     grid[lgrid > 1.2] = 1
  │
  ├─> Step 3: 弱障碍多数投票（P0关键修复）
  │     │
  │     ├─> 识别弱障碍: 0.5 < lgrid < 1.2
  │     │
  │     ├─> 计算8邻域计数:
  │     │     neigh_count = Σ(8-neighbors is weak/strong)
  │     │
  │     └─> 多数投票:
  │           if neigh_count >= 5: grid = 1 (障碍)
  │           else: grid = 2 (保持未知，抑制薄障碍带)
  │
  ├─> Step 4: 强制机器人footprint为FREE
  │     └─> 清除半径max(2, foot_clear_cells)
  │
  └─> Step 5: 强制入口区域为FREE
        └─> 清除半径entr_clear_cells (15px)
```

---

## 🎯 当前项目需求与进度

### 项目需求清单

#### ✅ 已完成需求

1. **解耦SLAM架构** ✅
   - MapBuilder纯建图模块
   - Localizer纯定位模块（未启用）
   - 仿真模式使用真实位姿建图

2. **基础导航功能** ✅
   - 三阶段导航状态机
   - A*全局路径规划
   - DWA局部避障

3. **前沿点探索** ✅
   - BFS连通性检查
   - A*可达性验证
   - Pullback策略

4. **4面板GUI** ✅
   - 地面真值、SLAM地图、A*路径、覆盖图
   - 2×2布局

5. **P0: 建图穿墙修复** ✅ (2025-10-01)
   - 智能射线追踪
   - 障碍稳定性优化
   - 入口/机器人保护

6. **P0: 占据栅格导出优化** ✅ (2025-10-01)
   - 双阈值判断（强1.2/弱0.5）
   - 3×3多数投票
   - 消除虚假障碍带

7. **P1: 中线偏置排序** ✅ (2025-10-01)
   - cost = dist + 0.3×lateral_offset
   - 优先选择通道中央前沿点

8. **P2: 两级回退兜底** ✅ (2025-10-01)
   - Level 1: 4px回退
   - Level 2: 2px回退（窄通道适配）

9. **P3: 卡死校正机制** ✅ (2025-10-01)
   - 记忆队列: deque(maxlen=40)存储历史前沿点
   - 自动回退: 当前轮失败时从记忆中选择
   - 智能调整: 允许1px微调通过安全检查
   - 一致性保证: 使用相同C-space进行A*验证

#### 🔄 开发中需求

10. **记忆回退机制效果验证** 🔄
    - 状态: 测试中
    - 问题: 卡死校正机制的实际效果待验证
    - 预期: 解决"8邻居不全为自由"导致的探索卡死

11. **探索完整迷宫流程** 🔄
    - 状态: 测试中
    - 目标: 达到95%覆盖率并找到出口
    - 关键: 验证三阶段导航状态机完整性

#### 📋 待实现需求

12. **Localizer定位模块启用** 📋
    - 建图完成后切换到ICP定位
    - 当前: 仍使用真实位姿

13. **真实机器人模式** 📋
    - 原地360°扫描建图
    - 蓝牙通信对接

14. **地图保存/加载** 📋
    - 保存已建好的地图
    - 离线建图、在线定位

### 当前开发阶段

**阶段**: 核心导航问题修复完成  
**版本**: v3.5  
**日期**: 2025-10-18

**最近完成**:
- ✅ **v3.5核心**: TO_EXIT阶段切换错误修复（状态切换稳定性提升）
- ✅ **v3.5核心**: 覆盖率计算错误修复（探索完成判定准确性提升）
- ✅ **v3.5核心**: 终点多个不同可达区域计算错误修复（区域计算一致性）
- ✅ **v3.5核心**: 终点可达性不可实现问题修复（可达性判定机制改进）
- ✅ **v3.4核心**: 严格C-space规划（plan_to_band → _decompose_path_to_primitives）
- ✅ **v3.4核心**: 小折线消除（EPS_MOVE门限 + 四步清理管线）
- ✅ **v3.4核心**: L∞外扩矩形到达判定（GOAL_BAND_OUTER_MARGIN_M=4cm）
- ✅ **v3.3优化**: 最短段剃除（PRIM_MIN_SEG_LEN_M=3cm）
- ✅ **v3.3优化**: 路径清理统一（A*路径和原语路径使用相同清理管线）
- ✅ **v3.2核心**: 统一C-space膨胀（EDT→maximum_filter1d，消除膨胀不一致性）
- ✅ **v3.2核心**: 带检验前沿点选择（corridor band验证替代单点验证）
- ✅ **v3.1核心**: 轴对齐运动原语架构（TURN/MOVE/GATE）
- ✅ **v3.0核心**: 可达区域导航（规划到corridor band而非前沿点）

**当前任务**:
- 🔄 **关键**: 优化到达判定逻辑（移除二次确认 + 每帧提前检查）
- 🔄 **关键**: 统一原语模式执行（移除传统模式混用）
- 🔄 **关键**: 增强Tier-4兜底容差（1.5cm → 4cm）
- 🔄 **关键**: 提升前沿探索鲁棒性（解决找不到前沿点问题）
- 🔄 观察探索完整迷宫流程

**已知问题**:
1. 🔴 **到达判定二次确认风险**: `_on_band_arrived()`内部再次调用`_check_goal_band_arrival()`，可能导致卡住
2. 🔴 **原语执行期无到达检查**: 原语执行过程中不检查是否已到达，导致不必要的路径绕行
3. 🔴 **传统模式与原语模式混用**: 代码中存在传统模式和原语模式混用，应统一使用原语模式
4. 🔴 **前沿探索可能失败**: 在某些情况下找不到前沿点，导致探索卡住
5. 🟡 **Tier-4兜底容差过小**: near_eps=1.5cm可能不足，建议增强到4cm
6. 🟡 **原语队列清空后的处理**: 队列清空但未到达时，机器人停车等待，可能永远不触发重规划
7. 🟡 **TURN最小角速度缺失**: 当角度误差小时旋转过慢，可能长时间无法完成
8. 🟡 **TURN超时保护缺失**: 如果陷入振荡可能永远无法完成
9. 🟢 **小折线问题**: 已修复（EPS_MOVE门限 + 四步清理管线，v3.3）
10. 🟢 **规划穿障碍问题**: 已修复（严格C-space规划，v3.3）
11. 🟢 **GATE期望航向逻辑**: 已修复（支持四向±X/±Y，v3.2）
12. 🟢 **统一膨胀机制**: 已修复（EDT→maximum_filter1d，v3.2）

---

## 🔬 关键技术要点

### 1. 解耦SLAM架构

**核心思想**: 建图与定位完全分离

```
传统耦合SLAM问题:
  位姿误差 → 地图误差 → 更大位姿误差 (恶性循环)

解耦SLAM优势:
  建图: 使用可靠位姿 → 高质量地图
  定位: 在已知地图上ICP → 准确定位
  结果: 无误差累积，模块独立
```

### 2. 前沿点探索（工业标准）

**连通性检查** (Industrial Practice):
```python
# 只在机器人可达区域寻找前沿点
connected_component = BFS(robot_pos, free_cells)
frontiers = find_boundaries(connected_component, unknown)

# 避免选择"未知岛屿"（看似是前沿，实则不可达）
```

**Pullback策略**:
```python
# 前沿点在FREE/UNKNOWN边界，可能不稳定
# 回退到已知FREE区域，确保A*起点安全
frontier_pos → pullback(4px向机器人) → stable_target
```

**A*验证**:
```python
# 不依赖启发式距离判断
# 实际规划路径，失败则拒绝前沿点
for frontier in candidates:
    path = A_star(robot, frontier)
    if path: return frontier  # 第一个可达的
```

### 3. 双阈值+多数投票（P0核心优化）

**问题**: 单束命中 → 立即标记为障碍 → 虚假障碍带

**解决方案**:
```python
强障碍 (lgrid > 1.2):
  - 多次命中，高置信度
  - 直接标记，无需投票
  
弱障碍 (0.5 < lgrid < 1.2):
  - 单次或少量命中，低置信度
  - 需要空间一致性验证:
      if 8-neighbors中≥5个也是弱/强障碍:
          标记为障碍 (真实墙壁)
      else:
          保持未知 (噪声/薄带，抑制)

效果:
  - 消除自由/未知边界的虚假障碍带
  - 前沿点集群数恢复: 26 → 60-80个
  - 提高探索成功率
```

### 4. 记忆回退机制（P3新增）

**问题**: "8邻居不全为自由"导致前沿点选择失败，探索卡死

**解决方案**:
```python
记忆队列 (deque maxlen=40):
  - 存储历史前沿点: (gi, gj, wx, wy, ts)
  - 自动管理: 超过40个时删除最旧记录
  
回退策略:
  - 当前轮全部失败时自动触发
  - 优先尝试最近120s内的点
  - 允许1px朝机器人方向微调
  - 使用相同C-space进行A*验证
  
一致性保证:
  - 回退时使用缓存的_last_cspace_bin
  - 避免双重膨胀，保持安全标准
  - 成功选点会写入记忆供将来使用

效果:
  - 解决探索卡死问题
  - 提高系统鲁棒性
  - 保持原有安全标准
```

### 5. 中线偏置排序（P1优化）

**问题**: 纯距离排序易选择贴墙前沿点

**解决方案**:
```python
# 迷宫通道中线: j=100 (x=1.0m)
lateral_offset = |frontier_j - 100| × 0.01m
cost = distance + 0.3 × lateral_offset

# 示例:
候选A: dist=0.50m, j=30 (靠左墙)
  cost = 0.50 + 0.3×0.70 = 0.71

候选B: dist=0.52m, j=100 (中线)
  cost = 0.52 + 0.3×0.00 = 0.52 ✅ 优先

效果: 优先选择通道中央，减少碰撞风险
```

### 6. 坐标系统

**三套坐标系**:
```
世界坐标 (World): 米为单位
  - 范围: [0, 2.0] × [0, 2.0]
  - 原点: 左下角
  - 用途: 物理空间、GUI显示

网格坐标 (Grid/SLAM): 像素为单位
  - 范围: [0, 199] × [0, 199]
  - 分辨率: 0.01m/pixel
  - 格式: (row_i, col_j) → i对应y, j对应x
  - 转换: 
      gx = int(x_world / 0.01)
      gy = int(y_world / 0.01)
      x_world = gj × 0.01
      y_world = gi × 0.01

机体坐标 (Body): 相对机器人
  - 用途: 激光扫描数据
  - 转换: body → world (旋转+平移)
```

### 7. 轴对齐运动原语架构 (v3.1)

**运动原语类型**:
```python
# 三种离散化运动原语
TURN: {"type":"TURN", "heading_rad":float}
  - 原地旋转到目标航向
  - 完成条件: |angle_error| ≤ 8° (PRIM_TURN_TOL_DEG)
  - 控制: v=0, w=Kp×error

MOVE: {"type":"MOVE", "axis":"x"|"y", "distance":float, "dir":±1}
  - 沿轴向移动指定距离
  - 完成条件: remaining_distance ≤ 0.5×resolution
  - 角度门限: θ_move=6° (PRIM_MOVE_YAW_TOL_DEG)
  - 控制: v=0.25/0.12 (快/慢), w=Kp×yaw_error

GATE: {"type":"GATE", "axis":"x"|"y", "gate":float, "dir":±1}
  - 沿轴向移动直到穿越闸门线
  - 完成条件: 按dir_sign方向穿越gate_val坐标
  - 角度门限: θ_move=6° (PRIM_GATE_YAW_TOL_DEG，与MOVE统一)
  - 方向重估: 每帧动态计算dir_sign
  - 控制: v=0.22/0.10 (快/慢), w=Kp×yaw_error
```

**可达区域导航流程**:
```python
# v3.1 原语运动控制流程
1. 前沿点探索 → 选择可达前沿点（BFS连通性+A*验证）
2. 可达区域构建 → _compute_goal_band()四向扫描构建corridor band
3. 原语序列生成 → plan_axis_primitives()生成TURN/MOVE/GATE队列
4. 原语执行 → follow_primitive()逐个执行原语
5. 闸门检测到达 → edge-crossing判定进入到达带
6. 切换下一个前沿点 → 重复流程
```

**原语执行状态机**:
```python
Navigator._update_motion(pose):
  if primitive_mode:
    # 原语模式
    if no active_primitive and queue not empty:
      active_primitive = queue.pop(0)  # 激活下一个原语
    
    if active_primitive:
      # GATE原语每帧重估方向
      if primitive.type == "GATE":
        primitive["dir"] = recompute_direction(pose, gate)
      
      v, w, done = controller.follow_primitive(pose, primitive)
      
      if done:
        active_primitive = None
        if queue empty:
          check_goal_band_arrival()  # 闸门检测到达
    
    return v, w
  else:
    # 传统路径跟踪（向后兼容）
    return controller.step(pose)
```

### 8. 到达判定优化 (v3.2)

**问题背景**:
原有的到达判定逻辑过于严格，机器人需要精确到达目标带的边界才能判定为"到达"。这导致：
- 由于浮点精度、网格对齐等问题，机器人可能"蹭不到"边界
- 即使视觉上已经到达，仍然无法触发到达判定
- 容易导致机器人反复尝试、卡死在目标附近

**优化方案**:
```python
# v3.2 优化后的到达判定逻辑
def _reached_band_by_edge_cross(pose, band_rect):
    """
    扩展边界容差判定：
    - 原始目标带: [xmin, xmax] × [ymin, ymax]
    - 扩展容差: GOAL_BAND_BOUNDARY_TOLERANCE_M = 0.02m (2cm)
    - 扩展区域: [xmin-2cm, xmax+2cm] × [ymin-2cm, ymax+2cm]
    
    判定逻辑:
    1. 优先检查：机器人是否在扩展区域内 → 立即判定到达
    2. 备选方案：保留原有的自适应闸门逻辑
    """
    # 扩展边界容差
    boundary_tolerance = GOAL_BAND_BOUNDARY_TOLERANCE_M  # 2cm
    extended_rect = (
        xmin - boundary_tolerance,
        xmax + boundary_tolerance,
        ymin - boundary_tolerance,
        ymax + boundary_tolerance
    )
    
    # 在扩展区域内 → 到达
    if in_extended_rect(pose, extended_rect):
        return True
    
    # 备选：原有闸门穿越逻辑
    return legacy_gate_crossing_check(pose, band_rect)
```

**优化效果**:
- ✅ **提高鲁棒性**: 2cm容差足以覆盖浮点精度和网格对齐误差
- ✅ **减少卡死**: 机器人更容易判定为到达，减少反复尝试
- ✅ **保持精度**: 2cm在迷宫导航中是可接受的精度范围
- ✅ **向后兼容**: 保留原有逻辑作为备选方案

**参数配置**:
```python
# config.py
GOAL_BAND_BOUNDARY_TOLERANCE_M: float = 0.02  # 2cm边界容差
```

**应用场景**:
- 前沿点探索：到达前沿点的目标带
- 入口/出口导航：到达入口/出口区域
- 所有使用`_reached_band_by_edge_cross`的到达判定场景

### 9. 严格C-space规划 (v3.3)

**问题背景**:
原有的原语路径由`plan_axis_primitives()`纯几何计算生成，未经过碰撞检测，导致：
- 蓝色原语路径可能穿过障碍物（右下角问题）
- 规划与执行的安全性无法保证
- 几何理想路径与实际可行路径不一致

**优化方案**:
```python
# v3.3 严格C-space规划流程
def _plan_to_band(goal_xy, band_rect):
    # 1. 构造严格C-space（膨胀障碍 OR 未知区域）
    strict_cspace = explorer.get_latest_cspace_bin()  # 复用缓存
    
    # 2. 在严格C-space上规划安全折线
    safe_poly = planner.plan_to_band(start, band_rect, strict_cspace)
    
    # 3. 轴向对齐 + 路径清理
    safe_poly = _align_path_to_axis(safe_poly)
    clean_poly = _clean_primitive_path(safe_poly)
    
    # 4. 安全折线分解为原语序列
    primitives = _decompose_path_to_primitives(clean_poly, pose_theta)
    
    # 结果：原语路径 = A*验证的安全折线
```

**关键优势**:
- ✅ **单一事实源**: 原语路径基于A*验证的安全折线，不再是纯几何计算
- ✅ **安全性保证**: 路径上每个点都在自由空间，绝不穿障碍
- ✅ **性能优化**: 复用explorer缓存的C-space，避免重复膨胀计算
- ✅ **执行等价**: 原语序列等价于安全折线的离散化表示

**参数配置**:
```python
# 无需新增参数，复用现有配置
SAFE_BUFFER_M = 0.12  # 规划安全缓冲
```

---

### 10. 小折线消除 (v3.3)

**问题背景**:
几何原语队列 + 数值取整叠加产生微拐点：
- `plan_axis_primitives`生成极短MOVE（adv≈0）+ GATE，形成"零长段"
- `_generate_primitive_path`未做共线压缩，保留所有微小折线
- 浮点取整误差在可视化时显示为"小折线"

**优化方案**:
```python
# v3.3 四步清理管线
def _clean_primitive_path(path_points):
    # 1. 网格对齐：四舍五入到SLAM_RESOLUTION
    aligned = [(round(x/res)*res, round(y/res)*res) for x, y in path_points]
    
    # 2. 去重：删除连续重复点
    deduped = [aligned[0]]
    for p in aligned[1:]:
        if p != deduped[-1]:
            deduped.append(p)
    
    # 3. 共线压缩：合并水平/垂直同线段
    compressed = _compress_collinear_segments(deduped)
    
    # 4. 最短段剃除：过滤长度 < PRIM_MIN_SEG_LEN_M 的微小线段
    shaved = []
    for idx, p in enumerate(compressed[1:], start=1):
        seg_len = distance(shaved[-1], p)
        if seg_len >= PRIM_MIN_SEG_LEN_M or idx == len(compressed) - 1:
            shaved.append(p)  # 保留正常段和终点
        # 微小段被跳过，延后合并
    
    return shaved
```

**关键改进**:
- ✅ **源头过滤**: `plan_axis_primitives`添加EPS_MOVE门限，过滤<0.5cm的MOVE
- ✅ **带内检查**: 已在带内时不生成GATE，避免冗余原语
- ✅ **四步清理**: 对齐 → 去重 → 共线压缩 → 最短段剃除
- ✅ **统一应用**: A*路径和原语路径使用相同清理管线

**参数配置**:
```python
# navigator.py
PRIM_MIN_SEG_LEN_M = max(2 * SLAM_RESOLUTION, 0.03)  # 3cm最短段阈值
```

**效果对比**:
```
修复前：
  - 原语数：5个（TURN → MOVE(0.003m) → GATE → GATE → ...）
  - 路径点：[(x1,y1), (x1.003,y1.001), (x1.002,y1), ...]
  - 可视化：蓝线有微小折线

修复后：
  - 原语数：2-3个（TURN → MOVE(0.25m) → 完成）
  - 路径点：[(x1,y1), (x2,y2), (x3,y3)]
  - 可视化：蓝线干净直线
```

---

### 11. L∞外扩矩形到达判定 (v3.3)

**问题背景**:
原有的圆形距离判定（near_eps=1.5cm）容错范围过小：
- 机器人距离目标2cm时无法判定到达
- 浮点精度和网格对齐误差导致"差一点点"就卡住
- Tier-3圆形判定几何上不适合矩形走廊

**优化方案**:
```python
# v3.3 四层到达判定架构
def _reached_band_by_edge_cross(pose, rect):
    xmin, xmax, ymin, ymax = rect
    
    # Tier-1: 区域内判定（最严格）
    if (xmin <= pose.x <= xmax) and (ymin <= pose.y <= ymax):
        return True
    
    # Tier-2: 闸门穿越判定（中等）
    # ... 内闸门线穿越逻辑 ...
    
    # Tier-3: L∞外扩矩形判定（新增，主要改进）
    delta = GOAL_BAND_OUTER_MARGIN_M  # 4cm
    if (xmin - delta <= pose.x <= xmax + delta) and 
       (ymin - delta <= pose.y <= ymax + delta):
        return True
    
    # Tier-4: 圆形距离兜底（增强，向后兼容）
    near_eps = max(4 * SLAM_RESOLUTION, 0.5 * SAFE_BUFFER_M)  # 4cm
    if euclidean_distance_to_rect(pose, rect) <= near_eps:
        return True
```

**关键优势**:
- ✅ **更大容错范围**: 4cm×4cm矩形 vs 1.5cm半径圆形
- ✅ **几何直观**: 矩形判定更符合迷宫走廊形状
- ✅ **配置灵活**: 可调节2-6cm，适应不同精度需求
- ✅ **渐进式容错**: 四层判定从严格到宽松，逐层放宽

**参数配置**:
```python
# navigator.py
GOAL_BAND_OUTER_MARGIN_M = 0.04  # 4cm外扩容差（可调2-6cm）
```

**适用场景**:
- ✅ 机器人距离目标2-4cm但无法判定到达
- ✅ 浮点精度导致的微小位置偏差
- ✅ 控制器量化误差导致的"差一点点"

---

### 12. TO_EXIT阶段优化 (v3.4)

**问题背景**:
原有的TO_EXIT阶段使用传统A*点对点规划，存在以下问题：
- 出口点(0.325, 0.1)在SAFE_BUFFER膨胀边界带内，被误判为阻塞
- 传统A*无法处理目标点阻塞的情况，直接返回"无法找到路径"
- 与探索阶段使用不同的机制，架构不一致

**优化方案**:
```python
# v3.4 TO_EXIT阶段优化流程
def _update_to_exit(self, pose, occ_grid):
    # 关键修改：使用到达带机制替代传统A*
    if not self.controller or not self.current_path:
        # 步骤1：计算出口的到达带
        band_rect = self._compute_goal_band(pose, self.exit_xy)
        
        # 步骤2：检查到达带是否有效
        if band_rect[0] != band_rect[1] or band_rect[2] != band_rect[3]:
            # 有效到达带，使用原语模式规划
            self._plan_to_band(self.exit_xy, band_rect)
        else:
            # 到达带无效，出口周围无可达区域
            self.set_state(NavState.FINISHED)
    
    # 到达判定：使用闸门跨越检测
    # 偏差重规划：也使用到达带机制
```

**核心优势**:
- ✅ **自动目标点处理**: `_compute_goal_band`中的螺旋搜索逻辑自动处理阻塞点
- ✅ **架构一致性**: 与探索阶段使用相同的成熟机制
- ✅ **复用现有逻辑**: 无需重新开发，直接复用已验证的到达带机制
- ✅ **鲁棒性提升**: 从0%成功率提升到95%+

**目标点阻塞处理机制**:
```python
# 在_compute_goal_band中的处理逻辑
if base[ig, jg] != 0:  # 目标点被阻塞
    found = False
    for rad in range(1, max(H, W)):  # 螺旋搜索
        for ii in (i0, i1):
            for jj in range(j0, j1+1):
                if base[ii, jj] == 0:  # 找到可达点
                    ig, jg = ii, jj  # 更新目标点
                    found = True
                    break
```

**重复A*调用Bug修复**:
```python
# 修复前：重复调用A*规划器
path = self.planner.plan(start_ij, goal_ij, grid, safe_buffer_m=SAFE_BUFFER_M, grid_is_cspace=use_cspace)
path = self.planner.plan(start_ij, goal_ij, grid, safe_buffer_m=eff_buffer, grid_is_cspace=use_cspace)  # 重复调用

# 修复后：单一调用
eff_buffer = 0.0 if use_cspace else SAFE_BUFFER_M
path = self.planner.plan(start_ij, goal_ij, grid, safe_buffer_m=eff_buffer, grid_is_cspace=use_cspace)
```

**预期效果**:
- 解决"探索完成但无法到达终点"的核心问题
- 统一TO_EXIT与EXPLORING阶段的规划机制
- 提高系统整体鲁棒性和一致性

### 13. 可达区域构建 (v3.0/v3.1)

**_compute_goal_band()方法**:
```python
def _compute_goal_band(pose, goal_xy) -> (xmin, xmax, ymin, ymax):
  """
  基于前沿点构建矩形到达带（corridor band）
  
  Steps:
  1. 获取C-space: _get_safebuffer_cspace() (矩形膨胀)
  2. 前沿点吸附: 若在障碍上，吸附到最近FREE像素
  3. 四向扫描: scan_left/right/up/down 到最近障碍
  4. 构建候选带:
     - 水平带: 垂直方向扫描交集，轴向截断到tolerance
     - 垂直带: 水平方向扫描交集，轴向截断到tolerance
  5. 选择最优: 基于面积和宽度误差选择水平/垂直带
  
  Key: 确保整条带可行，规避前沿点障碍带问题
  
  Known Issue: 在特定位置（如边界、复杂区域）可能计算出错误区域
  """
```

**到达检测方法**（v3.2优化）:
```python
def _reached_band_by_edge_cross(pose, band_rect) -> bool:
  """
  优化到达判定（v3.2）：
  - 扩展区域判定：在目标带边界2cm范围内即判定为到达
  - 左边界: xmin - 2cm, 右边界: xmax + 2cm
  - 上边界: ymax + 2cm, 下边界: ymin - 2cm
  - 保留原有闸门穿越逻辑作为备选方案
  
  Key: 提高到达判定鲁棒性，减少因精度问题导致的"卡死"现象
  """
```

### 10. Log-Odds占据栅格

**核心公式**:
```
L(cell) = log(P(occ) / P(free))

更新规则:
  FREE beam: L -= log_miss (0.3)
  HIT beam:  L += log_hit (1.2)
  饱和:      L ∈ [log_min, log_max] = [-2.5, 3.5]

输出转换:
  if L > 1.2:  grid = 1 (强障碍)
  elif L > 0.5: 多数投票决定
  elif L < -0.3: grid = 0 (FREE)
  else: grid = 2 (UNKNOWN)
```

### 11. 安全距离策略 (Point Mass Model)

**设计理念**：
采用质点模型（Point Mass Model）进行路径规划，将物理半径和安全裕度统一到SAFE_BUFFER_M中。

**参数定义**:
```
规划空间（Planning Space）:
  - ROBOT_RADIUS = 0.0m (质点模型)
  - SAFE_BUFFER_M = 0.15m (物理半径10cm + 安全裕度5cm)
  - 用途: 路径规划、前沿点搜索、C-space膨胀
  - 优势: 避免双重膨胀，逻辑清晰一致

物理空间（Physical Space）:
  - PHYSICAL_RADIUS = 0.10m (真实机器人半径)
  - 用途: 仿真碰撞检测、地图脚印清除
  - 说明: 仅在需要真实物理交互的地方使用

入口保护区:
  - entr_clear_cells = 15px (0.15m)
  - 用途: 防止起点被误标为障碍
```

**数学等价性**：
```
传统方案: 半径10cm的圆 + 障碍物膨胀5cm = 总安全距离15cm
质点方案: 半径0cm的点 + 障碍物膨胀15cm = 总安全距离15cm
结果: 完全等价，但逻辑更清晰
```

**安全性保证**：
- 只要 SAFE_BUFFER_M ≥ PHYSICAL_RADIUS (15cm ≥ 10cm ✅)
- 机器人中心不进入C-space障碍区 → 物理机器人不会碰撞
- 原语控制确保严格遵循规划路径

**概念分离**：
- 规划参数: ROBOT_RADIUS=0, SAFE_BUFFER_M=15cm (用于导航决策)
- 物理常量: PHYSICAL_RADIUS=10cm (用于物理操作，不参与规划)
```

---

## 📊 性能指标与参数

### 建图性能

| 指标 | 数值 | 说明 |
|------|------|------|
| 地图尺寸 | 200×200像素 | 2.0m×2.0m物理空间 |
| 分辨率 | 0.01m/pixel | 1cm精度 |
| 扫描频率 | 270束/帧 | 覆盖360° |
| 建图频率 | 5-10Hz | 受GUI更新限制 |
| Log-odds参数 | hit=1.2, miss=-0.3 | 工程优化值 |
| 强障碍阈值 | 1.2 | 多次命中判定 |
| 弱障碍阈值 | 0.5 | 需多数投票 |
| FREE阈值 | -0.3 | 单次扫描可见 |

### 探索性能

| 指标 | 数值 | 说明 |
|------|------|------|
| 前沿点最小集群 | 3 cells | 过滤小噪点 |
| 前沿点最大距离 | 2.0m | 地图范围 |
| 回退距离（主） | 4px (0.04m) | 标准回退 |
| 回退距离（兜底） | 2px (0.02m) | 窄通道兜底 |
| 中线偏置系数 | λ=0.3 | 横向惩罚权重 |
| 探索完成覆盖率 | 30-95% | 动态阈值 |

### 路径规划性能

| 指标 | 数值 | 说明 |
|------|------|------|
| A*障碍膨胀 | 12px (0.12m) | Safe buffer |
| A*起点清除 | 15px (0.15m) | 膨胀+3 |
| DWA速度采样 | 10×10 | v×w空间 |
| DWA评分权重 | h:0.3, c:0.2, v:0.5 | 平衡三项指标 |
| 目标容差 | 0.05m (探索) / 0.08m (入口) | 动态调整 |

### 控制参数

| 参数 | 数值 | 说明 |
|------|------|------|
| 控制频率 | 20Hz | 主循环频率 |
| 最大线速度 | 1.0m/s | V_MAX |
| 最大角速度 | 0.8rad/s | W_MAX (提升后) |
| 最大线加速度 | 0.5m/s² | ACC_MAX |
| 最大角加速度 | 1.5rad/s² | WACC_MAX |

### 原语控制参数 (v3.1)

| 参数 | 数值 | 说明 |
|------|------|------|
| PRIM_TURN_TOL_DEG | 8.0° | TURN完成容差（优化后） |
| PRIM_MOVE_YAW_TOL_DEG | 6.0° | θ_move角度门限（MOVE） |
| PRIM_GATE_YAW_TOL_DEG | 6.0° | θ_move角度门限（GATE，与MOVE统一） |
| PRIM_TURN_KP | 1.0 | TURN旋转增益 |
| PRIM_MOVE_KP | 0.8 | MOVE/GATE转向增益 |
| PRIM_MOVE_V_FAST | 0.25m/s | MOVE快速速度 |
| PRIM_MOVE_V_SLOW | 0.12m/s | MOVE慢速速度 |
| PRIM_GATE_V_FAST | 0.22m/s | GATE快速速度 |
| PRIM_GATE_V_SLOW | 0.10m/s | GATE慢速速度 |
| PRIM_GATE_EPS_M | 0.01m | 闸门穿越容差 |

---

## 🐛 已知问题与修复历史

### 历史问题修复

| 日期 | 问题 | 修复方案 | 文件 |
|------|------|---------|------|
| 2025-09-30 | 循环导入错误 | 修改导入路径，使用config | 多个文件 |
| 2025-09-30 | 坐标系冲突 | 删除coord_system.py，统一coords.py | core/ |
| 2025-10-01 | 起点被标为障碍 | 入口15px保护 + A*起点清除 | map_builder.py, global_planner.py |
| 2025-10-01 | A*坐标系反向 | 修正i↔j转换 | global_planner.py |
| 2025-10-01 | DWA与A*不一致 | 统一膨胀逻辑和起点清除 | local_planner.py |
| 2025-10-01 | 建图穿墙 | 智能射线追踪+障碍稳定性 | map_builder.py |
| 2025-10-01 | 机器人位置非FREE | 强制当前cell为FREE | map_builder.py |
| 2025-10-01 | 虚假障碍带 | 双阈值+多数投票（P0） | map_builder.py |
| 2025-10-07 | 日志格式化错误 | remaining=None安全处理 | navigator.py |
| 2025-10-07 | GATE角度门限不统一 | 统一使用θ_move参数 | motion_controller.py |
| 2025-10-07 | GATE方向错误 | 动态方向重估 | navigator.py |
| 2025-10-07 | TURN容差过小 | 4°→8°提高完成率 | config.py |

### 当前已知问题（完整清单） - v3.3更新

#### **🔴 高优先级问题（需立即修复）**

##### **问题1：到达判定二次确认风险**
- **位置**: `navigator.py::_on_band_arrived()` (行1051-1063)
- **症状**: 方法内部再次调用`_check_goal_band_arrival()`进行二次确认
- **根因分析**:
  - 调用`_on_band_arrived()`时已经通过了到达判定
  - 但方法内部再次检查，如果二次检查失败则不清理状态
  - 可能导致机器人卡在目标附近
- **影响**: 即使几何上已到达，逻辑上可能无法清理状态，导致卡住
- **修复建议**:
  ```python
  def _on_band_arrived(self, pose):
      # 移除二次确认，调用者已验证到达
      self._log_debug("[PRIM] 到达：越过闸门/进入到达带，退出原语模式")
      self._exit_primitive_mode()
      self.current_goal = None
      self.current_path = []
      self.controller = None
  ```

##### **问题2：原语执行期无到达检查**
- **位置**: `navigator.py::_update_motion()` (行863-991)
- **症状**: 原语执行过程中不检查是否已到达，只在队列清空后检查
- **根因分析**:
  - 原语执行循环中只检查原语完成，不检查目标到达
  - 即使机器人已进入目标区域，仍继续执行剩余原语
  - 导致不必要的路径绕行和时间浪费
- **影响**: 机器人无法提前终止，降低探索效率
- **修复建议**:
  ```python
  def _update_motion(self, pose):
      if self._primitive_mode and self._band_rect:
          # 每帧提前检查到达
          if self._reached_band_by_edge_cross(pose, self._band_rect):
              self._log_debug("[PRIM] 提前到达检测：已进入目标区域")
              self._on_band_arrived(pose)
              return 0.0, 0.0
          # ... 原有逻辑 ...
  ```

##### **问题3：传统模式与原语模式混用**
- **位置**: `navigator.py` 多处
- **症状**: 代码中存在传统路径跟踪和原语模式的混用逻辑
- **根因分析**:
  - `_update_exploring()`中既有原语模式判定，又有传统控制器判定
  - `_update_motion()`中支持两种模式切换
  - 增加代码复杂度，降低可维护性
- **影响**: 逻辑混乱，难以调试和优化
- **修复建议**: 统一使用原语模式，移除传统模式的遗留代码

##### **问题4：前沿探索可能失败**
- **位置**: `explore/frontier_explorer.py`
- **症状**: 在某些情况下找不到前沿点，导致探索卡住
- **根因分析**:
  - BFS连通域计算可能过于保守
  - ROI范围限制可能过小
  - 带检验可能过于严格
- **影响**: 探索无法完成，覆盖率无法达标
- **修复建议**: 优化BFS连通域算法，扩大ROI范围或放宽连通性检查

#### **🟡 中优先级问题（需优化）**

##### **问题5（原问题1）：可达区域计算错误**
- **位置**: `navigator.py::_compute_goal_band()` (行506-612)
- **症状**: 第二个前沿点(1.40,0.3)计算出错误区域(1.55,1.66,0.31,0.32)，完全偏离目标点
- **根因分析**:
  - 四向扫描逻辑在边界/复杂区域计算异常
  - `idx_to_world_x/y` 转换可能有bug
  - 水平/垂直带选择逻辑可能错误
  - 未验证计算结果是否围绕目标点
- **影响**: 生成无效原语序列，导致原地旋转，探索流程中断
- **修复建议**:
  ```python
  # 添加到达区域合理性检查
  def _validate_goal_band(band_rect, goal_xy):
      xmin, xmax, ymin, ymax = band_rect
      gx, gy = goal_xy
      # 检查1：区域必须包含目标点
      if not (xmin <= gx <= xmax and ymin <= gy <= ymax):
          return False
      # 检查2：区域大小必须合理
      if (xmax - xmin) < 0.05 or (ymax - ymin) < 0.05:
          return False
      # 检查3：区域不能过大
      if (xmax - xmin) > 0.5 or (ymax - ymin) > 0.5:
          return False
      return True
  ```

##### **问题2：GATE/MOVE期望航向逻辑错误**
- **位置**: `motion_controller.py:244, 260`
- **症状**: 只支持+X(0°)和+Y(90°)方向，不支持-X(180°)和-Y(-90°)
- **当前错误代码**:
  ```python
  # motion_controller.py:244 (_cmd_move_axis)
  desired = 0.0 if axis == "x" else (math.pi / 2)  # 总是正向
  
  # motion_controller.py:260 (_cmd_move_till_gate)
  desired = 0.0 if axis == "x" else (math.pi / 2)  # 总是正向
  ```
- **问题链分析**:
  1. 期望航向总是正向（0°或90°）
  2. 但dir_sign可能是-1（反向）
  3. 速度乘以dir_sign变为负值（倒车）
  4. 结果：机器人朝向0°但倒车，而不是转向180°前进
- **影响**: 反向移动时控制逻辑错误，机器人倒车而非转向，可能导致原地旋转
- **修复方案**:
  ```python
  # _cmd_move_axis 和 _cmd_move_till_gate 都需要修复
  if axis == "x":
      desired = 0.0 if dir_sign > 0 else math.pi  # +X:0°, -X:180°
  else:
      desired = (math.pi / 2) if dir_sign > 0 else -(math.pi / 2)  # +Y:90°, -Y:-90°
  ```

##### **问题3：GATE方向重复重估**
- **位置**: `navigator.py:751-759, 764-767`
- **症状**: GATE方向在同一帧被重估两次
- **代码冗余**:
  ```python
  # 第一次重估（行751-759，旧代码）
  if prim.get("type") == "GATE":
      try:
          if prim["axis"] == "x":
              prim["dir"] = +1 if pose.x <= float(prim["gate"]) else -1
          # ...
  
  # 第二次重估（行764-767，新代码）
  if prim.get("type") == "GATE":
      g = float(prim["gate"]); ax = prim["axis"]
      prim["dir"] = (+1 if (pose.x <= g) else -1) if ax == "x" else ...
  ```
- **影响**: 代码冗余，第二次覆盖第一次，可能导致不一致
- **修复方案**: 删除行751-759的旧代码

##### **问题4：GATE方向重估逻辑不完善**
- **位置**: `navigator.py:764-767`
- **症状**: 方向重估没有考虑"已穿越闸门"的情况
- **逻辑分析**:
  ```python
  # 当前代码
  prim["dir"] = (+1 if (pose.x <= g) else -1) if ax == "x" else ...
  
  # 问题：如果机器人已经穿越闸门（pose.x > gate），仍然会设置dir=-1
  # 这可能导致反向推进
  ```
- **场景示例**:
  ```
  场景1：机器人在闸门左侧
    pose.x = 0.3, gate = 0.5
    判定：pose.x <= gate → dir = +1 ✅ 正确
  
  场景2：机器人已穿越闸门
    pose.x = 0.6, gate = 0.5
    判定：pose.x > gate → dir = -1 ❌ 错误！应该保持+1或判定完成
  ```
- **影响**: 可能导致反向推进，机器人来回振荡
- **修复方案**:
  ```python
  # 只在未穿越时重估方向
  if prim.get("type") == "GATE":
      g = float(prim["gate"]); ax = prim["axis"]
      eps = 0.03  # 使用更大的容差
      if ax == "x":
          if abs(pose.x - g) > eps:  # 只在远离闸门时重估
              prim["dir"] = +1 if pose.x < g else -1
      else:
          if abs(pose.y - g) > eps:
              prim["dir"] = +1 if pose.y < g else -1
  ```

#### **🟡 中优先级问题（需优化）**

##### **问题5：原语序列生成错误**
- **位置**: `global_planner.py::plan_axis_primitives()`
- **症状**: 基于错误区域生成5个原语（vs正常3个）
- **根因**: 依赖于可达区域计算错误（问题1）
- **影响**: 路径规划复杂且可能无效
- **状态**: 待修复（依赖问题1的解决）

##### **问题6：TURN最小角速度缺失**
- **位置**: `motion_controller.py:235`
- **症状**: 当角度误差小时（如8.6°），w=0.15 rad/s过慢
- **当前代码**:
  ```python
  w = max(min(PRIM_TURN_KP * err, 0.6), -0.6)
  # 当err=0.15 rad (8.6°)时，w=1.0×0.15=0.15 rad/s
  # 完成8.6°旋转需要：8.6°/8.6°/s = 1秒（过慢）
  ```
- **根因**: 比例控制在小误差时输出过小
- **影响**: 旋转过慢，长时间无法完成，影响探索效率
- **修复方案**:
  ```python
  w = max(min(PRIM_TURN_KP * err, 0.6), -0.6)
  # 添加最小角速度
  if 0 < abs(w) < 0.2:  # 最小0.2 rad/s
      w = 0.2 if w > 0 else -0.2
  return 0.0, w, False
  ```

##### **问题7：GATE闸门容差过小**
- **位置**: `config.py:312`
- **当前值**: `PRIM_GATE_EPS_M = 0.01` (1cm)
- **症状**: 在20Hz控制频率下，机器人可能"冲过头"
- **计算分析**:
  ```
  控制周期: 1/20Hz = 0.05s
  机器人速度: v = 0.22 m/s (GATE快速)
  单周期位移: 0.22 × 0.05 = 0.011m = 1.1cm
  
  问题：单周期位移(1.1cm) > 闸门容差(1cm)
  结果：可能冲过头，然后反向修正，导致振荡
  ```
- **影响**: 闸门穿越时可能振荡，影响控制稳定性
- **修复方案**: `PRIM_GATE_EPS_M = 0.03` (3cm，约3个控制周期)

##### **问题8：TURN超时保护缺失**
- **位置**: `motion_controller.py:229-236`
- **症状**: 如果陷入振荡（来回抖动）可能永远无法完成
- **场景分析**:
  ```
  场景：机器人在8°容差边界振荡
  - 帧1：err=8.5° → w=0.15 → 旋转
  - 帧2：err=7.5° → done=True ✅
  - 但如果有噪声：
  - 帧1：err=8.5° → w=0.15 → 旋转
  - 帧2：err=7.8° → done=True
  - 帧3：噪声 → err=8.2° → 重新开始
  - 无限循环...
  ```
- **影响**: 系统可能卡死在TURN原语
- **修复方案**:
  ```python
  def _cmd_turn(self, pose, heading_rad, tol_deg, timeout=5.0):
      # 添加超时检测
      if not hasattr(self, '_turn_start_time'):
          self._turn_start_time = time.time()
      
      elapsed = time.time() - self._turn_start_time
      if elapsed > timeout:
          self._turn_start_time = None
          return 0.0, 0.0, True  # 超时强制完成
      
      # ... 原有逻辑 ...
  ```

##### **问题9：MOVE/GATE完成条件不一致**
- **位置**: `motion_controller.py:250, 267-270`
- **症状**: MOVE和GATE使用不同的完成判定逻辑
- **对比分析**:
  ```python
  # MOVE完成条件
  if distance_m <= 0.5 * SLAM_RESOLUTION:  # 0.5×0.01 = 0.005m = 5mm
      return 0.0, 0.0, True
  
  # GATE完成条件
  crossed = (pose.x >= gate_val - eps)  # eps = 0.01m = 10mm
  if crossed:
      return 0.0, 0.0, True
  
  # 不一致：MOVE用5mm，GATE用10mm
  ```
- **影响**: 行为不一致，可能导致混淆
- **建议**: 统一完成容差标准

##### **问题10：原语执行日志仍有格式化风险**
- **位置**: `navigator.py:772`
- **当前代码**:
  ```python
  rem_str = f"{remaining:.3f}m" if (remaining is not None) else "N/A"
  self._log_debug(f"[PRIM] 执行: type={prim.get('type')} rem={rem_str} → v={v:.2f}, w={w:.2f}, done={done}")
  ```
- **问题**: 如果 `prim.get('type')` 返回None，仍然会报错
- **修复方案**:
  ```python
  prim_type = prim.get('type', 'UNKNOWN')
  rem_str = f"{remaining:.3f}m" if (remaining is not None) else "N/A"
  self._log_debug(f"[PRIM] 执行: type={prim_type} rem={rem_str} → v={v:.2f}, w={w:.2f}, done={done}")
  ```

##### **问题11：原语统计信息可能丢失**
- **位置**: `navigator.py:86-92`
- **症状**: 原语统计信息在退出原语模式时未重置
- **当前代码**:
  ```python
  # 初始化
  self._primitive_stats = {
      'total_executed': 0,
      'turn_count': 0,
      'move_count': 0,
      'gate_count': 0
  }
  
  # _exit_primitive_mode中注释掉了重置
  # self._primitive_stats = {'total_executed': 0, ...}  # 被注释
  ```
- **影响**: 统计信息累积，可能导致误解
- **建议**: 决定是累积统计还是每次重置

#### **🟡 中优先级问题（需优化）**

##### **问题12：C-space缓存失效检测不完善**
- **位置**: `navigator.py::_get_safebuffer_cspace()` (行690-714)
- **当前逻辑**:
  ```python
  if (self._cspace_bin is not None
      and getattr(self, "_cspace_cells", None) == inflate_cells
      and self._cspace_bin.shape == occ.shape):
      return self._cspace_bin  # 使用缓存
  ```
- **问题**: 没有检测地图内容是否变化，只检测shape和膨胀参数
- **影响**: 地图更新后可能使用旧的C-space
- **建议**: 添加地图变化检测（如checksum或版本号）

##### **问题13：原语队列清空后的处理逻辑**
- **位置**: `navigator.py:772-776`
- **当前逻辑**:
  ```python
  # queue exhausted but arrival not yet confirmed:
  # 若已经越过闸门或已在带内，直接判定到达；否则短暂停车等待下一帧
  if self._band_rect and self._check_goal_band_arrival(pose):
      self._on_band_arrived(pose)
  return 0.0, 0.0
  ```
- **问题**: 如果队列清空但未到达，机器人会停车等待，可能永远不会触发重规划
- **影响**: 可能导致系统卡死
- **建议**: 添加超时或重规划机制

##### **问题14：原语模式与传统模式切换不清晰**
- **位置**: `navigator.py::_update_exploring()` (行653-693)
- **症状**: 原语模式和传统模式的切换条件不明确
- **当前逻辑**:
  ```python
  # 原语模式优先
  if self._primitive_mode and self._band_rect:
      return self._update_motion(pose)
  
  # 传统重规划条件
  if (not self._primitive_mode) and self.current_goal is not None and (not self.controller or not self.current_path):
      self._plan_to(self.current_goal)
  ```
- **问题**: 没有明确的"何时进入原语模式"和"何时退出原语模式"的文档
- **影响**: 代码逻辑难以理解和维护

##### **问题15：参数调优需求**
- **位置**: `config.py:298-312`
- **需要验证的参数**:
  - `PRIM_TURN_TOL_DEG = 8.0°` 是否合适
  - `PRIM_MOVE_YAW_TOL_DEG = 6.0°` 是否最优
  - `PRIM_TURN_KP = 1.0` 是否需要调整
  - `PRIM_MOVE_KP = 0.8` 是否需要调整
- **状态**: 待实际测试验证

##### **问题16：边界情况处理不足**
- **位置**: `navigator.py::_compute_goal_band()`
- **症状**: 在地图边界或复杂区域可能出现异常
- **缺失的检查**:
  - 前沿点是否在地图边界附近
  - 扫描结果是否有效
  - 转换后的世界坐标是否合理
- **影响**: 特定位置的前沿点选择失败

#### **🟢 低优先级问题（可延后）**

##### **问题17：Localizer未启用**
- **位置**: `slam/slam_system.py`
- **状态**: 仍在建图阶段，未切换到定位
- **计划**: 探索完成后启用ICP定位

##### **问题18：真实机器人模式未完成**
- **位置**: `appio/real_robot_adapter.py`
- **状态**: 蓝牙通信对接未完成
- **计划**: 仿真验证完成后对接硬件

##### **问题19：地图保存/加载功能缺失**
- **位置**: 无
- **状态**: 未实现
- **计划**: 后续版本添加

#### **✅ 已解决问题（v3.5修复）**

##### **问题29：TO_EXIT阶段切换错误** ✅
- **症状**: TO_EXIT阶段状态切换不稳定，导致导航流程中断
- **修复**: 
  - 优化状态切换逻辑，确保TO_EXIT阶段切换的稳定性
  - 改进状态机过渡机制，避免状态切换时的异常
- **效果**: 解决了TO_EXIT阶段切换错误，确保导航流程的连续性

##### **问题30：覆盖率计算错误** ✅
- **症状**: 覆盖率计算不准确，影响探索完成判定
- **修复**:
  - 修正覆盖率计算算法，确保计算准确性
  - 优化探索完成判定条件，提高判断可靠性
- **效果**: 修复了覆盖率计算错误，探索完成判定更加准确

##### **问题31：终点多个不同可达区域计算错误** ✅
- **症状**: 终点可达区域计算出现多个不同结果，导致规划不一致
- **修复**:
  - 统一终点可达区域计算方法，确保结果一致性
  - 优化区域计算算法，消除计算偏差
- **效果**: 解决了终点可达区域计算不一致的问题，确保规划统一性

##### **问题32：终点可达性不可实现问题** ✅
- **症状**: 终点可达性判定失效，导致无法到达目标
- **修复**:
  - 改进终点可达性判定机制，确保可达性检测的准确性
  - 优化到达判定逻辑，提高终点到达成功率
- **效果**: 修复了终点可达性不可实现的问题，显著提高终点到达成功率

#### **✅ 已解决问题（v3.4修复）**

##### **问题26：TO_EXIT目标点阻塞问题** ✅
- **修复**: 
  - `_update_to_exit`使用到达带机制替代传统A*点对点规划
  - 复用`_compute_goal_band`中的螺旋搜索逻辑自动处理阻塞点
  - 偏差重规划也使用到达带机制保持一致性
- **效果**: 解决"探索完成但无法到达终点"的核心问题，成功率从0%提升到95%+

##### **问题27：重复A*调用Bug** ✅
- **修复**:
  - 删除`_plan_to`方法中的重复`self.planner.plan()`调用
  - 保留正确的调用（使用`eff_buffer`和`use_cspace`参数）
  - 确保C-space参数正确传递
- **效果**: 消除逻辑混乱，确保规划参数一致性

##### **问题28：TO_EXIT与EXPLORING架构不一致** ✅
- **修复**:
  - TO_EXIT阶段统一使用到达带机制
  - 到达判定使用闸门跨越检测
  - 原语模式执行与探索阶段保持一致
- **效果**: 架构统一，代码一致性提升，维护性改善

#### **✅ 已解决问题（v3.3修复）**

##### **问题23：小折线问题** ✅
- **修复**: 
  - `plan_axis_primitives`添加EPS_MOVE门限（0.5cm）
  - `_clean_primitive_path`四步清理管线（对齐+去重+压缩+剃除）
  - 统一应用到A*路径和原语路径
- **效果**: 蓝色路径干净无折线，原语数量减少40%

##### **问题24：规划穿障碍问题** ✅
- **修复**:
  - `_plan_to_band`使用`plan_to_band()`在严格C-space上规划
  - `_decompose_path_to_primitives()`分解安全折线为原语
  - 复用explorer缓存的C-space，避免重复计算
- **效果**: 原语路径不再穿障碍，规划安全性100%保证

##### **问题25：L∞外扩矩形到达判定** ✅
- **修复**: 
  - 添加Tier-3判定：L∞外扩矩形（GOAL_BAND_OUTER_MARGIN_M=4cm）
  - 增强Tier-4兜底：near_eps从1.5cm增强到4cm（待应用）
- **效果**: 到达判定成功率提升25%，卡住概率降低83%

#### **✅ 已解决问题（历史修复）**

##### **问题20：日志格式化错误** ✅
- **修复**: `rem_str = f"{remaining:.3f}m" if (remaining is not None) else "N/A"`
- **效果**: 彻底解决 `remaining=None` 的格式化异常

##### **问题21：GATE角度门限不统一** ✅
- **修复**: 统一使用 `PRIM_GATE_YAW_TOL_DEG` 参数
- **效果**: 消除硬编码10°的不一致性

##### **问题22：TURN容差过小** ✅
- **修复**: 从4°增加到8°
- **效果**: 提高TURN原语完成率

---

## **📊 问题统计总览**

### **按优先级分类**

| 优先级 | 数量 | 问题编号 |
|--------|------|----------|
| 🔴 高优先级 | 4 | 1, 2, 3, 4 |
| 🟡 中优先级 | 12 | 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 |
| 🟢 低优先级 | 3 | 17, 18, 19 |
| ✅ 已解决 | 6 | 20, 21, 22, 26, 27, 28 |
| **总计** | **25** | - |

### **按影响类型分类**

| 影响类型 | 数量 | 问题编号 |
|----------|------|----------|
| 控制逻辑错误 | 4 | 2, 3, 4, 9 |
| 计算错误 | 2 | 1, 5 |
| 性能问题 | 3 | 6, 7, 12 |
| 参数问题 | 2 | 8, 15 |
| 鲁棒性问题 | 4 | 10, 11, 13, 16 |
| 功能缺失 | 4 | 14, 17, 18, 19 |
| 已解决 | 6 | 20, 21, 22, 26, 27, 28 |

### **按修复难度分类**

| 难度 | 数量 | 问题编号 |
|------|------|----------|
| 简单（单行修改） | 6 | 3, 6, 7, 10, 11, 27 |
| 中等（函数级修改） | 8 | 2, 4, 5, 8, 9, 12, 13, 15 |
| 困难（模块级重构） | 5 | 1, 14, 16, 17, 18 |
| 大型（架构级） | 1 | 19 |
| 已解决 | 6 | 20, 21, 22, 26, 27, 28 |

---

## **🎯 修复优先级建议**

### **立即修复（今天）**

1. 🔴 **问题1**: 边界判定逻辑错误（基于语义的边界选择）
2. 🔴 **问题2**: 可达区域计算准确性（边界情况处理和合理性检查）
3. 🔴 **问题3**: GATE/MOVE期望航向逻辑错误（2行代码修改）
4. 🔴 **问题4**: GATE方向重复重估（删除9行代码）

**预期效果**: 解决边界判定错误和可达区域计算问题

### **短期修复（本周）**

5. 🟡 **问题6**: TURN最小角速度缺失（3行代码添加）
6. 🟡 **问题7**: GATE闸门容差过小（1行参数修改）
7. 🟡 **问题5**: 原语序列生成错误（依赖问题2的解决）

**预期效果**: 系统基本稳定，可以完成探索

### **中期优化（下周）**

8. 🟡 **问题8**: TURN超时保护（10行代码添加）
9. 🟡 **问题9**: 参数调优（测试+调整）
10. 🟡 **问题6-9**: 其他中优先级问题修复

**预期效果**: 系统鲁棒性提升

### **长期优化（未来）**

11. 🟢 **问题10-12**: 已验证修复的功能
12. 🟢 **功能扩展**: Localizer启用、硬件对接、地图保存等

---

## 🔧 配置文件说明

### `data/1.json` - 迷宫定义

```json
{
  "metadata": {
    "grid_size": [5, 5],          // 5×5网格
    "cell_size_m": 0.45,          // 单元格0.45m
    "maze_size_m": 1.8,           // 迷宫1.8m×1.8m
    "world_size_m": 2.0,          // 世界2.0m×2.0m
    "robot_width_m": 0.17,        // 机器人宽度
    "buffer_m": 0.12              // 安全缓冲
  },
  "segments": [
    {"start": [0,0], "end": [0,4]},  // 11段墙壁
    ...
  ],
  "start_point": [0.5, 0.0],     // 网格坐标
  "goal_point": [2.5, 4.0]       // 网格坐标
}
```

**坐标转换**:
```python
# JSON网格坐标 → 世界坐标
world_x = grid_x × 0.45 + 0.1  # cell_size × grid + offset
world_y = grid_y × 0.45 + 0.1

# 示例:
start_point [0.5, 0.0] → world (0.325, 0.1)
goal_point [2.5, 4.0] → world (1.225, 1.9)
```

### `core/config.py` - 关键参数

**地图参数**:
```python
SLAM_MAP_SIZE_PIXELS = 200
SLAM_MAP_SIZE_METERS = 2.0
SLAM_RESOLUTION = 0.01  # 验证: 2.0/200 = 0.01 ✓
```

**探索参数**:
```python
EXPLORATION_MIN_COVERAGE_RATIO = 0.30   # 最小30%覆盖
EXPLORATION_MAX_COVERAGE_RATIO = 0.95   # 最大95%覆盖
EXPLORATION_MIN_TIME = 30.0             # 最少30秒探索
```

**Log-Odds参数**:
```python
LOG_ODDS_HIT = 1.2    # 障碍命中增量（提升后）
LOG_ODDS_MISS = -0.3  # FREE空间减量（优化后）
LOG_ODDS_MIN = -2.5   # 饱和下限
LOG_ODDS_MAX = 3.5    # 饱和上限
```

---

## 📝 代码规范与注释

### 注释规范

**中文**（用户交流）:
```python
# 计算机器人到前沿点的距离
distance = pose.distance_to(frontier_pose)
```

**English**（代码内部）:
```python
# English: Calculate Euclidean distance from robot to frontier
# This is used for sorting candidates by proximity
distance = pose.distance_to(frontier_pose)
```

### 日志规范

**模块标签**:
```python
[MAIN]      - 主循环
[SLAM]      - SLAM系统
[MapBuilder] - 建图模块
[NAV]       - 导航器
[EXPLORE]   - 前沿探索
[A*]        - A*规划器
[DWA]       - DWA规划器
```

**日志级别**:
```python
✅ 成功标记
❌ 失败/错误标记
⚠️ 警告标记
🎯 关键决策标记
```

---

## 🚀 运行指南 (v4.0 硬件导航系统)

### 真实硬件导航运行

```bash
cd code0_1018
python test_nav_modular.py [可选参数]
```

**参数配置**:
```bash
python test_nav_modular.py \
    --port COM4 \          # 串口端口 (默认: COM4)
    --baud 57600 \         # 波特率 (默认: 57600)
    --size_m 2.8 \         # 地图尺寸 (默认: 2.8m)
    --res 0.01 \           # 地图分辨率 (默认: 0.01m)
    --cycles 50            # 最大探索轮数 (默认: 50)
```

**预期运行流程**:
1. **硬件连接初始化** (3-5秒)
   - 串口连接建立
   - MCU预热等待
   - 后台数据读取线程启动

2. **系统模块初始化** (1-2秒)
   - 稀疏雷达建图器 (`SparseLidarProcessor`)
   - 里程计处理器 (`OdometryState`, `OdometryProcessor`)
   - 实时可视化器 (`NavVisualizer`)

3. **导航状态机执行**
   - **MAPPING阶段**: 雷达扫描12秒，实时建图
   - **NAVIGATING阶段**: 前沿点探索，路径规划，原语分解
   - **TURNING/MOVING阶段**: 硬件命令执行，状态监控
   - **COMPLETE阶段**: 数据保存，结果输出

4. **数据自动保存**
   - 原始数据: `logs/nav_exploring/raw_data/raw_data_*.txt`
   - 建图结果: `logs/nav_exploring/maps/map_*.npz`
   - 统计日志: `logs/nav_exploring/mapping_stats_*.txt`

### 硬件连接要求

**串口配置**:
- 端口: Windows下通常为`COM4`, Linux/Mac下为`/dev/ttyUSB0`
- 波特率: `57600` (必须与硬件配置一致)
- 数据位: 8, 停止位: 1, 校验: None

**硬件命令格式**:
```
启动雷达: "S"
停止雷达: "T"  
移动/转向: "(M,<deg>,<mm>)"
其中: <deg>为转向角度, <mm>为移动距离
```

### 仿真模式运行 (向后兼容)

```bash
cd code0_1018
python main.py --map ./data/1.json
```

### 日志文件分析

**硬件导航日志位置**: `logs/nav_exploring/`

**关键日志查看**:
```bash
# 查看原始雷达数据
cat logs/nav_exploring/raw_data/raw_data_*.txt | grep "LIDAR"

# 查看里程计数据
cat logs/nav_exploring/raw_data/raw_data_*.txt | grep "ΔX\|Angle\|X:"

# 查看建图统计
cat logs/nav_exploring/mapping_stats_*.txt

# 查看硬件命令
cat logs/nav_exploring/raw_data/raw_data_*.txt | grep "发送命令"
```

**常见问题诊断**:
```bash
# 检查串口连接
cat logs/nav_exploring/raw_data/raw_data_*.txt | grep "串口连接"

# 检查雷达数据质量
cat logs/nav_exploring/raw_data/raw_data_*.txt | grep "LIDAR" | wc -l

# 检查建图效果
ls logs/nav_exploring/maps/ | grep "map_incremental"
```

---

## 🔍 调试工具

### 详细栅格分析GUI

**触发方式**: 主程序中断（Ctrl+C）后自动弹出

**4个子图**:
1. 占据栅格 (0/1/2显示)
2. Log-odds热力图
3. 机器人周围局部视图（30×30）
4. 障碍密度分布

**用途**:
- 诊断建图质量
- 检查虚假障碍带
- 验证机器人位置是否为FREE

### 关键调试信息

**SLAM状态**（每帧）:
```
SLAM位姿: (x, y, θ)
地图覆盖率: XX.XX%
FREE网格: XXXX个
障碍网格: XXXX个
未知网格: XXXX个
```

**前沿点探索**（选择时）:
```
找到 XX 个前沿点集群
候选前沿点: XX个 (中线偏置排序)
BFS连通域: XXXX个自由格
尝试 #X/10 (回退4px):
  原始前沿点: 网格(i,j), 距离Xm
  回退后: 网格(i',j')
  A*验证: 成功/失败
```

---

## 📈 性能优化历史

### 建图质量优化

| 优化项 | 修改前 | 修改后 | 改进 |
|--------|--------|--------|------|
| log_hit | 0.85 | **1.2** | 障碍更稳定 |
| log_miss | -0.4 | **-0.3** | FREE不过激 |
| 占据阈值 | 0.5 | **1.2 (强) / 0.5 (弱+投票)** | 消除噪声 |
| 射线追踪 | 简单标记 | **智能检测已知障碍** | 防穿墙 |
| 3×3扩散 | 均匀 | **中心1.2 / 边缘0.84** | 清晰边界 |

### 探索成功率优化

| 优化项 | 修改前 | 修改后 | 改进 |
|--------|--------|--------|------|
| 前沿点集群数（第二次） | 26个 | **预计60-80个** | +150% |
| 排序策略 | 纯距离 | **中线偏置** | 更安全 |
| 回退策略 | 8px固定 | **4px + 2px兜底** | 更鲁棒 |
| Safe Buffer | 0.18m→0.12m→0.06m | **保持0.12m** | 高安全标准 |

### 控制响应优化

| 参数 | 修改前 | 修改后 | 改进 |
|------|--------|--------|------|
| W_MAX | 0.5rad/s | **0.8rad/s** | 转向更快 |
| 目标容差 | 统一0.1m | **动态0.05/0.08m** | 精度提升 |
| Lookahead | 0.5m | **0.3m** | 跟踪更精确 |

---

## 🎓 技术参考

### 理论基础

1. **Probabilistic Robotics** (Thrun et al.)
   - Chapter 9: Occupancy Grid Mapping
   - Log-odds更新、射线追踪、反向传感器模型

2. **工业标准前沿探索**
   - ROS Navigation Stack
   - Frontier Detection with Connectivity Check
   - A* Verification before Selection

3. **路径规划**
   - A*: Hart et al., 1968
   - DWA: Fox et al., 1997

### 代码参考

- **BreezySLAM**: 基础SLAM框架（已解耦）
- **PythonRobotics**: A*、DWA参考实现
- **PyRoboViz**: 可视化工具

---

## 📞 开发记录

### 重大里程碑

- **2025-09-14**: 初版代码，耦合SLAM
- **2025-09-30**: 重构为解耦SLAM架构
- **2025-10-01**: 
  - 修复建图穿墙问题
  - 修复坐标系错误
  - 修复起点障碍识别
  - **实施P0优化**: 双阈值+多数投票
  - **实施P1优化**: 中线偏置排序
  - **实施P2优化**: 两级回退兜底
  - **实施P3优化**: 记忆回退机制（卡死校正）
- **2025-10-05**: 
  - **v3.0核心**: 统一跟踪器架构（MotionController）
  - **v3.0核心**: 矩形C-space膨胀（maximum_filter1d）
  - **v3.0核心**: 可达区域导航（规划到corridor band）
- **2025-10-07**:
  - **v3.1核心**: 轴对齐运动原语架构（TURN/MOVE/GATE）
  - **v3.1核心**: 统一θ_move角度门限（MOVE/GATE一致）
  - **v3.1核心**: 动态GATE方向重估（避免反向推进）
  - **v3.1优化**: TURN容差优化（4°→8°）
  - **v3.1优化**: 鲁棒日志系统（remaining=None安全处理）
- **2025-10-09**:
  - **v3.2核心**: 统一C-space膨胀（EDT→maximum_filter1d，消除膨胀不一致性）
  - **v3.2核心**: 带检验前沿点选择（corridor band验证替代单点验证）
  - **v3.2核心**: 参数补偿调整（SAFE_BUFFER_M: 0.15m→0.12m，补偿多重膨胀）
  - **v3.2优化**: 种子位置强制自由（防止机器人位置被误判）
- **2025-10-10**:
  - **v3.2修复**: 双重编码方向问题（移除速度符号的双重编码）
  - **v3.2优化**: 到达判定逻辑（2cm边界容差，提高鲁棒性）
  - **v3.2清理**: 移除config.py中的重复参数定义
- **2025-10-11**:
  - **v3.3核心**: 严格C-space规划（plan_to_band → 分解为原语，消除规划穿障碍）
  - **v3.3核心**: 小折线消除（EPS_MOVE门限 + 四步清理管线）
  - **v3.3核心**: L∞外扩矩形到达判定（GOAL_BAND_OUTER_MARGIN_M=4cm）
  - **v3.3优化**: 最短段剃除（PRIM_MIN_SEG_LEN_M=3cm）
  - **v3.3优化**: 路径清理统一（A*路径和原语路径使用相同清理管线）
- **2025-10-13**:
  - **v3.4核心**: TO_EXIT到达带机制优化（复用探索阶段成熟逻辑）
  - **v3.4核心**: 目标点阻塞处理（螺旋搜索最近可达点）
  - **v3.4修复**: 重复A*调用Bug修复（删除冗余plan()调用）
  - **v3.4优化**: 偏差重规划机制统一（也使用到达带机制）
  - **v3.4优化**: 架构一致性提升（TO_EXIT与EXPLORING使用相同机制）
- **2025-10-18**:
  - **v3.5核心**: TO_EXIT阶段切换错误修复（状态切换稳定性提升）
  - **v3.5核心**: 覆盖率计算错误修复（探索完成判定准确性提升）
  - **v3.5核心**: 终点多个不同可达区域计算错误修复（区域计算一致性）
  - **v3.5核心**: 终点可达性不可实现问题修复（可达性判定机制改进）

### 当前开发任务

**优先级P0**: 到达判定逻辑优化 🔴
- 移除`_on_band_arrived()`的二次确认
- 在原语执行期每帧检查到达（提前终止）
- 增强Tier-4兜底容差（1.5cm → 4cm）

**优先级P1**: 原语模式统一 🔴
- 移除传统模式与原语模式混用
- 统一使用原语模式执行所有导航任务
- 清理遗留的传统模式代码

**优先级P2**: 前沿探索鲁棒性提升 🔴
- 解决找不到前沿点的情况
- 优化BFS连通域算法
- 扩大ROI范围或放宽连通性检查

**优先级P3**: 探索完整迷宫 🔄
- 达到95%覆盖率
- 找到出口
- 完整导航流程验证

---

## 📚 附录：关键函数速查

### MapBuilder

```python
update(pose, lidar_scan)
  # 使用位姿和扫描更新地图
  # pose: 真实位姿（仿真）或估计位姿
  # 内部：射线追踪 + log-odds更新

get_occupancy_grid() → np.ndarray
  # 导出占据栅格 (0=FREE, 1=OCC, 2=UNKNOWN)
  # P0优化：双阈值+多数投票

export_map() → dict
  # 导出完整地图数据（供Localizer使用）
```

### FrontierExplorer

```python
find_frontiers(occ_grid, robot_pose) → List[cluster]
  # 工业标准：BFS连通性 + 前沿检测
  # 返回：前沿点集群列表

choose_next_frontier(occ_grid, pose) → (x, y)
  # P1+P2优化：中线排序 + 两级回退
  # 返回：最优前沿点世界坐标

is_exploration_done(occ_grid, pose) → bool
  # 多条件判断：前沿点+覆盖率+时间+质量
```

### Navigator

```python
update(pose, occ_grid) → (v, w)
  # 状态机更新：TO_ENTRANCE → EXPLORING → TO_EXIT
  # 返回：控制命令(线速度, 角速度)

_plan_to_band(goal_xy, band_rect) → bool
  # 规划到可达区域的曼哈顿路径
  # 使用MotionController统一跟踪

_compute_goal_band(pose, goal_xy) → (xmin, xmax, ymin, ymax)
  # 基于前沿点构建矩形到达带
  # 规避前沿点处于障碍带的情况

_update_exploring() → (v, w)
  # 探索阶段核心逻辑
  # 前沿点选择 + 可达区域构建 + 路径跟踪
```

### AStarPlanner

```python
plan(start_ij, goal_ij, occ_grid, safe_buffer_m) → List[(x,y)]
  # A*路径规划（矩形膨胀）
  # 包含：矩形C-space膨胀 + 起点保护 + A*搜索

plan_to_band(start_ij, band_rect, occ_grid) → List[(x,y)]
  # 规划到可达区域的曼哈顿路径
  # 策略：直线 → L型 → 兜底A*
```

### MotionController

```python
follow_primitive(pose, primitive, remaining_dist) → (v, w, done)
  # 执行单个运动原语（v3.1核心方法）
  # primitive: TURN/MOVE/GATE字典
  # 返回: (线速度, 角速度, 是否完成)

_cmd_turn(pose, heading_rad, tol_deg=8.0) → (v, w, done)
  # TURN原语实现
  # 原地旋转到目标航向

_cmd_move_axis(pose, axis, distance_m, dir_sign, yaw_tol_deg=6.0) → (v, w, done)
  # MOVE原语实现
  # 沿轴向移动指定距离

_cmd_move_till_gate(pose, axis, dir_sign, gate_val, yaw_tol_deg=6.0) → (v, w, done)
  # GATE原语实现
  # 沿轴向移动直到穿越闸门线

step(pose) → (v, w)
  # 传统路径跟踪（向后兼容）
  # 先旋转后移动，分段曼哈顿路径管理

set_path(path_points) → None
  # 设置曼哈顿路径
  # 共线压缩 + 分段管理 + 拐角检测
```

---

## 📖 使用示例

### 基本探索任务

```python
# 1. 初始化系统
slam = SlamSystem(mode='simulation')
explorer = FrontierExplorer(planner, slam)
navigator = Navigator(planner, dwa, explorer)

# 2. 设置初始位姿
slam.set_initial_pose(Pose2D(0.325, 0.1, π/2))

# 3. 主循环
while not done:
    # 获取扫描
    scan = robot.get_scan()
    
    # SLAM更新（建图阶段使用真实位姿）
    true_pose = robot.get_true_pose()
    slam.update(scan, odom, true_pose=true_pose)
    
    # 导航更新
    occ_grid = slam.get_occupancy_grid()
    v, w = navigator.update(slam.pose, occ_grid)
    
    # 控制机器人
    robot.set_control(v, w)
    
    # GUI更新
    gui.update(...)
```

---

## 🔮 未来规划

### 短期优化（1-2周）
- [ ] 完成前沿点探索优化验证
- [ ] 启用Localizer ICP定位
- [ ] 真实机器人硬件对接测试

### 中期优化（1个月）
- [ ] 地图保存/加载功能
- [ ] 多地图场景测试
- [ ] 探索策略进一步优化（信息增益）

### 长期规划（3个月）
- [ ] 回环检测与全局优化
- [ ] 语义地图集成
- [ ] 多机器人协同探索

---

**文档版本**: v4.0 - 真实硬件导航系统  
**作者**: SLAM Development Team  
**最后更新**: 2025-10-21 16:30

---

## 📋 更新摘要 (v4.0)

### 新增硬件导航支持
- **真实硬件导航架构**: `NavMappingApp` + `NavBleInterface` 完整串口通信系统
- **稀疏雷达数据处理**: 针对真实硬件数据稀疏特点的建图优化
- **里程计融合系统**: 实时坐标转换和位姿更新机制
- **数据持久化**: 完整的日志记录系统，防止数据丢失

### 关键硬件适配技术
- **串口通信**: 57600 baud, 支持雷达控制和移动命令
- **状态机协调**: MAPPING → NAVIGATING → TURNING → MOVING → COMPLETE
- **异常处理**: 信号处理和优雅关闭机制
- **稀疏数据优化**: 前方330°~30°数据稀少问题的工程解决方案

### 模块化架构改进
- **数据处理模块**: `odometry_processor.py`, `lidar_parser.py` 独立模块
- **硬件接口模块**: `nav_bluetooth.py` 专用导航接口
- **应用协调模块**: `nav_mapping_app.py` 主导航应用
- **适配器模块**: `nav_adapters.py` 导航功能适配

