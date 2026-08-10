# 动态 Voronoi 封控设计规范

> **版本**：V1.0
> **日期**：2026-08-10
> **负责人**：陈思睿
> **对应包**：`ros2_ws/src/containment_pkg`
> **状态**：正式发布

---

## 目录

1. [文档概述](#1-文档概述)
2. [系统背景](#2-系统背景)
3. [术语与符号约定](#3-术语与符号约定)
4. [Voronoi 封控数学模型](#4-voronoi-封控数学模型)
5. [算法规范](#5-算法规范)
6. [ROS2 接口规范](#6-ros2-接口规范)
7. [节点行为规范](#7-节点行为规范)
8. [配置规范](#8-配置规范)
9. [数据流与集成规范](#9-数据流与集成规范)
10. [测试规范](#10-测试规范)
11. [已知限制与演进路线](#11-已知限制与演进路线)
12. [附录](#12-附录)

---

## 1. 文档概述

### 1.1 目的

本规范定义异构无人集群协同封控系统中**动态 Voronoi 封控模块**的设计标准，涵盖数学模型、算法逻辑、ROS2 接口、节点行为、配置参数、数据流和测试要求。作为 `containment_pkg` 的权威设计文档，供开发、联调和验收使用。

### 1.2 适用范围

- `containment_pkg` 的核心算法 `voronoi_enclose()` 及其 ROS2 节点封装 `EnclosureNode`
- 消息定义 `swarm_interfaces/msg/EnclosureCommand`、`EnclosureCommandArray`、`EnclosureTarget`、`EnclosureTargetArray`
- 与感知组（`tracker_node`）、调度组（`scheduler_node`）、坐标转换节点（`coord_transform_node`）的集成对接

### 1.3 相关文档

| 文档 | 路径 | 说明 |
|------|------|------|
| 动态 Voronoi 封控模型设计说明 | `docs/dynamic_voronoi_containment.md` | 数学模型基础描述 |
| 静态 Voronoi 封控初步模型 | `docs/containment_model.md` | 静态分区与覆盖率计算 |
| Topic 接口设计 V2 | `docs/interface/Topic接口设计V2.md` | 系统级 Topic 接口定义 |
| 封控组接入指南 | `docs/interface/封控组接入指南.md` | 启动与联调指南 |
| 坐标转换设计 | `docs/interface/坐标转换设计.md` | 像素→世界坐标转换规范 |
| containment_pkg README | `ros2_ws/src/containment_pkg/README.md` | 包级说明 |

---

## 2. 系统背景

### 2.1 项目定位

本项目面向第十九届"挑战杯"全国大学生课外学术科技作品竞赛——揭榜挂帅专项赛，构建**异构无人集群协同封控系统**。系统整合无人机与无人车平台，实现目标感知 → 跟踪 → 路径规划 → 任务调度 → **协同封控** → 控制执行的全链路闭环。

### 2.2 技术栈

| 技术 | 用途 |
|------|------|
| ROS2 Humble | 通信中间件 |
| Ubuntu 22.04 | 操作系统 |
| PX4 Autopilot | 无人机飞控 |
| RflySim | 数字孪生仿真 |
| SciPy / Shapely | Voronoi 图计算与几何运算 |
| NumPy | 向量与矩阵运算 |
| Python 3.10 | 开发语言 |

### 2.3 封控模块在系统中的位置

```
相机图像 ─► tracker_node (YOLOv8 + DeepSORT/BoT-SORT)
                   │
        ┌──────────┴──────────────┐
        ▼                         ▼
  /target_track             /enclosure_targets
  (TargetTrackArray)       (EnclosureTargetArray)
        │                         │
        ▼                         ▼
  coord_transform_node      EnclosureNode (本模块)
  (像素→世界坐标)           (Voronoi 封控计算)
        │                         │
        └────────┬────────────────┘
                 ▼
         /enclosure_command
         (EnclosureCommandArray)
                 │
                 ▼
          PX4 / RflySim 飞控执行
```

### 2.4 开发进度

| 阶段 | 状态 | 内容 |
|------|------|------|
| 静态 Voronoi 分区 | ✅ 已完成 | `static_voronoi_uav.py` 离线演示 |
| 动态 Voronoi 更新 | ✅ 已完成 | `dynamic_voronoi_uav.py` 动画演示 |
| ROS2 节点封装 | ✅ 已完成 | `enclosure_node.py` + `voronoi.py` |
| 消息接口定义 | ✅ 已完成 | `EnclosureCommand/Array`、`EnclosureTarget/Array` |
| 单元测试 | ✅ 已完成 | 算法测试 + 节点测试（3 + 3 组） |
| Lloyd 迭代优化 | 📋 规划中 | 预留 `iterations` 参数 |
| 加权 Voronoi | 📋 规划中 | 异构平台差异化分区 |
| 3D 封控扩展 | 📋 规划中 | 空地协同三维封控 |

---

## 3. 术语与符号约定

### 3.1 术语表

| 术语 | 定义 |
|------|------|
| **封控** | 以无人机/无人车为节点，围绕目标形成包围态势，限制目标移动空间 |
| **Voronoi 区域** | 空间中距离某生成点最近的所有点构成的子区域 |
| **生成点** | Voronoi 图的种子点，本模块中为无人机位置 |
| **有效半径** | 封控点距目标的距离，取 `enclosure_radius` 与 `min_dist` 的较大值 |
| **Dirty 标记** | 节点内部状态标志，标识自上次计算以来是否有新输入到达 |
| **待命指令** | 当无人机数量多于目标数量时，多余无人机收到的 NaN 坐标指令 |
| **Lloyd 迭代** | Voronoi 图的质心松弛算法，用于优化分区均匀性 |

### 3.2 符号表

| 符号 | 含义 | 单位 |
|------|------|------|
| $U = \{u_1, u_2, \dots, u_n\}$ | 无人机集合 | — |
| $P_i = (x_i, y_i)$ | 第 $i$ 架无人机位置 | m |
| $T = \{t_1, t_2, \dots, t_m\}$ | 目标集合 | — |
| $t_j = (x_j^t, y_j^t)$ | 第 $j$ 个目标位置 | m |
| $d_i(X)$ | 点 $X$ 到无人机 $u_i$ 的欧氏距离 | m |
| $V_i$ | 无人机 $u_i$ 的 Voronoi 区域 | — |
| $R$ | 封控有效半径（$R = \max(R_{enc}, R_{min})$） | m |
| $R_{enc}$ | 配置的封控半径参数 `enclosure_radius` | m |
| $R_{min}$ | 配置的最小距离参数 `min_dist` | m |
| $A$ | 任务区域总面积 | m² |
| $A_c$ | 无人机覆盖区域面积 | m² |
| $\text{Coverage}$ | 封控覆盖率 | % |

---

## 4. Voronoi 封控数学模型

### 4.1 Voronoi 区域划分

设任务区域 $\Omega \subset \mathbb{R}^2$，内部部署 $n$ 架无人机，每架无人机作为 Voronoi 生成点。第 $i$ 架无人机的 Voronoi 区域定义为：

$$V_i = \{X \in \Omega \mid d_i(X) < d_j(X), \forall j \neq i\}$$

其中 $d_i(X) = \|X - P_i\|$ 为点 $X$ 到无人机 $u_i$ 的欧氏距离。

**性质**：
- $\Omega = V_1 \cup V_2 \cup \dots \cup V_n$（完备划分）
- $V_i \cap V_j = \emptyset, \forall i \neq j$（区域不重叠，边界处等距）

### 4.2 封控点投影模型

运行时封控采用**目标中心径向投影**策略：

1. **目标-无人机分配**：每架无人机分配到距其最近的目标。
2. **外法向投影**：封控点沿目标→无人机方向的单位向量放置在距目标 $R$ 处。
3. **退化处理**：当无人机与目标几乎重合时，依次回退到目标→目标质心方向、等角分布方向。

数学表达：

$$\text{target}_i = t_{a(i)} + R \cdot \frac{P_i - t_{a(i)}}{\|P_i - t_{a(i)}\|}$$

其中 $a(i) = \arg\min_j \|P_i - t_j\|$ 为最近目标索引，$R = \max(R_{enc}, R_{min})$。

### 4.3 覆盖率计算

单架无人机的覆盖区域为以其封控点为圆心、$R$ 为半径的圆盘：

$$C_i = \{X \mid \|X - \text{target}_i\| \leq R\}$$

总覆盖区域：

$$C = \bigcup_{i=1}^{n} C_i$$

裁剪至任务区域：

$$C_{\text{eff}} = C \cap \Omega$$

覆盖率：

$$\text{Coverage} = \frac{\text{Area}(C_{\text{eff}})}{\text{Area}(\Omega)} \times 100\%$$

### 4.4 静态与动态对比

| 特性 | 静态 Voronoi | 动态 Voronoi |
|------|-------------|-------------|
| 无人机位置 | 固定 | 实时变化 |
| 区域划分 | 一次计算 | 每周期重算 |
| 封控范围 | 固定 | 随运动更新 |
| 目标移动 | 不跟随 | 自动跟随 |
| 适用场景 | 初始部署评估 | 实时封控任务 |
| 对应代码 | `static_voronoi_uav.py` | `dynamic_voronoi_uav.py` + `voronoi.py` |

### 4.5 Lloyd 松弛（预留）

Lloyd 迭代通过反复将生成点移动到其 Voronoi 区域质心来优化分区均匀性。当前实现保留了 `iterations` 参数接口但未启用，因确定性投影算法无需迭代求解。后续接入完整 Lloyd 优化时：

1. 计算各 Voronoi 区域质心。
2. 将生成点移动到质心。
3. 重新计算 Voronoi 图。
4. 重复至收敛或达到 `iterations` 次数。

---

## 5. 算法规范

### 5.1 核心函数：`voronoi_enclose()`

**文件**：`containment_pkg/voronoi.py`

**签名**：

```python
def voronoi_enclose(
    target_xy: np.ndarray,   # shape (M, 2)，目标位置
    drone_xy: np.ndarray,    # shape (N, 2)，无人机位置
    enclosure_radius: float,  # 封控半径（m）
    min_dist: float = 5.0,   # 最小距离（m）
    iterations: int = 50,    # Lloyd 迭代次数（预留，当前不使用）
) -> Tuple[np.ndarray, np.ndarray]:  # 返回 (封控点 (N,2), 有效半径 (N,))
```

**算法流程**：

```
输入: target_xy (M,2), drone_xy (N,2), enclosure_radius, min_dist
  │
  ├─ 1. 输入校验
  │    ├─ targets 为空 → 返回空数组
  │    ├─ shape 校验: target_xy (M,2), drone_xy (N,2)
  │    ├─ 有限值校验: 所有元素必须有限
  │    └─ 非负校验: enclosure_radius ≥ 0, min_dist ≥ 0, iterations ≥ 0
  │
  ├─ 2. 计算有效半径
  │    R = max(enclosure_radius, min_dist)
  │
  ├─ 3. 目标-无人机分配
  │    对每架无人机, 找距其最近的目标:
  │    a(i) = argmin_j ||drone_i - target_j||
  │    (批量计算: N×M 距离矩阵 → argmin)
  │
  ├─ 4. 生成封控点（逐无人机）
  │    for i in range(N):
  │      dir = drone_i - target_{a(i)}        # 外法向
  │      if ||dir|| < 1e-9:                   # 退化情况 1
  │        dir = target_{a(i)} - center       # 回退到目标质心方向
  │      if ||dir|| < 1e-9:                   # 退化情况 2
  │        angle = 2π * i / N                  # 回退到等角分布
  │        dir = [cos(angle), sin(angle)]
  │      target_i = target_{a(i)} + R * dir / ||dir||
  │
  └─ 5. 返回 (封控点数组, 有效半径数组)
       所有半径统一为 R
```

**输入约束**：

| 参数 | 类型 | 约束 | 说明 |
|------|------|------|------|
| `target_xy` | `np.ndarray` | shape (M, 2), M ≥ 0, 有限值 | 目标二维坐标 |
| `drone_xy` | `np.ndarray` | shape (N, 2), N ≥ 0, 有限值 | 无人机二维坐标 |
| `enclosure_radius` | `float` | ≥ 0 | 封控半径 |
| `min_dist` | `float` | ≥ 0, 默认 5.0 | 半径下限 |
| `iterations` | `int` | ≥ 0, 默认 50 | Lloyd 迭代（预留） |

**异常**：`ValueError` — shape 不匹配、含非有限值、参数为负。

**输出保证**：
- 返回封控点 shape 恒为 (N, 2)。
- 返回有效半径 shape 恒为 (N,)，所有值相等且 = max(enclosure_radius, min_dist)。
- 每个封控点严格在距目标 R 的圆周上（退化情况除外）。

### 5.2 退化情况处理

| 场景 | 条件 | 回退策略 |
|------|------|----------|
| 无人机在目标正上方 | ‖drone − target‖ < 1e-9 | 使用 target − center 方向 |
| 单目标且无人机在目标处 | 上述回退后仍 < 1e-9 | 等角分布 $\theta_i = 2\pi i / N$ |
| 无目标 | targets 为空 | 返回空封控点数组 |

### 5.3 待命指令生成

当无人机数量 $N$ 大于目标数量 $M$ 时：
- 前 $\min(N, M)$ 架无人机分配封控点。
- 第 $M+1$ 至 $N$ 架无人机收到**待命指令**：`target_x/y/z = NaN`，`enclosure_radius = 0.0`。

---

## 6. ROS2 接口规范

### 6.1 话题接口

| 方向 | 话题名 | 消息类型 | 说明 |
|------|--------|----------|------|
| **输入（主）** | `/target_track` | `swarm_interfaces/msg/TargetTrackArray` | 感知组发布的目标轨迹，读取 `tracks[].x/y` |
| **输入（兼容）** | `/enclosure_targets` | `swarm_interfaces/msg/EnclosureTargetArray` | 感知组发布的封控专用目标，读取 `targets[].x/y` |
| **输入** | `/drone_states` | `swarm_interfaces/msg/DroneStateArray` | 无人机状态，读取 `drones[].drone_id/x/y/z` |
| **输出** | `/enclosure_command` | `swarm_interfaces/msg/EnclosureCommandArray` | 每架无人机的封控目标点和有效半径 |

**输入优先级**：`/target_track` 为主来源；`/enclosure_targets` 为兼容回退。两者同时发布时，**最后到达的消息覆盖内部目标快照**。联调时建议只启用一个目标输入。

### 6.2 消息定义

#### EnclosureCommand

```
uint32   drone_id              # 无人机 ID
float64  target_x              # 封控目标点 X（m）
float64  target_y              # 封控目标点 Y（m）
float64  target_z              # 封控目标点 Z（m）
float32  enclosure_radius      # 有效封控半径（m）
```

#### EnclosureCommandArray

```
EnclosureCommand[] commands    # 每架无人机的封控指令
uint32             num_drones  # 无人机总数
```

#### EnclosureTarget（兼容输入用）

```
uint32     target_id           # 目标 ID
float64    x                   # 目标 X 位置
float64    y                   # 目标 Y 位置
float32    speed               # 目标速度大小
uint8      motion_mode         # 0=未知,1=静止,2=慢,3=快
float32    confidence          # 目标置信度
float32    box_x1/box_y1/box_x2/box_y2  # 包围盒
float32[5] pred_x/pred_y       # 未来5步预测位置
float32[10] history_x/history_y # 历史轨迹
```

#### EnclosureTargetArray

```
std_msgs/Header   header
uint32            frame_idx
EnclosureTarget[] targets
float32[8]        drone_x       # 无人机 X 位置（最多8架）
float32[8]        drone_y       # 无人机 Y 位置
uint8             num_drones
float32           enclosure_radius
float32           min_enclosure_dist
```

#### DroneState

```
uint32   drone_id
float64  x / y / z             # 位置
float64  vx / vy / vz          # 速度
bool     available            # 是否可用
uint8    PLATFORM_DRONE=0     # 平台类型常量
uint8    PLATFORM_CAR=1
uint8    platform_type
```

#### DroneStateArray

```
DroneState[]  drones
uint32        num_drones
```

#### TargetTrack（主输入用）

```
uint32     target_id
float64    x / y               # 坐标
float64    vx / vy            # 速度
float32    confidence
uint8      cls
bool       is_confirmed
float32    speed
uint8      motion_mode
float32[5] pred_x / pred_y / pred_conf  # 预测轨迹
```

### 6.3 坐标系约定

| 数据源 | 坐标系 | 说明 |
|--------|--------|------|
| `/target_track` | 像素坐标 | 感知组原始输出，需经 `coord_transform_node` 转换 |
| `/enclosure_targets` | 像素坐标 | 同上，兼容输入 |
| `/drone_states` | 世界坐标（ENU） | 飞控/模拟器输出 |
| `/enclosure_command` | 世界坐标（ENU） | 封控指令 |

**重要**：`/target_track` 和 `/enclosure_targets` 发布的是**像素坐标**。当需要世界米制坐标时，须由 `coord_transform_node`（感知组 C3 桥接节点）完成像素→相机系→机身系→世界系（ENU）的转换。封控节点假设输入的 `x/y` 已是**世界米制坐标**。坐标系定义参见[坐标转换设计](../interface/坐标转换设计.md)。

---

## 7. 节点行为规范

### 7.1 节点概要

| 属性 | 值 |
|------|-----|
| 节点名 | `enclosure_node` |
| 可执行入口 | `enclosure_node`（`setup.py` console_scripts） |
| 构建类型 | `ament_python` |
| 依赖 | `rclpy`、`numpy`、`swarm_interfaces` |
| 维护者 | chen |
| 许可证 | Apache-2.0 |

### 7.2 生命周期

```
rclpy.init()
    │
    ▼
EnclosureNode()  ──► 声明参数、创建订阅/发布/定时器
    │
    ▼
rclpy.spin(node) ──► 事件循环
    │
    │   ┌──────────────────────────────────────┐
    │   │  回调层（异步，只更新快照+置 dirty）    │
    │   │  on_target_track()    → 更新 targets   │
    │   │  on_enclosure_targets() → 更新 targets │
    │   │  on_drone()           → 更新 drones   │
    │   └──────────────────────────────────────┘
    │                   │
    │   ┌──────────────────────────────────────┐
    │   │  定时器层（周期 update_period 秒）      │
    │   │  tick():                              │
    │   │    if not dirty or not targets         │
    │   │      or not drones → return False      │
    │   │    _recalculate() → voronoi_enclose()  │
    │   │    → 发布 EnclosureCommandArray         │
    │   │    → 清除 dirty                         │
    │   └──────────────────────────────────────┘
    │
    ▼
node.destroy_node() + rclpy.shutdown()
```

### 7.3 Dirty 标记机制

节点采用**写时标记、读时计算**策略，解耦异步输入与定时计算：

1. **回调阶段**（异步触发）：收到任何输入消息 → 更新内部快照 → 置 `_dirty = True`。不执行计算。
2. **定时器阶段**（周期触发）：`tick()` 检查 `_dirty`。仅当 `_dirty == True` 且 `_targets` 和 `_drones` 均非空时才执行计算。
3. **计算后**：清除 `_dirty = False`。同一周期内后续的 `tick()` 不重复计算。
4. **下一条输入**：重新置 `_dirty = True`，开启下一次计算周期。

**设计意图**：避免高频输入消息触发过密的计算；一个 `update_period` 周期内至多一次输出。

### 7.4 待命指令规则

当无人机数量 $N >$ 目标数量 $M$ 时：

| 无人机索引 | 指令内容 |
|------------|----------|
| $0 \sim M-1$ | 正常封控点（`target_x/y/z` 为计算值，`enclosure_radius = R`） |
| $M \sim N-1$ | 待命（`target_x/y/z = NaN`，`enclosure_radius = 0.0`） |

待命无人机的 `drone_id` 仍然保留，下游节点可据此决定悬停或返航。

### 7.5 兼容回调别名

为兼容旧版调用方，节点提供别名：
- `_targets_callback` = `on_enclosure_targets`
- `_drones_callback` = `on_drone`

---

## 8. 配置规范

### 8.1 参数定义

| 参数名 | 类型 | 默认值 | 范围 | 说明 |
|--------|------|--------|------|------|
| `enclosure_radius` | `float` | 25.0 | [0, ∞) | 封控点距目标的有效半径（m） |
| `min_dist` | `float` | 5.0 | [0, ∞) | 半径下限（m），实际半径 = max(enclosure_radius, min_dist) |
| `update_period` | `float` | 1.0 | [0.01, ∞) | 动态重算周期（s），下限 0.01 防止过频 |

### 8.2 配置文件

**文件**：`config/containment.yaml`

```yaml
enclosure_node:
  ros__parameters:
    enclosure_radius: 25.0
    min_dist: 5.0
    update_period: 1.0
```

### 8.3 参数调优指南

| 场景 | enclosure_radius | min_dist | update_period | 说明 |
|------|-----------------|----------|---------------|------|
| 密集封控（多目标近距离） | 15.0 | 3.0 | 0.5 | 小半径高频更新 |
| 标准封控 | 25.0 | 5.0 | 1.0 | 默认配置 |
| 大范围封控（少目标远距离） | 50.0 | 10.0 | 2.0 | 大半径低频更新 |
| 快速目标追踪 | 20.0 | 5.0 | 0.2 | 高频更新跟上目标运动 |

**注意**：`update_period` 过小会增加 CPU 负载；建议根据目标运动速度和无人机数量合理设置。

### 8.4 启动方式

```bash
# 方式 1：使用配置文件启动
cd <ros2_ws>
colcon build --packages-select swarm_interfaces containment_pkg
source install/setup.bash
ros2 launch containment_pkg containment.launch.py

# 方式 2：命令行参数覆盖
ros2 run containment_pkg enclosure_node \
    --ros-args \
    -p enclosure_radius:=25.0 \
    -p min_dist:=5.0 \
    -p update_period:=1.0
```

### 8.5 Launch 文件规范

**文件**：`launch/containment.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="containment_pkg",
            executable="enclosure_node",
            name="enclosure_node",
            output="screen",
            parameters=["config/containment.yaml"],
        ),
    ])
```

---

## 9. 数据流与集成规范

### 9.1 输入源

| 来源 | 话题 | 发布节点 | 坐标系 | 角色 |
|------|------|----------|--------|------|
| 感知组（主） | `/target_track` | `tracker_node` | 像素 | 目标轨迹（YOLOv8 + DeepSORT） |
| 感知组（兼容） | `/enclosure_targets` | `tracker_node` | 像素 | 封控专用目标 |
| 飞控/模拟器 | `/drone_states` | PX4 / RflySim | 世界（ENU） | 无人机位姿 |

### 9.2 输出消费方

| 消费方 | 话题 | 用途 |
|--------|------|------|
| PX4 飞控 | `/enclosure_command` | 无人机位置控制目标 |
| RflySim | `/enclosure_command` | 仿真验证 |
| 调度组（可选） | `/enclosure_command` | 封控状态监控 |

### 9.3 坐标转换集成

感知组发布的 `/target_track` 为像素坐标。封控节点要求世界坐标。数据流必须经过坐标转换节点：

```
/target_track (像素) ─► coord_transform_node ─► /target_track_world (世界 ENU) ─► EnclosureNode
```

`coord_transform_node` 执行的转换链：
1. 像素 → 相机光学系（逆 K 矩阵反投影）
2. 地面平面假设投影
3. 相机光学系 → 机身系（安装角旋转）
4. 机身系 → 世界系（ENU 刚体变换）

详见[坐标转换设计](../interface/坐标转换设计.md)。

### 9.4 三链路联调

系统支持**感知→调度→封控**三链路联调，通过 `three_links.launch.py` 启动：

```bash
ros2 launch ros2_ws three_links.launch.py
```

联调验证要点：
- `tracker_node` 发布 `/target_track` 后，`enclosure_node` 在一个 `update_period` 内发布 `/enclosure_command`。
- 目标移动时，封控点跟随移动，偏差 ≤ `enclosure_radius`。
- 无人机数量变化时，`/enclosure_command.commands` 数量同步变化。

---

## 10. 测试规范

### 10.1 测试文件

| 文件 | 测试内容 | 依赖 |
|------|----------|------|
| `tests/test_voronoi.py` | 算法核心 `voronoi_enclose()` | numpy, containment_pkg |
| `tests/test_dynamic_voronoi.py` | 动态特性（确定性、目标运动、min_dist） | numpy, containment_pkg |
| `tests/test_enclosure_node.py` | ROS2 节点行为 | rclpy, swarm_interfaces |
| `tests/conftest.py` | pytest fixtures | pytest |

### 10.2 算法测试用例

| 用例名 | 验证内容 | 预期结果 |
|--------|----------|----------|
| `test_voronoi_enclose_basic` | 单目标4无人机对称布局 | 封控点均在半径25的圆周上 |
| `test_voronoi_enclose_radius` | 半径参数传递 | 返回半径 = 12.5 |
| `test_voronoi_more_drones_than_targets` | 5无人机2目标 | 输出 shape (5,2)，多余无人机待命 |
| `test_static_voronoi_is_deterministic` | 相同输入两次调用 | 结果完全一致（确定性） |
| `test_target_motion_changes_enclosure_region` | 目标从(0,0)移到(10,0) | 封控点整体平移10m |
| `test_min_dist_is_enforced` | enclosure_radius=2, min_dist=5 | 有效半径 = 5 |

### 10.3 节点测试用例

| 用例名 | 验证内容 | 预期结果 |
|--------|----------|----------|
| `test_target_track_and_drone_callbacks_update_state` | 回调更新内部状态 | targets/drones 正确存储，dirty=True |
| `test_tick_publishes_once_until_next_input` | dirty 机制 | 首次 tick 发布，第二次 tick 不发布（dirty已清），新输入后再次发布 |
| `test_multiple_targets_and_drones_publish_standby_for_extra_drone` | 3无人机2目标 | 3条指令，第3条 radius=0（待命） |

### 10.4 测试执行

```bash
# 在已 source ROS2 和 swarm_interfaces 的环境中
cd <ros2_ws>
source install/setup.bash
pytest containment_pkg/tests/ -v
```

**注意**：算法测试不依赖 ROS2；节点测试在缺少 ROS2 接口时会自动跳过（`pytest.importorskip("rclpy")`）。

### 10.5 验收标准

| 验收项 | 标准 |
|--------|------|
| 算法正确性 | 所有算法测试通过 |
| 节点行为 | 所有节点测试通过（ROS2 环境下） |
| 静态基线 | 固定发布一帧后 `/enclosure_command` 有输出 |
| 动态跟随 | 目标移动后封控点跟随，偏差 ≤ enclosure_radius |
| 待命正确 | 无人机多于目标时多余无人机 radius=0 |
| 计算性能 | 单次 `voronoi_enclose()` 耗时 < 10ms（10目标10无人机） |

---

## 11. 已知限制与演进路线

### 11.1 当前限制

| 限制 | 说明 | 影响 |
|------|------|------|
| 无 Lloyd 迭代 | `iterations` 参数预留但未实现 | 分区均匀性未优化 |
| 2D 平面 | 仅处理 x/y 坐标，z 直接透传 | 不支持三维封控 |
| 无坐标转换 | 节点假设输入已是世界坐标 | 需上游 `coord_transform_node` |
| 均匀半径 | 所有无人机使用相同有效半径 | 异构平台无差异化 |
| 无区域裁剪 | 运行时节点不做 Shapely 区域裁剪 | 封控点可能在任务区域外 |
| 覆盖率离线 | 覆盖率计算仅存在于演示脚本 | 运行时无覆盖率指标 |
| 无时间同步 | 不使用 `message_filters` 做时间对齐 | 目标和无人机状态可能不同步 |

### 11.2 演进路线

| 优先级 | 功能 | 说明 |
|--------|------|------|
| P1 | Lloyd 迭代优化 | 实现 `iterations` 参数，质心松弛优化分区 |
| P1 | 加权 Voronoi | 异构平台（无人机 vs 无人车）差异化分区 |
| P2 | 3D 封控扩展 | 支持 z 轴封控，空地协同立体包围 |
| P2 | 区域裁剪 | 运行时裁剪封控点到任务区域 |
| P2 | 时间同步 | `message_filters.ApproximateTimeSynchronizer` |
| P3 | 实时覆盖率 | 节点内部计算并发布覆盖率指标 |
| P3 | 动态半径 | 根据目标速度和运动模式自适应调整半径 |
| P3 | 预测封控 | 利用 `pred_x/y` 前置封控点 |

---

## 12. 附录

### 12.1 文件清单

| 文件 | 路径 | 说明 |
|------|------|------|
| 核心算法 | `containment_pkg/voronoi.py` | `voronoi_enclose()` 函数 |
| ROS2 节点 | `containment_pkg/enclosure_node.py` | `EnclosureNode` 类 |
| 静态演示 | `containment_pkg/static_voronoi_uav.py` | 离线静态 Voronoi 演示 |
| 动态演示 | `containment_pkg/dynamic_voronoi_uav.py` | 动态 Voronoi 动画演示 |
| 配置 | `config/containment.yaml` | 参数默认值 |
| 启动 | `launch/containment.launch.py` | ROS2 launch 文件 |
| 构建配置 | `setup.py` / `setup.cfg` / `package.xml` | ament_python 构建 |
| 测试 | `tests/test_voronoi.py` | 算法测试 |
| 测试 | `tests/test_dynamic_voronoi.py` | 动态特性测试 |
| 测试 | `tests/test_enclosure_node.py` | 节点行为测试 |
| 测试 | `tests/conftest.py` | pytest 公共 fixtures |

### 12.2 依赖矩阵

| 依赖 | 类型 | 用途 |
|------|------|------|
| `rclpy` | ROS2 | 节点与通信 |
| `numpy` | Python | 矩阵运算 |
| `swarm_interfaces` | 自定义 | 消息定义 |
| `scipy.spatial.Voronoi` | Python | Voronoi 图（仅演示脚本） |
| `shapely` | Python | 几何运算（仅演示脚本） |
| `matplotlib` | Python | 可视化（仅演示脚本） |

### 12.3 修订历史

| 版本 | 日期 | 修改人 | 说明 |
|------|------|--------|------|
| V1.0 | 2026-08-10 | 陈思睿 | 初始正式版本发布 |

---

> 本规范基于 `containment_pkg` 当前代码实现编写，随代码演进同步更新。
> 如有疑问，联系维护者 chen。
