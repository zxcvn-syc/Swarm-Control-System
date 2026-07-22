# 协同封控模块
# 无人机Voronoi封控模块（containment_pkg）


## 1. 模块简介

`containment_pkg` 是一个基于 ROS2 的无人机协同封控研究模块。

本模块实现基于 Voronoi 图的无人机区域划分与封控模型，用于研究多无人机在二维任务区域中的空间分配和覆盖能力。

当前版本主要实现：

- 无人机静态部署
- 静态 Voronoi 区域划分
- 无人机封控范围建模
- 封控覆盖率计算
- 封控结果可视化


---

# 2. 软件包结构


```
containment_pkg/
│
├── containment_pkg/
│   ├── __init__.py
│   └── static_voronoi_uav.py
│
├── resource/
│   └── containment_pkg
│
├── package.xml
├── setup.py
├── setup.cfg
│
└── README.md
```


---

# 3. 静态Voronoi封控模型设计


## 3.1 无人机部署模型


假设任务区域为二维平面区域：

\[
\Omega \subset R^2
\]


区域内部署 N 架无人机。


无人机位置集合表示为：

\[
P=\{p_1,p_2,...,p_N\}
\]


其中：

\[
p_i=(x_i,y_i)
\]


表示第 i 架无人机的位置坐标。



---

## 3.2 Voronoi区域划分模型


根据无人机之间的欧氏距离，对任务区域进行划分。


第 i 架无人机对应的 Voronoi 区域定义为：


\[
V_i=
\{x\in\Omega | d(x,p_i)<d(x,p_j),j\neq i\}
\]


其中：

- \(V_i\)：第 i 架无人机负责区域
- \(d(x,p_i)\)：目标点 x 到无人机 i 的距离


所有无人机区域满足：


\[
\Omega=
V_1\cup V_2\cup ...\cup V_N
\]


并且：


\[
V_i\cap V_j=\emptyset
\]


表示不同无人机负责区域之间不存在重复划分。



---

# 4. 无人机封控覆盖模型


假设每架无人机具有固定探测/封控半径：

\[
R_i
\]


第 i 架无人机覆盖区域表示为：


\[
C_i=
\{x||x-p_i|\leq R_i\}
\]


其中：

- \(C_i\)：第 i 架无人机覆盖区域
- \(R_i\)：无人机封控半径


所有无人机形成的总封控区域为：


\[
C=
\bigcup_{i=1}^{N}C_i
\]


---

# 5. 封控覆盖率计算


采用覆盖面积比例评价无人机封控能力。


覆盖率公式：


\[
Coverage=
\frac{Area(C)}
{Area(\Omega)}
\times100\%
\]


其中：


- \(Area(C)\)：无人机实际覆盖区域面积
- \(Area(\Omega)\)：任务区域总面积


覆盖率越高，表示无人机对目标区域的封控能力越强。



---

# 6. 仿真参数


| 参数 | 数值 |
| ---- | ---- |
|任务区域大小|100m × 100m|
|无人机数量|5架|
|无人机类型|静态部署|
|区域划分方法|Voronoi图|
|封控半径|25m|



---

# 7. 环境依赖


Python依赖库：


```bash
pip install numpy scipy matplotlib shapely
```



ROS2环境：

- ROS2 Humble（推荐）
- Python3
- colcon build工具



---

# 8. 运行方法


## Python直接运行


进入工作空间：


```bash
cd ~/ros2_ws
```


运行程序：


```bash
python3 src/containment_pkg/containment_pkg/static_voronoi_uav.py
```



---

## ROS2方式运行


编译工作空间：


```bash
colcon build
```


加载环境：


```bash
source install/setup.bash
```


运行节点：


```bash
ros2 run containment_pkg static_voronoi_uav
```



---

# 9. 输出结果


程序运行后输出：


## （1）Voronoi区域划分结果

显示每架无人机对应负责区域。


## （2）无人机覆盖范围

显示无人机有效封控范围。


## （3）封控覆盖率


示例：


```
无人机数量: 5

任务区域面积: 10000 m²

覆盖区域面积: xxxx m²

封控覆盖率: xx.xx %
```



---

# 10. 后续研究方向


本模块后续将进一步扩展：


## 动态Voronoi更新机制

包括：

- 无人机位置实时更新
- Voronoi区域动态重构
- 封控区域实时变化


## 多无人机协同控制

包括：

- 无人机任务分配
- 编队控制
- 协同封控策略


## Voronoi优化方法

包括：

- 加权Voronoi划分
- Lloyd算法优化部署
- 自适应覆盖优化



---

# 11. 作者信息


作者：Chen


版本：

v0.0.0
