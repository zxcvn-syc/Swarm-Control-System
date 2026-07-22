"""
Static Voronoi UAV Containment Demo

功能：
1. 无人机静态部署
2. Voronoi区域划分
3. 无人机覆盖范围计算
4. 封控覆盖率计算
5. 结果可视化

Author: UAV Containment Project
"""


import numpy as np
import matplotlib.pyplot as plt

from scipy.spatial import Voronoi, voronoi_plot_2d

from shapely.geometry import Polygon, Point
from shapely.ops import unary_union



# =========================
# 1. 参数设置
# =========================


# 任务区域大小
AREA_SIZE = 100


# 无人机位置(x,y)
uav_points = np.array([
    [20, 20],
    [80, 20],
    [50, 50],
    [20, 80],
    [80, 80]
])


# 无人机数量
uav_num = len(uav_points)


# 单架无人机探测/封控半径
coverage_radius = 25



# =========================
# 2. 创建任务区域
# =========================


mission_area = Polygon([
    (0, 0),
    (AREA_SIZE, 0),
    (AREA_SIZE, AREA_SIZE),
    (0, AREA_SIZE)
])



# =========================
# 3. 静态Voronoi区域划分
# =========================


vor = Voronoi(uav_points)


plt.figure(figsize=(7, 7))


voronoi_plot_2d(
    vor,
    show_vertices=False,
    show_points=False
)


# 绘制无人机

plt.scatter(
    uav_points[:,0],
    uav_points[:,1],
    c="red",
    s=100,
    label="UAV"
)



# UAV编号

for i, p in enumerate(uav_points):

    plt.text(
        p[0]+2,
        p[1]+2,
        "UAV"+str(i+1)
    )


plt.xlim(0, AREA_SIZE)
plt.ylim(0, AREA_SIZE)

plt.grid()

plt.title(
    "Static Voronoi UAV Partition"
)

plt.legend()

plt.savefig(
    "voronoi_partition.png",
    dpi=300
)

plt.show()



# =========================
# 4. UAV覆盖区域计算
# =========================


coverage_regions = []


for p in uav_points:

    circle = Point(
        p[0],
        p[1]
    ).buffer(
        coverage_radius
    )

    coverage_regions.append(circle)



# 合并覆盖区域

total_coverage = unary_union(
    coverage_regions
)


# 裁剪到任务区域

total_coverage = total_coverage.intersection(
    mission_area
)



# =========================
# 5. 覆盖率计算
# =========================


area_total = mission_area.area

area_cover = total_coverage.area


coverage_rate = (
    area_cover / area_total
)



print("==========================")
print("Static Voronoi UAV Containment")
print("==========================")

print(
    "无人机数量:",
    uav_num
)

print(
    "任务区域面积:",
    area_total
)

print(
    "覆盖区域面积:",
    round(area_cover, 2)
)

print(
    "封控覆盖率:",
    round(coverage_rate*100, 2),
    "%"
)



# =========================
# 6. 输出无人机信息
# =========================


for i,p in enumerate(uav_points):

    print("----------------------")

    print(
        "UAV:",
        i+1
    )

    print(
        "位置:",
        tuple(p)
    )

    print(
        "覆盖半径:",
        coverage_radius
    )



# =========================
# 7. 最终封控效果图
# =========================


fig, ax = plt.subplots(
    figsize=(7,7)
)



# 任务区域边界

x,y = mission_area.exterior.xy

ax.plot(
    x,
    y,
    linewidth=2
)



# UAV覆盖圆

for region in coverage_regions:

    x,y = region.exterior.xy

    ax.fill(
        x,
        y,
        alpha=0.25
    )



# UAV点

ax.scatter(
    uav_points[:,0],
    uav_points[:,1],
    c="red",
    s=100
)



for i,p in enumerate(uav_points):

    ax.text(
        p[0]+2,
        p[1]+2,
        "UAV"+str(i+1)
    )



ax.set_xlim(
    0,
    AREA_SIZE
)

ax.set_ylim(
    0,
    AREA_SIZE
)


ax.set_xlabel(
    "X(m)"
)

ax.set_ylabel(
    "Y(m)"
)


ax.set_title(
    "UAV Static Voronoi Containment Coverage"
)


ax.grid()



plt.savefig(
    "uav_containment_result.png",
    dpi=300
)


plt.show()
