#!/usr/bin/env python3

"""
Dynamic Voronoi UAV Containment Demo

功能：
1. UAV动态移动
2. Voronoi实时更新
3. 封控区域变化
4. 实时覆盖率计算
"""


import numpy as np

import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation

from scipy.spatial import Voronoi, voronoi_plot_2d

from shapely.geometry import Point, Polygon

from shapely.ops import unary_union



# =====================
# 参数
# =====================


AREA_SIZE = 100

UAV_NUM = 5

COVER_RADIUS = 20



# UAV初始位置

uav_positions = np.array([

    [20,20],
    [80,20],
    [50,50],
    [20,80],
    [80,80]

], dtype=float)



# UAV速度

velocity = np.array([

    [0.5,0.2],
    [-0.4,0.3],
    [0.2,-0.4],
    [0.3,-0.3],
    [-0.3,-0.2]

])



# 任务区域

mission_area = Polygon([

    (0,0),
    (100,0),
    (100,100),
    (0,100)

])



# =====================
# 覆盖率计算
# =====================


def calculate_coverage():

    circles=[]


    for p in uav_positions:

        circle = Point(
            p[0],
            p[1]
        ).buffer(
            COVER_RADIUS
        )

        circles.append(circle)



    total = unary_union(circles)


    total = total.intersection(
        mission_area
    )


    rate = (

        total.area /

        mission_area.area

    )


    return rate*100




# =====================
# 动态更新
# =====================


fig, ax = plt.subplots(
    figsize=(7,7)
)



def update(frame):

    global uav_positions


    ax.clear()



    # 更新无人机位置

    uav_positions[:] += velocity



    # 边界反弹

    for i in range(UAV_NUM):

        if uav_positions[i][0] < 0 or uav_positions[i][0]>100:

            velocity[i][0]*=-1


        if uav_positions[i][1] <0 or uav_positions[i][1]>100:

            velocity[i][1]*=-1



    # 重新计算Voronoi

    vor = Voronoi(
        uav_positions
    )


    voronoi_plot_2d(

        vor,

        ax=ax,

        show_vertices=False

    )



    # UAV显示

    ax.scatter(

        uav_positions[:,0],

        uav_positions[:,1],

        c="red",

        s=80

    )



    for i,p in enumerate(uav_positions):

        ax.text(

            p[0]+2,

            p[1]+2,

            "UAV"+str(i+1)

        )



    coverage = calculate_coverage()



    ax.set_title(

        "Dynamic Voronoi UAV Containment\nCoverage %.2f%%"
        %
        coverage

    )


    ax.set_xlim(0,100)

    ax.set_ylim(0,100)

    ax.grid()



ani = FuncAnimation(

    fig,

    update,

    frames=200,

    interval=100

)


plt.show()
