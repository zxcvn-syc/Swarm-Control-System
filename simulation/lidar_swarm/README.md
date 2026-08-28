# 六平台 LiDAR 量测仿真

本目录提供一个不依赖额外工具箱的 MATLAB R2026a 仿真。它在同一二维水平面中布置 3 架 UAV 和 3 辆 UGV，每个平台各有一个 LiDAR 量测源；仿真目标为一个行人和一辆车辆。

它建模的是距离/方位角量测与回波级融合，不是 Gazebo、PX4 或三维飞行避障仿真。因此，结果能验证分类、跨传感器锁定和鬼影剔除逻辑，不能替代 UAV 三维机体碰撞验证。

## 方法

- 每一时刻为有效可见目标生成带高斯噪声的距离、方位角和轮廓尺度量测，并转换到 world 坐标。
- 在单帧按空间半径聚类；平均轮廓小于 2 m 判为 `person`，否则判为 `vehicle`。
- 轨迹仅在至少 3 次时间确认且观测到至少 2 个独立传感器后锁定。
- 每个传感器交替生成一个快速移动的单传感器鬼影。它们不能满足跨传感器确认，超过丢失门限后记为拒绝。
- RNG 算法固定为 `twister`，种子为 `20260828`。

## 运行

```matlab
addpath("simulation/lidar_swarm")
results = runtests("simulation/lidar_swarm/tests");
assertSuccess(results)
run_lidar_swarm_simulation("simulation/lidar_swarm/results")
```

输出目录包含量测、真值、轨迹和指标 CSV，JSON 汇总以及确定性的 PNG 概览图。

## 已验证结果

使用 MATLAB R2026a 执行 `TestLidarSwarmSimulation` 的两个测试和完整结果生成，指标见 `results/lidar_swarm_metrics.csv`：

| 指标 | 结果 |
|---|---:|
| 平台 | 3 UAV + 3 UGV |
| 目标量测 / 鬼影量测 | 936 / 363 |
| 成功锁定真实目标 | 2 / 2 |
| 拒绝鬼影轨迹 | 358 |
| 鬼影误锁 | 0 |
| 已锁定目标分类准确率 | 1.0 |
| 平均锁定时延 | 1.0 s |

这些数值只对该固定随机种子、场景和门限有效；变更传感器布局、噪声、速度或鬼影模型后必须重新运行并比较结果，不能将它们外推为真实飞行性能。
