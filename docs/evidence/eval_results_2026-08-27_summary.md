# 三场景封控成功率测试统计（8.26 响应证据门槛）

- 总测试次数：60
- 有效判定（成功+失败）：59
- 无效（INVALID，无响应证据）：1
- **封控成功率（有效样本）：89.8%** （指标 6 目标 ≥85%）

> 说明：INVALID 指目标被回收/守住，但全程无平台进入 intercept_radius（默认 5 m），即封控无法归因于系统，故单列、不计入成功率分母。

## 各场景明细

| 场景 | 次数 | 成功 | 失败 | INVALID | 成功率 | 轨迹分布 |
|------|------|------|------|---------|--------|----------|
| border | 20 | 17 | 2 | 1 | 89.5% | oscillate=3, return=15, straight=2 |
| park | 20 | 18 | 2 | 0 | 90.0% | oscillate=4, return=14, straight=2 |
| security | 20 | 18 | 2 | 0 | 90.0% | oscillate=2, return=16, straight=2 |

## 失败 / 无效原因分布

| 场景 | 结果 | 原因 | 次数 |
|------|------|------|------|
| border | FAIL | escaped_monitor | 2 |
| border | INVALID | no_response_evidence | 1 |
| border | SUCCESS | held_within_monitor | 2 |
| border | SUCCESS | re_contained | 15 |
| park | FAIL | escaped_monitor | 2 |
| park | SUCCESS | held_within_monitor | 4 |
| park | SUCCESS | re_contained | 14 |
| security | FAIL | escaped_monitor | 2 |
| security | SUCCESS | held_within_monitor | 2 |
| security | SUCCESS | re_contained | 16 |
