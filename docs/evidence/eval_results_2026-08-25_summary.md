# 三场景封控成功率测试统计

- 总测试次数：60
- 综合成功率：**96.7%** （目标 ≥85%）

| 场景 | 次数 | 成功 | 成功率 | 轨迹分布 |
|------|------|------|--------|----------|
| border | 20 | 18 | 90.0% | oscillate=6, return=12, straight=2 |
| park | 20 | 20 | 100.0% | oscillate=2, return=18 |
| security | 20 | 20 | 100.0% | oscillate=4, return=16 |

## 失败原因分布

| 场景 | 原因 | 次数 |
|------|------|------|
| border | escaped_monitor | 2 |
| border | held_within_monitor | 6 |
| border | re_contained | 12 |
| park | held_within_monitor | 2 |
| park | re_contained | 18 |
| security | held_within_monitor | 4 |
| security | re_contained | 16 |
