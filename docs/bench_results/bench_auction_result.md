# Auction 引擎耗时基准 (agents × tasks)

基线: unknown | seed: 42 | 平台: Intel64 Family 6 Model 186 Stepping 2, GenuineIntel | 时间: 2026-08-13 11:25:16

重复次数: 10 次取中位数 | Python: 3.13.14

## Greedy Assign

| Agents \\ Tasks | 10 | 20 | 50 |
|---|---|---|---|
| 8 |  0.06 ms |  0.10 ms |  0.19 ms |
| 16 |  0.09 ms |  0.13 ms |  0.31 ms |
| 32 |  0.14 ms |  0.20 ms |  0.42 ms |

## Auction Engine

| Agents \\ Tasks | 10 | 20 | 50 |
|---|---|---|---|
| 8 |  0.66 ms |  1.21 ms |  4.63 ms |
| 16 |  1.13 ms |  2.33 ms |  5.38 ms |
| 32 |  2.29 ms |  4.11 ms |  5.73 ms |
