# 感知跟踪交付入库说明（组长审查版）

入库时间：2026-08-28
交付人：杨诗钰
审查人：张瑜暄（组长，本次审查由组长执行并入库）

## 一、交付内容

三场景（park / security / border）真实 YOLO 检测 + IoU Tracker 跟踪链路的完整交付：

- `01_detection/`：各场景 GT 对齐后的检测结果（conf=0.50）与置信度阈值扫描结果
- `02_tracking/`：各场景 Tracker 输出与跟踪评估汇总
- `03_tracking_gt/`：带 gt_id 的 Tracking GT（90 帧、210 个 GT 实例）
- `04_code/`：检测生成、跟踪评估、Mock 链路脚本
- `05_mock_chain/`：Mock Tracker 到 EnclosureTarget 的接口验证数据（仅验证接口格式，不代表真实性能）
- `UAV感知与跟踪模块阶段交付报告.docx`：原始交付报告（未改动）

## 二、审查结论

交付真实、可复现。审查时用交付的 CSV 原样重跑了 `04_code/tracking_success_eval.py`，
报告中总体跟踪成功率 74.76%（157/210 实例）、各场景成功率、检测框数（107/635/16）
全部复现一致。`threshold_results.csv` 中 conf=0.50 对应 P=70.26% / R=72.76%，与项目
此前定案口径一致。评估脚本逻辑规范（逐帧、同类别映射、IoU>=0.30），无硬编码结果。
Mock 结果与真实结果在报告与数据中均明确区分。

## 三、入库时修正的两处 GT 标注问题

原始交付的 `03_tracking_gt/` 存在两处标注问题，入库版本已修正：

1. security 场景 gt_id=1 被两个不同目标复用（一辆 car 和一辆 motorcycle，各 30 帧
   混在同一 gt_id 下）。修正：motorcycle 目标改为独立 gt_id=3。
   该问题导致原始评估把两个目标的轨迹交错计入同一身份，ID Switch 虚高。
2. security / border 两个 GT 文件的 scene 列误写为 "park"。修正为对应场景名。
   （原评估脚本按文件名加载，此项不影响原始结果。）

原始 GT 未入库；如需对照，见杨诗钰原始交付（20260828）。

## 四、原始结果与修正结果对照

| 指标 | 原始交付 | 修正后 | 说明 |
|---|---|---|---|
| 总体跟踪成功率 | 74.76% | 74.76% | 不受影响（逐实例匹配） |
| 总体 ID Switch | 14 | 1 | 原始 14 次中 13 次为 gt_id 复用造成的假象 |
| security GT 目标数 | 2 | 3 | 摩托车目标独立计数 |
| security 逐目标 | — | car1 100%、car2 100%、motorcycle 26.67%（8/30 帧） | 见下 |

修正后 security 场景的真实结论与原报告分析不同：
- 两辆 car 目标 30/30 帧全部跟踪成功，无 ID Switch；
- 摩托车目标仅 8/30 帧成功（26.67%），是该场景成功率偏低的主因——瓶颈在
  小型目标检测召回，而非轨迹身份切换。

`02_tracking/tracking_success_result_fixed.csv` 为原始交付结果（保留存档），
`02_tracking/tracking_success_result_corrected.csv` 为修正 GT 后的评估结果。

**对外口径建议：跟踪成功率 74.76%（210 GT 实例、IoU>=0.30 逐帧匹配）、ID Switch 1。**
报告/PPT 引用时以修正后数字为准。

## 五、复现方法

```bash
cd 04_code
python tracking_success_eval.py \
  --gt-dir ../03_tracking_gt \
  --track-dir ../02_tracking \
  --output /tmp/tracking_success_result.csv
```

脚本已参数化：不传参数时默认使用原作者本机路径（D:\UAV_detection），行为不变。

其余脚本（`generate_gt_aligned_detections.py`、`run_tracker_eval.py`、Mock 系列）
保持交付原样，路径常量为原作者本机绝对路径，仓库内复现需自行调整。

## 六、遗留事项

- 真实 ROS2 topic 联调（Tracker 输出接入 /enclosure_targets）待团队在 ROS2 环境验证；
- border 场景检测仅 16/30 帧有输出，若需提升该场景指标，优先提升前端检测召回；
- 原报告（docx）中 security 分析结论与 ID Switch=14 的表述基于原始 GT，
  引用时以本文件第四节数字为准。
