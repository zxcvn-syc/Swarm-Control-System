杨诗钰_感知跟踪交付 说明
==========================

本文件夹为无人机感知与跟踪任务的交付内容，按以下目录组织：

01_detection/  检测结果
  - threshold_results.csv            置信度阈值筛选结果
  - detections_eval_park.csv         停车场场景检测评估结果
  - detections_eval_security.csv     安防场景检测评估结果
  - detections_eval_border.csv       边境场景检测评估结果

02_tracking/  跟踪结果
  - tracking_eval_park.csv           停车场场景跟踪评估结果
  - tracking_eval_security.csv       安防场景跟踪评估结果
  - tracking_eval_border.csv         边境场景跟踪评估结果
  - tracking_success_result_fixed.csv 跟踪成功率评估结果（修正版）

03_tracking_gt/  跟踪真值（Ground Truth）
  - tracking_gt_park.csv             停车场场景跟踪真值
  - tracking_gt_security.csv         安防场景跟踪真值
  - tracking_gt_border.csv           边境场景跟踪真值

04_code/  代码
  - generate_gt_aligned_detections.py 生成与真值对齐的检测结果
  - run_tracker_eval.py               跟踪评估主脚本
  - tracking_success_eval.py          跟踪成功率评估脚本
  - mock_tracker.py                   模拟跟踪器
  - mock_enclosure_data.py            模拟围栏数据生成
  - mock_publisher.py                 模拟数据发布器

05_mock_chain/  模拟链路数据
  - mock_targets.csv                 模拟目标数据
  - enclosure_targets_mock.csv       模拟围栏目标数据

说明：
- 原始工程位于 D:\UAV_detection，本交付为整理后的关键成果文件。
- 评估文件均以 CSV 格式提供，UTF-8 编码。
