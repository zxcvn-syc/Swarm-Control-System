# 公开无人机交通数据接入证据（2026-08-20）

本次接入目标是把演示和后续训练输入切换到真实公开数据源，避免使用占位视频或模拟样本。大体积数据只保存在本机外部数据目录，不提交到 Git。

## 本机数据目录

```text
F:/codex-cursor-plugins/vision-recognition/datasets
```

## 已下载并验收

| 数据集 | 用途 | 本地归档 | 验收结果 |
|---|---|---|---|
| VisDrone2019-DET-val | 无人机视角目标检测验证 | `visdrone_det_val/archive/visdrone_det_val.zip`，81,638,851 bytes，SHA256 `abeea063037e5d20398837deb11084e652402a34ddf4f207bdf541a6f2a35ef9` | 已解压；548 张图片、548 个标注文件；样例标注 128 个框 |
| MiTra Data_T1 | 车辆轨迹和态势训练 | `mitra_data_t1/archive/mitra_data_t1.zip`，84,632,006 bytes，SHA256 `aca7647c0e8295ac1488e578d9342b7be6c8977327bebb3845025b7096cd47ad` | 已解压；7 个 CSV；合计 4,149,069 条轨迹数据行 |

## 已登记但未声称下载

| 数据集 | 状态 | 后续动作 |
|---|---|---|
| VisDrone2019-MOT-val | 已登记官方 Google Drive 文件 ID | 用于多目标跟踪前需人工下载/授权确认 |
| UAVDT-Benchmark-M | 已登记官方数据入口 | 用于无人机车辆跟踪主训练集前需确认授权和下载大包 |
| AU-AIR annotations/images | 已登记官方 Google Drive 文件 ID | 先下载 annotations，再下载 images |
| DRIFT | 已登记 HuggingFace 加载命令 | 用于 OBB/轨迹/拥堵传播分析前再下载 |

## 可复现命令

```bash
python scripts/prepare_public_uav_data.py list
python scripts/prepare_public_uav_data.py download visdrone_det_val --extract --inspect
python scripts/prepare_public_uav_data.py download mitra_data_t1 --extract --inspect
python scripts/prepare_public_uav_data.py manual uavdt_benchmark_m
python scripts/prepare_public_uav_data.py manual auair_images
python scripts/prepare_public_uav_data.py manual drift
```

## 真实性边界

- “已下载并验收”仅指本机脚本完成下载、解压、计数和 SHA256 记录。
- “已登记”不等于已下载，不能写成训练已完成或数据已全部接入。
- VisDrone 检测集适合先跑检测/抽帧可视化；MiTra Data_T1 适合轨迹预测、拥堵态势、封控决策逻辑训练。
