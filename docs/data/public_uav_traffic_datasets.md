# 公开无人机/高位交通数据接入清单

本文档把演示/训练输入切换到可追溯公开数据源。大数据文件只落在本机数据目录，不提交进 Git。

默认数据目录：

```bash
F:/codex-cursor-plugins/vision-recognition/datasets
```

可通过环境变量覆盖：

```bash
export CVTRACK_DATASETS_ROOT=/path/to/datasets
```

## 优先级

| 优先级 | 数据集 | 主要用途 | 获取方式 | 当前动作 |
|---|---|---|---|---|
| P0 | VisDrone DET val/train/test-dev | 无人机视角车辆/行人检测，先跑通检测训练和抽样可视化 | GitHub release 镜像可直接下载 | 先下载 `visdrone_det_val` 做真实样本校验 |
| P0 | VisDrone VID/MOT val | 视频检测、多目标跟踪、轨迹生成 | 官方 Google Drive，val 约 1.49/1.48 GB | 下载前确认带宽，优先作为跟踪训练集 |
| P0 | UAVDT-Benchmark-M | 无人机车辆检测/MOT，道路/路口/高速/收费站 | 官方 Google Drive，research purpose only | 作为车辆跟踪主数据集，需人工确认授权和大包下载 |
| P1 | AU-AIR | 低空交通监控，含 GPS/IMU/速度等无人机传感数据 | 官方 Google Drive，图片 2.2 GB，标注 3.9 MB | 先下 annotations，再下 images |
| P1 | MiTra Data_T1..T9 | 车辆轨迹、封控态势、合流/分流/换道行为 | OPARA 直链，轨迹包 80-514 MB；原视频日志 30-71 GB/包 | 先下载 `mitra_data_t1`，暂不下载视频日志大包 |
| P1 | DRIFT | 4K 无人机交通 OBB、YOLO/ByteTrack、轨迹分析 | HuggingFace `Hj-Lee/The-DRIFT` | 用作轨迹/OBB训练与拥堵传播分析 |
| P2 | UA-DETRAC/CityFlow/VIRAT/SDD/highD/inD/rounD | 固定高位监控、跨摄像头 ReID、轨迹预测补充 | 各官方站点/申请 | 只作补充，不替代无人机主数据 |

## 命令

列出所有已登记数据源：

```bash
python scripts/prepare_public_uav_data.py list
```

下载并抽样校验 VisDrone 检测验证集：

```bash
python scripts/prepare_public_uav_data.py download visdrone_det_val --extract --inspect
```

下载并校验 MiTra 第一组轨迹包：

```bash
python scripts/prepare_public_uav_data.py download mitra_data_t1 --extract --inspect
```

查看需要人工下载的大包命令：

```bash
python scripts/prepare_public_uav_data.py manual uavdt_benchmark_m
python scripts/prepare_public_uav_data.py manual auair_images
python scripts/prepare_public_uav_data.py manual visdrone_mot_val
python scripts/prepare_public_uav_data.py manual drift
```

## 当前已实测数据

| 数据集 | 本地归档 | SHA256 | 解压/检查结果 |
|---|---|---|---|
| VisDrone2019-DET-val | `F:/codex-cursor-plugins/vision-recognition/datasets/visdrone_det_val/archive/visdrone_det_val.zip`，81,638,851 bytes | `abeea063037e5d20398837deb11084e652402a34ddf4f207bdf541a6f2a35ef9` | 548 张图片、548 个标注文件；抽样标注 128 个框 |
| MiTra Data_T1 | `F:/codex-cursor-plugins/vision-recognition/datasets/mitra_data_t1/archive/mitra_data_t1.zip`，84,632,006 bytes | `aca7647c0e8295ac1488e578d9342b7be6c8977327bebb3845025b7096cd47ad` | 7 个 CSV，合计 4,149,069 条轨迹数据行 |

对应 `manifest.json` 保存在各自数据集目录下，作为后续训练/演示取样的真实数据依据。

## 使用边界

- 只有脚本实际下载、解压、计数并写入 `manifest.json` 的数据，才可写成“已获取/已验证”。
- Google Drive 和 HuggingFace 大包如果只登记了 ID 或命令，只能写成“可复现入口已确认”，不能写成已下载。
- 真实数据可作为检测/跟踪/轨迹训练输入，但不自动代表真机飞行、实地部署或地理测绘精度。
