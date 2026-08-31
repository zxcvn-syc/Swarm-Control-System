# 真机插机连接与一键会话

> 适用范围：一架 PX4 飞控、ROS 2 Humble、MAVROS 和本项目的单机 `uav0` 操作台。
> 该入口只负责连接飞控和启动锁定的浏览器操作台。它不会自动解锁、切换 Offboard、上传任务、起飞、降落、返航或启动路径控制 bridge。

## 一次性配置

飞控必须先以 USB 或数传 USB 适配器直通给 Linux/VM。Windows 主机不能同时占用同一个串口。
在 VMware 中选择 **虚拟机 -> 可移动设备 -> 对应 Pixhawk/FTDI/数传设备 -> 连接**；设备应从
Windows 断开并显示在虚拟机中。台架阶段默认卸桨。

插入设备后执行：

```bash
cd ~/Swarm-Control-System-operator-console
./scripts/real_uav_session.sh --discover
```

从输出中选择 `/dev/serial/by-id/...` 的稳定路径。只有在该目录不存在时才使用 `/dev/ttyACM0` 或 `/dev/ttyUSB0`。在 QGroundControl/PX4 中确认连接到该物理端口的 MAVLink 实例、系统 ID 与**实际波特率**；不能根据设备类型猜测波特率。

安装并填写本机配置。填写完成后该文件包含真实串口信息，必须保持 `root:root` 和 `0600`，不得提交到 Git：

```bash
sudo install -d -m 0750 /etc/swarm-control
sudo install -m 0600 \
  config/real_uav_connection.template.env \
  /etc/swarm-control/real_uav_connection.env
sudoedit /etc/swarm-control/real_uav_connection.env
```

至少填写：

- `FCU_DEVICE`：稳定的 `/dev/serial/by-id/...` 路径；
- `FCU_BAUD`：PX4 对应 MAVLink 实例的已确认波特率；
- `ROS_DOMAIN_ID`：真机专用域，不能与 SITL 或回放共享；
- `MAVROS_NAMESPACE`：默认 `uav0/mavros`，必须与本项目的话题和操作台一致。

PX4 的 MAVLink 端口配置属于机型、固件和接线相关设置。MAVROS 与 QGroundControl 不得同时打开同一个 USB/串口；如需 QGC，使用独立的遥测链路或在启动 MAVROS 前退出 QGC。

## 每次插机启动

先连接电源、RC 和飞控，完成现场的物理安全检查。插入飞控或数传 USB，确认设备出现在
`--discover` 输出后，从仓库根目录运行：

```bash
./scripts/real_uav_session.sh --monitor
```

脚本会执行以下无动作步骤：

1. 校验配置、设备路径和 ROS workspace；
2. 启动命名空间为 `/uav0/mavros` 的 MAVROS；
3. 最多等待 40 秒，要求 `/uav0/mavros/state` 明确报告 `connected: true`；
4. 启动默认 `LOCKED`、保持请求为真的安全监督器与只读浏览器操作台；
5. 将 MAVROS 和面板日志写到 `~/flight_evidence/<日期>/session-<时间>/`。

任一检查失败时，脚本终止并关闭它启动的 MAVROS；先修复电缆、VM USB 直通、PX4 MAVLink 端口或波特率，不能用反复重试替代排障。

Windows 上保留已有 SSH 隧道后，浏览器访问 `http://127.0.0.1:18080`。若端口变更，按
`DASHBOARD_PORT` 同步修改隧道。脚本终端必须保持运行；按 `Ctrl-C` 会结束本次 MAVROS 与操作台会话。

## 显式受控会话

只有在[真机验证准备与放行手册](real_uav_flight_readiness.md)的 `bench`、`perception`、`decision` 和 `flight` 证据完成、飞行负责人放行后，才可启动：

```bash
./scripts/real_uav_session.sh --controls
```

此模式生成权限 `0600` 的单次 `control-token`，并将路径和审计日志保存在本次 session 目录。仅在本机终端读取 token 后填入浏览器；不要发送、截图或保存在 shell 历史、Git 和报告中。控制面板仍只提供带确认短语和审计的 `ARM`、`DISARM`、`POSCTL`、`ALTCTL` 和受安全门约束的 `OFFBOARD` 请求。其完整按钮条件见[真机飞手操作台与接口手册](real_uav_operator_interface.md)。

`--controls` 不会启动 `px4_offboard_bridge`。路径控制只能在系留/防护网阶段、经飞行负责人单独放行后，按接口手册的 bridge 命令启动，且 `auto_arm:=false` 必须保持不变。

## 排障

| 症状 | 首先检查 | 不可做的事 |
| --- | --- | --- |
| `--discover` 没有设备 | VMware USB 直通、线缆、Windows 是否占用 | 手工编造 `/dev/ttyACM0` 路径。 |
| MAVROS 未 `connected: true` | `FCU_DEVICE`、波特率、PX4 MAVLink 实例、系统 ID | 直接打开 Offboard 或改变失效保护参数。 |
| 页面没有 MAVROS 状态 | `ROS_DOMAIN_ID=71`、`MAVROS_NAMESPACE=uav0/mavros`、SSH 隧道端口 | 把浏览器控制端口开放到 LAN。 |
| 受控按钮不可用 | 完成预检，确认安全门、目标锁定、审计日志和 token | 用 HTTP 脚本循环请求或绕过确认短语。 |

参考：PX4 [Companion Computer](https://docs.px4.io/main/en/companion_computer/pixhawk.html) 与 MAVROS [ROS 2 Humble 文档](https://docs.ros.org/en/humble/p/mavros/)。
