# 具身智能空地协同搜索救援赛 — 参赛Demo

## 项目简介

本Demo实现了一个基于大模型驱动的"无人机侦察搜索 + 无人车运输救援"空地协同系统。选手通过自然语言指令控制无人机(UAV)和无人车(UGV)在模拟巷战环境中完成五个任务：侦察、同步、运输、验证、返航。

**核心架构**：大模型解析自然语言 → ROS指令分发 → 无人机/无人车自主执行 → 多模态视觉反馈

## 目录结构

```
demo/
├── README.md                      # 本文档
├── start_simulation.sh            # 一键启动仿真环境
├── llm_commander.py               # 大模型交互节点 (FM9G4B-V)
├── mission_executor.py            # 任务调度执行节点
├── FM9G4B-V -> ../../model/...    # 大模型目录(软链接)
├── px4_ns.launch                  # ROS launch文件
├── rostrans.param                 # ROS传输参数
├── kill_ros_pid.sh                # 清理ROS进程
├── udp_port_free.sh               # 释放UDP端口
├── MulticopterNOpx4.so            # 无人机综合模型
│
├── prompts/
│   └── mission_knowledge.txt      # 大模型知识库 (选手修改此文件)
│
├── uav/
│   ├── uav_recon.py               # 无人机: 侦察建图/坐标转换/观察飞行/返航
│   └── CreateScene.py             # 仿真场景初始化
│
├── ugv/
│   ├── ugv_navigation.py          # 无人车: 自主导航/物资释放/返航
│   ├── movebase.launch            # move_base + AMCL 启动文件
│   └── movebase_params/           # 导航参数
│       ├── costmap_common_params.yaml
│       ├── global_costmap_params.yaml
│       ├── local_costmap_params.yaml
│       └── dwa_local_planner_params.yaml
│
└── models/
    ├── best.pt                    # YOLO模型 (小车+人员识别)
    ├── yolov8n.pt                 # YOLOv8n通用模型
    └── bestv1.pgm                 # 参考地图 (建图高度层选择)
```

> 注: 传感器数据转换(PointCloud2→LaserScan、里程计→TF、cmd_vel转发)已内置在 `ugv_navigation.py` 的 `UGVController` 类中，无需额外启动脚本。

## 外部依赖

| 依赖 | 路径 | 说明 |
|------|------|------|
| RflySim仿真平台 | `/home/ubuntu/PX4PSP/` | CopterSim, Firmware, RflySimUE5等 |
| ROS工作空间 | `/root/gpufree-data/ws_livox/` | FAST-LIO建图 + livox_ros_driver |
| 大模型 | `/root/gpufree-data/model/FM9G4B-V/` | 九格多模态大模型 |
| RosTrans | `/opt/rostrans` | ROS1-ROS2桥接 |

## 环境要求

| 组件 | 版本/说明 |
|------|-----------|
| 操作系统 | Ubuntu 20.04 |
| ROS | Noetic (full desktop) |
| Python | 3.8+ |
| CUDA | 11.0+ (需GPU) |
| Python包 | torch, transformers, ultralytics, open3d, opencv-python, pillow, cv_bridge |
| ROS包 | ros-noetic-move-base, ros-noetic-amcl, ros-noetic-map-server |

## 快速开始

### 1. 启动仿真环境

```bash
cd ~/PX4PSP/demo
bash start_simulation.sh
```

此脚本依次启动: RflySim3D(UE5引擎) → CopterSim(飞控仿真) → PX4 SITL → roscore/mavros/RosTrans → CreateScene(场景摆放)

> [截图: 仿真启动后的RflySim3D窗口和各xterm终端]

### 2. 启动任务调度节点

新终端:
```bash
cd ~/PX4PSP/demo
python3 mission_executor.py
```

### 3. 启动大模型交互节点

新终端:
```bash
cd ~/PX4PSP/demo
python3 llm_commander.py
```

看到 `>>` 提示符后输入自然语言指令即可。

> [截图: 两个终端分别运行 mission_executor / llm_commander]

## 五任务流程

| 任务 | 自然语言示例 | 指令名 | 评分 |
|------|-------------|--------|------|
| 1. 侦察 | "无人机去冲突区域侦察" | `uav_reconnaissance_mission` | 10分 |
| 2. 同步 | "报告侦察结果并同步给无人车" | `result_sync` | 20分 |
| 3. 运输 | "无人车送药品到救援点" | `car_navigation` | 10分 |
| 4. 验证 | "检测救援物品是否送达" | `thing_detect` | 20分 |
| 5. 返航 | "所有设备返回营区" | `back_home` | 20分 |

**一次性执行全部**: 输入"执行全部任务"或"依次完成所有任务"

> [截图: llm_commander终端中大模型输出指令]

### 各任务详细说明

**任务1 — 无人机侦察** (`uav/uav_recon.py` → `run_reconnaissance_task`)
- 无人机按预设航点飞行，同时运行FAST-LIO进行3D点云建图
- 可选开启YOLOv8实时目标识别
- 航点定义在 `mission_executor.py` 的 `WAYPOINTS` 列表中

**任务2 — 同步结果** (`uav/uav_recon.py` → `convert_pcd_to_2d_map`)
- 将3D点云转换为2D占据栅格地图 (`height_maps_final/processed_map.pgm`)
- YOLO检测救援人员和无人车位置
- 自动完成MAP→UAV坐标系转换
- 输出 `targets.json` (救援目标坐标，含MAP和UAV两套坐标)

> [截图: 生成的二维导航地图]

**任务3 — 无人车运输** (`ugv/ugv_navigation.py` → `run_ugv_mission`)
- 启动 move_base + AMCL 导航栈
- UGVController内置传感器数据转换，无需额外节点
- 读取 `targets.json` 目标坐标，自主规划路径
- 到达后通过UE4接口释放医疗物资

> [截图: rviz中无人车导航路径]

**任务4 — 视觉验证** (`mission_executor.py` + `llm_commander.py`)
- 无人机飞至救援人员上方悬停
- 拍摄下视摄像头画面 (`/rflysim/sensor3/img_rgb`)
- base64编码通过 `/vision_query` 话题发送
- FM9G4B-V多模态大模型分析，判断物资是否送达
- 结果通过 `/vision_result` 返回

**任务5 — 全员返航**
- 无人机 (`uav_recon.py` → `return_to_home`): 爬升→水平→下降→自动降落上锁
- 无人车 (`ugv_navigation.py` → `navigate_back_to_start`): 复用导航栈返回起点

## 核心ROS话题

| 话题 | 方向 | 说明 |
|------|------|------|
| `/mission_command` | LLM→Executor | 大模型解析出的指令 |
| `/mission_feedback` | Executor→LLM | 任务执行状态反馈 |
| `/vision_query` | Executor→LLM | 视觉查询(base64图片+问题) |
| `/vision_result` | LLM→Executor | 多模态分析结果 |
| `/mavros/local_position/pose` | UAV | 无人机当前位姿(NED) |
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | →UAV | 无人机速度控制 |
| `/ugv/mavros/state` | UGV | 无人车飞控状态 |
| `/rflysim/sensor3/img_rgb` | UAV | 无人机下视摄像头 |
| `/rflysim/sensor0/vehicle_lidar` | UGV | 无人车激光雷达(PointCloud2) |

## 大模型知识库

`prompts/mission_knowledge.txt` 定义大模型系统提示词。选手需自行优化:
- **指令映射**: 自然语言 → 5个标准指令名
- **任务背景**: 军事救援场景
- **响应格式**: LLM输出 `**command xxx**` 格式

## 自定义与调试

### 修改无人机航点

编辑 `mission_executor.py` 中的 `WAYPOINTS`:
```python
WAYPOINTS = [
    [0, 0, 1.7],      # [x, y, 高度(m)]
    [4.1, 0.2, 1.7],
    ...
]
```

### 调整导航参数

编辑 `ugv/movebase_params/dwa_local_planner_params.yaml`:
- `max_vel_x`: 最高线速度 (默认0.6)
- `occdist_scale`: 避障权重 (默认1.2)
- `inflation_radius`: 障碍物膨胀半径 (costmap_common_params.yaml, 默认0.35)

### 校准UAV坐标系

编辑 `uav/uav_recon.py` 中的转换参数:
```python
MAP_TO_UAV_CX = 3.2   # UAV_x = -MAP_y + CX
MAP_TO_UAV_CY = 7.9   # UAV_y = MAP_x + CY
```

### 调试任务4视觉验证

测试后检查:
- `/tmp/mission_vision_debug.jpg` — 任务端原始画面
- `/tmp/mission_vision_debug_received.jpg` — 大模型端收到画面
- `/tmp/vision_debug.log` — 查询和模型回复

## 注意事项

1. **执行顺序**: 任务有依赖关系(1→2→3→4→5)，不可跳跃
2. **坐标文件**: 任务2生成`targets.json`，任务3和4依赖此文件
3. **传感器桥接**: 无需额外启动，UGVController内置全部转换
4. **大模型终端**: 保持`llm_commander.py`运行，它同时处理指令和视觉查询
5. **仿真清理**: 异常退出后运行 `bash kill_ros_pid.sh`

## 文件说明

| 文件 | 选手修改? | 内容 |
|------|----------|------|
| `prompts/mission_knowledge.txt` | **是** | RAG知识库,控制指令解析 |
| `mission_executor.py` | 可 | 航点、任务逻辑 |
| `uav/uav_recon.py` | 可 | 建图参数、坐标系校准 |
| `ugv/ugv_navigation.py` | 可 | 导航策略、超时参数 |
| `ugv/movebase_params/*.yaml` | 可 | 规划器、代价地图 |
| `llm_commander.py` | 否 | 大模型加载与通信 |
| `start_simulation.sh` | 否 | 仿真启动 |
