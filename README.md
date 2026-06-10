[🇨🇳 中文](./readme.md) | [🇬🇧 English](./README_EN.md)

# MachineDog — 宇树 Go2 机器狗自主行人跟随系统

基于 YOLOv8 + KCF + 卡尔曼滤波的 AI 感知模块，部署于 Jetson Nano，通过 UDP 控制宇树 (Unitree) Go2 四足机器狗实现自主行人检测、跟踪与跟随。

![demo](./2183_1781068026.gif)

---

## 系统架构

```
┌──────────────────┐     双目相机      ┌────────────────────┐     UDP 运动指令    ┌──────────────────┐
│  Unitree Go2     │ ───────────────▶ │  Jetson Nano (机载)  │ ─────────────────▶ │  Unitree Go2     │
│  双目深度相机      │                 │                     │                   │  运动控制板       │
└──────────────────┘                 │  • YOLOv8 行人检测   │                   └──────────────────┘
                                      │  • KCF 实时跟踪      │
                                      │  • 丢失重检测恢复     │
                                      │  • UDP 发送跟随信号  │
                                      └────────────────────┘
```

---

## 项目结构

```
Pedestrian-detection/
│
├── Tracking/                          # 【AI 感知层】行人检测与跟踪
│   ├── Detect.py                      #   基础版 YOLOv8 ONNX 检测器
│   ├── detect1.py                     #   ⭐ 部署版：YOLOv8 + UDP 通信
│   ├── Track.py                       #   KCF 核相关滤波跟踪器 + 置信度评估
│   ├── Kalman.py                      #   卡尔曼滤波运动预测
│   ├── TrackingProgram.py             #   检测-跟踪-重检测主循环
│   └── best.onnx                      #   YOLOv8 预训练模型
│
├── go22_ws/                           # 【机器人控制层】ROS Catkin 工作空间
│   └── src/
│       ├── UnitreecameraSDK/          #   宇树双目相机 SDK (C++)
│       │   └── Face_tracking.cpp      #     早期方案：背景减除 + 肤色检测
│       │
│       ├── unitree_legged_sdk/        #   宇树 Go2 运动控制 SDK
│       │   ├── example_py/            #     Python 示例：walk / velocity / torque
│       │   ├── python_wrapper/        #     pybind11 C++ → Python 绑定
│       │   └── lib/python/arm64/      #     Jetson ARM64 预编译 .so
│       │
│       └── faceLightSDK_Nano/         #   面部 LED 灯效控制
│
├── 2183_1781068026.gif               # 项目演示 GIF
├── readme.md                          # 中文文档
└── README_EN.md                       # 英文文档
```

---

## 工作流程

```
首帧
    └── YOLOv8 检测所有行人
        └── 选择面积最大的作为目标
            └── 初始化 KCF 跟踪器

后续帧
    └── KCF 跟踪目标
        └── 置信度检查（最大响应值 & APCE）
            ├── [通过] → 继续跟踪，更新模型 → UDP 发送跟随信号
            └── [失败] → 重新 YOLOv8 检测，重新匹配目标
```

---

## 模块说明

### AI 感知层 — `Tracking/`

#### `detect1.py` — 部署版行人检测器 ⭐

实际部署在 Jetson Nano 上的核心检测模块。与 `Detect.py` 基础版相比，新增 **UDP Socket 通信**，检测到行人时向机器狗控制板发送信号。

#### `Detect.py` — 基础版行人检测器

基于 YOLOv8 ONNX 模型，通过 OpenCV DNN 进行行人检测。支持 Letterbox 缩放、NMS 过滤和自动目标选择（面积最大框）。

> **注意**: 附带的 `best.onnx` 模型训练轮次有限，检测效果一般，建议替换为自训练模型。

#### `Track.py` — KCF 跟踪器

基于核相关滤波 (KCF) 的单目标跟踪器。

**特性:**
- HOG 特征提取
- 多尺度自适应搜索（检测框随目标尺寸自动变化）
- 双重置信度指标：**最大响应值** + **APCE**
- 长时跟踪：基于过去 N 帧（默认 30）的统计动态阈值
- 卡尔曼滤波辅助运动预测

> CN 颜色特征和 LBP 纹理特征接口已预留但未启用，后续可融合提升效果。

#### `Kalman.py` — 卡尔曼滤波

8 维状态向量 `[x, y, w, h, vx, vy, vw, vh]`，匀速运动模型。在 KCF 跟踪置信度不足时提供位置预测。

#### `TrackingProgram.py` — 调试主程序

串联检测 → 跟踪 → 重检测的完整循环。读取视频文件或摄像头帧，调度检测器和跟踪器，在桌面环境渲染结果（用于调试和演示）。

### 机器人控制层 — `go22_ws/`

#### `unitree_legged_sdk/` — 宇树 Go2 运动控制

宇树官方 SDK，通过 UDP 高层协议控制机器狗的步态、姿态、速度等。包含 Python wrapper（基于 pybind11），可直接从 Python 发送运动指令。

#### `UnitreecameraSDK/` — 双目相机

宇树双目深度相机 SDK，支持获取原始帧、校正帧、深度图和点云。其中的 `Face_tracking.cpp` 是第一代方案：基于 MOG2 背景减除 + 肤色检测的行人检测，后续被 YOLOv8 方案替代。

#### `faceLightSDK_Nano/` — 面部 LED

控制 Go2 面部 LED 灯阵的 SDK，支持自定义灯效和音效播放。

---

## 设计思路

### 业务痛点

在机器人跟随、安防监控等场景中，逐帧 YOLO 检测计算开销大、实时性差，难以部署到资源受限的边缘设备。同时目标短暂遮挡后容易丢失，无法满足长时稳定跟踪的业务需求。

### 方案选型与理由

设计「检测初始化 + KCF 跟踪 + 丢失重检测」的混合架构：

| 阶段 | 选型 | 理由 |
|------|------|------|
| 检测 | YOLOv8 → ONNX (OpenCV DNN) | 精度高，ONNX 导出脱离 PyTorch 依赖，可轻量部署 |
| 跟踪 | KCF (核相关滤波) | 单帧推理亚毫秒级，算力消耗极低，适合边缘设备 |
| 特征 | HOG | 计算效率高，对光照变化鲁棒 |
| 选目标 | 面积最大 | 适配「跟随最近行人」的业务规则，可替换为其他策略 |
| 丢失恢复 | 全图重检测 + 特征比对 | 无需复杂的 ReID 模型，利用已有 KCF 特征做匹配 |

> 不选 DeepSORT / StrongSORT 的原因：本项目为单目标跟踪，匈牙利匹配和 ReID 模块会引入不必要的计算开销。

### 关键难点与解决方案

**难点 1：KCF 在目标形变、遮挡时易漂移**
- 引入 **APCE**（Average Peak-to-Correlation Energy）作为辅助置信度，与 max response 组成双重判据
- 基于过去 30 帧的历史均值做动态阈值，比固定阈值更能适应场景变化

**难点 2：长时间跟踪导致模型退化**
- 每次跟踪成功以学习率 0.012 渐进更新模型参数和特征
- 平衡稳定性与适应性，避免单帧噪声污染模型

**难点 3：目标远近变化导致跟踪框不匹配**
- 多尺度自适应搜索：在 0.95~1.05 范围内以 0.05 步长搜索最优尺度
- 对放大尺度施加 0.8 的抑制系数，防止跟踪框不合理膨胀

**难点 4：卡尔曼滤波辅助效果不佳**
- 原因：匀速运动模型在目标突然变速/转向时预测偏差大
- 策略：卡尔曼仅作为 KCF 搜索的初始位置参考，不直接使用其预测结果

### 业务效果

- 跟踪模式下单帧推理耗时从 YOLO 逐帧检测的 ~几十ms 降至 **亚毫秒级**
- 长时跟踪通过置信度判据 + 丢失重检测实现了持续锁定
- 模块化解耦设计（Detect / Track / Kalman），可快速替换检测器或跟踪器迁移至其他场景

---

## 快速开始

1. **安装依赖:**
   ```bash
   pip install numpy opencv-python
   ```
2. **准备模型:**

   将 YOLOv8 的 ONNX 模型文件放入项目目录（或使用附带的 `best.onnx`）。
3. **配置并运行:**

   修改 `TrackingProgram.py` 中的配置:
   ```python
   video_path = "path/to/your/video.mp4"   # 视频路径，设为 0 使用摄像头
   model_path = "./best.onnx"
   model_input_size = 416
   debug = False
   ```
   然后运行:
   ```bash
   python TrackingProgram.py
   ```
4. **快捷键:**
   | 按键    | 功能    |
   | ----- | ----- |
   | `空格`  | 暂停/继续 |
   | `Esc` | 退出    |

***

## 依赖环境

| 包      | 版本     |
| ------ | ------ |
| Python | 3.9.17 |
| NumPy  | 1.25.2 |
| OpenCV | 4.6.0  |

> 基于 Anaconda YOLO 环境测试。

### 机器人控制层额外依赖

| 组件 | 说明 |
|------|------|
| ROS (Robot Operating System) | Catkin 工作空间 `go22_ws` |
| CMake | 编译 C++ SDK |
| Boost / pybind11 | Python 绑定 |
| Unitree Go2 固件 | H0.1.7 ≥ v0.1.35 |

***

---

## 技术演进

本项目包含两代检测方案，体现了从传统 CV 到深度学习的工程迭代：

| 维度 | 方案 A: `Face_tracking.cpp` | 方案 B: `detect1.py` |
|------|---------------------------|---------------------|
| 方法 | MOG2 背景减除 + 肤色检测 | YOLOv8 深度学习 + KCF 跟踪 |
| 特征 | 运动前景 + RGB 肤色阈值 | CNN 语义特征 + HOG 纹理 |
| 鲁棒性 | 低（易受光照、背景干扰） | 高（端到端学习） |
| 实时性 | 高（传统 CV，无需 GPU） | 中（YOLO 推理开销，KCF 可弥补） |
| 通信 | ROS Topic (注释掉) | UDP Socket → 机器狗 |

> 从方案 A→B 的演进展示了「先验证可行性，再优化精度」的工程实践，在面试中可作为技术选型权衡的典型案例。

---

## 系统集成与数据流

### 部署版检测器 (`detect1.py`) 核心逻辑

```python
# 初始化 UDP 客户端
self.udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 检测到人 → 发送信号到机器狗
if target_pos is not None:
    self.udp_client.sendto(b'1', (self.udp_target_ip, self.udp_target_port))
```

### 机器狗运动控制 (`example_walk.py`)

```python
# UDP 高层协议控制
udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.161", 8082)
cmd = sdk.HighCmd()
cmd.mode = 2         # 连续行走
cmd.velocity = [-0.2, 0]  # 前进速度
cmd.yawSpeed = 0     # 转向
```

### 完整数据流

```
双目相机 → Jetson Nano 获取图像
  → YOLOv8 检测行人 → KCF 跟踪目标 → 置信度检查
    → [检测到人] UDP 发送 b'1' → Go2 运动控制板
      → Go2 执行跟随动作（前进/转向/停止）
    → [丢失目标] 全图重检测 → 特征匹配恢复
```

---

## 硬件部署

```
┌─────────────────────────────────────────┐
│            硬件拓扑                       │
│                                         │
│  Go2 双目相机 ──USB──▶ Jetson Nano       │
│                         (机载计算)        │
│                           │              │
│                          UDP (无线)       │
│                           │              │
│                           ▼              │
│                      Go2 运动控制板       │
│                           │              │
│                      Go2 12 个电机        │
│                                         │
│  网络:                                    │
│    Jetson ↔ Go2: 192.168.123.x 网段       │
│    UDP 检测信号端口: 8900                 │
│    UDP 运动控制端口: 8082                 │
└─────────────────────────────────────────┘
```

---

## 参数说明

### AI 感知层 `Tracking/`

### `TrackingProgram.py`

| 参数                 | 默认值     | 说明                |
| ------------------ | ------- | ----------------- |
| `video_path`       | `None`  | 视频路径，设为 `0` 使用摄像头 |
| `model_path`       | `None`  | ONNX 模型文件路径       |
| `model_input_size` | `416`   | 模型输入尺寸（416×416）   |
| `debug`            | `False` | 调试可视化开关           |

### `Detect.py` — `Detector`

| 参数           | 默认值                                   | 说明                 |
| ------------ | ------------------------------------- | ------------------ |
| `model_path` | —                                     | ONNX 模型路径          |
| `input_size` | —                                     | 模型输入尺寸             |
| `conf_thres` | `0.3`                                 | 置信度阈值（低于该值的检测框被丢弃） |
| `class_desc` | `["pedestrian", "partially visible"]` | 类别名称               |

### `Track.py` — `Tracker`

| 参数                 | 默认值     | 说明               |
| ------------------ | ------- | ---------------- |
| `roi_scale_weight` | `0.8`   | 尺度增益抑制系数（>1 时抑制） |
| `min_scale`        | `0.95`  | 最小缩放比例           |
| `max_scale`        | `1.05`  | 最大缩放比例           |
| `scale_step`       | `0.05`  | 尺度搜索步长           |
| `response_thres`   | `0.5`   | 最大响应值阈值系数        |
| `apce_thres`       | `0.5`   | APCE 阈值系数        |
| `keep_hist_cnt`    | `30`    | 历史帧统计数量          |
| `learning_rate`    | `0.012` | KCF 模型更新率        |

***

<br />

