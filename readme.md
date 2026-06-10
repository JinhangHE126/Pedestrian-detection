[🇨🇳 中文](./readme.md) | [🇬🇧 English](./README_EN.md)

# 行人检测

基于 YOLOv8 + KCF + 卡尔曼滤波的长时行人检测与跟踪系统。

![demo](./2183_1781068026.gif)

***

## 项目结构

```
Pedestrian-detection/
├── TrackingProgram.py   # 主程序入口
├── Detect.py            # 行人检测器
├── Track.py             # KCF 跟踪器 + 置信度评估
├── Kalman.py            # 卡尔曼滤波
├── best.onnx            # 预训练模型
└── readme.md
```

***

## 工作流程

```
首帧
    └── YOLOv8 检测所有行人
        └── 选择面积最大的作为目标
            └── 初始化 KCF 跟踪器

后续帧
    └── KCF 跟踪目标
        └── 置信度检查（最大响应值 & APCE）
            ├── [通过] → 继续跟踪，更新模型
            └── [失败] → 重新 YOLOv8 检测，重新匹配目标
```

***

## 模块说明

### `Detect.py` — 行人检测

基于 YOLOv8 ONNX 模型，通过 OpenCV DNN 进行行人检测。支持 Letterbox 缩放、NMS 过滤和自动目标选择（面积最大框）。

> **注意**: 附带的 `best.onnx` 模型训练轮次有限，检测效果一般，建议替换为自训练模型。

### `Track.py` — KCF 跟踪器

基于核相关滤波 (KCF) 的单目标跟踪器。

**特性:**

- HOG 特征提取
- 多尺度自适应搜索（检测框随目标尺寸自动变化）
- 双重置信度指标：**最大响应值** + **APCE**
- 长时跟踪：基于过去 N 帧（默认 30）的统计动态阈值
- 卡尔曼滤波辅助运动预测

> CN 颜色特征和 LBP 纹理特征接口已预留但未启用，后续可融合提升效果。

### `Kalman.py` — 卡尔曼滤波

8 维状态向量 `[x, y, w, h, vx, vy, vw, vh]`，匀速运动模型。在 KCF 跟踪置信度不足时提供位置预测。

### `TrackingProgram.py` — 主程序

串联检测 → 跟踪 → 重检测的完整循环。读取视频帧，调度检测器和跟踪器，渲染结果。

***

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

***

## 参数说明

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

