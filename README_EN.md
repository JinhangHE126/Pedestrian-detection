[🇨🇳 中文](./readme.md) | [🇬🇧 English](./README_EN.md)

# Pedestrian Detection

A long-term pedestrian detection and tracking system based on YOLOv8 + KCF + Kalman Filter.

![demo](./2183_1781068026.gif)

---

## Project Structure

```
Pedestrian-detection/
├── TrackingProgram.py   # Main entry
├── Detect.py            # YOLOv8 pedestrian detector
├── Track.py             # KCF tracker + confidence evaluation
├── Kalman.py            # Kalman filter for motion prediction
├── best.onnx            # Pre-trained YOLOv8 ONNX model
├── readme.md            # Chinese documentation
└── README_EN.md         # English documentation
```

---

## Workflow

```
First Frame
    └── YOLOv8 detects all pedestrians
        └── Select the largest bounding box as target
            └── Initialize KCF tracker

Subsequent Frames
    └── KCF tracks the target
        └── Confidence check (max response & APCE)
            ├── [Pass] Continue tracking & update model
            └── [Fail] Re-detect with YOLOv8 & re-identify target
```

---

## Modules

### `Detect.py` — Pedestrian Detector

Uses YOLOv8 ONNX model via OpenCV DNN for pedestrian detection. Supports Letterbox resize, NMS filtering, and automatic target selection (largest bounding box by area).

> **Note**: The included `best.onnx` was trained with limited compute resources and may have suboptimal accuracy. Replace with your own trained model for better results.

### `Track.py` — KCF Tracker

Implements Kernelized Correlation Filter (KCF) for single-object tracking.

**Features:**
- HOG feature extraction
- Multi-scale adaptive search (bounding box scales with target)
- Dual confidence metrics: **max response** + **APCE** (Average Peak-to-Correlation Energy)
- Long-term tracking: maintains a history of past N frames (default 30) for dynamic thresholding
- Kalman filter assisted motion prediction

> CN (Color Names) and LBP features are reserved in code but not yet enabled. Feature fusion (HOG + CN) is planned for future improvement.

### `Kalman.py` — Kalman Filter

8-dimensional state vector `[x, y, w, h, vx, vy, vw, vh]` with constant-velocity motion model. Used to predict target position when the KCF tracker loses confidence.

### `TrackingProgram.py` — Main Entry

Orchestrates the full detection → tracking → re-detection loop. Reads video frames, delegates to Detector and Tracker, and renders results.

---

## Quick Start

1. **Install dependencies:**

   ```bash
   pip install numpy opencv-python
   ```

2. **Prepare your model:**

   Place a YOLOv8 ONNX model file in the project directory (or use the included `best.onnx`).

3. **Configure and run:**

   Edit `TrackingProgram.py` to set:

   ```python
   video_path = "path/to/your/video.mp4"   # or 0 for webcam
   model_path = "./best.onnx"
   model_input_size = 416
   debug = False
   ```

   Then run:

   ```bash
   python TrackingProgram.py
   ```

4. **Controls:**

   | Key | Action |
   |-----|--------|
   | `Space` | Pause / Resume |
   | `Esc`   | Exit |

---

## Dependencies

| Package | Version |
|---------|---------|
| Python  | 3.9.17  |
| NumPy   | 1.25.2  |
| OpenCV  | 4.6.0   |

> Tested with Anaconda YOLO environment.

---

## Parameters

### `TrackingProgram.py`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `video_path` | `None` | Video file path (set to `0` for webcam) |
| `model_path` | `None` | Path to YOLOv8 ONNX model |
| `model_input_size` | `416` | Model input resolution (416×416) |
| `debug` | `False` | Enable debug visualization |

### `Detect.py` — `Detector`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_path` | — | Path to ONNX model |
| `input_size` | — | Model input size |
| `conf_thres` | `0.3` | Confidence threshold (boxes below this are discarded) |
| `class_desc` | `["pedestrian", "partially visible"]` | Class names |

### `Track.py` — `Tracker`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `roi_scale_weight` | `0.8` | Scale penalty factor (>1 suppressed) |
| `min_scale` | `0.95` | Minimum scale ratio |
| `max_scale` | `1.05` | Maximum scale ratio |
| `scale_step` | `0.05` | Scale search step |
| `response_thres` | `0.5` | Max-response confidence coefficient |
| `apce_thres` | `0.5` | APCE confidence coefficient |
| `keep_hist_cnt` | `30` | Number of past frames for averaging |
| `learning_rate` | `0.012` | KCF model update rate |

---

## Limitations & TODOs

- [ ] The bundled `best.onnx` model was trained with limited epochs — **replace with a better model**
- [ ] CN (Color Names) feature extraction is not yet integrated — **fusing HOG + CN** would improve accuracy
- [ ] Kalman filter assistance needs tuning — current effect is suboptimal
- [ ] Currently selects the largest pedestrian as target; could be extended to **multi-target tracking**
- [ ] Add camera (webcam) mode as a configurable option
