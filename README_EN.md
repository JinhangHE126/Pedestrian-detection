[🇨🇳 中文](./readme.md) | [🇬🇧 English](./README_EN.md)

# MachineDog — Autonomous Pedestrian Following for Unitree Go2

An embodied AI system that enables the Unitree Go2 quadruped robot to autonomously detect, track, and follow pedestrians. Deploys YOLOv8 + KCF + Kalman Filter on Jetson Nano, with UDP-based robot motion control.

![demo](./2183_1781068026.gif)

---

## System Architecture

```
┌──────────────────┐   Stereo Camera   ┌────────────────────┐   UDP Commands   ┌──────────────────┐
│  Unitree Go2     │ ────────────────▶ │  Jetson Nano (Edge) │ ───────────────▶ │  Unitree Go2     │
│  Stereo Camera   │                   │                     │                  │  Motion Control  │
└──────────────────┘                   │  • YOLOv8 Detection │                  └──────────────────┘
                                        │  • KCF Tracking     │
                                        │  • Re-detection     │
                                        │  • UDP Signaling    │
                                        └────────────────────┘
```

---

## Project Structure

```
Pedestrian-detection/
│
├── Tracking/                          # [AI Perception] Pedestrian detection & tracking
│   ├── Detect.py                      #   Basic YOLOv8 ONNX detector
│   ├── detect1.py                     #   ⭐ Production: YOLOv8 + UDP communication
│   ├── Track.py                       #   KCF tracker + confidence evaluation
│   ├── Kalman.py                      #   Kalman filter for motion prediction
│   ├── TrackingProgram.py             #   Detect-Track-ReDetect main loop
│   └── best.onnx                      #   Pre-trained YOLOv8 model
│
├── go22_ws/                           # [Robot Control] ROS Catkin workspace
│   └── src/
│       ├── UnitreecameraSDK/          #   Unitree stereo camera SDK (C++)
│       │   └── Face_tracking.cpp      #     Gen 1: background subtraction + skin detection
│       │
│       ├── unitree_legged_sdk/        #   Unitree Go2 locomotion SDK
│       │   ├── example_py/            #     Python examples: walk / velocity / torque
│       │   ├── python_wrapper/        #     pybind11 C++ → Python bindings
│       │   └── lib/python/arm64/      #     Pre-compiled .so for Jetson ARM64
│       │
│       └── faceLightSDK_Nano/         #   Facial LED light control
│
├── 2183_1781068026.gif               # Demo GIF
├── readme.md                          # Chinese documentation
└── README_EN.md                       # English documentation
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
            ├── [Pass] Continue → update model → UDP send follow signal
            └── [Fail] Re-detect with YOLOv8 & re-identify target
```

---

## Modules

### AI Perception Layer — `Tracking/`

#### `detect1.py` — Production Detector ⭐

The core module actually deployed on Jetson Nano. Extends `Detect.py` with **UDP Socket communication** to send follow signals to the robot's motion controller when a pedestrian is detected.

#### `Detect.py` — Basic Detector

Uses YOLOv8 ONNX model via OpenCV DNN for pedestrian detection. Supports Letterbox resize, NMS filtering, and automatic target selection (largest bounding box by area).

> **Note**: The included `best.onnx` was trained with limited compute resources and may have suboptimal accuracy. Replace with your own trained model for better results.

#### `Track.py` — KCF Tracker

Implements Kernelized Correlation Filter (KCF) for single-object tracking.

**Features:**
- HOG feature extraction
- Multi-scale adaptive search (bounding box scales with target)
- Dual confidence metrics: **max response** + **APCE**
- Long-term tracking: maintains a history of past N frames (default 30) for dynamic thresholding
- Kalman filter assisted motion prediction

> CN (Color Names) and LBP features are reserved in code but not yet enabled. Feature fusion (HOG + CN) is planned for future improvement.

#### `Kalman.py` — Kalman Filter

8-dimensional state vector `[x, y, w, h, vx, vy, vw, vh]` with constant-velocity motion model. Used to predict target position when the KCF tracker loses confidence.

#### `TrackingProgram.py` — Debug Entry Point

Orchestrates the full detection → tracking → re-detection loop. Reads video files or webcam frames, dispatches to Detector and Tracker, and renders results on desktop.

### Robot Control Layer — `go22_ws/`

#### `unitree_legged_sdk/` — Go2 Locomotion Control

Unitree's official SDK for controlling the robot's gait, posture, and velocity via UDP high-level protocol. Includes a Python wrapper (pybind11) for direct Python-based motion commands.

#### `UnitreecameraSDK/` — Stereo Camera

Unitree stereo depth camera SDK, supporting raw frames, rectified frames, depth maps, and point clouds. Contains `Face_tracking.cpp` — the first-generation approach using MOG2 background subtraction + skin color detection, later superseded by the YOLOv8 pipeline.

#### `faceLightSDK_Nano/` — Facial LED

SDK for controlling the Go2's facial LED array, with custom light effects and sound playback.

---

## Design Rationale

### Business Problem

In robot following and surveillance scenarios, per-frame YOLO detection is computationally expensive and hard to deploy on resource-constrained edge devices. Additionally, short occlusions cause target loss with detection-only approaches, disrupting downstream business logic (e.g., robot path planning).

### Solution Design & Trade-offs

A hybrid "Detect → Track → Re-detect" pipeline was designed:

| Phase | Choice | Rationale |
|-------|--------|-----------|
| Detection | YOLOv8 → ONNX (OpenCV DNN) | High accuracy; ONNX export removes PyTorch runtime dependency for lightweight deployment |
| Tracking | KCF (Kernelized Correlation Filter) | Sub-millisecond per-frame inference; extremely low compute for edge devices |
| Features | HOG | Computationally efficient, robust to illumination changes |
| Target Selection | Largest bounding box | Matches "follow the nearest pedestrian" logic; easily swappable |
| Failure Recovery | Full-frame re-detect + feature matching | No extra ReID model needed; reuses existing KCF features |

> DeepSORT / StrongSORT were not chosen because Hungarian matching and ReID modules add unnecessary overhead for single-target tracking.

### Engineering Challenges & Solutions

**Challenge 1: KCF drift under deformation & occlusion**
- Introduced **APCE** (Average Peak-to-Correlation Energy) as a second confidence metric alongside max response
- Dynamic threshold based on a rolling 30-frame history, adapting to scene variations better than a fixed threshold

**Challenge 2: Model degradation during long-term tracking**
- Incremental model update with a low learning rate (0.012) after each successful frame
- Balances stability (resisting single-frame noise) with adaptability (accommodating gradual appearance changes)

**Challenge 3: Target scale changes causing tracking box mismatch**
- Multi-scale adaptive search: evaluates 0.95~1.05 scale range with 0.05 step
- Applies a suppression factor (0.8) to penalize unreasonable box expansion

**Challenge 4: Kalman filter underperformance**
- Root cause: constant-velocity motion model fails under abrupt acceleration or turning
- Strategy: Kalman serves only as an initial search position hint for KCF; final position relies on KCF response

### Business Impact

- Tracking-mode inference drops from ~tens of ms (per-frame YOLO) to **sub-millisecond**
- Long-term target lock maintained via dual confidence gating + automatic re-detection
- Modular design (Detect / Track / Kalman) enables quick migration to other target types by swapping modules

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

### Robot Control Layer Additional Dependencies

| Component | Description |
|-----------|-------------|
| ROS (Robot Operating System) | Catkin workspace `go22_ws` |
| CMake | C++ SDK compilation |
| Boost / pybind11 | Python bindings |
| Unitree Go2 Firmware | H0.1.7 ≥ v0.1.35 |

---

---

## Technical Evolution

This project includes two generations of detection approaches, demonstrating iterative engineering from traditional CV to deep learning:

| Dimension | Gen 1: `Face_tracking.cpp` | Gen 2: `detect1.py` |
|-----------|---------------------------|---------------------|
| Method | MOG2 background subtraction + skin color detection | YOLOv8 deep learning + KCF tracking |
| Features | Motion foreground + RGB skin threshold | CNN semantic features + HOG texture |
| Robustness | Low (sensitive to lighting, background) | High (end-to-end learned) |
| Real-time | High (traditional CV, no GPU) | Medium (YOLO inference, offset by KCF) |
| Communication | ROS Topic (commented out) | UDP Socket → robot controller |

> The Gen 1 → Gen 2 evolution showcases "validate feasibility first, then optimize accuracy" — an excellent case study for technical decision-making in interviews.

---

## System Integration & Data Flow

### Production Detector (`detect1.py`) Core Logic

```python
# Initialize UDP client
self.udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Pedestrian detected → send signal to robot
if target_pos is not None:
    self.udp_client.sendto(b'1', (self.udp_target_ip, self.udp_target_port))
```

### Robot Motion Control (`example_walk.py`)

```python
# UDP high-level protocol control
udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.161", 8082)
cmd = sdk.HighCmd()
cmd.mode = 2           # continuous walking
cmd.velocity = [-0.2, 0]  # forward speed
cmd.yawSpeed = 0       # turn rate
```

### End-to-End Data Flow

```
Stereo Camera → Jetson Nano captures frames
  → YOLOv8 detects pedestrians → KCF tracks target → confidence check
    → [Detected] UDP sends b'1' → Go2 motion control board
      → Go2 executes follow action (forward/turn/stop)
    → [Lost] Full-frame re-detect → feature matching recovery
```

---

## Hardware Deployment

```
┌─────────────────────────────────────────┐
│         Hardware Topology               │
│                                         │
│  Go2 Stereo Camera ──USB──▶ Jetson Nano │
│                            (Onboard)    │
│                              │          │
│                            UDP (WiFi)   │
│                              │          │
│                              ▼          │
│                       Go2 Control Board │
│                              │          │
│                       Go2 12 Motors     │
│                                         │
│  Network:                               │
│    Jetson ↔ Go2: 192.168.123.x subnet   │
│    Detection signal port: 8900          │
│    Motion control port: 8082            │
└─────────────────────────────────────────┘
```

---

## Parameters

### AI Perception — `Tracking/`

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
