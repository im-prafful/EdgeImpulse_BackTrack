# 🚀 BackTrack — AI-Powered Wearable Posture Coach

### ESP32-S3 • MPU-9250 • Edge Impulse • Blynk • Wearable Haptics

BackTrack is a neckband-style wearable that detects poor posture in real time using a 9-axis IMU and an Edge Impulse TinyML model deployed on the ESP32-S3. When bad posture is detected, the device sends a gentle vibration nudge to help users subconsciously improve their spine alignment.

Designed for developers, students, remote workers, office professionals, and anyone who spends long seated hours.

---

## 🧭 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Architecture](#hardware-architecture)
- [Repository Structure](#repository-structure)
- [Firmware](#firmware)
- [Python Tools](#python-tools)
- [Edge Impulse Workflow](#edge-impulse-workflow)
- [Mobile App (Blynk)](#mobile-app-blynk)
- [Setup & Installation](#setup--installation)
- [Data Collection](#data-collection)
- [Model Deployment](#model-deployment)
- [Demo](#demo)
- [Future Enhancements](#future-enhancements)
- [License](#license)
- [Author](#author)

---

## 📘 Overview

Backtrack tackles the common issue of “tech-neck” and slouched posture by monitoring neck orientation in real time. Using an MPU-9250 IMU mounted on a discreet neckband, the ESP32-S3 processes motion data and performs on-device inference using an Edge Impulse TinyML model. When unhealthy posture is detected, the device provides haptic feedback and sends status updates to a mobile dashboard via Blynk.

---

## ✨ Features

- **Edge AI Posture Classification** (runs fully offline on ESP32-S3)
- **Real-time IMU Monitoring** using a 9-axis MPU-9250 sensor
- **Haptic Feedback** through a vibration motor
- **Mobile Dashboard** for live posture state, sensitivity control, and battery level
- **Personalized Calibration** to adapt to each user’s natural posture
- **Data Logging & Visualization** tools for modeling and debugging

---

## ⚙️ Hardware Architecture

```
MPU-9250 IMU → ESP32-S3 MCU → Edge Impulse Model (.eim)
                         │
                 Vibration Motor (Haptics)
                         │
                  Blynk Mobile Dashboard
```

---

## 🧱 Repository Structure

```
EdgeImpulse_BackTrack/
│
├── arduino/
│   ├── mpu9250_firmware/
│   │   ├── mpu9250_firmware.ino
│   │   ├── calibration_utils.h
│   │   ├── calibration_utils.cpp
│   │   ├── posture_detection.h
│   │   ├── posture_detection.cpp
│   │   ├── vibration_feedback.h
│   │   ├── vibration_feedback.cpp
│   │   ├── blynk_integration.h
│   │   ├── blynk_integration.cpp
│   │   └── README.md
│   │
│   └── data_logger/
│       ├── imu_data_logger.ino
│       ├── imu_data_parser.h
│       ├── imu_data_parser.cpp
│       └── README.md
│
├── edge_impulse/
│   ├── ei_posture_model.eim
│   ├── model_metadata.json
│   ├── scripts/
│   │   ├── upload_to_edge_impulse.py
│   │   ├── preprocess_data.py
│   │   └── visualize_data.ipynb
│   └── README.md
│
├── src/
│   ├── pyserial_interface.py
│   ├── preprocess_data.py
│   ├── live_plotter.py
│   ├── blynk_interface.py
│   ├── __init__.py
│   ├── requirements.txt
│   └── README.md
│
├── data/
│   ├── raw/
│   ├── processed/
│   └── calibration_samples.csv
│
├── .env.example
├── .gitignore
├── LICENSE
├── CONTRIBUTING.md
└── README.md
```

---

## 🔥 Firmware

Located in:

```
arduino/mpu9250_firmware/
```

Modules include:

- `mpu9250_firmware.ino` — main control loop
- `calibration_utils.*` — baseline posture calibration
- `posture_detection.*` — posture classification logic
- `vibration_feedback.*` — haptic motor patterns
- `blynk_integration.*` — optional WiFi/Blynk sync

The firmware reads IMU data, runs inference, triggers haptics, and streams status to the dashboard.

---

## 🐍 Python Tools

Located in:

```
src/
```

Tools included:

- `pyserial_interface.py` — log IMU data for Edge Impulse
- `preprocess_data.py` — clean & normalize datasets
- `live_plotter.py` — visualize IMU signals
- `blynk_interface.py` — test/dashboard integration
- `requirements.txt` — dependencies list

These tools support your ML workflow and debugging.

---

## 🌿 Edge Impulse Workflow

1. Collect IMU posture data (good, bad, neutral)
2. Upload to Edge Impulse Studio
3. Configure spectral features / neural network
4. Train & validate posture model
5. Export as `.eim` file
6. Copy into `/edge_impulse/`
7. ESP32 loads the model at runtime

---

## 📲 Mobile App (Blynk)

Features:

- Live posture state (Good / Neutral / Slouched)
- Live IMU values
- Vibration sensitivity control
- Battery status
- Push notification alerts
- Calibration button

App communicates via WiFi or BLE through Blynk IoT Cloud.

---

## 🛠 Setup & Installation

### 1. Clone the repository

```
git clone https://github.com/<yourusername>/EdgeImpulse_BackTrack.git
```

### 2. Install dependencies

```
pip install -r src/requirements.txt
```

### 3. Configure environment

```
cp .env.example .env
```

### 4. Flash firmware

Open in Arduino IDE:

```
arduino/mpu9250_firmware/mpu9250_firmware.ino
```

---

## 📡 Data Collection

Run:

```
python src/pyserial_interface.py --port COM3 --baud 115200 --out data/raw/session1.csv
```

Collect multiple sessions (neutral, slouched, forward tilt, etc.).

---

## 🤖 Model Deployment

Export model as `.eim` from Edge Impulse and place it in:

```
edge_impulse/ei_posture_model.eim
```

Device will automatically begin inference.

---

## 🎥 Demo

A typical demonstration includes:

- Wearing the neckband
- Slouch detection
- Vibration feedback
- Blynk dashboard updates
- Edge Impulse model workflow overview

---

## 🔮 Future Enhancements

- BLE-only mode (no WiFi required)
- Dual-IMU advanced spine curvature tracking
- OLED wearable display
- Auto-sleep & battery optimization
- Flutter-based custom mobile app
- Cloud analytics dashboard

---

## 📜 License

MIT License.

---

## 👤 Author

**1. Prafful Mishra**  
AI & Cloud Engineer, TCS

**2. Rohan Mahishi**
Data Scientist, Tredence

Built for Edge Impulse Hackathon 2025
