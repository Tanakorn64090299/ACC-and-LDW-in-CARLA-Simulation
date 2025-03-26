# 🚗 ADAS Simulator using CARLA: ACC + LDW System

> **The Application of CARLA Simulator for Safety-Assisted Driving Design**  
> Final Senior Project - Burapha University (2024)

## 📌 Project Overview

This project explores the use of **CARLA**, an open-source autonomous driving simulator, to develop and evaluate **Advanced Driver Assistance Systems (ADAS)**. The focus is on two key systems:

- **Adaptive Cruise Control (ACC)** – maintains safe distance using radar and PID control.
- **Lane Departure Warning (LDW)** – detects unintentional lane departure using camera-based lane detection and alerts.

The system is integrated and visualized in CARLA via a unified Python interface with live HUD, logging, and multi-modal driving support.

---

## 🧠 Key Features

### ✅ Adaptive Cruise Control (ACC)
- Utilizes `sensor.other.radar` to detect lead vehicle.
- PID controller adjusts throttle/brake to maintain safe distance.
- Emergency braking and smooth acceleration logic.
- Real-time logging and performance visualization.

### ✅ Lane Departure Warning (LDW)
- Detects lane lines using camera and computer vision (Canny, perspective transform).
- Warns when the vehicle deviates from lane without signal.
- Visual + audio alert with side-screen red bars and "beep.wav".

### ✅ Unified ADAS Demo
- Combined LDW + ACC system in `ADAS(ACC+LDW).py`.
- Supports `MANUAL`, `ACC`, `AUTOPILOT`, and `FULL_AUTOPILOT` modes.
- Visual HUD shows speed, time, ACC status, lane status.
- Real-time logging with CSV + automatic plotting (PNG/PDF).

---

## 📁 File Structure

📦 ADAS_CARLA_Project/ │ ├── ADAS(ACC+LDW).py # Main script: LDW + ACC + HUD integration ├── acc_metrics_logger.py # Logging ACC data to CSV & plot to PNG/PDF ├── T_sensor_manager.py # Radar setup and smoothing ├── T_acc_control.py # ACC Controller with PID logic │

---
## 🎬 Demo Videos

### 🚗 1. ADAS Simulation: ACC + LDW + HUD
[![Watch the demo](https://img.youtube.com/vi/XGa_XzYuVyA/maxresdefault.jpg)](https://youtu.be/XGa_XzYuVyA)

---

### 📈 2. ACC Logging & Performance Analysis
[![Watch the demo](https://img.youtube.com/vi/IOnO5zjLFlw/maxresdefault.jpg)](https://youtu.be/IOnO5zjLFlw)

---

### 🧠 3. Lane Departure Warning (LDW) Visual Alert
[![Watch the demo](https://img.youtube.com/vi/su1HLTpOqd8/maxresdefault.jpg)](https://youtu.be/su1HLTpOqd8)

---
yaml
Copy
Edit

---

## 🧪 How to Run the Simulation

### 🛠️ Prerequisites
- CARLA Simulator (0.9.13+ recommended)
- Python 3.8–3.10
- Required Python libraries:
  ```bash
  pip install pygame numpy opencv-python matplotlib pandas
▶️ Start Simulation
Run CARLA server:

bash
Copy
Edit
./CarlaUE4.sh -opengl
Run the main script:

bash
Copy
Edit
python ADAS(ACC+LDW).py
Switch driving modes using your code’s key input (WASD or autopilot toggle).

👨‍💻 Authors

นายธนกร มงคลเดชสวัสดิ์ – [64090299]

นายวุฒิชัย อิ่มวงค์ – [64090330]

Faculty of Logistics, Burapha University
Academic Year 2567 (2024)

CARLA Simulator Community & Open-source contributors

📄 License
This project uses open-source components (CARLA, Python libraries). The custom logic is licensed under MIT. See LICENSE for details.
