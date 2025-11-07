# FaboIMUrig — Baseline v0.4

Firmware for a **multi-sensor wearable calibration rig** forming the sensing backbone of the Faboratory’s **Upper-Body Soft Exosuit Project** at Yale University.

This version introduces a **scalable node-based architecture** for multi-IMU calibration, including modular quaternion handling, reference-pose calibration, heading alignment for GameRV mode, and preliminary data-streaming for machine learning.

---

## 🧠 Project Context

The Faboratory’s wearable robotics project aims to develop an **intelligent upper-body soft exosuit** capable of detecting human motion and responding with assistive actuation.

The system integrates:
- **Soft actuators**
- **Inertial Measurement Units (IMUs)**
- **Capacitive touch sensors**

Together, these subsystems enable the exosuit to perceive **posture**, **muscle activity**, and **joint motion** in real time.  
This firmware serves as the foundation for **motion capture, calibration, and dataset collection** toward future perception and control models.

---

## ⚙️ Hardware Configuration

| Component | Purpose | I²C Address / Channel | Notes |
|------------|----------|-----------------------|-------|
| **Adafruit Feather nRF52840 Sense** | Main MCU | — | USB serial + built-in LSM6DSL IMU |
| **TCA9548A Multiplexer** | I²C routing | 0x70 | Enables multiple BNO085s on one bus |
| **BNO085 IMUs (x4)** | Orientation sensing | CH0–CH3 → 0x4A | Mounted on upper-body segments |
| **MPR121 (x2)** | Capacitive touch sensing | 0x5A, 0x5B | Reads distributed capacitive electrodes |

> The current build supports **up to four BNO085 nodes** and **two MPR121 boards**, scalable via the node array structure.

---

## 🧩 Current Firmware Features (Baseline v0.4)

✅ **Multi-node sensor framework**
- Unified `BnoNode` class for IMU management  
- Each node handles its own I²C channel, orientation state, reference pose, and heading offset  
- Scales automatically via `NUM_NODES` constant

✅ **Comprehensive calibration**
- **LSM6 gyro bias estimation** (mean of 300 samples)
- **Per-node zero-pose averaging** (`z` command) with hemisphere correction  
- **Heading alignment for GameRV** (aligns yaw across all sensors relative to chest node)  
- Placeholder **mount-alignment correction** (`q_mount`) ready for anatomical calibration

✅ **Quaternion math library (`Quat.h/.cpp`)**
- Full support for conjugate, multiplication, normalization, dot product  
- Euler conversion, yaw extraction, angle-from-identity, and vector rotation utilities  
- Clean, unit-tested arithmetic used across all IMU nodes

✅ **Data acquisition & streaming**
- Unified CSV logging of:
  - Built-in LSM6 acceleration + gyro  
  - Calibrated quaternions for each IMU node (`q_body`)  
  - Optional yaw and motion magnitude per node  
  - Touch sensor masks (A/B)  
- Streaming frequency: ~50 Hz

✅ **Serial command interface**

| Key | Function |
|-----|-----------|
| `z` | Capture averaged zero pose for all IMUs (reference pose) |
| `g` | Switch all IMUs to Game Rotation Vector mode (gyro + accel only) |
| `r` | Switch all IMUs to Rotation Vector mode (gyro + accel + mag) |

---

## 🧪 System Operation

On startup, the firmware:
1. Initializes all I²C devices and selects each TCA9548A channel sequentially.  
2. Performs **gyro bias calibration** on the Feather’s built-in LSM6DSL IMU.  
3. Brings up all BNO085 nodes in the specified fusion mode.  
4. Captures continuous quaternion updates at ~100 Hz per node, aggregated and streamed at 50 Hz.  
5. Logs IMU quaternions, built-in IMU readings, and capacitive touch data in synchronized CSV format.  

Press `'z'` to define a neutral **reference pose**; this captures and averages the quaternions for each IMU, enabling consistent motion tracking relative to that baseline.

---

## 🧭 Calibration Progress

This version implements a **modular calibration pipeline**. Each step refines the alignment between sensors and the human body.

| Stage | Description | Status |
|--------|--------------|--------|
| **1. Baseline calibration** | IMU bias removal, zero-pose reference, quaternion stability | ✅ Completed |
| **2. Heading alignment** | Synchronize yaw across all IMUs (GameRV offset correction) | ✅ Implemented |
| **3. Mount alignment** | Apply per-segment orientation correction for anatomical axes | ⚙️ In progress |
| **4. Touch–motion mapping** | Correlate capacitive readings with IMU motion states | 🔄 Planned |
| **5. Dataset schema & logging** | Structured CSV output for ML training | ✅ Implemented |
| **6. Visualization & fusion** | Real-time 3D visualization and fusion across IMUs | 🔄 Future |

---

## 💻 Technical Notes

- **Sampling frequency:** ~50 Hz  
- **Fusion modes:** GameRV (relative yaw), RotationRV (absolute yaw)  
- **Quaternion composition:** `q_body = inv(q_ref) * q_world * q_mount`  
- **Heading alignment:** Removes yaw offset between sensors (GameRV only)  
- **Mount correction:** Identity by default; customizable per sensor  
- **Touch sensors:** Read continuously via MPR121 (`touchA`, `touchB`)  
- **Bias calibration:** LSM6 gyro bias estimated automatically at startup  

---

## 🧪 What’s Next

- [ ] Implement guided **mount alignment calibration** for body frames  
- [ ] Add real-time visualization in Python or Processing  
- [ ] Optimize logging format and feature extraction for ML  
- [ ] Introduce low-pass filtering for noisy signals  
- [ ] Begin large-scale motion dataset collection  
- [ ] Correlate capacitive signals with IMU-based motion states  

---

## 🧩 Build Instructions

1. **Board:** Adafruit Feather nRF52840 Sense  
2. **Libraries:**
   - `Adafruit_LSM6DSL`
   - `SparkFun_BNO080_Arduino_Library`
   - `Adafruit_MPR121`
3. **Upload via:** Arduino IDE 2.x  
4. **Baud rate:** 115200  
5. **Serial interface:** CSV output (compatible with Python logging script)

---

## 🧾 Version History

| Version | Key Features | Date |
|----------|---------------|------|
| **v0.4 (Current)** | Multi-node `BnoNode` system, heading alignment for GameRV, modular quaternion math, CSV data logging for ML | Nov 2025 |
| **v0.3** | Dual-IMU + touch integration, LSM6 bias calibration, zero-pose averaging, stable quaternion streaming | Oct 2025 |
| **v0.2** | Initial IMU bring-up through TCA9548A multiplexer, first stable sensor readout | Sept 2025 |
| **v0.1 (Prototype)** | Single BNO085 + LSM6 proof of concept for sensor communication | Aug 2025 |

> Future versions (v0.5+) will introduce guided mount alignment, real-time visualization, and early dataset collection pipelines.

---

## 📚 Background

This firmware lays the foundation for the **sensor calibration and data collection phase** of the Faboratory’s soft exosuit project.  
The node-based architecture ensures that each IMU can be independently calibrated, synchronized, and fused into a coherent motion model.

Future development will focus on:
- **Mount alignment calibration** using guided reference poses  
- **Sensor fusion refinement** across multiple modalities  
- **Machine learning-based pose estimation**  
- **Closed-loop assistive control** based on multimodal sensing  

---

> **Maintainer:** David Antwi (Alienware2000)  
> Developed at the **Faboratory**  
> Yale University, 2025
