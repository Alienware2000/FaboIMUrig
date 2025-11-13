# FaboIMUrig — Baseline v0.5

Firmware for a **multi-sensor wearable calibration rig** forming the sensing backbone of the Faboratory’s **Upper-Body Soft Exosuit Project** at Yale University.

This version adds the first implementation of **joint-angle estimation** between IMU pairs, using both the **Bone-Vector** and **Quaternion-Difference** methods.

It builds on the modular node-based sensor architecture introduced in v0.4 and prepares the firmware for **real-time biomechanical interpretation** of upper-limb motion.

---

## 🧠 Project Context

The Faboratory’s soft exosuit project aims to create an **intelligent wearable system** capable of detecting human motion, interpreting user intent, and responding with soft robotic actuation.

The system integrates:
- **BNO085 IMUs** for 3D orientation
- **LSM6DSL IMU** on the Feather for reference sensing
- **MPR121 capacitive touch sensors**
- **Soft extension actuators**

This firmware acts as the **calibration, synchronization, and data-streaming layer** for:
- Motion capture  
- Joint-angle estimation  
- Touch-motion dataset generation  
- ML-based intention modeling  
- Future closed-loop assistive control  

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
  - Joint-angle measurements (bone & quat methods)  
  - Optional yaw and motion magnitude per node  
  - Touch sensor masks (A/B)  
- Streaming frequency: ~50 Hz

✅ **Serial command interface**

| Key | Function |
|-----|----------|
| `z` | Capture reference pose (averaged) |
| `g` | Switch to GameRV mode |
| `r` | Switch to RotationVector mode |
| `1` | Select Bone-Vector joint-angle mode |
| `2` | Select Quaternion-Diff joint-angle mode |

---

## 🧭 System Operation

On startup, the firmware:
1. Initializes multiplexer and all IMU nodes  
2. Performs LSM6 gyro bias calibration  
3. Selects fusion mode (default: GameRV)  
4. Streams synchronized IMU + touch data at 50 Hz  
5. Computes joint angles between specified IMU pairs  

Press `'z'` while arm is in a neutral pose to capture:
- per-node `q_ref`  
- yaw alignment offsets (GameRV)  

### Joint-angle measurement  
Occurs **after**:
- reference-pose correction  
- heading synchronization  
- mount-alignment (when available)  

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
|--------|--------------|------|
| **v0.5-pre (Current)** | Joint-angle estimation (BoneVector & QuaternionDiff), updated data pipeline | Dec 2025 |
| **v0.4** | Node-based IMU architecture, heading alignment, modular quaternion math | Nov 2025 |
| **v0.3** | Dual IMU + touch, zero-pose averaging, stable quaternions | Oct 2025 |
| **v0.2** | First TCA9548A multi-IMU bring-up | Sept 2025 |
| **v0.1** | Single-node IMU prototype | Aug 2025 |

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
