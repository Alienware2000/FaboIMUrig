# FaboIMUrig — Baseline v0.3

Firmware for a **multi-sensor wearable calibration rig** that forms the sensing backbone of the Faboratory’s **Upper-Body Soft Exosuit Project** at Yale University.

This stage of development focuses on **bringing up the embedded sensing system**, ensuring reliable I²C communication across multiple IMUs and capacitive touch sensors, and performing baseline calibration routines that produce stable, repeatable data for future learning and control.

---

## 🧠 Project Context

The Faboratory’s wearable robotics project aims to develop an **intelligent upper-body soft exosuit** capable of detecting human motion and responding with assistive actuation.

The system integrates:
- **Soft actuators**
- **Inertial Measurement Units (IMUs)**
- **Capacitive sensors**

Together, these subsystems enable the exosuit to perceive **posture**, **muscle activity**, and **joint motion** in real time.  
Over the past three months, work has focused on bringing up the **embedded sensing platform** that will power these perception pipelines.

---

## ⚙️ Hardware Configuration

| Component | Purpose | I²C Address / Channel | Notes |
|------------|----------|-----------------------|-------|
| **Adafruit Feather nRF52840 Sense** | Main MCU | — | USB serial + built-in IMU |
| **TCA9548A Multiplexer** | I²C routing | 0x70 | Connects multiple BNO085 IMUs |
| **BNO085 IMUs (x2)** | Orientation sensing | CH0 → 0x4A, CH1 → 0x4A | Mounted on different body segments |
| **MPR121 (x2)** | Capacitive touch sensing | 0x5A, 0x5B | Reads distributed capacitive elements |

> The current configuration tests **two BNO085 sensors** and **two MPR121 modules**, establishing a scalable pattern for expansion.

---

## 🧩 Current Firmware Features (Baseline v0.3)

✅ **Core bring-up completed**
- Verified reliable I²C communication via TCA9548A multiplexer  
- Integrated multiple IMUs and capacitive sensors  
- Built-in IMU (LSM6DSL) bias calibration implemented  

✅ **Baseline calibration implemented**
- **LSM6 gyro bias estimation** (average of 300 samples)
- **BNO zero-pose averaging** (`z` command) with hemisphere check  
- Produces stable quaternion + Euler data for both IMUs

✅ **Data acquisition & streaming**
- Real-time streaming of roll/pitch/yaw at ~50 Hz  
- Optional CSV-ready serial output format  

✅ **Serial command interface**
| Key | Function |
|-----|-----------|
| `z` | Capture averaged zero pose for both IMUs |
| `g` | Switch to Game Rotation Vector (gyro + accel only) |
| `r` | Switch to Rotation Vector (includes magnetometer) |

---

## 🧪 System Operation

On startup, the firmware:
1. Initializes all I²C devices through the TCA9548A.  
2. Performs **gyro bias calibration** on the Feather’s internal LSM6DSL IMU.  
3. Activates both BNO085s and begins quaternion updates at 100 Hz.  
4. Streams roll–pitch–yaw data at 50 Hz over USB serial.  

The `'z'` command triggers a **zeroing routine** that averages quaternions for both IMUs, defining a neutral orientation reference. Touch readings from both MPR121 boards are captured continuously for later mapping with motion data.

---

## 🧭 Calibration Progress

The firmware currently supports **baseline calibration** routines to stabilize sensor data and align orientation frames.  
This stage establishes the foundation for the next calibration levels:

| Stage | Description | Status |
|--------|--------------|--------|
| **1. Baseline calibration** | IMU bias removal, zero-pose reference, quaternion stability | ✅ Completed |
| **2. Heading & mount alignment** | Align yaw between sensors; correct for mounting offsets | 🔄 Planned |
| **3. Touch-to-motion mapping** | Correlate capacitive sensor readings with IMU joint angles | 🔄 Planned |
| **4. Smoothing & filtering** | Temporal smoothing and deadband thresholds for live control | 🔄 Planned |
| **5. Visualization & fusion** | Combine motion and touch data in real-time 3D visualization | 🔄 Future |

---

## 💻 Technical Notes

- **Sampling frequency:** ~50 Hz  
- **Quaternion math:** Handled in `Quat.h/.cpp` (normalization, multiplication, conjugate, dot product, Euler conversion)
- **Built-in IMU bias calibration:** runs automatically at startup
- **BNO085 mode switching:** accessible through serial commands (`g`, `r`)

---

## 🧪 What’s Next

- [ ] Implement heading alignment between IMUs  
- [ ] Introduce smoothing and low-pass filters  
- [ ] Add yaw-only calibration mode  
- [ ] Log serial data to CSV for analysis  
- [ ] Visualize roll/pitch/yaw in a 3D dashboard  
- [ ] Begin correlating capacitive signals with joint angles  

---

## 🧩 Build Instructions

1. **Board:** Adafruit Feather nRF52840 Sense  
2. **Libraries:**
   - `Adafruit_LSM6DSL`
   - `SparkFun_BNO080_Arduino_Library`
   - `Adafruit_MPR121`
3. **Upload via:** Arduino IDE 2.x  
4. **Baud rate:** 115200 (variable) 

---

## 📚 Background

This firmware is part of an ongoing effort to build a **multi-modal sensing layer** for a soft exosuit.  
Accurate calibration ensures the system can translate human motion into actuator commands and model joint kinematics for adaptive assistance.

Future stages will extend this baseline toward **sensor fusion**, **learning-based pose estimation**, and **human-in-the-loop control**.

---

> **Maintainer:** David Antwi (Alienware2000)  
> Developed at the **Faboratory**   
> Yale University, 2025
