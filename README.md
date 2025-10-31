# FaboIMUrig (Baseline v0.3)

Feather nRF52840 Sense + TCA9548A + 2× BNO085 + 2× MPR121.

## Features
- LSM6 gyro bias at boot
- Two BNO085 via TCA9548A (channels 0 & 1)
- Press `z` to capture zero pose (averaged + hemisphere fix)
- Press `g` for Game Rotation Vector; `r` for Rotation Vector
- Prints roll/pitch/yaw for both IMUs at ~50 Hz
- Quaternion math in `Quat.{h,cpp}`

## Build
- Arduino IDE 2.x  
- Board: Adafruit Feather nRF52840 Sense  
- Libraries: Adafruit LSM6DSL, SparkFun BNO080/BNO085, Adafruit MPR121

## Wiring
- TCA9548A on I²C addr 0x70
- BNO085 #0 on TCA CH0 @ 0x4A, BNO085 #1 on TCA CH1 @ 0x4A
- MPR121 at 0x5A and 0x5B

## Serial Controls
- `z` — zero pose (averaged)
- `g` — Game Rotation Vector (gyro+accel)
- `r` — Rotation Vector (uses magnetometer)

## Next
- Add CSV logging
- Segment-frame alignment poses
- Optional yaw-only alignment toggle
