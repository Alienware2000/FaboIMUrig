#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include "Quat.h"

// Simple fusion mode selector
enum class FusionMode : uint8_t { GameRV, RotationRV };

struct BnoNode {
  // wiring / device
  BNO080*  dev           = nullptr;
  uint8_t  tcaChannel    = 0;     // TCA9548A channel (0..7)
  uint8_t  tcaAddr       = 0x70;  // default TCA address
  uint8_t  bnoAddr       = 0x4A;  // BNO I2C address by default
  FusionMode mode        = FusionMode::GameRV;

  // orientation state
  Quat   q_world;                 // live orientation from sensor (world frame)
  Quat   q_ref;                   // snapshot at zero
  bool   haveRef        = false;

  // heading alignment (for GameRV)
  float  yawOffsetDeg   = 0.0f;
  bool   haveYawOffset  = false;

  // mount (sensor → body)
  Quat   q_mount        = Quat(0,0,0,1);

  // telemetry
  uint32_t lastUpdateMS = 0;

  // --- ctor so brace-init works on Arduino toolchain ---
  BnoNode() = default;
  BnoNode(BNO080* dev_, uint8_t tcaCh_, uint8_t tcaAddr_ = 0x70,
          uint8_t bnoAddr_ = 0x4A, FusionMode mode_ = FusionMode::GameRV)
  : dev(dev_), tcaChannel(tcaCh_), tcaAddr(tcaAddr_),
    bnoAddr(bnoAddr_), mode(mode_) /*, q_world(), q_ref(), q_mount(0,0,0,1) */ {}

  // --- API ---
  bool begin(uint8_t tca_addr, uint8_t bno_addr, uint8_t rate_ms=10);
  void setFusion(FusionMode m, uint8_t rate_ms=10);
  bool poll();                          // returns true if updated

  // Capture a stable zero pose by averaging a short window
  void captureZero(uint16_t samples=300, uint16_t msBetween=5);

  // helpers
  float yawDegWorld() const;

  // Relative since zero; if no zero yet, return q_world
  Quat  qRel() const;                   // inv(q_ref) * q_world (or q_world)

  // Relative + mount correction (sensor → body segment)
  Quat  qBody() const;                  // qRel() * q_mount

private:
  void selectOnMux() const;             // select TCA channel
};
