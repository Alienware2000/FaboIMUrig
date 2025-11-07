#include "BnoNode.h"

// ------------------ MUX SELECTION ------------------
void BnoNode::selectOnMux() const {
  // Each TCA channel corresponds to one BNO on the bus.
  Wire.beginTransmission(tcaAddr);
  Wire.write(1 << tcaChannel);    // activate this channel only
  Wire.endTransmission();
}

// ------------------ INITIALIZATION ------------------
bool BnoNode::begin(uint8_t tca_addr, uint8_t bno_addr, uint8_t rate_ms) {
  tcaAddr = tca_addr;
  bnoAddr = bno_addr;

  selectOnMux();
  delay(120);                      // settle after mux switch

  if (!dev || !dev->begin(bnoAddr, Wire)) return false;

  setFusion(mode, rate_ms);        // set initial fusion mode

  // initialize quaternions and state
  q_world = Quat(); q_ref = Quat(); q_mount = Quat(0,0,0,1);
  haveRef = haveYawOffset = false;
  lastUpdateMS = millis();
  return true;
}

// ------------------ FUSION MODE ------------------
void BnoNode::setFusion(FusionMode m, uint8_t rate_ms) {
  mode = m;
  selectOnMux();

  if (mode == FusionMode::GameRV) {
    dev->enableGameRotationVector(rate_ms);   // gyro+accel; no magnetometer
  } else {
    dev->enableRotationVector(rate_ms);       // uses magnetometer (absolute yaw)
  }
}

// ------------------ POLL (update sensor quaternion) ------------------
bool BnoNode::poll() {
  selectOnMux();
  if (!dev->dataAvailable()) return false;

  q_world.i = dev->getQuatI();
  q_world.j = dev->getQuatJ();
  q_world.k = dev->getQuatK();
  q_world.r = dev->getQuatReal();
  qNormalize(q_world);  // ensure unit length

  lastUpdateMS = millis();
  return true;
}

// ------------------ ZERO CALIBRATION ------------------
void BnoNode::captureZero(uint16_t samples, uint16_t msBetween) {
  // Average stable q_world with hemisphere fix
  Quat acc(0,0,0,0);
  Quat first = q_world;

  for (uint16_t n=0; n<samples; ++n) {
    // ensure we have fresh data
    poll();
    Quat s = q_world;

    // keep on same hemisphere as first
    if (qDot(s, first) < 0) { s.i=-s.i; s.j=-s.j; s.k=-s.k; s.r=-s.r; }

    acc.i += s.i; acc.j += s.j; acc.k += s.k; acc.r += s.r;
    delay(msBetween);
  }

  qNormalize(acc);
  q_ref = acc;
  haveRef = true;
}

// ------------------ HELPERS ------------------
float BnoNode::yawDegWorld() const {
  return yawDeg(q_world);
}

// ------------------ RELATIVE ORIENTATION ------------------
// Relative since zero; if no zero yet, return q_world
Quat BnoNode::qRel() const {
  if (!haveRef) return q_world;
  // inv(q_ref) * q_world = q_ref* ⊗ q_world
  return qMul(qConj(q_ref), q_world);
}

// ------------------ FINAL CALIBRATED ORIENTATION ------------------
// Relative + mount correction (sensor → body segment)
Quat BnoNode::qBody() const {
  // 1. Start from relative orientation since zero
  Quat rel = qRel();

  // 2. If using GameRV, apply heading offset (align yaw between sensors)
  if (mode == FusionMode::GameRV && haveYawOffset) {
    rel = qMul(quatFromYawDeg(-yawOffsetDeg), rel);  // apply heading alignment
  }

  // 3. Apply mount correction (sensor frame -> body segment frame)
  return qMul(rel, q_mount);  // apply mount alignment
}
