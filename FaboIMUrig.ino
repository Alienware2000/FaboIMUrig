// ====== Faboratory IMU Rig — Baseline v0.3 ======
// Features: LSM6 gyro bias, BNO zero-pose (averaged), two BNOs via TCA, touch read
// What’s intentionally NOT included yet: smoothing, heading mode switch, yaw alignment, mount alignment

// Feather nRF52840 Sense + TCA9548A + 2x BNO085 (SparkFun lib) + 2x MPR121
// Prints one CSV frame per loop: t_ms, ax,ay,az, gx,gy,gz, q0(i,j,k,w), q1(i,j,k,w), touchA, touchB

#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_LSM6DSL.h>               // Feather Sense built-in IMU
#include "Adafruit_MPR121.h"                // Touch
#include <SparkFun_BNO080_Arduino_Library.h>// SparkFun BNO080/BNO085 lib

// ---------- I2C addresses ----------
#define TCA_ADDR   0x70
#define LSM6_ADDR  0x6A
#define BNO_ADDR   0x4A   // Adafruit BNO085 (ADR low)
#define MPR1_ADDR  0x5A
#define MPR2_ADDR  0x5B

// ---------- TCA channels ----------
const uint8_t BNO_CH0 = 0;   // BNO #0 on CH0 (SD0/SC0)
const uint8_t BNO_CH1 = 1;   // BNO #1 on CH1 (SD1/SC1)

// ---------- Devices ----------
Adafruit_LSM6DSL lsm6;
Adafruit_MPR121  cap1, cap2;
BNO080           bno0, bno1; // SparkFun driver supports multi-instances

struct Quat { 
  float i, j, k, r;
  Quat() : i(0), j(0), k(0), r(1) {}
  Quat(float ii, float jj, float kk, float rr) : i(ii), j(jj), k(kk), r(rr) {}
 };

Quat q0, q1; // default = (0,0,0,1)
Quat q0_ref(0,0,0,1), q1_ref(0,0,0,1); 
bool haveRef0=false, haveRef1=false;

// Convert a quaternion (i,j,k,w) to Euler (deg) for readability
static void quatToEuler(float qi, float qj, float qk, float qr,
                        float &roll, float &pitch, float &yaw) {
  // roll (x-axis rotation)
  float sinr_cosp = 2.0f * (qr * qi + qj * qk);
  float cosr_cosp = 1.0f - 2.0f * (qi * qi + qj * qj);
  roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / PI;

  // pitch (y-axis rotation)
  float sinp = 2.0f * (qr * qj - qk * qi);
  if (fabsf(sinp) >= 1.0f)
    pitch = copysignf(90.0f, sinp); // clamp
  else
    pitch = asinf(sinp) * 180.0f / PI;

  // yaw (z-axis rotation)
  float siny_cosp = 2.0f * (qr * qk + qi * qj);
  float cosy_cosp = 1.0f - 2.0f * (qj * qj + qk * qk);
  yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / PI;
}

static Quat qConj(const Quat& q){ return Quat(-q.i, -q.j, -q.k, q.r); }
static Quat qMul(const Quat& a, const Quat& b) {
  return Quat(
    a.r*b.i + a.i*b.r + a.j*b.k - a.k*b.j,
    a.r*b.j - a.i*b.k + a.j*b.r + a.k*b.i,
    a.r*b.k + a.i*b.j - a.j*b.i + a.k*b.r,
    a.r*b.r - a.i*b.i - a.j*b.j - a.k*b.k
  );
}

static void qNormalize(Quat &q) {
  float n = sqrtf(q.i*q.i + q.j*q.j + q.k*q.k + q.r*q.r);
  if (n > 0) { q.i/=n; q.j/=n; q.k/=n; q.r/=n; }
}

static float qDot(const Quat& a, const Quat& b) {
  return a.i*b.i + a.j*b.j + a.k*b.k + a.r*b.r;
}

// Average current q0/q1 over a short window; stores into q0_ref/q1_ref
static void captureZeroAveraged(uint16_t samples=300, uint16_t msBetween=5) {
  Quat acc0(0,0,0,0), acc1(0,0,0,0);
  Quat first0 = q0, first1 = q1;

  for (uint16_t n=0; n<samples; ++n) {
    // Re-use most recent q0/q1 (your loop updates them continuously)
    Quat s0 = q0, s1 = q1;

    // Keep samples on same hemisphere as the first (avoid cancellation)
    if (qDot(s0, first0) < 0) { s0.i=-s0.i; s0.j=-s0.j; s0.k=-s0.k; s0.r=-s0.r; }
    if (qDot(s1, first1) < 0) { s1.i=-s1.i; s1.j=-s1.j; s1.k=-s1.k; s1.r=-s1.r; }

    // Accumulate
    acc0.i += s0.i; acc0.j += s0.j; acc0.k += s0.k; acc0.r += s0.r;
    acc1.i += s1.i; acc1.j += s1.j; acc1.k += s1.k; acc1.r += s1.r;

    delay(msBetween);

  }

  // Normalize to get the mean orientation
  qNormalize(acc0); qNormalize(acc1);
  q0_ref = acc0; q1_ref = acc1;
  haveRef0 = haveRef1 = true;
}

static float qAngleFromIdentityDeg(const Quat& q) {
  float w = fmaxf(-1.0f, fminf(1.0f, q.r)); // clamp
  return 2.0f * acosf(w) * 180.0f / PI;
}

float gxb=0, gyb=0, gzb=0; // gyro bias

void calibrateGyroBias() {
  sensors_event_t a,g,t;
  const int N = 300;
  gxb = gyb = gzb = 0;
  for (int i=0; i<N; i++) {
    lsm6.getEvent(&a,&g,&t);
    gxb += g.gyro.x; gyb += g.gyro.y; gzb += g.gyro.z;
    delay(5);
  }
  gxb/=N; gyb/=N; gzb/=N;
  Serial.print("LSM6 gyro bias: ");
  Serial.print(gxb,5); Serial.print(", ");
  Serial.print(gyb,5); Serial.print(", ");
  Serial.println(gzb,5);
}

static void tcaSelect(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);   // one-hot select
  Wire.endTransmission();
}

static bool beginBNO(BNO080& dev, uint8_t ch) {
  tcaSelect(ch);
  delay(120);                          // settle after mux switch
  if (!dev.begin(BNO_ADDR, Wire)) return false;
  dev.enableRotationVector(10);        // 10 ms -> ~100 Hz
  return true;
}

static void printHeader() {
  Serial.println("t_ms,ax,ay,az,gx,gy,gz,q0i,q0j,q0k,q0w,q1i,q1j,q1k,q1w,touchA,touchB");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(100000);               // use 400k later when stable
  delay(50);

  // Built-in IMU
  if (!lsm6.begin_I2C(LSM6_ADDR, &Wire)) {
    Serial.println("ERR: LSM6DSL not found at 0x6A");
    while (1) delay(10);
  }
  lsm6.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  lsm6.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  lsm6.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6.setGyroDataRate(LSM6DS_RATE_104_HZ);

  Serial.println("Hold still ~2s for gyro bias...");
  calibrateGyroBias();

  // Two BNOs via TCA (SparkFun lib)
  if (!beginBNO(bno0, BNO_CH0)) { Serial.println("ERR: BNO080 #0 init failed"); while (1) delay(10); }
  if (!beginBNO(bno1, BNO_CH1)) { Serial.println("ERR: BNO080 #1 init failed"); while (1) delay(10); }

  // Touch
  if (!cap1.begin(MPR1_ADDR)) { Serial.println("ERR: MPR121 #1 not found"); while (1) delay(10); }
  if (!cap2.begin(MPR2_ADDR)) { Serial.println("ERR: MPR121 #2 not found"); while (1) delay(10); }
  // cap1.setThresholds(8,4); cap2.setThresholds(8,4); // optional tuning

  printHeader();
}

void loop() {
  static uint32_t next_ms = 0;
  uint32_t now = millis();
  if ((int32_t)(now - next_ms) < 0) return;
  next_ms = now + 20;                  // ~50 Hz output cadence

  // Built-in IMU (raw accel/gyro)
  sensors_event_t a, g, t;
  lsm6.getEvent(&a, &g, &t);
  float ax=a.acceleration.x, ay=a.acceleration.y, az=a.acceleration.z;
  float gx=(g.gyro.x - gxb),         gy=(g.gyro.y - gyb),         gz=(g.gyro.z - gzb);

  // BNO #0 quaternion
  tcaSelect(BNO_CH0);
  // tiny settle can help with some mux boards
  // delayMicroseconds(200);
  if (bno0.dataAvailable()) {
    // SparkFun driver updates internal fields automatically when dataAvailable() is true
    q0.i = bno0.getQuatI();
    q0.j = bno0.getQuatJ();
    q0.k = bno0.getQuatK();
    q0.r = bno0.getQuatReal();
  }

  // BNO #1 quaternion
  tcaSelect(BNO_CH1);
  // delayMicroseconds(200);
  if (bno1.dataAvailable()) {
    q1.i = bno1.getQuatI();
    q1.j = bno1.getQuatJ();
    q1.k = bno1.getQuatK();
    q1.r = bno1.getQuatReal();
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') {
      Serial.println("Zeroing... hold the pose ~0.5s");
      captureZeroAveraged(100, 5);
      Serial.println("Zero pose captured for (averaged)");
    }
    if (c == 'g') {
      tcaSelect(BNO_CH0); bno0.enableGameRotationVector(10);
      tcaSelect(BNO_CH1); bno1.enableGameRotationVector(10);
      Serial.println("Mode: Game Rotation Vector (gyro+accel, no magnetometer)");
    }
    if (c == 'r') {
      tcaSelect(BNO_CH0); bno0.enableRotationVector(10);
      tcaSelect(BNO_CH1); bno1.enableRotationVector(10);
      Serial.println("Mode: Rotation Vector (uses magnetometer for absolute yaw)");
    }

  }

  // Touch bitmasks
  uint16_t touchA = cap1.touched();
  uint16_t touchB = cap2.touched();

  // --- Pretty, labeled one-liner (easy to eyeball). Comment out if you want only CSV.

  Quat q0_rel = haveRef0 ? qMul(qConj(q0_ref), q0) : q0;
  Quat q1_rel = haveRef1 ? qMul(qConj(q1_ref), q1) : q1;

  const float deadbangDeg = 2.0f; // choose 2-3 deg
  Quat q0_disp = q0_rel, q1_disp = q1_rel;

  if (qAngleFromIdentityDeg(q0_rel) < deadbangDeg) { q0_disp = Quat(0,0,0,1); }
  if (qAngleFromIdentityDeg(q1_rel) < deadbangDeg) { q1_disp = Quat(0,0,0,1); }

  // convert these to Euler for display:
  float r0,p0,y0, r1,p1,y1;
  // quatToEuler(q0_rel.i, q0_rel.j, q0_rel.k, q0_rel.r, r0,p0,y0);
  // quatToEuler(q1_rel.i, q1_rel.j, q1_rel.k, q1_rel.r, r1,p1,y1);
  
  quatToEuler(q0_disp.i, q0_disp.j, q0_disp.k, q0_disp.r, r0,p0,y0);
  quatToEuler(q1_disp.i, q1_disp.j, q1_disp.k, q1_disp.r, r1,p1,y1);

  // Serial.print("t="); Serial.print(now);
  // Serial.print(" | IMU(acc)[m/s^2]: ");
  // Serial.print(ax,2); Serial.print(","); Serial.print(ay,2); Serial.print(","); Serial.print(az,2);
  // Serial.print(" | Gyro(x,y,z): ");
  // Serial.print(gx,2); Serial.print(","); Serial.print(gy,2); Serial.print(","); Serial.print(gz,2);
  // Serial.print(" | BNO0 quat(i,j,k,w): ");
  // Serial.print(q0.i,3); Serial.print(","); Serial.print(q0.j,3); Serial.print(","); Serial.print(q0.k,3); Serial.print(","); Serial.print(q0.r,3);
  Serial.print(" | BNO0 r/p/y[deg]: ");
  Serial.print(r0,1); Serial.print(","); Serial.print(p0,1); Serial.print(","); Serial.print(y0,1);
  // Serial.print(" | BNO1 quat(i,j,k,w): ");
  // Serial.print(q1.i,3); Serial.print(","); Serial.print(q1.j,3); Serial.print(","); Serial.print(q1.k,3); Serial.print(","); Serial.print(q1.r,3);
  Serial.print(" | BNO1 r/p/y[deg]: ");
  Serial.print(r1,1); Serial.print(","); Serial.print(p1,1); Serial.print(","); Serial.println(y1,1);
  // Serial.print(" | touchA=0x"); Serial.print(touchA, HEX);
  // Serial.print(" touchB=0x"); Serial.println(touchB, HEX);


  // Emit one CSV frame
  // Serial.print(now); Serial.print(',');
  // Serial.print(ax,3); Serial.print(','); Serial.print(ay,3); Serial.print(','); Serial.print(az,3); Serial.print(',');
  // Serial.print(gx,3); Serial.print(','); Serial.print(gy,3); Serial.print(','); Serial.print(gz,3); Serial.print(',');

  // Serial.print(q0.i,4); Serial.print(','); Serial.print(q0.j,4); Serial.print(',');
  // Serial.print(q0.k,4); Serial.print(','); Serial.print(q0.r,4); Serial.print(',');

  // Serial.print(q1.i,4); Serial.print(','); Serial.print(q1.j,4); Serial.print(',');
  // Serial.print(q1.k,4); Serial.print(','); Serial.print(q1.r,4); Serial.print(',');

  // Serial.print(touchA); Serial.print(','); Serial.println(touchB);
}
