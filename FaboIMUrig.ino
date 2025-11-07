// ====== Faboratory IMU Rig — Baseline v0.4 (refactor) ======
// Features now: LSM6 gyro bias, per-node zero (averaged), fusion mode switch,
// heading alignment offset capture (for GameRV), mount placeholders, CSV print.

// Cal pipeline in this build:
// - LSM6 gyro bias
// - per-node zero (averaged)
// - fusion mode switch (g/r)
// - heading offsets captured at zero (for GameRV drift-consistency)
// - mount alignment placeholder (identity) inside each node
// Output: CSV with raw LSM6 + q_world/q_rel/yaw per first two nodes (expand easily)

#include <Arduino.h>
#include <Wire.h>

#include "Quat.h"
#include "BnoNode.h"

#include <Adafruit_LSM6DSL.h>
#include "Adafruit_MPR121.h"

// ---------- addresses ----------
#define TCA_ADDR   0x70
#define LSM6_ADDR  0x6A
#define BNO_ADDR   0x4A        // ADR low (Adafruit BNO085)
#define MPR1_ADDR  0x5A
#define MPR2_ADDR  0x5B

// ---------- devices ----------
Adafruit_LSM6DSL lsm6;
Adafruit_MPR121  cap1, cap2;

// ---------- TCA channels ----------
const uint8_t BNO_CH0 = 0;
const uint8_t BNO_CH1 = 1;
const uint8_t BNO_CH2 = 2;
const uint8_t BNO_CH3 = 3;

// How many IMUs today? 
// Scale-out: choose how many BNOs you mounted
constexpr uint8_t NUM_NODES = 2;

// Drivers (SparkFun) + nodes (our wrapper/state)
BNO080   bnoDrv[NUM_NODES];
BnoNode  node[NUM_NODES] = {
  BnoNode(&bnoDrv[0], BNO_CH0, TCA_ADDR, BNO_ADDR, FusionMode::GameRV), // chest?
  BnoNode(&bnoDrv[1], BNO_CH1, TCA_ADDR, BNO_ADDR, FusionMode::GameRV), // upper arm?
  // BnoNode(&bnoDrv[2], BNO_CH2, TCA_ADDR, BNO_ADDR, FusionMode::GameRV), // forearm?
  // BnoNode(&bnoDrv[3], BNO_CH3, TCA_ADDR, BNO_ADDR, FusionMode::GameRV), // spare
};

// ---------- LSM6 gyro bias ----------
float gxb=0, gyb=0, gzb=0;

static void calibrateGyroBias() {
  sensors_event_t a,g,t;
  const int N = 300;
  gxb = gyb = gzb = 0;
  for (int i=0; i<N; ++i) {
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

// example for 2 nodes; extend by pattern
static void printHeader() {
  // Serial.println(
  //   "t_ms,"
  //   "ax,ay,az,gx,gy,gz,"
  //   "bno0_qr,qi,qj,qk,"
  //   "bno1_qr,qi,qj,qk,"
  //   // "bno2_qr,qi,qj,qk,"
  //   // "bno3_qr,qi,qj,qk,"
  //   "touchA,touchB"
  // );

  // Serial.println(
  // "t_ms,ax,ay,az,gx,gy,gz,"
  // "n0_w,n0_i,n0_j,n0_k,n0_yaw,"
  // "n1_w,n1_i,n1_j,n1_k,n1_yaw,"
  // "touchA,touchB"
  // );

  Serial.println(
    "t_ms,"
    "ax,ay,az,gx,gy,gz,"
    "n0_world_w,n0_world_i,n0_world_j,n0_world_k,"
    "n0_body_w,n0_body_i,n0_body_j,n0_body_k,n0_yaw_deg,"
    "n1_world_w,n1_world_i,n1_world_j,n1_world_k,"
    "n1_body_w,n1_body_i,n1_body_j,n1_body_k,n1_yaw_deg,"
    "touchA,touchB"
  );

}

// Minimal I2C scanner with TCA channels
void scanI2C() {
  Serial.println("\n--- Base bus scan (no TCA channel selected) ---");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  found 0x"); Serial.println(addr, HEX);
    }
  }

  // Known TCA address(es) to try:
  const uint8_t tcaCandidates[] = {0x70, 0x71, 0x72, 0x73};
  for (uint8_t ti=0; ti<sizeof(tcaCandidates); ++ti) {
    uint8_t tca = tcaCandidates[ti];
    Serial.print("\nTrying TCA at 0x"); Serial.println(tca, HEX);

    // Ping TCA itself
    Wire.beginTransmission(tca);
    int ping = Wire.endTransmission();
    if (ping != 0) { Serial.println("  not present"); continue; }

    for (uint8_t ch=0; ch<8; ++ch) {
      // select channel
      Wire.beginTransmission(tca);
      Wire.write(1 << ch);
      Wire.endTransmission();
      delay(3);

      Serial.print("  CH"); Serial.print(ch); Serial.println(" scan:");
      bool any=false;
      for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
          Serial.print("    found 0x"); Serial.println(addr, HEX);
          any=true;
        }
      }
      if (!any) Serial.println("    (no devices)");
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(100000);
  delay(50);
  // scanI2C();
  // while(true) { delay(1000); } // stop here for now


  // LSM6
  if (!lsm6.begin_I2C(LSM6_ADDR, &Wire)) {
    Serial.println("ERR: LSM6DSL not found at 0x6A"); while (1) delay(10);
  }
  lsm6.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  lsm6.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  lsm6.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6.setGyroDataRate(LSM6DS_RATE_104_HZ);

  Serial.println("Hold still ~2s for gyro bias...");
  calibrateGyroBias();

  // BNO nodes
  for (uint8_t i=0; i<NUM_NODES; ++i) {
    if (!node[i].begin(TCA_ADDR, BNO_ADDR, /*rate_ms=*/10)) {
      Serial.print("ERR: BNO080 #"); Serial.print(i); Serial.println(" init failed");
      while (1) delay(10);
    }
  }

  // Touch
  if (!cap1.begin(MPR1_ADDR)) { Serial.println("ERR: MPR121 #1 not found"); while(1) delay(10); }
  if (!cap2.begin(MPR2_ADDR)) { Serial.println("ERR: MPR121 #2 not found"); while(1) delay(10); }

  printHeader();

}

// Helper to set all nodes’ fusion
static void setAllFusion(FusionMode m){
  for (uint8_t i=0; i<NUM_NODES; ++i) node[i].setFusion(m, 10);
}

void loop() {
  static uint32_t next_ms = 0;
  uint32_t now = millis();
  if ((int32_t)(now - next_ms) < 0) return; // now < next_ms
  next_ms = now + 20;   // ~50 Hz output cadence

  // --- poll BNOs ---
  for (uint8_t i=0; i<NUM_NODES; ++i) node[i].poll();

  // --- built-in IMU ---
  sensors_event_t a, g, t;
  lsm6.getEvent(&a, &g, &t);
  float ax=a.acceleration.x, ay=a.acceleration.y, az=a.acceleration.z;
  float gx=(g.gyro.x - gxb), gy=(g.gyro.y - gyb), gz=(g.gyro.z - gzb);

  // --- keyboard control ---
  while (Serial.available()) {
    char c = Serial.read();

    if (c == 'z') {
      Serial.println("Zeroing... hold the pose ~0.5s");
      for (uint8_t i=0; i<NUM_NODES; ++i) node[i].captureZero(100, 5);
      Serial.println("Zero pose captured (averaged) for all nodes");

      // Heading offsets (only needed for GameRV consistency)
      // assume node[0] is chest
      float yawRef = yawDeg(node[0].q_ref);
      for (uint8_t i=0; i<NUM_NODES; ++i) {
        node[i].yawOffsetDeg  = yawDeg(node[i].q_ref) - yawRef; // choose node 0 as reference (chest)
        node[i].haveYawOffset = true;
      }
    }

    if (c == 'g') { setAllFusion(FusionMode::GameRV);  Serial.println("Mode: GameRV"); }
    if (c == 'r') { setAllFusion(FusionMode::RotationRV); Serial.println("Mode: RotationRV"); }
  }

  // --- build derived states ---

  // Example: use the first two nodes in the CSV for now
  BnoNode& n0 = node[0];
  BnoNode& n1 = node[1];

  // Optional all encompassing step 
  Quat q0_body = n0.qBody();
  Quat q1_body = n1.qBody();

  // Relative since zero
  // Quat q0_rel = n0.qRel();
  // Quat q1_rel = n1.qRel();

  // // Apply heading alignment for GameRV (subtract constant yaw about world +Z)
  // if (n0.haveYawOffset && n0.mode == FusionMode::GameRV)
  //   q0_rel = qMul(quatFromYawDeg(-n0.yawOffsetDeg), q0_rel);
  // if (n1.haveYawOffset && n1.mode == FusionMode::GameRV)
  //   q1_rel = qMul(quatFromYawDeg(-n1.yawOffsetDeg), q1_rel);

  // // Mount correction (identity unless you edit nX.q_mount)
  // Quat q0_body = qMul(q0_rel, n0.q_mount);
  // Quat q1_body = qMul(q1_rel, n1.q_mount);

  // --- build derived states for all nodes ---
  // Quat q_body[NUM_NODES];   // array to hold each node’s calibrated orientation
  // float yaw_body_deg[NUM_NODES];

  // for (uint8_t i = 0; i < NUM_NODES; ++i) {
  //   q_body[i] = node[i].qBody();   // automatically applies heading + mount
  //   yaw_body_deg[i] = yawDeg(q_body[i]);  // optional, human-friendly
  // }

  // for (uint8_t i = 0; i < NUM_NODES; ++i) {
  //   Serial.print(q_body[i].r,4); Serial.print(',');
  //   Serial.print(q_body[i].i,4); Serial.print(',');
  //   Serial.print(q_body[i].j,4); Serial.print(',');
  //   Serial.print(q_body[i].k,4); Serial.print(',');
  //   Serial.print(yaw_body_deg[i],2);
  //   Serial.print(i < NUM_NODES-1 ? ',' : ',');
  // }

  // yaw alignment for display/feature (only when offset available)
  // float y0 = yawDeg(q0_body);
  // float y1 = yawDeg(q1_body);
  // if (node0.haveYawOffset) y0 -= node0.yawOffsetDeg;
  // if (node1.haveYawOffset) y1 -= node1.yawOffsetDeg;

  // touch
  uint16_t touchA = cap1.touched();
  uint16_t touchB = cap2.touched();

  // --- CSV frame (stable schema; expand later to q2/q3 if you want) ---
  Serial.print(now); Serial.print(',');
  Serial.print(ax,3); Serial.print(','); Serial.print(ay,3); Serial.print(','); Serial.print(az,3); Serial.print(',');
  Serial.print(gx,3); Serial.print(','); Serial.print(gy,3); Serial.print(','); Serial.print(gz,3); Serial.print(',');

  // n0 world, body, yaw
  Serial.print(n0.q_world.r,4); Serial.print(','); Serial.print(n0.q_world.i,4); Serial.print(',');
  Serial.print(n0.q_world.j,4); Serial.print(','); Serial.print(n0.q_world.k,4); Serial.print(',');
  Serial.print(q0_body.r,4);    Serial.print(','); Serial.print(q0_body.i,4);    Serial.print(',');
  Serial.print(q0_body.j,4);    Serial.print(','); Serial.print(q0_body.k,4);    Serial.print(',');
  Serial.print(yawDeg(q0_body),2); Serial.print(',');

  // n1 world, body, yaw
  Serial.print(n1.q_world.r,4); Serial.print(','); Serial.print(n1.q_world.i,4); Serial.print(',');
  Serial.print(n1.q_world.j,4); Serial.print(','); Serial.print(n1.q_world.k,4); Serial.print(',');
  Serial.print(q1_body.r,4);    Serial.print(','); Serial.print(q1_body.i,4);    Serial.print(',');
  Serial.print(q1_body.j,4);    Serial.print(','); Serial.print(q1_body.k,4);    Serial.print(',');
  Serial.print(yawDeg(q1_body),2); Serial.print(',');

  Serial.print(touchA); Serial.print(','); Serial.println(touchB);

  // ========================= CSV OUTPUT =========================
  // Serial.print(now); Serial.print(',');

  // // --- Built-in LSM6 data ---
  // Serial.print(ax,3); Serial.print(','); Serial.print(ay,3); Serial.print(','); Serial.print(az,3); Serial.print(',');
  // Serial.print(gx,3); Serial.print(','); Serial.print(gy,3); Serial.print(','); Serial.print(gz,3); Serial.print(',');

  // // --- BNO body orientations (final calibrated) ---
  // for (uint8_t i=0; i<NUM_NODES; ++i) {
  //   Quat qb = node[i].qBody();
  //   Serial.print(qb.r,4); Serial.print(',');
  //   Serial.print(qb.i,4); Serial.print(',');
  //   Serial.print(qb.j,4); Serial.print(',');
  //   Serial.print(qb.k,4);
  //   if (i < NUM_NODES-1) Serial.print(',');
  // }

  // // --- Touch sensors ---
  // Serial.print(',');
  // Serial.print(cap1.touched()); Serial.print(',');
  // Serial.println(cap2.touched());

}

