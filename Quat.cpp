#include "Quat.h"

/* Conjugate: inverse for unit quaternions */
Quat qConj(const Quat& q){
  return Quat(-q.i, -q.j, -q.k, q.r);
}

/*
  Hamilton product a ⊗ b.
  If q rotates vectors by v' = q ⊗ (0, v) ⊗ q*,
  then applying b followed by a is a ⊗ b.
*/
Quat qMul(const Quat& a, const Quat& b) {
  return Quat(
    a.r*b.i + a.i*b.r + a.j*b.k - a.k*b.j,  // i
    a.r*b.j - a.i*b.k + a.j*b.r + a.k*b.i,  // j
    a.r*b.k + a.i*b.j - a.j*b.i + a.k*b.r,  // k
    a.r*b.r - a.i*b.i - a.j*b.j - a.k*b.k   // r (scalar)
  );
}

/* Project to the unit sphere in R^4 */
void qNormalize(Quat &q) {
  float n = sqrtf(q.i*q.i + q.j*q.j + q.k*q.k + q.r*q.r);
  if (n > 0.f) { q.i/=n; q.j/=n; q.k/=n; q.r/=n; }
}

/* Standard 4D dot product */
float qDot(const Quat& a, const Quat& b) {
  return a.i*b.i + a.j*b.j + a.k*b.k + a.r*b.r;
}

/*
  Angle from identity in degrees.
  Clamp r into [-1, 1] to avoid NaN due to numeric noise.
*/
float qAngleFromIdentityDeg(const Quat& q) {
  float r = fmaxf(-1.0f, fminf(1.0f, q.r));
  return 2.0f * acosf(r) * 180.0f / PI;
}

/*
  ZYX Euler from quaternion (i, j, k, r).
  References:
    roll  = atan2(2(r*i + j*k), 1 - 2(i^2 + j^2))
    pitch = asin (2(r*j - k*i))         with clamp
    yaw   = atan2(2(r*k + i*j), 1 - 2(j^2 + k^2))
*/
void quatToEuler(float qi, float qj, float qk, float qr,
                 float &roll, float &pitch, float &yaw) {

  // roll about X
  float sinr_cosp = 2.0f * (qr * qi + qj * qk);
  float cosr_cosp = 1.0f - 2.0f * (qi * qi + qj * qj);
  roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / PI;

  // pitch about Y
  float sinp = 2.0f * (qr * qj - qk * qi);
  if (fabsf(sinp) >= 1.0f) pitch = copysignf(90.0f, sinp);  // clamp
  else                     pitch = asinf(sinp) * 180.0f / PI;

  // yaw about Z
  float siny_cosp = 2.0f * (qr * qk + qi * qj);
  float cosy_cosp = 1.0f - 2.0f * (qj * qj + qk * qk);
  yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / PI;
}
