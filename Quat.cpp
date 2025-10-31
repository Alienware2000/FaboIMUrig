#include "Quat.h"

Quat qConj(const Quat& q){ return Quat(-q.i, -q.j, -q.k, q.r); }

Quat qMul(const Quat& a, const Quat& b) {
  return Quat(
    a.r*b.i + a.i*b.r + a.j*b.k - a.k*b.j,
    a.r*b.j - a.i*b.k + a.j*b.r + a.k*b.i,
    a.r*b.k + a.i*b.j - a.j*b.i + a.k*b.r,
    a.r*b.r - a.i*b.i - a.j*b.j - a.k*b.k
  );
}

void qNormalize(Quat &q) {
  float n = sqrtf(q.i*q.i + q.j*q.j + q.k*q.k + q.r*q.r);
  if (n > 0.f) { q.i/=n; q.j/=n; q.k/=n; q.r/=n; }
}

float qDot(const Quat& a, const Quat& b) {
  return a.i*b.i + a.j*b.j + a.k*b.k + a.r*b.r;
}

float qAngleFromIdentityDeg(const Quat& q) {
  float w = fmaxf(-1.0f, fminf(1.0f, q.r));
  return 2.0f * acosf(w) * 180.0f / PI;
}

void quatToEuler(float qi, float qj, float qk, float qr,
                 float &roll, float &pitch, float &yaw) {
  float sinr_cosp = 2.0f * (qr * qi + qj * qk);
  float cosr_cosp = 1.0f - 2.0f * (qi * qi + qj * qj);
  roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / PI;

  float sinp = 2.0f * (qr * qj - qk * qi);
  if (fabsf(sinp) >= 1.0f) pitch = copysignf(90.0f, sinp);
  else pitch = asinf(sinp) * 180.0f / PI;

  float siny_cosp = 2.0f * (qr * qk + qi * qj);
  float cosy_cosp = 1.0f - 2.0f * (qj * qj + qk * qk);
  yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / PI;
}
