#pragma once
#include <Arduino.h>
#include <math.h>

struct Quat {
  float i, j, k, r;
  Quat() : i(0), j(0), k(0), r(1) {}
  Quat(float ii, float jj, float kk, float rr) : i(ii), j(jj), k(kk), r(rr) {}
};

Quat qConj(const Quat& q);
Quat qMul (const Quat& a, const Quat& b);
void  qNormalize(Quat& q);
float qDot(const Quat& a, const Quat& b);
float qAngleFromIdentityDeg(const Quat& q);
void  quatToEuler(float qi, float qj, float qk, float qr,
                  float &roll, float &pitch, float &yaw);
