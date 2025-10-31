#pragma once
#include <Arduino.h>
#include <math.h>

/*
  Quaternion used here stores the vector part as (i, j, k) and the scalar part in r.
  Many texts use (x, y, z, w). Here r plays the role of w.
  We expect unit quaternions for orientations:
    i^2 = j^2 = k^2 = ijk = -1
  Unit length is required for correct rotation math.
*/
struct Quat {
  float i, j, k, r;

  // Identity rotation by default: (0, 0, 0, 1)
  Quat() : i(0), j(0), k(0), r(1) {}

  // Convenience constructor
  Quat(float ii, float jj, float kk, float rr) : i(ii), j(jj), k(kk), r(rr) {}
};

/*
  Conjugate:
    For a unit quaternion q = (v, r) with v = (i, j, k),
    q* = (-v, r).
  For unit quaternions, conjugate equals inverse.
*/
Quat qConj(const Quat& q);

/*
  Hamilton product:
    c = a ⊗ b
  Composition order matters. Treat quaternions like rotation operators.
  If q_sensor is the sensor orientation and q_ref is a reference pose,
  a common relative rotation is q_rel = q_ref* ⊗ q_sensor.
*/
Quat qMul (const Quat& a, const Quat& b);

/*
  Normalize to unit length. Safe to call often.
  No change if the norm is zero (defensive guard).
*/
void  qNormalize(Quat& q);

/*
  Dot product in R^4. Useful to:
    1) test antipodal quaternions q and -q
    2) pick a consistent hemisphere before averaging
*/
float qDot(const Quat& a, const Quat& b);

/*
  Angle of a unit quaternion from identity:
    If q = (v, r) represents a rotation by angle theta around axis v̂,
    then r = cos(theta/2). So theta = 2 * acos(r).
  Returns degrees.
*/
float qAngleFromIdentityDeg(const Quat& q);

/*
  Helper for human readability. Converts quaternion to roll, pitch, yaw.
  Uses aerospace ZYX convention:
    roll  = rotation about X
    pitch = rotation about Y
    yaw   = rotation about Z
  Beware of gimbal lock around pitch = ±90 degrees.
*/
void  quatToEuler(float qi, float qj, float qk, float qr,
                  float &roll, float &pitch, float &yaw);
