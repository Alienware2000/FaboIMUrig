#pragma once
#include "Quat.h"
#include "BnoNode.h"

enum JointAngleMode : uint8_t {
  BoneVector = 0,    // angle between bone directions (from qBody)
  QuaternionDiff = 1 // angle of relative quaternion
};

// High-level API: joint angle (degrees) between two nodes (e.g., upper arm = A, forearm = B).
// By default we return the "anatomical" supplement: 180° - θ so straight ≈ 180°, flexed < 180°.
float jointAngleDeg(const BnoNode& A, const BnoNode& B, JointAngleMode mode, bool anatomicalSupplement=true);
float jointAngleBoneVector(const Quat& qBodyA, const Quat& qBodyB);

// --- helpers (used by both paths) ---
inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline float rad2deg(float r) { return r * 180.0f / PI; }
inline float deg2rad(float d) { return d * PI / 180.0f; }

// Compute the angle (deg) between two *unit* vectors (defensive-normalized). Returns [0..180].
float angleBetweenUnitVecsDeg(const Vec3& u, const Vec3& v);

// Return the "bone direction" in world from a node's calibrated orientation.
// Convention: bone = local +Y axis of the sensor in its *body* frame.
Vec3 boneDirWorld(const BnoNode& n);