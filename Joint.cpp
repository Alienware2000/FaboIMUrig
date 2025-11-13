#include "Joint.h"
#include <math.h>

// Normalize a Vec3 defensively
static Vec3 norm3(const Vec3& v) {
  float n = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
  if (n <= 1e-9f) return {0,0,0};
  return { v.x/n, v.y/n, v.z/n };
}

float angleBetweenUnitVecsDeg(const Vec3& a, const Vec3& b) {
  Vec3 ua = norm3(a), ub = norm3(b);
  float dot = ua.x*ub.x + ua.y*ub.y + ua.z*ub.z;
  dot = clampf(dot, -1.0f, 1.0f);          // numeric safety
  return rad2deg(acosf(dot));              // [0..180]
}

Vec3 boneDirWorld(const BnoNode& n) {
  // Local +Y (0,1,0) is the bone axis by our mounting convention.
  static const Vec3 localY = {0,1,0};
  Quat qb = n.qBody();                     // already: qRel ⊗ q_mount
  return rotateVecByQuat(localY, qb);
}

static float angleFromQuat(const Quat& q) {
  // unit quaternion assumed; clamp real part for safety
  float r = clampf(q.r, -1.0f, 1.0f);
  return rad2deg(2.0f * acosf(r));         // [0..360], but in practice [0..180] for principal
}

float jointAngleDeg(const BnoNode& A, const BnoNode& B, JointAngleMode mode, bool anatomicalSupplement) {
  float theta = 0.0f;

  if (mode == JointAngleMode::BoneVector) {
    Vec3 u = boneDirWorld(A);
    Vec3 v = boneDirWorld(B);
    theta = angleBetweenUnitVecsDeg(u, v);       // 0° if parallel, 180° if opposite
  } else {
    // Quaternion-difference method on calibrated orientations
    Quat qA = A.qBody();
    Quat qB = B.qBody();
    Quat qRel = qMul(qB, qConj(qA));             // rotate A into B
    qNormalize(qRel);
    theta = angleFromQuat(qRel);                 // principal rotation angle
  }

  // Anatomical convention: straight ≈ 180°, flexed smaller.
  if (anatomicalSupplement) theta = 180.0f - theta;

  // Clamp to a reasonable elbow range [0..180]
  if (theta < 0)   theta = 0;
  if (theta > 180) theta = 180;
  return theta;
}

// Compute angle between two IMU "bone" directions using q_body.
// Assumes +Y is the bone axis in sensor frame.
float jointAngleBoneVector(const Quat& qBodyA, const Quat& qBodyB) {
  // 1) Bone direction in *body frame* (sensor’s +Y)
  const Vec3 boneLocal = {0.0f, 1.0f, 0.0f};

  // 2) Rotate into world
  Vec3 a = rotateVecByQuat(boneLocal, qBodyA);
  Vec3 b = rotateVecByQuat(boneLocal, qBodyB);

  // 3) Normalize defensively
  auto norm = [](const Vec3& v) {
    return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
  };

  float na = norm(a);
  float nb = norm(b);

  // If something is totally degenerate, just say angle=0 for now
  if (na < 1e-6f || nb < 1e-6f) {
    return 0.0f;
  }

  a.x /= na; a.y /= na; a.z /= na;
  b.x /= nb; b.y /= nb; b.z /= nb;

  // 4) Dot product → cos(theta)
  float dot = a.x*b.x + a.y*b.y + a.z*b.z;

  // 5) Clamp to [-1, 1] to avoid NaNs from float error
  if (dot >  1.0f) dot =  1.0f;
  if (dot < -1.0f) dot = -1.0f;

  float thetaRad = acosf(dot);
  float thetaDeg = thetaRad * 180.0f / PI;

  return thetaDeg;   // Use as-is, or 180 - thetaDeg if that’s your convention
}
