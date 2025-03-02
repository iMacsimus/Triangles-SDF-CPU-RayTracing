#include "quaternion.hpp"

LiteMath::float4x4 Quaternion::rotationMatrix() const noexcept {
  float x2 = x * x;
  float y2 = y * y;
  float z2 = z * z;
  float xy = x * y;
  float xz = x * z;
  float yz = y * z;
  float wx = w * x;
  float wy = w * y;
  float wz = w * z;

  LiteMath::float4x4 mat;
  // Rotation part (3x3)
  mat(0, 0) = 1.0f - 2.0f * (y2 + z2);
  mat(0, 1) = 2.0f * (xy - wz);
  mat(0, 2) = 2.0f * (xz + wy);
  mat(0, 3) = 0.0f;

  mat(1, 0) = 2.0f * (xy + wz);
  mat(1, 1) = 1.0f - 2.0f * (x2 + z2);
  mat(1, 2) = 2.0f * (yz - wx);
  mat(1, 3) = 0.0f;

  mat(2, 0) = 2.0f * (xz - wy);
  mat(2, 1) = 2.0f * (yz + wx);
  mat(2, 2) = 1.0f - 2.0f * (x2 + y2);
  mat(2, 3) = 0.0f;

  // Homogeneous part (identity row and column)
  mat(3, 0) = 0.0f;
  mat(3, 1) = 0.0f;
  mat(3, 2) = 0.0f;
  mat(3, 3) = 1.0f;

  return mat;
}
