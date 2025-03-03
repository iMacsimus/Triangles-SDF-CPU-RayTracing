#pragma once

#include <cmath>
#include <iostream>

#include <LiteMath/LiteMath.h>

struct Quaternion : LiteMath::float4 {
public:
  Quaternion() noexcept : LiteMath::float4{0.0f, 0.0f, 0.0f, 1.0f} {}
  Quaternion(float t_x, float t_y, float t_z, float t_w) noexcept
      : LiteMath::float4{t_x, t_y, t_z, t_w} {}
  explicit Quaternion(const float4 &v) noexcept : LiteMath::float4{v} {}
  Quaternion(const Quaternion &) noexcept = default;
  Quaternion(Quaternion &&) noexcept = default;

public:
  Quaternion &operator=(const Quaternion &) noexcept = default;
  Quaternion &operator=(Quaternion &&) noexcept = default;

public:
  const LiteMath::float4 &asFloat4() const noexcept {
    return static_cast<const LiteMath::float4 &>(*this);
  }
  LiteMath::float4 &asFloat4() noexcept {
    return static_cast<LiteMath::float4 &>(*this);
  }
  Quaternion conjugate() const noexcept { return Quaternion(-x, -y, -z, w); }
  float magnitude() const noexcept { return LiteMath::length(*this); }
  Quaternion normalized() const noexcept {
    float n = magnitude();
    return {x / n, y / n, z / n, w / n};
  }
  LiteMath::float4x4 rotationMatrix() const noexcept;

public:
  ~Quaternion() = default;
};

inline Quaternion operator*(const Quaternion &q1, const Quaternion &q2) {
  return Quaternion{q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
                    q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
                    q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
                    q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z};
}

inline LiteMath::float3 rotateVector(const LiteMath::float3 &v,
                                     const Quaternion &q) {
  Quaternion p(v.x, v.y, v.z, 0.0f); // Convert vector to a pure quaternion
  Quaternion rotated = q * p * q.conjugate(); // Rotate the vector
  return {rotated.x, rotated.y, rotated.z};
}