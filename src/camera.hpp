#pragma once

#include <LiteMath/LiteMath.h>

#include "quaternion.hpp"

class Camera {
public:
  Camera() noexcept = default;
  Camera(const LiteMath::float3 &position, const LiteMath::float3 &target,
         const LiteMath::float3 &up = LiteMath::float3{0.0f, 1.0f,
                                                       0.0f}) noexcept;
  Camera(const Camera &) noexcept = default;
  Camera(Camera &&) noexcept = default;

public:
  Camera &operator=(const Camera &) noexcept = default;
  Camera &operator=(Camera &&) noexcept = default;

public:
  void resetPosition(const LiteMath::float3 &newPosition) noexcept;
  void resetTarget(const LiteMath::float3 &newTarget) noexcept;
  void rotate(float dx, float dy) noexcept;
  LiteMath::float4x4 lookAtMatrix() const noexcept {
    return LiteMath::lookAt(m_position, m_target, m_up);
  }
  LiteMath::float3 position() const noexcept { return m_position; }
  LiteMath::float3 target() const noexcept { return m_target; }
  LiteMath::float3 up() const noexcept { return m_up; }
  LiteMath::float3 right() const noexcept;
  LiteMath::float3 forward() const noexcept {
    return LiteMath::normalize(target() - position());
  }

private:
  void updateVectors() noexcept;
  void updateOrientation() noexcept;

private:
  LiteMath::float3 m_position;
  LiteMath::float3 m_target;
  LiteMath::float3 m_up = {0.0f, 1.0f, 0.0f};
  Quaternion m_orientation;
  float m_sensetivity = 0.01f;

public:
  ~Camera() = default;
};
