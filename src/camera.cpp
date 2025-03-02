#include "camera.hpp"

using namespace LiteMath;

void Camera::rotate(float dx, float dy) noexcept {
  float pitch = dy * m_sensetivity; // Rotation around the X-axis
  float yaw = dx * m_sensetivity;   // Rotation around the Y-axis

  Quaternion qPitch(std::sin(pitch / 2.0f), 0.0f, 0.0f, std::cos(pitch / 2.0f));
  Quaternion qYaw(0.0f, std::sin(yaw / 2.0f), 0.0f, std::cos(yaw / 2.0f));

  // Combine the rotations (yaw first, then pitch)
  m_orientation =
      static_cast<Quaternion>(qPitch * qYaw * m_orientation).normalized();

  updateVectors();
}

void Camera::updateVectors() noexcept {
  float3 forward = {0.0f, 0.0f, -1.0f};
  float3 rotatedForward = normalize(rotateVector(forward, m_orientation));

  float distance = length(m_target - m_position);
  m_position = m_target - rotatedForward * distance;
  m_up = normalize(rotateVector({0.0f, 1.0f, 0.0f}, m_orientation));
}

Camera::Camera(const float3 &position, const float3 &target,
               const float3 &up) noexcept
    : m_position(position), m_target(target), m_up(up) {
  auto mat = lookAtMatrix();
  m_up = to_float3(mat.col(1));
  updateOrientation();
}

void Camera::updateOrientation() noexcept {
  auto m = lookAtMatrix();
  float t = 0.0f; // trace

  if (m(2, 2) < 0) {
    if (m(0, 0) > m(1, 1)) {
      t = 1 + m(0, 0) - m(1, 1) - m(2, 2);
      m_orientation = Quaternion(t, m(0, 1) + m(1, 0), m(2, 0) + m(0, 2),
                                 m(1, 2) - m(2, 1));
    } else {
      t = 1 - m(0, 0) + m(1, 1) - m(2, 2);
      m_orientation = Quaternion(m(0, 1) + m(1, 0), t, m(1, 2) + m(2, 1),
                                 m(2, 0) - m(0, 2));
    }
  } else {
    if (m(0, 0) < -m(1, 1)) {
      t = 1 - m(0, 0) - m(1, 1) + m(2, 2);
      m_orientation = Quaternion(m(2, 0) + m(0, 2), m(1, 2) + m(2, 1), t,
                                 m(0, 1) - m(1, 0));
    } else {
      t = 1 + m(0, 0) + m(1, 1) + m(2, 2);
      m_orientation = Quaternion(m(1, 2) - m(2, 1), m(2, 0) - m(0, 2),
                                 m(0, 1) - m(1, 0), t);
    }
  }
  m_orientation.asFloat4() *= 0.5f / std::sqrt(t);
}

LiteMath::float3 Camera::right() const noexcept {
  return to_float3(lookAtMatrix().col(0));
}

void Camera::resetPosition(const LiteMath::float3 &newPosition) noexcept {
  m_position = newPosition;
  updateOrientation();
  m_up = rotateVector({0.0f, 1.0f, 0.0f}, m_orientation);
}

void Camera::resetTarget(const LiteMath::float3 &newTarget) noexcept {
  m_target = newTarget;
  updateOrientation();
  m_up = rotateVector({0.0f, 1.0f, 0.0f}, m_orientation);
}