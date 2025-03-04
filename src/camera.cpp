#include "camera.hpp"

using namespace LiteMath;

void Camera::rotate(float dx, float dy) noexcept {
  float pitch = dy * m_sensetivity; // Rotation around the X-axis
  float yaw = dx * m_sensetivity;   // Rotation around the Y-axis

  Quaternion qYaw = angleAxis(yaw, up());
  Quaternion qPitch = angleAxis(pitch, right());

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
}

Camera::Camera(const float3 &position, const float3 &target,
               const float3 &up) noexcept
    : m_position(position), m_target(target) {
  updateOrientation(up);
}

void Camera::updateOrientation(float3 up) noexcept {
  auto m = LiteMath::lookAt(position(), target(), up);
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
  normalize(m_orientation);
}

void Camera::resetPosition(const LiteMath::float3 &newPosition) noexcept {
  m_position = newPosition;
  updateOrientation(up());
}

void Camera::resetTarget(const LiteMath::float3 &newTarget) noexcept {
  m_target = newTarget;
  updateOrientation(up());
}