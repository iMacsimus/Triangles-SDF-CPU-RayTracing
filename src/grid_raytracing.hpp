#pragma once

#include <string>
#include <vector>

#include "LiteMath/LiteMath.h"

#include "raytracing.hpp"

struct SDFGrid final : IScene {
  LiteMath::uint3 size;
  std::vector<float> values;
  float sdf(LiteMath::uint3 coords) const noexcept {
    return values[(coords.x * size.y + coords.y) * size.z + coords.z];
  }
  float sdf(LiteMath::float3 point) const noexcept;
  LiteMath::float3 normal(LiteMath::float3 point) const noexcept;
  virtual HitInfo intersect(const LiteMath::float3 &rayPos,
                            const LiteMath::float3 &rayDir, float tNear,
                            float tFar) const;
};
void loadSDFGrid(SDFGrid &scene, const std::string &path);
