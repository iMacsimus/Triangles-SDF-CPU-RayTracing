#pragma once

#include <string>
#include <vector>

#include "LiteMath/LiteMath.h"

#include "mesh.h"
#include "raytracing.hpp"

struct SDFGrid final : IScene {
public:
  LiteMath::uint3 size;
  std::vector<float> values;

public:
  float sdf(LiteMath::uint3 coords) const noexcept {
    return values[(coords.z * size.y + coords.y) * size.x + coords.x];
  }
  float &sdf(LiteMath::uint3 coords) noexcept {
    return values[(coords.z * size.y + coords.y) * size.x + coords.x];
  }
  float sdf(LiteMath::int3 coords) const noexcept {
    return values[(coords.z * size.y + coords.y) * size.x + coords.x];
  }
  float &sdf(LiteMath::int3 coords) noexcept {
    return values[(coords.z * size.y + coords.y) * size.x + coords.x];
  }
  float sdf(LiteMath::float3 point) const noexcept;
  LiteMath::float3 normal(LiteMath::float3 point) const noexcept;
  virtual HitInfo intersect(const LiteMath::float3 &rayPos,
                            const LiteMath::float3 &rayDir, float tNear,
                            float tFar) const;
};

SDFGrid makeGridFromMesh(LiteMath::uint3 size, const cmesh4::SimpleMesh &mesh);
void loadSDFGrid(SDFGrid &scene, const std::string &path);
void saveSDFGrid(const SDFGrid &scene, const std::string &path);
