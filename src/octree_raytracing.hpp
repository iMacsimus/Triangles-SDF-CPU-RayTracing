#pragma once

#include <cinttypes>
#include <vector>

#include "raytracing.hpp"

struct SDFOctreeNode {
  float values[8];
  uint32_t childrenOffset = 0;
  bool isLeaf() const noexcept { return childrenOffset == 0; }
  bool isEmpty() const noexcept {
    return std::all_of(values, values + 8,
                       [](float val) { return val > 10.0f; }) ||
           std::all_of(values, values + 8,
                       [](float val) { return val == 0.0f; });
  }
};

struct SDFOctree final : public IScene {
public:
  HitInfo intersect(const LiteMath::float3 &rayPos,
                    const LiteMath::float3 &rayDir, float tNear,
                    float tFar) const override;

private:
  HitInfo intersectNode(size_t nodeID, const LiteMath::float3 &rayPos,
                        const LiteMath::float3 &rayDir, float tNear, float tFar,
                        const LiteMath::BBox3f &nodeBox = {
                            LiteMath::float3{-1.0f},
                            LiteMath::float3{1.0f}}) const;

  float nodeSDF(size_t nodeID, const LiteMath::BBox3f &nodeBox,
                LiteMath::float3 point) const;
  float nodeSDF(size_t nodeID, LiteMath::uint3 point) const {
    return nodes[nodeID].values[(point.x << 2) + (point.y << 1) + point.z];
  }
  LiteMath::float3 nodeNormal(size_t nodeID, const LiteMath::BBox3f &nodeBox,
                              LiteMath::float3 point) const;
  HitInfo intersectLeaf(size_t nodeID, const LiteMath::BBox3f &nodeBox,
                        const LiteMath::float3 &rayPos,
                        const LiteMath::float3 &rayDir, float tNear,
                        float tFar) const;

public:
  std::vector<SDFOctreeNode> nodes;
};

void loadSDFOctree(SDFOctree &scene, const std::string &path);