#pragma once

#include <LiteMath/Image2d.h>
#include <LiteMath/LiteMath.h>

#include "mesh.h"
#include "camera.hpp"

struct FrameBuffer {
  LiteImage::Image2D<uint32_t> color;
  LiteImage::Image2D<float> t;
  void resize(uint32_t newWidth, uint32_t newHeight) {
    color.resize(newWidth, newHeight);
    t.resize(newWidth, newHeight);
  }
  void clear() {
    color.clear(0);
    t.clear(std::numeric_limits<float>::infinity());
  }
};

inline LiteMath::BBox3f update_box(LiteMath::BBox3f box, LiteMath::float4 v) {
  v /= v.w;
  box.boxMin = LiteMath::min(box.boxMin, to_float3(v));
  box.boxMax = LiteMath::max(box.boxMax, to_float3(v));
  return box;
}

inline LiteMath::BBox3f calc_bbox(const cmesh4::SimpleMesh &mesh) {
  LiteMath::BBox3f bbox;
  bbox.boxMin = LiteMath::float3{std::numeric_limits<float>::infinity()};
  bbox.boxMax = -bbox.boxMin;
  for (auto &v : mesh.vPos4f) {
    bbox = update_box(bbox, v);
  }
  return bbox;
}

inline LiteMath::BBox3f calc_bbox(cmesh4::SimpleMesh &mesh, size_t start,
                                  size_t end) {
  LiteMath::BBox3f bbox;
  bbox.boxMin = LiteMath::float3{std::numeric_limits<float>::infinity()};
  bbox.boxMax = -bbox.boxMin;
  for (size_t id = start; id < end; ++id) {
    auto v = mesh.vPos4f[mesh.indices[id]];
    bbox = update_box(bbox, v);
  }
  return bbox;
}

inline LiteMath::BBox3f calc_bbox(cmesh4::SimpleMesh &mesh,
                                  const uint32_t ids[3]) {
  LiteMath::BBox3f bbox;
  bbox.boxMin = LiteMath::float3{std::numeric_limits<float>::infinity()};
  bbox.boxMax = -bbox.boxMin;
  bbox = update_box(bbox, mesh.vPos4f[ids[0]]);
  bbox = update_box(bbox, mesh.vPos4f[ids[1]]);
  bbox = update_box(bbox, mesh.vPos4f[ids[2]]);
  return bbox;
}

inline float surfaceArea(LiteMath::BBox3f box) {
  LiteMath::float3 delta = box.boxMax - box.boxMin;
  return 2 * (delta.x * delta.y + delta.x * delta.z + delta.y * delta.z);
}

struct HitInfo {
  bool hitten = false;
  float t = std::numeric_limits<float>::infinity();
  LiteMath::float3 normal = {0.0f, 1.0f, 0.0f};
  LiteMath::float3 albedo = {1.0f, 1.0f, 1.0f};
  float reflectiveness = 0.0f;
};

class IScene {
public:
  virtual HitInfo intersect(const LiteMath::float3 &rayPos,
                            const LiteMath::float3 &rayDir, float tNear,
                            float tFar) const = 0;
  virtual ~IScene() {}
};

struct Renderer {
public:
  LiteMath::float3 lightPos;
  bool enableShadows = false;
  bool enableReflections = false;

public:
  float draw(IScene &scene, FrameBuffer &frameBuffer, const Camera &camera,
             const LiteMath::float4x4 projInv);
};