#pragma once

#include <LiteMath/Image2d.h>
#include <LiteMath/LiteMath.h>

#include "camera.hpp"
#include "mesh.h"

struct TrivialMeshRenderer {
public:
  cmesh4::SimpleMesh mesh;
  Camera camera;
  LiteMath::float4x4 projInv;

public:
  float draw(LiteImage::Image2D<uint32_t> &buffer) const;
};

inline LiteMath::BBox3f calc_bbox(const cmesh4::SimpleMesh &mesh) {
  LiteMath::BBox3f bbox;
  for (auto &v : mesh.vPos4f) {
    auto pos = to_float3(v / v.w);
    bbox.boxMin = min(bbox.boxMin, pos);
    bbox.boxMax = max(bbox.boxMax, pos);
  }
  return bbox;
}
