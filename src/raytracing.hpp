#pragma once

#include <LiteMath/Image2d.h>
#include <LiteMath/LiteMath.h>

#include "camera.hpp"
#include "mesh.h"

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

inline LiteMath::BBox3f update_box(LiteMath::BBox3f box, LiteMath::float3 v) {
  box.boxMin = LiteMath::min(box.boxMin, v);
  box.boxMax = LiteMath::max(box.boxMax, v);
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

class SceneUnion : public IScene {
public:
  SceneUnion(std::shared_ptr<IScene> pFirst, std::shared_ptr<IScene> pSecond)
      : m_pFirst(pFirst), m_pSecond(pSecond) {}
  HitInfo intersect(const LiteMath::float3 &rayPos,
                    const LiteMath::float3 &rayDir, float tNear,
                    float tFar) const override {
    HitInfo intersect1 = m_pFirst->intersect(rayPos, rayDir, tNear, tFar);
    HitInfo intersect2 = m_pSecond->intersect(rayPos, rayDir, tNear, tFar);
    return (intersect1.t < intersect2.t) ? intersect1 : intersect2;
  }

private:
  std::shared_ptr<IScene> m_pFirst, m_pSecond;
};

enum class ShadingMode { Normal, Lambert, Color };

struct Renderer {
public:
  LiteMath::float3 lightPos;
  bool enableShadows = true;
  bool enableReflections = true;
  ShadingMode shadingMode = ShadingMode::Lambert;

public:
  float draw(const IScene &scene, FrameBuffer &frameBuffer,
             const Camera &camera, const LiteMath::float4x4 projInv) const;

private:
  std::pair<LiteMath::float4, float>
  intersectionColor(const IScene &scene, const LiteMath::float3 &rayPos,
                    const LiteMath::float3 &rayDir, float tNear, float tFar,
                    float tPrev, int maxDepth = 2) const;
};

class Plane final : public IScene {
public:
  Plane(const LiteMath::float3 &normal, float offset = 0)
      : m_normal(normal), m_offset(offset) {
    recalcBasis();
  }
  Plane(const LiteMath::float3 &normal, const LiteMath::float3 &pointOnPlane)
      : m_normal(normal), m_offset(dot(normal, pointOnPlane)) {
    recalcBasis();
  }
  Plane(float a, float b, float c, float d) {
    m_normal = LiteMath::normalize(LiteMath::float3{a, b, c});
    m_offset = -d / LiteMath::length(LiteMath::float3{a, b, c});
    recalcBasis();
  }

public:
  HitInfo intersect(const LiteMath::float3 &rayPos,
                    const LiteMath::float3 &rayDir, float tNear,
                    float tFar) const {
    HitInfo result;

    // dot(o+t*d, n) - offset == 0 <=> dot(o, n) + t*dot(d, n) - offset == 0
    // t = (offset - dot(o, n)) / (dot(d, n))
    float dividor = LiteMath::dot(rayDir, m_normal);
    if (std::abs(dividor) < 1e-8f) {
      return result; // no hit
    }

    float t = (m_offset - dot(rayPos, m_normal)) / dividor;

    if (t < tNear || t > tFar) {
      return result; // no hit
    }

    LiteMath::float3 intersectionPoint = rayPos + t * rayDir;
    int x = static_cast<int>(std::ceil(dot(intersectionPoint, m_basis1)));
    int y = static_cast<int>(std::ceil(dot(intersectionPoint, m_basis2)));
    LiteMath::float3 color =
        (x + y) % 2 == 0 ? LiteMath::float3(0.0f) : LiteMath::float3(1.0f);

    result.hitten = true;
    result.t = t;
    result.normal = m_normal;
    result.albedo = color;
    result.reflectiveness = 0.3f;
    return result;
  }

private:
  void recalcBasis() {
    LiteMath::float3 absNormal = LiteMath::abs(m_normal);
    if (absNormal.x > absNormal.y && absNormal.x > absNormal.z) {
      m_basis1 =
          LiteMath::normalize(LiteMath::float3(m_normal.y, -m_normal.x, 0));
    } else {
      m_basis1 =
          LiteMath::normalize(LiteMath::float3(0, m_normal.z, -m_normal.y));
    }
    m_basis2 = LiteMath::normalize(LiteMath::cross(m_basis1, m_normal));
  }

private:
  LiteMath::float3 m_normal;
  float m_offset; // plane <=> dot(normal, point) + offset = 0
  LiteMath::float3 m_basis1;
  LiteMath::float3 m_basis2;
};

#define SWAP(x, y)                                                             \
  if (t[x] > t[y]) {                                                           \
    std::swap(t[x], t[y]);                                                     \
    std::swap(children[x], children[y]);                                       \
  }
inline void sort8(float t[8], size_t children[8]) {
  SWAP(0, 1);
  SWAP(2, 3);
  SWAP(4, 5);
  SWAP(6, 7);
  SWAP(0, 2);
  SWAP(1, 3);
  SWAP(4, 6);
  SWAP(5, 7);
  SWAP(1, 2);
  SWAP(5, 6);
  SWAP(0, 4);
  SWAP(3, 7);
  SWAP(1, 5);
  SWAP(2, 6);
  SWAP(1, 4);
  SWAP(3, 6);
  SWAP(2, 4);
  SWAP(3, 5);
  SWAP(3, 4);
}