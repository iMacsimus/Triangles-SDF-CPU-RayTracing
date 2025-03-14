#include <fstream>

#include "grid_raytracing.hpp"

using namespace LiteMath;

float SDFGrid::sdf(LiteMath::float3 point) const noexcept {
  point = (point + 1.0f) / 2.0f;
  point *=
      float3{static_cast<float>(size.x - 1), static_cast<float>(size.y - 1),
             static_cast<float>(size.z - 1)};

  float3 c0f = floor(point);
  float3 c1f = ceil(point);

  uint3 c0 = {static_cast<uint32_t>(c0f.x), static_cast<uint32_t>(c0f.y),
              static_cast<uint32_t>(c0f.z)};

  uint3 c1 = {static_cast<uint32_t>(c1f.x), static_cast<uint32_t>(c1f.y),
              static_cast<uint32_t>(c1f.z)};

  float3 p_c0f = point - c0f;
  float3 c1f_p = c1f - point;

  if (c1.x == c0.x) [[unlikely]] {
    p_c0f.x = 1.0f;
    c1f_p.x = 0.0f;
  }

  if (c1.y == c0.y) [[unlikely]] {
    p_c0f.y = 1.0f;
    c1f_p.y = 0.0f;
  }

  if (c1.z == c0.z) [[unlikely]] {
    p_c0f.z = 1.0f;
    c1f_p.z = 0.0f;
  }

  float p[8] = {};
  p[0] = sdf(uint3{c0.x, c0.y, c0.z});
  p[1] = sdf(uint3{c0.x, c0.y, c1.z});
  p[2] = sdf(uint3{c0.x, c1.y, c0.z});
  p[3] = sdf(uint3{c0.x, c1.y, c1.z});
  p[4] = sdf(uint3{c1.x, c0.y, c0.z});
  p[5] = sdf(uint3{c1.x, c0.y, c1.z});
  p[6] = sdf(uint3{c1.x, c1.y, c0.z});
  p[7] = sdf(uint3{c1.x, c1.y, c1.z});

  float res = 0.0f;
  res += p[0] * c1f_p.x * c1f_p.y * c1f_p.z;
  res += p[1] * c1f_p.x * c1f_p.y * p_c0f.z;
  res += p[2] * c1f_p.x * p_c0f.y * c1f_p.z;
  res += p[3] * c1f_p.x * p_c0f.y * p_c0f.z;

  res += p[4] * p_c0f.x * c1f_p.y * c1f_p.z;
  res += p[5] * p_c0f.x * c1f_p.y * p_c0f.z;
  res += p[6] * p_c0f.x * p_c0f.y * c1f_p.z;
  res += p[7] * p_c0f.x * p_c0f.y * p_c0f.z;

  return res;
}

constexpr float NORMAL_EPS = 1e-3f;
LiteMath::float3 SDFGrid::normal(LiteMath::float3 point) const noexcept {
  float xLeft =
      (point.x - NORMAL_EPS >= -1.0f) ? point.x - NORMAL_EPS : point.x;
  float xRight =
      (point.x + NORMAL_EPS <= 1.0f) ? point.x + NORMAL_EPS : point.x;
  float yLeft =
      (point.y - NORMAL_EPS >= -1.0f) ? point.y - NORMAL_EPS : point.y;
  float yRight =
      (point.y + NORMAL_EPS <= 1.0f) ? point.y + NORMAL_EPS : point.y;
  float zLeft =
      (point.z - NORMAL_EPS >= -1.0f) ? point.z - NORMAL_EPS : point.z;
  float zRight =
      (point.z + NORMAL_EPS <= 1.0f) ? point.z + NORMAL_EPS : point.z;

  float dx = (sdf(float3{xRight, point.y, point.z}) -
              sdf(float3{xLeft, point.y, point.z}));

  float dy = (sdf(float3{point.x, yRight, point.z}) -
              sdf(float3{point.x, yLeft, point.z}));

  float dz = (sdf(float3{point.x, point.y, zRight}) -
              sdf(float3{point.x, point.y, zLeft}));

  return normalize(float3{dx, dy, dz});
}

constexpr float HIT_EPS = 1e-3f;

HitInfo SDFGrid::intersect(const LiteMath::float3 &rayPos,
                           const LiteMath::float3 &rayDir, float tNear,
                           float tFar) const {
  HitInfo result;

  auto boxIntersection = BBox3f{float3{-1.0f}, float3{1.0f}}.Intersection(
      rayPos, 1.0f / rayDir, tNear, tFar);
  if (boxIntersection.t1 > boxIntersection.t2) {
    return result; // no hit
  }

  float t = boxIntersection.t1;
  float3 curPoint = rayPos + t * rayDir;
  curPoint = max(curPoint, float3{-1.0f});
  curPoint = min(curPoint, float3{1.0f});

  while (all_of(curPoint <= float3{1.0f}) &&
         all_of(curPoint >= float3{-1.0f})) {
    float curSdf = sdf(curPoint);

    if (curSdf < HIT_EPS) {
      result.hitten = true;
      result.t = t + curSdf;
      result.normal = normal(curPoint);
      break;
    }

    t += curSdf;
    curPoint = rayPos + t * rayDir;
  }

  return result;
}

void loadSDFGrid(SDFGrid &scene, const std::string &path) {
  std::ifstream fs(path, std::ios::binary);
  fs.read((char *)&scene.size, 3 * sizeof(unsigned));
  scene.values.resize(scene.size.x * scene.size.y * scene.size.z);
  fs.read((char *)scene.values.data(),
          scene.size.x * scene.size.y * scene.size.z * sizeof(float));
  fs.close();
}
