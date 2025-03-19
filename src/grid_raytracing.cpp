#include <fstream>

#include "grid_raytracing.hpp"

using namespace LiteMath;

constexpr size_t degree = 3;

float getKnot3(size_t i, size_t count) {
  size_t knotsCount = count + 1 - degree;
  if (i <= degree) [[unlikely]] {
    return 0.0f;
  } else if (i >= count) [[unlikely]] {
    return 1.0f;
  } else [[likely]] {
    return static_cast<float>(i - degree) / static_cast<float>(knotsCount-1);
  }
}

size_t getSpan3(float t, size_t count) {
  size_t knotsCount = count + 1 - degree;
  size_t span = static_cast<size_t>(t * static_cast<float>(knotsCount - 1));
  if (span == knotsCount - 1) {
    span -= 1;
  }
  return span + degree;
}

// values must start at index span-degree = span-3
float bspline3(const float *values, size_t count, float t) {
  float left[degree+1] = {};
  float right[degree+1] = {};
  size_t span = getSpan3(t, count);

  float N[degree+1] = {};
  N[0] = 1.0f;
  for (size_t j = 1; j <= degree; ++j) {
    left[j] = t - getKnot3(span + 1 - j, count);
    right[j] = getKnot3(span + j, count) - t;
    float saved = 0.0f;
    for (size_t r = 0; r < j; ++r) {
      float temp = N[r] / (right[r + 1] + left[j - r]);
      N[r] = saved + right[r + 1] * temp;
      saved = left[j - r] * temp;
    }
    N[j] = saved;
  }

  float res = 0.0f;
  for (size_t i = 0; i <= degree; ++i) {
    res += values[i] * N[i];
  }
  return res;
}

float SDFGrid::sdfBSpline3(LiteMath::float3 point) const noexcept {
  point = (point + 1.0f) / 2.0f;

  size_t spanX = getSpan3(point.x, size.x);
  size_t spanY = getSpan3(point.y, size.y);
  size_t spanZ = getSpan3(point.z, size.z);

  float valuesZ[degree + 1] = {};
  for (size_t curZ = spanZ - degree; curZ <= spanZ; ++curZ) {
    float valuesY[degree + 1] = {};
    for (size_t curY = spanY - degree; curY <= spanY; ++curY) {
      const float *valuesX =
          values.data() + (curZ * size.y + curY) * size.x + spanX - degree;
      valuesY[curY + degree - spanY] = bspline3(valuesX, size.x, point.x);
    }
    valuesZ[curZ + degree - spanZ] = bspline3(valuesY, size.y, point.y);
  }

  float result = bspline3(valuesZ, size.z, point.z);
  return result;
}

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
  float3 pointMin = clamp(point-NORMAL_EPS, float3{-1.0f}, float3{1.0f});
  float3 pointMax = clamp(point+NORMAL_EPS, float3{-1.0f}, float3{1.0f});

  float dx = (sdfBSpline3(float3{pointMax.x, point.y, point.z}) -
              sdfBSpline3(float3{pointMin.x, point.y, point.z}));

  float dy = (sdfBSpline3(float3{point.x, pointMax.y, point.z}) -
              sdfBSpline3(float3{point.x, pointMin.y, point.z}));

  float dz = (sdfBSpline3(float3{point.x, point.y, pointMax.z}) -
              sdfBSpline3(float3{point.x, point.y, pointMin.z}));

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
