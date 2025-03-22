#include <array>
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
  std::cout << "Grid Size: " << scene.size.x << "x" << scene.size.y << "x"
            << scene.size.z << std::endl;
  scene.values.resize(scene.size.x * scene.size.y * scene.size.z);
  fs.read((char *)scene.values.data(),
          scene.size.x * scene.size.y * scene.size.z * sizeof(float));
  fs.close();
}

void saveSDFGrid(const SDFGrid &scene, const std::string &path) {
  std::ofstream fs(path, std::ios::binary);
  fs.write((const char *)&scene.size, 3 * sizeof(unsigned));
  fs.write((const char *)scene.values.data(),
           scene.size.x * scene.size.y * scene.size.z * sizeof(float));
  fs.flush();
  fs.close();
}

float3 closestPointTriangle(float3 const &p, float3 const &a, float3 const &b,
                            float3 const &c) {
  const float3 ab = b - a;
  const float3 ac = c - a;
  const float3 ap = p - a;

  const float d1 = dot(ab, ap);
  const float d2 = dot(ac, ap);
  if (d1 <= 0.f && d2 <= 0.f)
    return a; // #1

  const float3 bp = p - b;
  const float d3 = dot(ab, bp);
  const float d4 = dot(ac, bp);
  if (d3 >= 0.f && d4 <= d3)
    return b; // #2

  const float3 cp = p - c;
  const float d5 = dot(ab, cp);
  const float d6 = dot(ac, cp);
  if (d6 >= 0.f && d5 <= d6)
    return c; // #3

  const float vc = d1 * d4 - d3 * d2;
  if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) {
    const float v = d1 / (d1 - d3);
    return a + v * ab; // #4
  }

  const float vb = d5 * d2 - d1 * d6;
  if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
    const float v = d2 / (d2 - d6);
    return a + v * ac; // #5
  }

  const float va = d3 * d6 - d5 * d4;
  if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) {
    const float v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return b + v * (c - b); // #6
  }

  const float denom = 1.f / (va + vb + vc);
  const float v = vb * denom;
  const float w = vc * denom;
  return a + v * ab + w * ac; // #0
}

inline void sort3(float aa[3]) {
  if (aa[0] > aa[1]) {
    std::swap(aa[0], aa[1]);
  }
  if (aa[1] > aa[2]) {
    std::swap(aa[1], aa[2]);
  }
  if (aa[0] > aa[1]) {
    std::swap(aa[0], aa[1]);
  }
}

void redistancing(SDFGrid &grid) {
  float h = 2.0f / static_cast<float>(hmax(grid.size - 1));
  float f = 1.0f;
  constexpr float eps = 1e-8f;

  std::vector<bool> frozen(grid.size.x * grid.size.y * grid.size.z, false);
  for (size_t i = 0; i < grid.values.size(); ++i) {
    frozen[i] = !std::isinf(grid.values[i]);
  }

  int3 ssize = {static_cast<int>(grid.size.x), static_cast<int>(grid.size.y),
                static_cast<int>(grid.size.z)};

  for (size_t sweep = 0; sweep < 8; ++sweep) {
    std::array dirX = (sweep / 4 == 0) ? std::array{0, ssize.x, +1}
                                       : std::array{ssize.x - 1, -1, -1};
    std::array dirY = ((sweep % 4) / 2 == 0) ? std::array{0, ssize.y, +1}
                                             : std::array{ssize.y - 1, -1, -1};
    std::array dirZ = (sweep % 2 == 0) ? std::array{0, ssize.z, +1}
                                       : std::array{ssize.z - 1, -1, -1};

    for (int x = dirX[0]; x != dirX[1]; x += dirX[2]) {
      for (int y = dirY[0]; y != dirY[1]; y += dirY[2]) {
        for (int z = dirZ[0]; z != dirZ[1]; z += dirZ[2]) {
          int flatCoord = (z * ssize.y + y) * ssize.x + x;
          if (frozen[flatCoord]) {
            continue;
          }

          float aa[3] = {};
          int ym1 = clamp(y - 1, 0, ssize.y - 1);
          int yp1 = clamp(y + 1, 0, ssize.y - 1);
          int xm1 = clamp(x - 1, 0, ssize.x - 1);
          int xp1 = clamp(x + 1, 0, ssize.x - 1);
          int zm1 = clamp(z - 1, 0, ssize.z - 1);
          int zp1 = clamp(z + 1, 0, ssize.z - 1);

          aa[0] = std::min(std::abs(grid.sdf(int3{xm1, y, z})),
                           std::abs(grid.sdf(int3{xp1, y, z})));
          aa[1] = std::min(std::abs(grid.sdf(int3{x, ym1, z})),
                           std::abs(grid.sdf(int3{x, yp1, z})));
          aa[2] = std::min(std::abs(grid.sdf(int3{x, y, zm1})),
                           std::abs(grid.sdf(int3{x, y, zp1})));
          sort3(aa);

          float d_curr = aa[0] + h * f;
          float d_new;
          if (d_curr <= (aa[1] + eps)) {
            d_new = d_curr;
          } else {
            float a = 2.0f;
            float b = -2.0f * (aa[0] + aa[1]);
            float c = aa[0] * aa[0] + aa[1] * aa[1] - h * h * f * f;
            float D = sqrt(b * b - 4.0f * a * c);

            d_curr = ((-b + D) > (-b - D) ? (-b + D) : (-b - D)) / (2.0f * a);

            if (d_curr <= (aa[2] + eps))
              d_new = d_curr;
            else {
              a = 3.0f;
              b = -2.0f * (aa[0] + aa[1] + aa[2]);
              c = aa[0] * aa[0] + aa[1] * aa[1] + aa[2] * aa[2] - h * h * f * f;
              D = sqrt(b * b - 4.0f * a * c);
              d_new = ((-b + D) > (-b - D) ? (-b + D) : (-b - D)) / (2.0f * a);
            }
          }
          if (std::abs(grid.values[flatCoord]) > d_new) {
            float sign = -1.0f;
            if (grid.sdf(int3{xm1, y, z}) > 0 ||
                grid.sdf(int3{xp1, y, z}) > 0 ||
                grid.sdf(int3{x, ym1, z}) > 0 ||
                grid.sdf(int3{x, yp1, z}) > 0 ||
                grid.sdf(int3{x, y, zm1}) > 0 ||
                grid.sdf(int3{x, y, zp1}) > 0) {
              sign = 1.0f;
            }
            grid.values[flatCoord] = d_new * sign;
          }
        }
      }
    }
  }
}

SDFGrid makeGridFromMesh(LiteMath::uint3 size, const cmesh4::SimpleMesh &mesh) {
  float3 sizef = {static_cast<float>(size.x), static_cast<float>(size.y),
                  static_cast<float>(size.z)};

  SDFGrid result;
  result.size = size;
  result.values = std::vector<float>(size.x * size.y * size.z,
                                     std::numeric_limits<float>::infinity());

  for (size_t trID = 0; trID < mesh.TrianglesNum(); ++trID) {
    float3 vertices[3] = {to_float3(mesh.vPos4f[mesh.indices[trID * 3]]),
                          to_float3(mesh.vPos4f[mesh.indices[trID * 3 + 1]]),
                          to_float3(mesh.vPos4f[mesh.indices[trID * 3 + 2]])};

    BBox3f trBox;
    trBox.boxMin = float3{std::numeric_limits<float>::infinity()};
    trBox.boxMax = -trBox.boxMin;

    trBox = update_box(trBox, vertices[0]);
    trBox = update_box(trBox, vertices[1]);
    trBox = update_box(trBox, vertices[2]);

    float3 minCoordf = floor((trBox.boxMin + 1.0f) / 2.0f * (sizef - 1.0f));
    minCoordf = clamp(minCoordf-1.0f, float3{0.0f}, sizef-1.0f);
    float3 maxCoordf = ceil((trBox.boxMax + 1.0f) / 2.0f * (sizef - 1.0f));
    maxCoordf = clamp(maxCoordf+1.0f, float3{0.0f}, sizef-1.0f);

    uint3 minCoord = {static_cast<uint32_t>(minCoordf.x),
                      static_cast<uint32_t>(minCoordf.y),
                      static_cast<uint32_t>(minCoordf.z)};
    uint3 maxCoord = {static_cast<uint32_t>(maxCoordf.x),
                      static_cast<uint32_t>(maxCoordf.y),
                      static_cast<uint32_t>(maxCoordf.z)};

    uint3 areaSize = maxCoord - minCoord + 1;
    uint32_t areaCount = areaSize.x * areaSize.y * areaSize.z;
    for (uint32_t flatCoord = 0; flatCoord < areaCount; ++flatCoord) {
      uint3 coord = {};
      coord.x = flatCoord % areaSize.x;
      coord.y = (flatCoord / areaSize.x) % areaSize.y;
      coord.z = flatCoord / (areaSize.x * areaSize.y);
      coord += minCoord;

      float3 point = {static_cast<float>(coord.x), static_cast<float>(coord.y),
                      static_cast<float>(coord.z)};
      point /= sizef;
      point *= 2.0f;
      point -= 1.0f;

      float3 trPoint =
          closestPointTriangle(point, vertices[0], vertices[1], vertices[2]);

      float3 pointToTriangle = trPoint - point;
      float3 normal =
          cross(vertices[1] - vertices[0], vertices[2] - vertices[0]);
      float sign = dot(normal, -pointToTriangle) >= 0 ? 1.0f : -1.0f;
      float distance = length(pointToTriangle);

      if (std::abs(result.sdf(coord)) > distance) {
        result.sdf(coord) = distance * sign;
      }
    }
  }

  redistancing(result);

  return result;
}
