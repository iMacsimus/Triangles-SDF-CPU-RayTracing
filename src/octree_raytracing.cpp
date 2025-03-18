#include <fstream>

#include "octree_raytracing.hpp"
#include <ray_pack_ispc.h>

using namespace LiteMath;

void loadSDFOctree(SDFOctree &scene, const std::string &path) {
  std::ifstream fs(path, std::ios::binary);
  unsigned sz = 0;
  fs.read((char *)&sz, sizeof(unsigned));
  scene.nodes.resize(sz);
  fs.read((char *)scene.nodes.data(),
          scene.nodes.size() * sizeof(SDFOctreeNode));
  fs.close();
}

float SDFOctree::nodeSDF(size_t nodeID, const LiteMath::BBox3f &nodeBox,
                         LiteMath::float3 point) const {
  point = (point - nodeBox.boxMin) / (nodeBox.boxMax - nodeBox.boxMin);
  point = clamp(point, float3{0.0000001f}, float3{0.9999999f});

  float3 c0f = floor(point);
  float3 c1f = ceil(point);

  uint3 c0 = {static_cast<uint32_t>(c0f.x), static_cast<uint32_t>(c0f.y),
              static_cast<uint32_t>(c0f.z)};

  uint3 c1 = {static_cast<uint32_t>(c1f.x), static_cast<uint32_t>(c1f.y),
              static_cast<uint32_t>(c1f.z)};

  float3 p_c0f = point - c0f;
  float3 c1f_p = c1f - point;

  float p[8] = {};
  p[0] = nodeSDF(nodeID, uint3{c0.x, c0.y, c0.z});
  p[1] = nodeSDF(nodeID, uint3{c0.x, c0.y, c1.z});
  p[2] = nodeSDF(nodeID, uint3{c0.x, c1.y, c0.z});
  p[3] = nodeSDF(nodeID, uint3{c0.x, c1.y, c1.z});
  p[4] = nodeSDF(nodeID, uint3{c1.x, c0.y, c0.z});
  p[5] = nodeSDF(nodeID, uint3{c1.x, c0.y, c1.z});
  p[6] = nodeSDF(nodeID, uint3{c1.x, c1.y, c0.z});
  p[7] = nodeSDF(nodeID, uint3{c1.x, c1.y, c1.z});

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

constexpr float NORMAL_EPS = 1e-6f;
LiteMath::float3 SDFOctree::nodeNormal(size_t nodeID,
                                       const LiteMath::BBox3f &nodeBox,
                                       LiteMath::float3 point) const {
  point = (point - nodeBox.boxMin) / (nodeBox.boxMax - nodeBox.boxMin);
  point = clamp(point, float3{0.0000001f}, float3{0.9999999f});

  float3 c0f = floor(point);
  float3 c1f = ceil(point);

  uint3 c0 = {static_cast<uint32_t>(c0f.x), static_cast<uint32_t>(c0f.y),
              static_cast<uint32_t>(c0f.z)};

  uint3 c1 = {static_cast<uint32_t>(c1f.x), static_cast<uint32_t>(c1f.y),
              static_cast<uint32_t>(c1f.z)};

  float3 p_c0f = point - c0f;
  float3 c1f_p = c1f - point;
  float3 dp_c0f = float3{1.0f};
  float3 dc1f_p = float3{-1.0f};

  float p[8] = {};
  p[0] = nodeSDF(nodeID, uint3{c0.x, c0.y, c0.z});
  p[1] = nodeSDF(nodeID, uint3{c0.x, c0.y, c1.z});
  p[2] = nodeSDF(nodeID, uint3{c0.x, c1.y, c0.z});
  p[3] = nodeSDF(nodeID, uint3{c0.x, c1.y, c1.z});
  p[4] = nodeSDF(nodeID, uint3{c1.x, c0.y, c0.z});
  p[5] = nodeSDF(nodeID, uint3{c1.x, c0.y, c1.z});
  p[6] = nodeSDF(nodeID, uint3{c1.x, c1.y, c0.z});
  p[7] = nodeSDF(nodeID, uint3{c1.x, c1.y, c1.z});

  float dfdx = p[0] * dc1f_p.x * c1f_p.y * c1f_p.z +
               p[1] * dc1f_p.x * c1f_p.y * p_c0f.z +
               p[2] * dc1f_p.x * p_c0f.y * c1f_p.z +
               p[3] * dc1f_p.x * p_c0f.y * p_c0f.z +
               p[4] * dp_c0f.x * c1f_p.y * c1f_p.z +
               p[5] * dp_c0f.x * c1f_p.y * p_c0f.z +
               p[6] * dp_c0f.x * p_c0f.y * c1f_p.z +
               p[7] * dp_c0f.x * p_c0f.y * p_c0f.z;

  float dfdy = p[0] * c1f_p.x * dc1f_p.y * c1f_p.z +
               p[1] * c1f_p.x * dc1f_p.y * p_c0f.z +
               p[2] * c1f_p.x * dp_c0f.y * c1f_p.z +
               p[3] * c1f_p.x * dp_c0f.y * p_c0f.z +
               p[4] * p_c0f.x * dc1f_p.y * c1f_p.z +
               p[5] * p_c0f.x * dc1f_p.y * p_c0f.z +
               p[6] * p_c0f.x * dp_c0f.y * c1f_p.z +
               p[7] * p_c0f.x * dp_c0f.y * p_c0f.z;

  float dfdz = p[0] * c1f_p.x * c1f_p.y * dc1f_p.z +
               p[1] * c1f_p.x * c1f_p.y * dp_c0f.z +
               p[2] * c1f_p.x * p_c0f.y * dc1f_p.z +
               p[3] * c1f_p.x * p_c0f.y * dp_c0f.z +
               p[4] * p_c0f.x * c1f_p.y * dc1f_p.z +
               p[5] * p_c0f.x * c1f_p.y * dp_c0f.z +
               p[6] * p_c0f.x * p_c0f.y * dc1f_p.z +
               p[7] * p_c0f.x * p_c0f.y * dp_c0f.z;
  
  return normalize(float3{dfdx, dfdy, dfdz});
}

constexpr float HIT_EPS = 1e-4f;

HitInfo SDFOctree::intersectLeaf(size_t nodeID, const LiteMath::BBox3f &nodeBox,
                                 const LiteMath::float3 &rayPos,
                                 const LiteMath::float3 &rayDir, float tNear,
                                 float tFar) const {
  HitInfo result;

  if (nodes[nodeID].isEmpty()) {
    return result; // no hit
  }

  if (std::all_of(nodes[nodeID].values, nodes[nodeID].values + 8,
                  [](float v) { return v >= HIT_EPS; })) {
    return result; // no hit
  }

  auto boxIntersection =
      nodeBox.Intersection(rayPos, 1.0f / rayDir, tNear, tFar);
  if (boxIntersection.t1 > boxIntersection.t2) {
    return result; // no hit
  }

  float t = boxIntersection.t1;
  float3 curPoint = rayPos + t * rayDir;
  curPoint = max(curPoint, nodeBox.boxMin);
  curPoint = min(curPoint, nodeBox.boxMax);

  while (all_of(curPoint <= float3{nodeBox.boxMax}) &&
         all_of(curPoint >= float3{nodeBox.boxMin})) {
    float curSdf = nodeSDF(nodeID, nodeBox, curPoint);

    if (curSdf < HIT_EPS) {
      result.hitten = true;
      result.t = t + curSdf;
      result.normal = nodeNormal(nodeID, nodeBox, curPoint);
      break;
    }

    t += curSdf;
    curPoint = rayPos + t * rayDir;
  }

  return result;
}

HitInfo SDFOctree::intersectNode(size_t nodeID, const LiteMath::float3 &rayPos,
                                 const LiteMath::float3 &rayDir, float tNear,
                                 float tFar,
                                 const LiteMath::BBox3f &nodeBox) const {
  auto &node = nodes[nodeID];
  if (node.isLeaf()) {
    return intersectLeaf(nodeID, nodeBox, rayPos, rayDir, tNear, tFar);
  }

  ispc::Box8 boxesSOA;
  ispc::divide_box_8(reinterpret_cast<const ispc::Box *>(&nodeBox), &boxesSOA);

  float ts[8] = {};
  float3 invDir = 1.0f / rayDir;
  ispc::intersect_box_8(&boxesSOA, rayPos.M, invDir.M, tNear, tFar, ts);
  size_t children[8] = {0, 1, 2, 3, 4, 5, 6, 7};
  sort8(ts, children);

  HitInfo result;
  for (size_t i = 0; i < 8; ++i) {
    size_t childID = children[i];
    float t = ts[i];
    if (t > 0 && (!result.hitten || result.t > t)) {
      BBox3f childBox;
      childBox.boxMin = { boxesSOA.xMin[childID], boxesSOA.yMin[childID], boxesSOA.zMin[childID] };
      childBox.boxMax = { boxesSOA.xMax[childID], boxesSOA.yMax[childID], boxesSOA.zMax[childID] };
      auto childHit = intersectNode(node.childrenOffset + childID, rayPos,
                                    rayDir, tNear, tFar, childBox);
      if (childHit.hitten) {
        result = childHit;
        break;
      }
    }
  }

  return result;
}

HitInfo SDFOctree::intersect(const LiteMath::float3 &rayPos,
                             const LiteMath::float3 &rayDir, float tNear,
                             float tFar) const {
  return intersectNode(0, rayPos, rayDir, tNear, tFar);
}