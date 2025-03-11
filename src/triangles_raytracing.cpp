#include <execution>
#include <chrono>
#include <omp.h>
#include <random>

#include "triangles_raytracing.hpp"
#include <ray_pack_ispc.h>

using namespace LiteMath;
using namespace LiteImage;

constexpr float EMPTY_NODE_TRAVERSE_COST = 0.2f;
struct DivisionResult {
  bool isDivided = false;
  size_t dividerId = -1;
};

struct Triple {
  uint32_t indices[3];
};

template <size_t Axes> auto makeComp(cmesh4::SimpleMesh &mesh) {
  return [&mesh](const Triple &tr1, const Triple &tr2) {
    auto box1 = calc_bbox(mesh, tr1.indices);
    auto box2 = calc_bbox(mesh, tr2.indices);
    return box1.boxMax[Axes] < box2.boxMax[Axes];
  };
}

BVHBuilder::DivisionResult BVHBuilder::tryDivide(std::vector<uint32_t> &indices,
                                                 size_t start, size_t end,
                                                 uint32_t axes) {
  switch (axes) {
  case 0:
    std::sort(std::execution::par_unseq, reinterpret_cast<Triple *>(indices.data() + start),
              reinterpret_cast<Triple *>(indices.data() + end),
              makeComp<0>(m_mesh));
    break;
  case 1:
    std::sort(std::execution::par_unseq, reinterpret_cast<Triple *>(indices.data() + start),
              reinterpret_cast<Triple *>(indices.data() + end),
              makeComp<1>(m_mesh));
    break;
  case 2:
    std::sort(std::execution::par_unseq, reinterpret_cast<Triple *>(indices.data() + start),
              reinterpret_cast<Triple *>(indices.data() + end),
              makeComp<2>(m_mesh));
    break;
  default:
    break;
  }

  for (size_t boxID = start / 3; boxID != end / 3; ++boxID) {
    auto &box = m_leftBoxes[boxID];
    if (boxID == start / 3) {
      box.boxMin = float3{std::numeric_limits<float>::infinity()};
      box.boxMax = -box.boxMin;
    } else {
      box = m_leftBoxes[boxID - 1];
    }

    box = update_box(box, m_mesh.vPos4f[indices[boxID * 3]]);
    box = update_box(box, m_mesh.vPos4f[indices[boxID * 3 + 1]]);
    box = update_box(box, m_mesh.vPos4f[indices[boxID * 3 + 2]]);
  }

  for (size_t reversedID = start / 3; reversedID != end / 3; ++reversedID) {
    size_t boxID = end / 3 - reversedID + start / 3 - 1;
    auto &box = m_rightBoxes[boxID];
    if (reversedID == start / 3) {
      box.boxMin = float3{std::numeric_limits<float>::infinity()};
      box.boxMax = -box.boxMin;
    } else {
      box = m_rightBoxes[boxID + 1];
    }

    box = update_box(box, m_mesh.vPos4f[indices[boxID * 3]]);
    box = update_box(box, m_mesh.vPos4f[indices[boxID * 3 + 1]]);
    box = update_box(box, m_mesh.vPos4f[indices[boxID * 3 + 2]]);
  }

  DivisionResult result;
  result.sah = static_cast<float>(end - start) / 3.0f;
  float parentSurfaceArea = surfaceArea(m_leftBoxes[end / 3 - 1]);
  for (size_t divider = start + 3; divider < end; divider += 3) {
    auto leftBox = m_leftBoxes[divider / 3 - 1];
    auto rightBox = m_rightBoxes[divider / 3];
    float leftCount = static_cast<float>(divider - start) / 3.0f;
    float rightCount = static_cast<float>(end - start) / 3.0f - leftCount;
    float curSAH = EMPTY_NODE_TRAVERSE_COST +
                   surfaceArea(leftBox) / parentSurfaceArea * leftCount +
                   surfaceArea(rightBox) / parentSurfaceArea * rightCount;
    if (curSAH < result.sah) {
      result.sah = curSAH;
      result.dividerId = divider;
      result.isDivided = true;
    }
  }

  // align to 8
  if (result.isDivided && (result.dividerId - start) % 24 != 0) {
    size_t divider1 = (result.dividerId - 1) / 24 * 24;
    size_t divider2 = ((result.dividerId - 1) / 24 + 1) * 24;
    size_t nearest =
        (result.dividerId - divider1 <= divider2 - result.dividerId) ? divider1
                                                                     : divider2;
    size_t other = divider1 + divider2 - nearest;

    if (start < nearest && nearest < end) {
      result.dividerId = nearest;
    } else if (start < other && other < end) {
      result.dividerId = other;
    }
  }

  return result;
}

BVHBuilder::DivisionResult BVHBuilder::tryDivide(size_t start, size_t end) {
  if (end - start <= 8 * 3) {
    return DivisionResult{};
  }

  auto &indicesX = m_mesh.indices;
  auto &indicesY = m_indicesY;
  auto &indicesZ = m_indicesZ;

  std::copy(indicesX.begin() + start, indicesX.begin() + end,
            indicesY.begin() + start);
  std::copy(indicesX.begin() + start, indicesX.begin() + end,
            indicesZ.begin() + start);

  auto curSAH = static_cast<float>(end - start) / 3.0f;

  DivisionResult divX = tryDivide(m_mesh.indices, start, end, 0);
  DivisionResult divY = tryDivide(m_indicesY, start, end, 1);
  DivisionResult divZ = tryDivide(m_indicesZ, start, end, 2);

  float minSAH = std::min({curSAH, divX.sah, divY.sah, divZ.sah});
  if (divX.sah == minSAH) {
    return divX;
  } else if (divY.sah == minSAH) {
    std::copy(indicesY.begin() + start, indicesY.begin() + end,
              indicesX.begin() + start);
    return divY;
  } else if (divZ.sah == minSAH) {
    std::copy(indicesZ.begin() + start, indicesZ.begin() + end,
              indicesX.begin() + start);
    return divZ;
  }

  return DivisionResult{};
}

void BVHBuilder::createNode(size_t offset, size_t start, size_t end) {
  BVH8Node node;

  size_t dividors[20] = {};
  size_t dividorsCount = 0;
  ChipQueue<std::pair<size_t, size_t>, 40> candidates;
  candidates.tryEnqueue({start, end});
  while (!candidates.isEmpty()) {
    auto [curStart, curEnd] = candidates.dequeue();
    if (dividorsCount == 7) {
      break;
    }
    auto result = tryDivide(curStart, curEnd);
    if (result.isDivided) {
      dividors[dividorsCount++] = result.dividerId;
      candidates.tryEnqueue({curStart, result.dividerId});
      candidates.tryEnqueue({result.dividerId, curEnd});
    }
  }

  if (dividorsCount == 0) {
    if (end - start > 24) { // maximum 8 triangles per list
      dividors[dividorsCount++] = ((start / 3 + end / 3) / 2) * 3;
    } else {
      node.isLeaf = true;
      node.leafInfo.startIndex = static_cast<uint32_t>(start);
      node.leafInfo.count = static_cast<uint32_t>(end - start);
#pragma omp critical
      {
        m_nodes[offset] = node;
      }
      return;
    }
  }

  assert(dividorsCount <= 7);
  dividorsCount = std::min(dividorsCount, 7ul);

  node.isLeaf = false;
  node.children.realCount = static_cast<uint32_t>(dividorsCount + 1);
  std::sort(dividors, dividors + dividorsCount);
  for (size_t child = 0; child < dividorsCount + 1; ++child) {
    size_t leftBound = (child == 0) ? start : dividors[child - 1];
    size_t rightBound = (child == dividorsCount) ? end : dividors[child];
    BBox3f childBox = calc_bbox(m_mesh, leftBound, rightBound);
    assert(all_of(childBox.boxMin < childBox.boxMax));
    node.children.boxes.xMin[child] = childBox.boxMin.x;
    node.children.boxes.yMin[child] = childBox.boxMin.y;
    node.children.boxes.zMin[child] = childBox.boxMin.z;
    node.children.boxes.xMax[child] = childBox.boxMax.x;
    node.children.boxes.yMax[child] = childBox.boxMax.y;
    node.children.boxes.zMax[child] = childBox.boxMax.z;
  }

#pragma omp critical
  {
    node.children.offset = static_cast<uint32_t>(m_nodes.size());
    for (size_t child = 0; child < dividorsCount + 1; ++child)
      m_nodes.emplace_back();
    m_nodes[offset] = node;
  }

  for (size_t child = 0; child < dividorsCount + 1; ++child) {
    size_t leftBound = (child == 0) ? start : dividors[child - 1];
    size_t rightBound = (child == dividorsCount) ? end : dividors[child];
#pragma omp task
    {
      createNode(node.children.offset + child, leftBound, rightBound);
    }
  }
}

void BVHBuilder::perform(cmesh4::SimpleMesh mesh) {
  auto b = std::chrono::high_resolution_clock::now();
  m_mesh = std::move(mesh);

  m_leftBoxes.resize(m_mesh.TrianglesNum());
  m_rightBoxes.resize(m_mesh.TrianglesNum());
  m_indicesY.resize(m_mesh.IndicesNum());
  m_indicesZ.resize(m_mesh.IndicesNum());

  m_nodes = {BVH8Node{}};
  m_nodes.reserve(m_mesh.TrianglesNum() * 2);
#pragma omp parallel num_threads(omp_get_max_threads())
  {
#pragma omp single
    {
      createNode(0, 0, m_mesh.indices.size());
    }
  }

  m_leftBoxes.clear();
  m_rightBoxes.clear();
  m_indicesY.clear();
  m_indicesZ.clear();

  m_leftBoxes.shrink_to_fit();
  m_rightBoxes.shrink_to_fit();
  m_indicesY.shrink_to_fit();
  m_indicesZ.shrink_to_fit();
  auto e = std::chrono::high_resolution_clock::now();
  float t = static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(e-b).count())/1e3f;
  std::cout << "BVH construction: " << t << "ms" << std::endl;
}

HitInfo BVHBuilder::intersect(const LiteMath::float3 &rayPos,
                              const LiteMath::float3 &rayDir, float tNear,
                              float tFar) const {
  return traverseNode(0, rayPos, rayDir, tNear, tFar);
}

#define SWAP(x, y)                                                             \
  if (t[x] > t[y]) {                                                           \
    std::swap(t[x], t[y]);                                                     \
    std::swap(children[x], children[y]);                                       \
  }
void sort8(float t[8], size_t children[8]) {
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

HitInfo BVHBuilder::traverseNode(size_t index, LiteMath::float3 rayPos,
                                 LiteMath::float3 rayDir, float tNear,
                                 float tFar) const {
  auto &node = m_nodes[index];
  HitInfo result;
  if (!node.isLeaf) {
    float t[8] = {};
    float3 invDir = 1.0f / rayDir;
    ispc::intersect_box_8(
        reinterpret_cast<const ispc::Box8 *>(&node.children.boxes), rayPos.M,
        invDir.M, tNear, tFar, t);
    size_t children[8] = {0, 1, 2, 3, 4, 5, 6, 7};
    sort8(t, children);

    for (size_t i = 0; i < 8; ++i) {
      size_t childID = children[i];
      if (childID >= node.children.realCount || (t[i] < 0) ||
          (result.hitten && result.t < t[i]))
        continue;

      HitInfo cur = traverseNode(node.children.offset + childID, rayPos, rayDir,
                                 tNear, tFar);
      if (cur.hitten && (!result.hitten || result.t > cur.t)) {
        result = cur;
      }
    }
  } else {
    uint32_t start = node.leafInfo.startIndex;
    uint32_t end = start + node.leafInfo.count;
    uint32_t trianglesCount = (end - start) / 3;
    assert(trianglesCount <= 8);
    trianglesCount = std::min(trianglesCount, 8u);

    ispc::Triangle8 triagles = {};
    for (uint32_t trID = 0; trID < trianglesCount; ++trID) {
      uint32_t i0 = m_mesh.indices[start + trID * 3];
      uint32_t i1 = m_mesh.indices[start + trID * 3 + 1];
      uint32_t i2 = m_mesh.indices[start + trID * 3 + 2];
      float4 v0 = m_mesh.vPos4f[i0];
      float4 v1 = m_mesh.vPos4f[i1];
      float4 v2 = m_mesh.vPos4f[i2];
      v0 /= v0.w;
      v1 /= v1.w;
      v2 /= v2.w;

      triagles.v0.x[trID] = v0.x;
      triagles.v0.y[trID] = v0.y;
      triagles.v0.z[trID] = v0.z;
      triagles.v1.x[trID] = v1.x;
      triagles.v1.y[trID] = v1.y;
      triagles.v1.z[trID] = v1.z;
      triagles.v2.x[trID] = v2.x;
      triagles.v2.y[trID] = v2.y;
      triagles.v2.z[trID] = v2.z;
    }
    ispc::HitInfo8 hits;
    ispc::intersect_1_ray_8_triangles(&triagles, rayPos.M, rayDir.M, &hits);

    for (size_t trID = 0; trID < trianglesCount; ++trID) {
      if (hits.hitten[trID] && (!result.hitten || result.t > hits.t[trID])) {
        result.hitten = true;
        result.normal = {hits.norm_x[trID], hits.norm_y[trID],
                         hits.norm_z[trID]};
        result.t = hits.t[trID];
      }
    }
  }

  return result;
}