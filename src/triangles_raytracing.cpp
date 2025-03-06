#include <chrono>
#include <omp.h>
#include <random>

#include "triangles_raytracing.hpp"
#include <ray_pack_ispc.h>

using namespace LiteMath;
using namespace LiteImage;

constexpr float EMPTY_NODE_TRAVERSE_COST = 0.2f;

struct HitInfo {
  bool hitten = false;
  float t;
  float3 normal;
};

float TrivialMeshRenderer::draw(LiteImage::Image2D<uint32_t> &buffer) const {
  int width = buffer.width();
  int height = buffer.height();
  BBox3f bbox = calc_bbox(mesh);
  float3 rayPos = camera.position();
  auto viewMatrix = camera.lookAtMatrix();
  auto viewInv = inverse4x4(viewMatrix);
  auto b = std::chrono::high_resolution_clock::now();
#ifdef NDEBUG
#pragma omp parallel for schedule(dynamic)
#endif
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; x += 8) {
      ispc::Ray8 rays;
      bool someOneIntersects = false;
      for (int rayID = 0; rayID < 8; ++rayID) {
        rays.orig_x[rayID] = rayPos.x;
        rays.orig_y[rayID] = rayPos.y;
        rays.orig_z[rayID] = rayPos.z;
        float4 rayDir4 = EyeRayDir4f(
            static_cast<float>(x + rayID), static_cast<float>(y),
            static_cast<float>(width), static_cast<float>(height), projInv);
        rayDir4.w = 0.0f;
        rayDir4 = viewInv * rayDir4;
        float3 rayDir = to_float3(rayDir4);
        rays.dir_x[rayID] = rayDir.x;
        rays.dir_y[rayID] = rayDir.y;
        rays.dir_z[rayID] = rayDir.z;

        auto boxHit = bbox.Intersection(rayPos, 1.0f / rayDir, 0.0f, 100.0f);
        if (boxHit.t1 <= boxHit.t2) {
          someOneIntersects = true;
        }
      }

      if (!someOneIntersects)
        continue;

      ispc::HitInfo8 hits;
      ispc::intersect_triangles_8(
          &rays, &hits,
          reinterpret_cast<const ispc::float4 *>(mesh.vPos4f.data()),
          mesh.indices.data(), static_cast<uint32_t>(mesh.indices.size()));

      for (int rayID = 0; rayID < 8; ++rayID) {
        if ((x + rayID >= width) || !hits.hitten[rayID])
          continue;
        float3 color = {hits.norm_x[rayID], hits.norm_y[rayID],
                        hits.norm_z[rayID]};
        color += 1.0f;
        color /= 2.0f;
        buffer[int2{x + rayID, height - y - 1}] =
            color_pack_rgba(to_float4(color, 1.0f));
      }
    }
  }
  auto e = std::chrono::high_resolution_clock::now();
  return static_cast<float>(
             std::chrono::duration_cast<std::chrono::microseconds>(e - b)
                 .count()) /
         1e3f;
}

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
    std::sort(reinterpret_cast<Triple *>(indices.data() + start),
              reinterpret_cast<Triple *>(indices.data() + end),
              makeComp<0>(m_mesh));
    break;
  case 1:
    std::sort(reinterpret_cast<Triple *>(indices.data() + start),
              reinterpret_cast<Triple *>(indices.data() + end),
              makeComp<1>(m_mesh));
    break;
  case 2:
    std::sort(reinterpret_cast<Triple *>(indices.data() + start),
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
  auto &node = m_nodes[offset];
  size_t dividors[7] = {};
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
  assert(dividorsCount <= 7);
  dividorsCount = std::min(dividorsCount, 7ul);

  if (dividorsCount == 0) {
    node.isLeaf = true;
    node.leafInfo.startIndex = static_cast<uint32_t>(start);
    node.leafInfo.count = static_cast<uint32_t>(end - start);
    return;
  }

  node.isLeaf = false;
  node.children.offset = static_cast<uint32_t>(m_nodes.size());
  node.children.realCount = static_cast<uint32_t>(dividorsCount + 1);
  std::sort(dividors, dividors + dividorsCount);
  for (size_t child = 0; child < dividorsCount + 1; ++child) {
    m_nodes.push_back({});
    size_t leftBound = (child == 0) ? start : dividors[child - 1];
    size_t rightBound = (child == dividorsCount) ? end : dividors[child];
    BBox3f childBox = calc_bbox(m_mesh, leftBound, rightBound);
    node.children.boxes.xMin[child] = childBox.boxMin.x;
    node.children.boxes.yMin[child] = childBox.boxMin.y;
    node.children.boxes.zMin[child] = childBox.boxMin.z;
    node.children.boxes.xMax[child] = childBox.boxMax.x;
    node.children.boxes.yMax[child] = childBox.boxMax.y;
    node.children.boxes.zMax[child] = childBox.boxMax.z;
  }

  for (size_t child = 0; child < dividorsCount + 1; ++child) {
    size_t leftBound = (child == 0) ? start : dividors[child - 1];
    size_t rightBound = (child == dividorsCount) ? end : dividors[child];
    createNode(node.children.offset + child, leftBound, rightBound);
  }
}

void BVHBuilder::perform(cmesh4::SimpleMesh mesh) {
  m_mesh = std::move(mesh);

  m_leftBoxes.resize(m_mesh.TrianglesNum());
  m_rightBoxes.resize(m_mesh.TrianglesNum());
  m_indicesY.resize(m_mesh.IndicesNum());
  m_indicesZ.resize(m_mesh.IndicesNum());

  m_nodes = {BVH8Node{}};
  createNode(0, 0, m_mesh.indices.size());

  m_leftBoxes.clear();
  m_rightBoxes.clear();
  m_indicesY.clear();
  m_indicesZ.clear();

  m_leftBoxes.shrink_to_fit();
  m_rightBoxes.shrink_to_fit();
  m_indicesY.shrink_to_fit();
  m_indicesZ.shrink_to_fit();
}