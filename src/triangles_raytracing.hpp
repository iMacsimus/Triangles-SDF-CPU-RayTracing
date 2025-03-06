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

struct Box8 {
  float xMin[8];
  float yMin[8];
  float zMin[8];
  float xMax[8];
  float yMax[8];
  float zMax[8];
};

struct BVHLeafInfo {
  uint32_t startIndex;
  uint32_t count;
};

struct BVH8ChildrenInfo {
  Box8 boxes;
  uint32_t realCount;
  uint32_t offset;
};

struct BVH8Node {
  union {
    BVH8ChildrenInfo children;
    BVHLeafInfo leafInfo;
  };
  bool isLeaf = false;
};

class BVHBuilder {
public:
  void perform(cmesh4::SimpleMesh mesh);
  cmesh4::SimpleMesh &&result() { return std::move(m_mesh); }

private:
  struct DivisionResult {
    bool isDivided = false;
    size_t dividerId = -1;
    float sah = std::numeric_limits<float>::infinity();
  };
  DivisionResult tryDivide(size_t start, size_t end);
  DivisionResult tryDivide(std::vector<uint32_t> &indices, size_t start,
                           size_t end, uint32_t axes);
  void createNode(size_t offset, size_t start, size_t end);

private:
  std::vector<BVH8Node> m_nodes;
  std::vector<LiteMath::BBox3f> m_leftBoxes;
  std::vector<LiteMath::BBox3f> m_rightBoxes;
  std::vector<uint32_t> m_indicesY;
  std::vector<uint32_t> m_indicesZ;
  cmesh4::SimpleMesh m_mesh;
};

template <typename T, int MaxSize> class ChipQueue {
public:
  ChipQueue() = default;

private:
  T data[MaxSize];
  int front = 0;
  int rear = -1;
  int count = 0;

public:
  bool tryEnqueue(T value) {
    if (count == MaxSize) {
      return false;
    }
    rear = (rear + 1) % MaxSize;
    data[rear] = value;
    count++;
    return true;
  }
  T dequeue() {
    T value = data[front];
    front = (front + 1) % MaxSize;
    count--;
    return value;
  }
  int size() const { return count; }
  bool isEmpty() const { return count == 0; }
};