#pragma once

#include <LiteMath/Image2d.h>
#include <LiteMath/LiteMath.h>

#include "camera.hpp"
#include "mesh.h"
#include "raytracing.hpp"

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

class BVHBuilder final: public IScene {
public:
  void perform(cmesh4::SimpleMesh mesh);
  HitInfo intersect(const LiteMath::float3 &rayPos,
                    const LiteMath::float3 &rayDir, float tNear,
                    float tFar) const override;
  cmesh4::SimpleMesh &&result() { return std::move(m_mesh); }
  size_t nodesCount() const noexcept { return m_nodes.size(); }

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
  HitInfo traverseNode(size_t index, LiteMath::float3 rayPos,
                       LiteMath::float3 rayDir, float tNear, float tFar) const;

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