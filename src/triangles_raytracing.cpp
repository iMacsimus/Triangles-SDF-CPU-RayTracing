#include "triangles_raytracing.hpp"

using namespace LiteMath;
using namespace LiteImage;

void trace_triangles(LiteImage::Image2D<uint32_t> &buffer,
                     [[maybe_unused]] const cmesh4::SimpleMesh &mesh,
                     const Camera &camera, LiteMath::float4x4 projInv) {
  int width = buffer.width();
  int height = buffer.height();

  float3 rayPos = camera.position();
  auto viewMatrix = camera.lookAtMatrix();
  auto viewInv = inverse4x4(viewMatrix);

#ifdef NDEBUG
#pragma omp parallel for schedule(dynamic)
#endif
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      float4 rayDir4 = EyeRayDir4f(static_cast<float>(x), static_cast<float>(y),
                                   static_cast<float>(width),
                                   static_cast<float>(height), projInv);
      rayDir4.w = 0.0f;
      rayDir4 = viewInv * rayDir4;
      float3 rayDir = to_float3(rayDir4);

      BBox3f bbox = {float3{-1.0f}, float3{1.0f}};

      auto hit = bbox.Intersection(rayPos, 1.0f / rayDir, 0,
                                   std::numeric_limits<float>::infinity());

      if (hit.t1 > hit.t2)
        continue;

      float3 color{0.0f};
      color[hit.face] = 1.0f;

      buffer[int2{x, height - 1 - y}] =
          LiteMath::color_pack_bgra(to_float4(color, 1.0f));
    }
  }
}