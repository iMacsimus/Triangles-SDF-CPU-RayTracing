#include "chrono"

#include "raytracing.hpp"

using namespace LiteMath;
using namespace LiteImage;

float Renderer::draw(IScene &scene, FrameBuffer &frameBuffer,
                     const Camera &camera, const LiteMath::float4x4 projInv) {
  auto &[colorBuf, tBuf] = frameBuffer;
  int width = colorBuf.width();
  int height = colorBuf.height();
  float3 rayPos = camera.position();
  auto viewMatrix = camera.lookAtMatrix();
  auto viewInv = inverse4x4(viewMatrix);
  auto b = std::chrono::high_resolution_clock::now();
#ifdef NDEBUG
#pragma omp parallel for schedule(dynamic)
#endif
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int2 xy = {x, height - y - 1};
      float4 rayDir4 = EyeRayDir4f(
          static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f,
          static_cast<float>(width), static_cast<float>(height), projInv);
      rayDir4.w = 0.0f;
      rayDir4 = viewInv * rayDir4;
      float3 rayDir = to_float3(rayDir4);
      auto hit = scene.intersect(rayPos, rayDir, 0.0f, 100.0f);
      if (hit.hitten && hit.t < tBuf[xy]) {
        float4 color = to_float4(hit.normal, 1.0f);
        color += 1.0f;
        color /= 2.0f;
        colorBuf[xy] = color_pack_rgba(color);
        tBuf[xy] = hit.t;
      }
    }
  }
  auto e = std::chrono::high_resolution_clock::now();
  return static_cast<float>(
             std::chrono::duration_cast<std::chrono::microseconds>(e - b)
                 .count()) /
         1e3f;
}