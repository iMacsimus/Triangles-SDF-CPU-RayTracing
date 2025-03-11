#include "chrono"

#include "raytracing.hpp"

using namespace LiteMath;
using namespace LiteImage;

inline float3 Lambert(const float3 &lightDir, const float3 &normal,
                      const float3 &albedo) {
  return std::max(dot(-lightDir, normal), 0.0f) * albedo;
}

std::pair<float4, float>
Renderer::intersectionColor(const IScene &scene, const float3 &rayPos,
                            const float3 &rayDir, float tNear, float tFar,
                            float tPrev, int maxDepth) const {
  auto hit = scene.intersect(rayPos, rayDir, tNear, std::min(tFar, tPrev));
  if (!hit.hitten) {
    return {float4(0.0f, 0.0f, 0.0f, 1.0f),
            std::numeric_limits<float>::infinity()};
  }

  if (dot(hit.normal, rayDir) > 0) {
    hit.normal *= -1.0f;
  }

  float4 color;
  if (shadingMode == ShadingMode::Normal) {
    color = to_float4(hit.normal, 1.0f);
    color = (color + 1.0f) / 2.0f;
  } else if (shadingMode == ShadingMode::Color) {
    color = to_float4(hit.albedo, 1.0f);
  } else if (shadingMode == ShadingMode::Lambert) {
    bool lightIsVisible = true;
    float3 point = rayPos + hit.t * rayDir;
    if (enableShadows) {
      float3 shadowDir = normalize(lightPos - point);
      HitInfo shadowHit =
          scene.intersect(point + shadowDir * 0.1f, shadowDir, 0.01f, 100.0f);
      lightIsVisible = !shadowHit.hitten;
    }
    if (!lightIsVisible) {
      color = to_float4(hit.albedo * 0.1f, 1.0f);
    } else {
      color = to_float4(
          LiteMath::min(hit.albedo * 0.1f + Lambert(normalize(point - lightPos),
                                                    hit.normal, hit.albedo),
                        float3(1.0f)),
          1.0f);
    }
    if (enableReflections) {
      if (enableReflections && maxDepth > 1 && hit.reflectiveness > 0.0f) {
        float3 reflectDir = normalize(reflect(rayDir, hit.normal));
        float tmp = std::numeric_limits<float>::infinity();
        float4 reflectedColor =
            intersectionColor(scene, point + 0.02f * reflectDir, reflectDir,
                              0.01f, 100.0f, tmp, maxDepth - 1)
                .first;
        color = color * (1.0f - hit.reflectiveness) + hit.reflectiveness *
                reflectedColor;
      }
    }
  }
  return {color, hit.t};
}

float Renderer::draw(const IScene &scene, FrameBuffer &frameBuffer,
                     const Camera &camera,
                     const LiteMath::float4x4 projInv) const {
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
      auto [color, tNew] =
          intersectionColor(scene, rayPos, rayDir, 0.01f, 100.0f, tBuf[xy]);
      if (!std::isinf(tNew)) {
        tBuf[xy] = tNew;
        colorBuf[xy] = color_pack_rgba(color);
      }
    }
  }
  auto e = std::chrono::high_resolution_clock::now();
  return static_cast<float>(
             std::chrono::duration_cast<std::chrono::microseconds>(e - b)
                 .count()) /
         1e3f;
}