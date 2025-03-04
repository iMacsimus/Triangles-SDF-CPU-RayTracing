#include "triangles_raytracing.hpp"
#include <omp.h>
#include <random>
#include <ray_pack_ispc.h>

using namespace LiteMath;
using namespace LiteImage;

struct HitInfo {
  bool hitten = false;
  float t;
  float3 normal;
};

void trace_triangles(LiteImage::Image2D<uint32_t> &buffer,
                     const cmesh4::SimpleMesh &mesh, LiteMath::BBox3f bbox,
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
}