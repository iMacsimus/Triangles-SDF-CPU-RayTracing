#pragma once

#include <LiteMath/Image2d.h>
#include <LiteMath/LiteMath.h>

#include "camera.hpp"
#include "mesh.h"

void trace_triangles(LiteImage::Image2D<uint32_t> &buffer,
                     const cmesh4::SimpleMesh &mesh, 
                     LiteMath::BBox3f bbox,
                     const Camera &camera,
                     LiteMath::float4x4 projInv);