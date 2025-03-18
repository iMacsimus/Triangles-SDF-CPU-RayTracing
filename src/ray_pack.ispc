typedef float<4> float4;
typedef float<3> float3;
typedef float<2> float2;

inline
float3 normalize(const float3 &ray)
{
  float length = sqrt(ray[0]*ray[0] + ray[1]*ray[1] + ray[2]*ray[2]);
  return ray / length;
}

inline float3 shuffle_yzx(float3 a) { float3 res = { a.y, a.z, a.x}; return res; }

inline
float3 cross(const float3 &a, const float3 &b)
{
  const float3 a_yzx = shuffle_yzx(a);
  const float3 b_yzx = shuffle_yzx(b);
  return shuffle_yzx(a*b_yzx - a_yzx*b);
}

inline
float3 cross(const uniform float3 &a, const float3 &b)
{
  const float3 a_yzx = shuffle_yzx(a);
  const float3 b_yzx = shuffle_yzx(b);
  return shuffle_yzx(a*b_yzx - a_yzx*b);
}

inline
float dot(const float4 &a, const float4 &b)
{
  return a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w;
}

inline
float dot(const float3 &a, const float3 &b)
{
  return a.x*b.x+a.y*b.y+a.z*b.z;
}

inline
float dot(const uniform float3 &a, const float3 &b)
{
  return a.x*b.x+a.y*b.y+a.z*b.z;
}

inline 
float4 to_float4(const float3 &src, float w) {
  float4 res = { src.x, src.y, src.z, w };
  return res;
}

inline 
float3 to_float3(const float4 &src) {
  float3 res = { src.x, src.y, src.z };
  return res;
}

inline 
uniform float3 to_float3(const uniform float4 &src) {
  uniform float3 res = { src.x, src.y, src.z };
  return res;
}

inline 
float length(float2 val) {
  return sqrt(val.x*val.x+val.y*val.y);
}

struct Ray8
{
  float orig_x[8];
  float orig_y[8];
  float orig_z[8];
  float dir_x[8];
  float dir_y[8];
  float dir_z[8];
};

struct HitInfo8
{
  uint hitten[8];
  float t[8];
  float norm_x[8];
  float norm_y[8];
  float norm_z[8];
};

struct HitInfo
{
  uint hitten;
  float t;
  float3 norm;
};

static HitInfo triangle_intersection(const varying float3 &orig, 
                              const varying float3 &dir,
                              const uniform float3 &v0, 
                              const uniform float3 &v1,
                              const uniform float3 &v2) {
  HitInfo res;
  res.hitten = false;
  res.t = -1.0f;
  float3 e1 = v1 - v0;
  float3 e2 = v2 - v0;
  res.norm = normalize(cross(e1, e2));
  float3 pvec = cross(dir, e2);
  float det = dot(e1, pvec);

  if (det < 1e-8f && det > -1e-8f) {
    return res;
  }

  float inv_det = 1 / det;
  float3 tvec = orig - v0;
  float u = dot(tvec, pvec) * inv_det;
  if (u < 0.0f || u > 1.0f) {
    return res;
  }

  float3 qvec = cross(tvec, e1);
  float v = dot(dir, qvec) * inv_det;
  if (v < 0.0f || u + v > 1.0f) {
    return res;
  }
  res.t = dot(e2, qvec) * inv_det;
  res.hitten = true;
  return res;
}

static HitInfo triangle_intersection(const uniform float3 &orig, 
                              const uniform float3 &dir,
                              const varying float3 &v0, 
                              const varying float3 &v1,
                              const varying float3 &v2) {
  HitInfo res;
  res.hitten = false;
  res.t = -1.0f;
  float3 e1 = v1 - v0;
  float3 e2 = v2 - v0;
  res.norm = normalize(cross(e1, e2));
  float3 pvec = cross(dir, e2);
  float det = dot(e1, pvec);

  if (det < 1e-8f && det > -1e-8f) {
    return res;
  }

  float inv_det = 1 / det;
  float3 tvec = orig - v0;
  float u = dot(tvec, pvec) * inv_det;
  if (u < 0.0f || u > 1.0f) {
    return res;
  }

  float3 qvec = cross(tvec, e1);
  float v = dot(dir, qvec) * inv_det;
  if (v < 0.0f || u + v > 1.0f) {
    return res;
  }
  res.t = dot(e2, qvec) * inv_det;
  res.hitten = true;
  return res;
}

export
void intersect_triangles_8(
    const Ray8 * uniform pRays, 
    HitInfo8 * uniform pHitInfo,
    const float4 * uniform vPos4f,
    const uint * uniform indices,
    uniform uint indicesCount) {
  uniform uint trianglesCount = indicesCount/3;
  foreach(rayID = 0...8) {
    float3 rayPos = { pRays->orig_x[rayID], pRays->orig_y[rayID], pRays->orig_z[rayID] };
    float3 rayDir = { pRays->dir_x[rayID], pRays->dir_y[rayID], pRays->dir_z[rayID] };
    HitInfo res;
    res.hitten = false;
    res.t = 0.0f;
    for (uniform uint i = 0; i < trianglesCount; ++i) {
      uniform float4 v0 = vPos4f[indices[3 * i]];
      uniform float4 v1 = vPos4f[indices[3 * i + 1]];
      uniform float4 v2 = vPos4f[indices[3 * i + 2]];
      v0 /= v0.w;
      v1 /= v1.w;
      v2 /= v2.w;
      HitInfo cur = triangle_intersection(rayPos, rayDir, to_float3(v0), to_float3(v1), to_float3(v2));
      if (cur.t <= 0)
        continue;
        
      if (!res.hitten || res.t > cur.t)
        res = cur;
    }

    pHitInfo->hitten[rayID] = res.hitten;
    pHitInfo->t[rayID] = res.t;
    pHitInfo->norm_x[rayID] = res.norm.x;
    pHitInfo->norm_y[rayID] = res.norm.y;
    pHitInfo->norm_z[rayID] = res.norm.z;
  }
}

struct Box8
{
  float xMin[8];
  float yMin[8];
  float zMin[8];
  float xMax[8];
  float yMax[8];
  float zMax[8];
};

struct Box 
{
  float boxMin[3];
  float boxMax[3];
};

export void divide_box_8(const Box * uniform box, Box8 * uniform result) {
  uniform float3 boxMin = { box->boxMin[0], box->boxMin[1], box->boxMin[2] };
  uniform float3 boxMax = { box->boxMax[0], box->boxMax[1], box->boxMax[2] };
  uniform float3 center = (boxMin+boxMax) / 2.0f;
  uniform float3 minMaxDiff = center - boxMin;

  foreach(id = 0...8) {
    int x = id >> 2;
    int y = (id & 0b11) >> 1;
    int z = id & 0b1;

    result->xMin[id] = (x == 0) ? boxMin.x : center.x;
    result->yMin[id] = (y == 0) ? boxMin.y : center.y;
    result->zMin[id] = (z == 0) ? boxMin.z : center.z;

    result->xMax[id] = result->xMin[id]+minMaxDiff.x;
    result->yMax[id] = result->yMin[id]+minMaxDiff.y;
    result->zMax[id] = result->zMin[id]+minMaxDiff.z;
  }
}

export
void intersect_box_8(
    const Box8 * uniform pBoxes,
    const float uniform orig[3],
    const float uniform dirInverted[3],
    float uniform tNear, float uniform tFar,
    float uniform pResults[8]) {
  float3 rayPos = { orig[0], orig[1], orig[2] };
  float3 invDir = { dirInverted[0], dirInverted[1], dirInverted[2] };

  foreach(boxID = 0...8) {
    float3 boxMin = { pBoxes->xMin[boxID], pBoxes->yMin[boxID], pBoxes->zMin[boxID] };
    float3 boxMax = { pBoxes->xMax[boxID], pBoxes->yMax[boxID], pBoxes->zMax[boxID] };

    float3 t1 = (boxMin-rayPos) * invDir; //TODO safe invDir
    float3 t2 = (boxMax-rayPos) * invDir;

    float3 tMin3 = { min(t1.x, t2.x), min(t1.y, t2.y), min(t1.z, t2.z) };
    float3 tMax3 = { max(t1.x, t2.x), max(t1.y, t2.y), max(t1.z, t2.z) };

    float tMin = max(tMin3.x, max(tMin3.y, tMin3.z));
    float tMax = min(tMax3.x, min(tMax3.y, tMax3.z));

    tMin = max(tMin, tNear);
    tMax = min(tMax, tFar);

    if (tMax < 0 || tMin > tMax) {
      pResults[boxID] = -1.0f;
    } else {
      pResults[boxID] = tMin;
    }
  }
}

struct Point8
{
  float x[8];
  float y[8];
  float z[8];
};

struct Triangle8
{
  Point8 v0;
  Point8 v1;
  Point8 v2;
};

export
void intersect_1_ray_8_triangles(
    const Triangle8 * uniform pTriangles,
    const float uniform orig[3],
    const float uniform dir[3],
    HitInfo8 * uniform pResults) {
  uniform float3 rayPos = { orig[0], orig[1], orig[2] };
  uniform float3 rayDir = { dir[0], dir[1], dir[2] };

  foreach(trID = 0...8) {
    float3 v0 = { pTriangles->v0.x[trID], pTriangles->v0.y[trID], pTriangles->v0.z[trID] };
    float3 v1 = { pTriangles->v1.x[trID], pTriangles->v1.y[trID], pTriangles->v1.z[trID] };
    float3 v2 = { pTriangles->v2.x[trID], pTriangles->v2.y[trID], pTriangles->v2.z[trID] };

    HitInfo hit = triangle_intersection(rayPos, rayDir, v0, v1, v2);
    pResults->hitten[trID] = hit.hitten;
    pResults->t[trID] = hit.t;
    pResults->norm_x[trID] = hit.norm.x;
    pResults->norm_y[trID] = hit.norm.y;
    pResults->norm_z[trID] = hit.norm.z;
  }
}