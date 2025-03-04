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
  res.t = 0.0f;
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