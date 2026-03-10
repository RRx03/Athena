#pragma once
#include <functional>
#include <simd/simd.h>
#include <string>
#include <unordered_map>

class FieldCache {
public:
  void invalidateAll();
  void invalidateField(const std::string &fieldName);
  float getOrCompute(const std::string &field, simd::float3 point,
                     const std::function<float()> &compute);

private:
  struct CacheKey {
    std::string field;
    float x, y, z;
    bool operator==(const CacheKey &o) const;
  };
  struct CacheKeyHash {
    size_t operator()(const CacheKey &k) const;
  };
  std::unordered_map<CacheKey, float, CacheKeyHash> _cache;
};
