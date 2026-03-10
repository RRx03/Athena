#pragma once
#include "ProblemDefinition.hpp"
#include <simd/simd.h>

class CoordinateResolver {
public:
  struct Frame {
    simd::float3 origin;
    simd::float3 axis;
    simd::float3 up;
    simd::float3 right;
  };
  static Frame resolveFrame(const ProblemDefinition &problem);
  static simd::float3 toSolverFrame(simd::float3 p, const Frame &f);
  static simd::float3 toUserFrame(simd::float3 p, const Frame &f);
};
