#pragma once
#include "ExpressionEvaluator.hpp"
#include "ProblemDefinition.hpp"
#include <simd/simd.h>
#include <string>

class PhysicsModule {
public:
  virtual ~PhysicsModule() = default;
  virtual std::string name() const = 0;
  virtual void setup(const ProblemDefinition &problem,
                     const ExpressionEvaluator::Context &ctx) = 0;
  virtual void solve(const ExpressionEvaluator::Context &ctx) = 0;
  virtual float fieldAt(const std::string &quantity, simd::float3 point) const = 0;
};
