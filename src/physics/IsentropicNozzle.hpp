#pragma once
#include "PhysicsModule.hpp"
#include <vector>

class IsentropicNozzle : public PhysicsModule {
public:
  std::string name() const override { return "isentropic_1d"; }
  void setup(const ProblemDefinition &problem,
             const ExpressionEvaluator::Context &ctx) override;
  void solve(const ExpressionEvaluator::Context &ctx) override;
  float fieldAt(const std::string &quantity, simd::float3 point) const override;

private:
  float _gamma = 1.4f, _R_gas = 287.0f, _T0 = 300.0f, _P0 = 101325.0f;

  struct Station {
    float z, r, mach, pressure, temperature, velocity, density;
  };
  std::vector<Station> _stations;
  Station interpolate(float z) const;
};
