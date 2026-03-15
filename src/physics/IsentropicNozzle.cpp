#include "physics/IsentropicNozzle.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

// ═══════════════════════════════════════════════════════════════
// Isentropic equations
// ═══════════════════════════════════════════════════════════════

float IsentropicNozzle::areaRatioFromMach(float M, float gamma) {
  float gp1 = gamma + 1.0f;
  float gm1 = gamma - 1.0f;
  float exp = gp1 / (2.0f * gm1);
  float base = (2.0f / gp1) * (1.0f + 0.5f * gm1 * M * M);
  return (1.0f / std::max(M, 1e-6f)) * std::pow(base, exp);
}

float IsentropicNozzle::machFromAreaRatio(float areaRatio, float gamma,
                                          bool supersonic) {
  if (areaRatio < 1.0f)
    areaRatio = 1.0f;
  if (std::abs(areaRatio - 1.0f) < 1e-6f)
    return 1.0f;

  float M = supersonic ? (1.0f + std::sqrt(areaRatio - 1.0f)) : 0.5f;

  for (int iter = 0; iter < 100; iter++) {
    float f = areaRatioFromMach(M, gamma) - areaRatio;
    float eps = 1e-6f;
    float fp = (areaRatioFromMach(M + eps, gamma) -
                areaRatioFromMach(M - eps, gamma)) / (2.0f * eps);
    if (std::abs(fp) < 1e-12f)
      break;
    float dM = -f / fp;
    M += dM;
    if (supersonic)
      M = std::max(1.001f, M);
    else
      M = std::clamp(M, 0.001f, 0.999f);
    if (std::abs(dM) < 1e-8f)
      break;
  }
  return M;
}

float IsentropicNozzle::tempRatio(float M, float gamma) {
  return 1.0f / (1.0f + 0.5f * (gamma - 1.0f) * M * M);
}

float IsentropicNozzle::pressRatio(float M, float gamma) {
  return std::pow(1.0f + 0.5f * (gamma - 1.0f) * M * M,
                  -gamma / (gamma - 1.0f));
}

float IsentropicNozzle::densRatio(float M, float gamma) {
  return std::pow(1.0f + 0.5f * (gamma - 1.0f) * M * M,
                  -1.0f / (gamma - 1.0f));
}

float IsentropicNozzle::chokedMassFlow(float throatArea, float gamma,
                                       float R_gas, float T0, float P0) {
  float gp1 = gamma + 1.0f;
  float gm1 = gamma - 1.0f;
  float exp = gp1 / (2.0f * gm1);
  float factor = std::sqrt(gamma / (R_gas * T0)) * std::pow(2.0f / gp1, exp);
  return P0 * throatArea * factor;
}

// ═══════════════════════════════════════════════════════════════
// Setup
// ═══════════════════════════════════════════════════════════════

void IsentropicNozzle::setup(const ProblemDefinition &problem,
                             const ExpressionEvaluator::Context &ctx) {
  auto it = problem.materials.find("propellant");
  if (it != problem.materials.end()) {
    auto &props = it->second.properties;
    if (props.count("gamma"))
      _gamma = props.at("gamma");
    if (props.count("R_specific"))
      _R_gas = props.at("R_specific");
  }

  auto wallIt = problem.materials.find("wall");
  if (wallIt != problem.materials.end()) {
    auto &wProps = wallIt->second.properties;
    if (wProps.count("density"))
      _wallDensity = wProps.at("density");
  }

  for (const auto &bc : problem.boundaryConditions) {
    if (bc.id == "inlet") {
      for (const auto &[name, dist] : bc.distributions) {
        if (name == "total_temperature" && !dist.expression.empty())
          try { _T0 = std::stof(dist.expression); } catch (...) {}
        if (name == "total_pressure" && !dist.expression.empty())
          try { _P0 = std::stof(dist.expression); } catch (...) {}
      }
    }
    if (bc.id == "outlet") {
      for (const auto &[name, dist] : bc.distributions) {
        if (name == "static_pressure" && dist.type == "target")
          try { _P_ambient = std::stof(dist.expression); } catch (...) {}
      }
    }
  }
}

// ═══════════════════════════════════════════════════════════════
// Build Profile — TOUJOURS conique depuis r_throat, r_exit, z_throat
// ═══════════════════════════════════════════════════════════════

void IsentropicNozzle::buildProfile(const ExpressionEvaluator::Context &ctx) {
  _stations.clear();

  auto get = [&](const std::string &name, float def) -> float {
    auto it = ctx.parameters.find(name);
    return (it != ctx.parameters.end()) ? it->second : def;
  };

  float r_throat = get("r_throat", 0.015f);
  float r_exit = get("r_exit", 0.035f);
  float L_nozzle = get("L_nozzle", 0.3f);
  float z_throat = get("z_throat", 0.12f);

  // ── CLAMP z_throat to valid range ──
  // The throat MUST be inside the nozzle, between 10% and 60% of length
  z_throat = std::clamp(z_throat, L_nozzle * 0.1f, L_nozzle * 0.6f);

  // Inlet radius: larger than both throat and exit
  float r_inlet = std::max(r_exit, r_throat) * 1.3f;

  // Ensure convergent-divergent: r_throat must be the minimum
  r_throat = std::min(r_throat, std::min(r_inlet, r_exit) * 0.95f);

  float A_throat = M_PI * r_throat * r_throat;
  int N = 50;

  for (int i = 0; i <= N; i++) {
    float t = (float)i / N;
    float z = t * L_nozzle;
    float r;

    if (z <= z_throat) {
      float frac = z / std::max(z_throat, 1e-10f);
      r = r_inlet + frac * (r_throat - r_inlet);
    } else {
      float frac = (z - z_throat) / std::max(L_nozzle - z_throat, 1e-10f);
      r = r_throat + frac * (r_exit - r_throat);
    }

    r = std::max(r, 0.001f);

    Station s;
    s.z = z;
    s.r = r;
    s.areaRatio = (M_PI * r * r) / A_throat;
    _stations.push_back(s);
  }

  // Find throat (minimum area ratio)
  _throatIndex = 0;
  float minAR = 1e10f;
  for (int i = 0; i < (int)_stations.size(); i++) {
    if (_stations[i].areaRatio < minAR) {
      minAR = _stations[i].areaRatio;
      _throatIndex = i;
    }
  }
}

// ═══════════════════════════════════════════════════════════════
// Solve
// ═══════════════════════════════════════════════════════════════

void IsentropicNozzle::solve(const ExpressionEvaluator::Context &ctx) {
  buildProfile(ctx);

  float A_throat = M_PI * _stations[_throatIndex].r *
                   _stations[_throatIndex].r;

  for (int i = 0; i < (int)_stations.size(); i++) {
    Station &s = _stations[i];
    bool supersonic = (i > _throatIndex);

    s.areaRatio = (M_PI * s.r * s.r) / A_throat;
    s.areaRatio = std::max(s.areaRatio, 1.0f);

    if (i == _throatIndex) {
      s.mach = 1.0f;
    } else {
      s.mach = machFromAreaRatio(s.areaRatio, _gamma, supersonic);
    }

    s.temperature = _T0 * tempRatio(s.mach, _gamma);
    s.pressure = _P0 * pressRatio(s.mach, _gamma);
    s.density = s.pressure / (_R_gas * s.temperature);
    s.velocity = s.mach * std::sqrt(_gamma * _R_gas * s.temperature);
  }

  _massFlow = chokedMassFlow(A_throat, _gamma, _R_gas, _T0, _P0);

  const Station &exitS = _stations.back();
  _exitMach = exitS.mach;

  float A_exit = M_PI * exitS.r * exitS.r;
  _thrust = _massFlow * exitS.velocity +
            (exitS.pressure - _P_ambient) * A_exit;
  _isp = _thrust / (_massFlow * 9.80665f);
}

// ═══════════════════════════════════════════════════════════════
// Interpolation
// ═══════════════════════════════════════════════════════════════

IsentropicNozzle::Station IsentropicNozzle::interpolate(float z) const {
  if (_stations.empty())
    return {z, 0, 1, 1, _P0, _T0, 0, 0};
  if (z <= _stations.front().z)
    return _stations.front();
  if (z >= _stations.back().z)
    return _stations.back();

  for (size_t i = 0; i + 1 < _stations.size(); i++) {
    if (z >= _stations[i].z && z <= _stations[i + 1].z) {
      float dz = _stations[i + 1].z - _stations[i].z;
      float t = (dz > 1e-10f) ? (z - _stations[i].z) / dz : 0.0f;
      const auto &a = _stations[i];
      const auto &b = _stations[i + 1];
      return {z,
              a.r + t * (b.r - a.r),
              a.areaRatio + t * (b.areaRatio - a.areaRatio),
              a.mach + t * (b.mach - a.mach),
              a.pressure + t * (b.pressure - a.pressure),
              a.temperature + t * (b.temperature - a.temperature),
              a.velocity + t * (b.velocity - a.velocity),
              a.density + t * (b.density - a.density)};
    }
  }
  return _stations.back();
}

// ═══════════════════════════════════════════════════════════════
// fieldAt
// ═══════════════════════════════════════════════════════════════

float IsentropicNozzle::fieldAt(const std::string &quantity,
                                simd::float3 point) const {
  if (quantity == "thrust")
    return _thrust;
  if (quantity == "specific_impulse" || quantity == "isp")
    return _isp;
  if (quantity == "mass_flow")
    return _massFlow;
  if (quantity == "exit_mach")
    return _exitMach;

  float z = point.z;
  Station s = interpolate(z);

  if (quantity == "mach")
    return s.mach;
  if (quantity == "pressure" || quantity == "static_pressure")
    return s.pressure;
  if (quantity == "temperature" || quantity == "static_temperature")
    return s.temperature;
  if (quantity == "velocity")
    return s.velocity;
  if (quantity == "density")
    return s.density;
  if (quantity == "area_ratio")
    return s.areaRatio;
  if (quantity == "radius")
    return s.r;
  if (quantity == "total_pressure")
    return _P0;
  if (quantity == "total_temperature")
    return _T0;

  if (quantity == "total_mass") {
    float mass = 0.0f;
    for (size_t i = 0; i + 1 < _stations.size(); i++) {
      float dz = _stations[i + 1].z - _stations[i].z;
      float r_avg = (_stations[i].r + _stations[i + 1].r) * 0.5f;
      float P = (_stations[i].pressure + _stations[i + 1].pressure) * 0.5f;
      float sigma = 690e6f;
      float t_wall = (P * r_avg) / std::max(sigma - 0.6f * P, 1.0f);
      mass += _wallDensity * 2.0f * M_PI * r_avg * t_wall * dz;
    }
    return mass;
  }

  if (quantity == "wall_temp_max")
    return _T0;
  if (quantity == "von_mises_max")
    return 0.0f;
  if (quantity == "bounding_box") {
    float maxR = 0;
    for (const auto &st : _stations)
      maxR = std::max(maxR, st.r);
    return maxR;
  }

  return 0.0f;
}