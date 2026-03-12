#include "physics/PressureVessel.hpp"
#include <algorithm>
#include <cmath>

float PressureVessel::wallThickness(float pressure, float radius,
                                    float yieldStrength,
                                    float safetyFactor) {
  float allowable = yieldStrength / safetyFactor;
  float denom = allowable - 0.6f * pressure;
  if (denom <= 0)
    return radius; // Pression trop élevée — paroi = rayon (erreur physique)
  return (pressure * radius) / denom;
}

void PressureVessel::setup(const ProblemDefinition &problem,
                           const ExpressionEvaluator::Context &ctx) {
  auto it = problem.materials.find("wall");
  if (it != problem.materials.end()) {
    auto &props = it->second.properties;
    if (props.count("yield_strength"))
      _yieldStrength = props.at("yield_strength");
    if (props.count("safety_factor"))
      _safetyFactor = props.at("safety_factor");
  }
}

void PressureVessel::solve(const ExpressionEvaluator::Context &ctx) {
  _stations.clear();
  _maxThickness = 0.0f;

  // On a besoin du champ de pression et du profil de rayon.
  // On utilise le fieldEvaluator du contexte (qui pointe vers
  // IsentropicNozzle) pour obtenir pression et rayon à chaque z.
  if (!ctx.fieldEvaluator)
    return;

  // Récupérer L_nozzle
  float L = 0.3f;
  auto lit = ctx.parameters.find("L_nozzle");
  if (lit != ctx.parameters.end())
    L = lit->second;

  int N = 50;
  for (int i = 0; i <= N; i++) {
    float z = L * i / N;
    simd::float3 pt = {0, 0, z};

    WallStation ws;
    ws.z = z;
    ws.radius = ctx.fieldEvaluator("radius", pt);
    ws.pressure = ctx.fieldEvaluator("pressure", pt);
    ws.thickness =
        wallThickness(ws.pressure, ws.radius, _yieldStrength, _safetyFactor);

    // Contrainte de Von Mises (approximation cylindre mince)
    // σ_hoop = P × r / t (contrainte circonférentielle)
    // σ_axial = P × r / (2t) (contrainte axiale)
    // σ_VM = √(σ_h² + σ_a² - σ_h×σ_a) pour état plan de contrainte
    if (ws.thickness > 1e-10f) {
      float sigma_h = ws.pressure * ws.radius / ws.thickness;
      float sigma_a = sigma_h * 0.5f;
      ws.vonMises = std::sqrt(sigma_h * sigma_h + sigma_a * sigma_a -
                              sigma_h * sigma_a);
    } else {
      ws.vonMises = 1e12f; // Épaisseur nulle = contrainte infinie
    }

    _maxThickness = std::max(_maxThickness, ws.thickness);
    _stations.push_back(ws);
  }
}

PressureVessel::WallStation
PressureVessel::interpolate(float z) const {
  if (_stations.empty())
    return {z, 0, 0, 0.003f, 0};

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
      WallStation ws;
      ws.z = z;
      ws.radius = a.radius + t * (b.radius - a.radius);
      ws.pressure = a.pressure + t * (b.pressure - a.pressure);
      ws.thickness = a.thickness + t * (b.thickness - a.thickness);
      ws.vonMises = a.vonMises + t * (b.vonMises - a.vonMises);
      return ws;
    }
  }
  return _stations.back();
}

float PressureVessel::fieldAt(const std::string &quantity,
                              simd::float3 point) const {
  float z = point.z;
  auto ws = interpolate(z);

  if (quantity == "wall_thickness")
    return ws.thickness;
  if (quantity == "von_mises" || quantity == "von_mises_stress")
    return ws.vonMises;
  if (quantity == "von_mises_max") {
    float maxVM = 0;
    for (const auto &s : _stations)
      maxVM = std::max(maxVM, s.vonMises);
    return maxVM;
  }
  if (quantity == "wall_thickness_max")
    return _maxThickness;

  return 0.0f;
}
