#include "physics/IsentropicNozzle.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

// ═══════════════════════════════════════════════════════════════
// Équations isentropiques statiques
// ═══════════════════════════════════════════════════════════════

float IsentropicNozzle::areaRatioFromMach(float M, float gamma) {
  float gp1 = gamma + 1.0f;
  float gm1 = gamma - 1.0f;
  float exp = gp1 / (2.0f * gm1);
  float base = (2.0f / gp1) * (1.0f + 0.5f * gm1 * M * M);
  return (1.0f / M) * std::pow(base, exp);
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
  float factor = std::sqrt(gamma / (R_gas * T0)) *
                 std::pow(2.0f / gp1, exp);
  return P0 * throatArea * factor;
}

// ═══════════════════════════════════════════════════════════════
// Setup — Lire les propriétés du gaz depuis le problème
// ═══════════════════════════════════════════════════════════════

void IsentropicNozzle::setup(const ProblemDefinition &problem,
                             const ExpressionEvaluator::Context &ctx) {
  // Lire gamma et R depuis les matériaux
  auto it = problem.materials.find("propellant");
  if (it != problem.materials.end()) {
    auto &props = it->second.properties;
    if (props.count("gamma"))
      _gamma = props.at("gamma");
    if (props.count("R_specific"))
      _R_gas = props.at("R_specific");
  }

  // Lire T0 et P0 depuis les distributions BC inlet
  for (const auto &bc : problem.boundaryConditions) {
    if (bc.id == "inlet") {
      for (const auto &[name, dist] : bc.distributions) {
        if (name == "total_temperature" && !dist.expression.empty()) {
          try { _T0 = std::stof(dist.expression); } catch (...) {}
        }
        if (name == "total_pressure" && !dist.expression.empty()) {
          try { _P0 = std::stof(dist.expression); } catch (...) {}
        }
      }
    }
    // Lire P_ambient depuis la BC outlet target
    if (bc.id == "outlet") {
      for (const auto &[name, dist] : bc.distributions) {
        if (name == "static_pressure" && dist.type == "target") {
          try { _P_ambient = std::stof(dist.expression); } catch (...) {}
        }
      }
    }
  }
}

// ═══════════════════════════════════════════════════════════════
// Build Profile — Construire r(z) depuis les paramètres courants
// ═══════════════════════════════════════════════════════════════

void IsentropicNozzle::buildProfile(
    const ExpressionEvaluator::Context &ctx) {
  _stations.clear();

  // Lire les paramètres clés
  float r_throat = 0.015f, r_exit = 0.035f, L_nozzle = 0.3f,
        z_throat = 0.12f;

  auto get = [&](const std::string &name, float def) -> float {
    auto it = ctx.parameters.find(name);
    return (it != ctx.parameters.end()) ? it->second : def;
  };

  r_throat = get("r_throat", r_throat);
  r_exit = get("r_exit", r_exit);
  L_nozzle = get("L_nozzle", L_nozzle);
  z_throat = get("z_throat", z_throat);

  // Collecter les points de contrôle du profil (profile_r[0..N])
  std::vector<float> profileR;
  for (int i = 0; i < 100; i++) {
    std::string key = "profile_r[" + std::to_string(i) + "]";
    auto it = ctx.parameters.find(key);
    if (it == ctx.parameters.end())
      break;
    profileR.push_back(it->second);
  }

  // Détecter si les profile_r sont l'init par défaut (toutes les valeurs
  // identiques = milieu des bornes, pas un vrai profil).
  // Dans ce cas, on utilise le profil conique.
  bool useProfileR = false;
  if (profileR.size() >= 3) {
    float minR = *std::min_element(profileR.begin(), profileR.end());
    float maxR = *std::max_element(profileR.begin(), profileR.end());
    // Si l'écart entre min et max est > 1%, c'est un vrai profil
    useProfileR = (maxR - minR) > 0.01f * (maxR + minR) * 0.5f;
  }

  // Construire les stations uniformément réparties le long de z
  int N = std::max(50, (int)profileR.size() * 3);
  float A_throat = M_PI * r_throat * r_throat;

  for (int i = 0; i <= N; i++) {
    float t = (float)i / N;
    float z = t * L_nozzle;

    // Interpoler le rayon
    float r;
    if (!useProfileR) {
      // Profil conique simple
      float r_inlet = get("r_inlet", r_exit * 1.2f);
      if (z <= z_throat) {
        float frac = (z_throat > 1e-10f) ? z / z_throat : 0.0f;
        r = r_inlet + frac * (r_throat - r_inlet);
      } else {
        float denom = L_nozzle - z_throat;
        float frac = (denom > 1e-10f) ? (z - z_throat) / denom : 0.0f;
        r = r_throat + frac * (r_exit - r_throat);
      }
    } else {
      // Interpolation linéaire par morceaux des points de contrôle
      // Les points sont répartis uniformément de z=0 à z=L_nozzle
      float idx_f = t * (profileR.size() - 1);
      int idx_lo = std::clamp((int)idx_f, 0, (int)profileR.size() - 2);
      float frac = idx_f - idx_lo;
      r = profileR[idx_lo] * (1.0f - frac) + profileR[idx_lo + 1] * frac;
    }

    // Garantir un rayon minimum
    r = std::max(r, 0.001f);

    Station s;
    s.z = z;
    s.r = r;
    s.areaRatio = (M_PI * r * r) / A_throat;

    _stations.push_back(s);
  }

  // Identifier la station au col (area ratio minimum)
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
// Solve — Calculer tous les champs pour la géométrie courante
// ═══════════════════════════════════════════════════════════════

void IsentropicNozzle::solve(const ExpressionEvaluator::Context &ctx) {
  buildProfile(ctx);

  float A_throat = M_PI * _stations[_throatIndex].r *
                   _stations[_throatIndex].r;

  // Calculer les grandeurs à chaque station
  for (int i = 0; i < (int)_stations.size(); i++) {
    Station &s = _stations[i];

    // Déterminer si on est en subsonique ou supersonique
    bool supersonic = (i > _throatIndex);

    // Recalculer A/A* par rapport au vrai col
    s.areaRatio = (M_PI * s.r * s.r) / A_throat;
    s.areaRatio = std::max(s.areaRatio, 1.0f); // Minimum = 1 (au col)

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

  // Grandeurs globales
  _massFlow = chokedMassFlow(A_throat, _gamma, _R_gas, _T0, _P0);

  const Station &exitS = _stations.back();
  _exitMach = exitS.mach;

  float A_exit = M_PI * exitS.r * exitS.r;
  _thrust = _massFlow * exitS.velocity +
            (exitS.pressure - _P_ambient) * A_exit;
  _isp = _thrust / (_massFlow * 9.80665f);
}

// ═══════════════════════════════════════════════════════════════
// Interpolation — Trouver les grandeurs en un point z arbitraire
// ═══════════════════════════════════════════════════════════════

IsentropicNozzle::Station
IsentropicNozzle::interpolate(float z) const {
  if (_stations.empty())
    return {z, 0, 1, 1, _P0, _T0, 0, 0};

  // Clamp z dans la plage
  if (z <= _stations.front().z)
    return _stations.front();
  if (z >= _stations.back().z)
    return _stations.back();

  // Recherche linéaire (suffisant pour ~20-50 stations)
  for (size_t i = 0; i + 1 < _stations.size(); i++) {
    if (z >= _stations[i].z && z <= _stations[i + 1].z) {
      float t = (_stations[i + 1].z - _stations[i].z) > 1e-10f
                    ? (z - _stations[i].z) /
                          (_stations[i + 1].z - _stations[i].z)
                    : 0.0f;
      Station s;
      const auto &a = _stations[i];
      const auto &b = _stations[i + 1];
      s.z = z;
      s.r = a.r + t * (b.r - a.r);
      s.areaRatio = a.areaRatio + t * (b.areaRatio - a.areaRatio);
      s.mach = a.mach + t * (b.mach - a.mach);
      s.pressure = a.pressure + t * (b.pressure - a.pressure);
      s.temperature = a.temperature + t * (b.temperature - a.temperature);
      s.velocity = a.velocity + t * (b.velocity - a.velocity);
      s.density = a.density + t * (b.density - a.density);
      return s;
    }
  }

  return _stations.back();
}

// ═══════════════════════════════════════════════════════════════
// fieldAt — Évalue un champ physique en un point 3D
//
// Le point est en coordonnées du solveur. Pour le quasi-1D,
// seule la coordonnée z compte (axiale). La coordonnée r
// (radiale) est ignorée car les propriétés sont uniformes
// sur chaque section.
//
// Grandeurs disponibles :
//   "mach", "pressure", "temperature", "velocity",
//   "density", "area_ratio", "radius",
//   "thrust" (global), "isp" (global), "mass_flow" (global)
// ═══════════════════════════════════════════════════════════════

float IsentropicNozzle::fieldAt(const std::string &quantity,
                                simd::float3 point) const {
  // Grandeurs globales (indépendantes du point)
  if (quantity == "thrust")
    return _thrust;
  if (quantity == "specific_impulse" || quantity == "isp")
    return _isp;
  if (quantity == "mass_flow")
    return _massFlow;

  // Grandeurs locales (dépendent de z)
  // En axisymétrique, le point est @[r, z] → point.x = r, point.z = z
  // En cartésien, z est la composante axiale (point.z)
  float z = point.z; // Coordonnée axiale

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

  // Total conditions (constantes en isentropique)
  if (quantity == "total_pressure")
    return _P0;
  if (quantity == "total_temperature")
    return _T0;

  return 0.0f;
}