#pragma once
#include "PhysicsModule.hpp"
#include <cmath>
#include <vector>

// ═══════════════════════════════════════════════════════════════
// IsentropicNozzle — Écoulement quasi-1D isentropique
//
// Hypothèses :
//   - Gaz parfait caloriquement parfait (γ constant)
//   - Écoulement isentropique (pas de chocs, pas de friction)
//   - Quasi-1D (propriétés uniformes sur chaque section)
//   - Axisymétrique autour de l'axe z
//
// Le profil de la tuyère est défini par les paramètres du contexte :
//   - param:r_throat, param:r_exit → rayons clés
//   - param:profile_r[0..N] → points de contrôle intermédiaires
//   - param:z_throat → position axiale du col
//   - param:L_nozzle → longueur totale
//
// Champs disponibles via fieldAt() :
//   "mach", "pressure", "temperature", "velocity",
//   "density", "area_ratio", "thrust", "isp", "mass_flow"
// ═══════════════════════════════════════════════════════════════

class IsentropicNozzle : public PhysicsModule {
public:
  std::string name() const override { return "isentropic_1d"; }

  void setup(const ProblemDefinition &problem,
             const ExpressionEvaluator::Context &ctx) override;

  void solve(const ExpressionEvaluator::Context &ctx) override;

  float fieldAt(const std::string &quantity, simd::float3 point) const override;

  // Accès direct aux grandeurs globales (pour CostEvaluator)
  float thrust() const { return _thrust; }
  float isp() const { return _isp; }
  float massFlow() const { return _massFlow; }
  float exitMach() const { return _exitMach; }

private:
  // Propriétés du gaz (lues depuis materials)
  float _gamma = 1.4f;
  float _R_gas = 287.0f;
  float _T0 = 300.0f;
  float _P0 = 101325.0f;
  float _P_ambient = 101325.0f;
  float _wallDensity = 8190.0f; // kg/m³ (Inconel 718 par défaut)

  // Grandeurs globales calculées
  float _thrust = 0.0f;
  float _isp = 0.0f;
  float _massFlow = 0.0f;
  float _exitMach = 1.0f;

  // Stations le long de l'axe
  struct Station {
    float z;         // Position axiale (m)
    float r;         // Rayon interne (m)
    float areaRatio; // A / A*
    float mach;
    float pressure;    // Pa
    float temperature; // K
    float velocity;    // m/s
    float density;     // kg/m³
  };
  std::vector<Station> _stations;
  int _throatIndex = 0; // Index de la station au col

  // Interpolation linéaire sur les stations
  Station interpolate(float z) const;

  // ── Équations isentropiques ──

  // A/A* = f(M, γ)
  static float areaRatioFromMach(float M, float gamma);

  // Résolution inverse : M = f(A/A*, γ, supersonic?)
  static float machFromAreaRatio(float areaRatio, float gamma, bool supersonic);

  // Rapports isentropiques
  static float tempRatio(float M, float gamma);
  static float pressRatio(float M, float gamma);
  static float densRatio(float M, float gamma);

  // Débit massique au col
  static float chokedMassFlow(float throatArea, float gamma, float R_gas,
                              float T0, float P0);

  // Construit le profil r(z) depuis les paramètres du contexte
  void buildProfile(const ExpressionEvaluator::Context &ctx);
};