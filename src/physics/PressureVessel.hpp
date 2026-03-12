#pragma once
#include "PhysicsModule.hpp"

// ═══════════════════════════════════════════════════════════════
// PressureVessel — Épaisseur de paroi minimale
//
// Formule du réservoir cylindrique sous pression interne :
//   t = (P × r) / (σ_yield / SF - 0.6 × P)
//
// où :
//   P = pression interne (Pa)
//   r = rayon interne (m)
//   σ_yield = limite élastique du matériau (Pa)
//   SF = facteur de sécurité
//
// Le module calcule t(z) le long de l'axe en utilisant la
// pression locale (fournie par IsentropicNozzle) et le rayon
// local (des paramètres).
// ═══════════════════════════════════════════════════════════════

class PressureVessel : public PhysicsModule {
public:
  std::string name() const override { return "pressure_vessel"; }

  void setup(const ProblemDefinition &problem,
             const ExpressionEvaluator::Context &ctx) override;

  void solve(const ExpressionEvaluator::Context &ctx) override;

  float fieldAt(const std::string &quantity,
                simd::float3 point) const override;

  // Épaisseur max calculée (pour le paramètre dérivé wall_thickness)
  float maxThickness() const { return _maxThickness; }

private:
  float _yieldStrength = 1035e6f;
  float _safetyFactor = 1.5f;
  float _maxThickness = 0.003f;

  // Référence au module isentropique pour les pressions locales
  const PhysicsModule *_flowModule = nullptr;

  struct WallStation {
    float z;
    float radius;
    float pressure;
    float thickness;
    float vonMises;
  };
  std::vector<WallStation> _stations;

  WallStation interpolate(float z) const;

  static float wallThickness(float pressure, float radius,
                             float yieldStrength, float safetyFactor);
};
