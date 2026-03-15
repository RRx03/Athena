#pragma once
#include "ExpressionEvaluator.hpp"
#include "FieldConstraintEvaluator.hpp"
#include "ProblemDefinition.hpp"
#include <map>
#include <string>

// ═══════════════════════════════════════════════════════════════
// CostEvaluator — Fonction de coût pondérée multi-objectif
//
// Agrège les violations des contraintes scalaires et des field
// constraints en un coût scalaire unique que l'optimiseur minimise.
//
// Stratégie :
//   - "minimize" / "maximize" → valeur directe (normalisée)
//   - ">=", "<=", "<", ">" → 0 si satisfait, violation² sinon
//   - Field constraints priorité 1 → max violation (sécurité)
//   - Field constraints priorité 2+ → L2 (performance)
//   - Évaluation paresseuse : si une contrainte P1 est massivement
//     violée, skip les contraintes de priorité basse
// ═══════════════════════════════════════════════════════════════

class CostEvaluator {
public:
  struct CostBreakdown {
    float total = 0.0f;
    std::map<std::string, float> terms; // constraint_id → contribution
    bool feasible = true; // Toutes les P1 satisfaites ?
  };

  static CostBreakdown
  evaluate(const ProblemDefinition &problem,
           const ExpressionEvaluator::Context &ctx,
           const std::vector<FieldConstraintEvaluator::Violation> &fieldViolations);
};
