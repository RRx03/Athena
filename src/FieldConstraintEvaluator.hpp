#pragma once
#include "ExpressionEvaluator.hpp"
#include "ProblemDefinition.hpp"
#include <vector>

// ═══════════════════════════════════════════════════════════════
// FieldConstraintEvaluator — Phase 6
//
// Discrétise les domaines paramétriques (1D, 2D, 3D, point, surface)
// et évalue les relations lhs op rhs à chaque échantillon.
//
// Pour chaque field_constraint, produit un Violation :
//   - maxViolation : pire écart (pour priorité 1 / sécurité)
//   - l2Violation  : norme L2 normalisée (pour priorité 2+ / perf)
//   - numViolated  : nombre d'échantillons qui violent la relation
// ═══════════════════════════════════════════════════════════════

class FieldConstraintEvaluator {
public:
  struct Violation {
    std::string constraintId;
    float maxViolation = 0.0f; // max_i |violation_i|
    float l2Violation = 0.0f;  // sqrt(sum(violation_i²) / N)
    int numSamples = 0;
    int numViolated = 0;   // Nombre d'échantillons en violation
    bool satisfied = true; // Tous les échantillons sont dans la tolérance
  };

  // Évalue toutes les field constraints dans le contexte donné
  static std::vector<Violation>
  evaluate(const std::vector<FieldConstraint> &constraints,
           const ExpressionEvaluator::Context &ctx);

  // Évalue une seule field constraint
  static Violation evaluateOne(const FieldConstraint &fc,
                               const ExpressionEvaluator::Context &ctx);

private:
  // Évalue la violation en un point (lhs op rhs)
  // Retourne 0 si satisfait, sinon la magnitude de la violation
  static float evaluateRelation(const std::string &lhs, const std::string &op,
                                const std::string &rhs, float tolerance,
                                const ExpressionEvaluator::Context &ctx);
};