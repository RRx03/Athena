#include "FieldConstraintEvaluator.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

// ═══════════════════════════════════════════════════════════════
// evaluateRelation — Évalue la violation d'une relation en un point
//
//   lhs = rhs (±tolerance)  → violation = max(0, |lhs-rhs| - tol)
//   lhs >= rhs              → violation = max(0, rhs - lhs)
//   lhs <= rhs              → violation = max(0, lhs - rhs)
//   lhs > rhs               → violation = max(0, rhs - lhs + eps)
//   lhs < rhs               → violation = max(0, lhs - rhs + eps)
//
// Retourne 0 si satisfait, sinon la magnitude de la violation.
// ═══════════════════════════════════════════════════════════════

float FieldConstraintEvaluator::evaluateRelation(
    const std::string &lhsExpr, const std::string &op,
    const std::string &rhsExpr, float tolerance,
    const ExpressionEvaluator::Context &ctx) {

  float lhs = ExpressionEvaluator::evaluate(lhsExpr, ctx);
  float rhs = ExpressionEvaluator::evaluate(rhsExpr, ctx);

  if (op == "=" || op == "==") {
    float diff = std::abs(lhs - rhs);
    // Tolérance relative : tolerance est une fraction (ex: 0.05 = 5%)
    float absThreshold = tolerance * std::max(std::abs(rhs), 1e-10f);
    return std::max(0.0f, diff - absThreshold);
  }

  if (op == ">=") {
    return std::max(0.0f, rhs - lhs); // Violé si lhs < rhs
  }

  if (op == "<=") {
    return std::max(0.0f, lhs - rhs); // Violé si lhs > rhs
  }

  if (op == ">") {
    return std::max(0.0f, rhs - lhs + 1e-6f);
  }

  if (op == "<") {
    return std::max(0.0f, lhs - rhs + 1e-6f);
  }

  throw std::runtime_error("FieldConstraintEvaluator: opérateur inconnu '" +
                           op + "'");
}

// ═══════════════════════════════════════════════════════════════
// evaluateOne — Évalue une seule field constraint
// ═══════════════════════════════════════════════════════════════

FieldConstraintEvaluator::Violation
FieldConstraintEvaluator::evaluateOne(
    const FieldConstraint &fc,
    const ExpressionEvaluator::Context &ctx) {

  Violation result;
  result.constraintId = fc.id;

  // ── Domain type: point ──
  if (fc.domain.type == "point") {
    result.numSamples = 1;
    float v = evaluateRelation(fc.lhs, fc.op, fc.rhs, fc.tolerance, ctx);
    result.maxViolation = v;
    result.l2Violation = v;
    result.numViolated = (v > 0) ? 1 : 0;
    result.satisfied = (v == 0);
    return result;
  }

  // ── Domain type: parametric_1d ──
  if (fc.domain.type == "parametric_1d") {
    if (fc.domain.variables.empty() || fc.domain.samples.empty())
      return result;

    const std::string &varName = fc.domain.variables[0];
    int N = fc.domain.samples[0];
    result.numSamples = N;

    // Résoudre les bornes du domaine
    auto rangeIt = fc.domain.ranges.find(varName);
    if (rangeIt == fc.domain.ranges.end())
      return result;

    float lo = ExpressionEvaluator::evaluate(rangeIt->second.first, ctx);
    float hi = ExpressionEvaluator::evaluate(rangeIt->second.second, ctx);

    float sumSq = 0.0f;

    for (int i = 0; i < N; i++) {
      float t = (N > 1) ? (float)i / (N - 1) : 0.0f;
      float val = lo + t * (hi - lo);

      // Créer un contexte local avec la variable de domaine injectée
      ExpressionEvaluator::Context localCtx = ctx;
      localCtx.domainVariables[varName] = val;

      float v = evaluateRelation(fc.lhs, fc.op, fc.rhs, fc.tolerance, localCtx);

      result.maxViolation = std::max(result.maxViolation, v);
      sumSq += v * v;
      if (v > 0)
        result.numViolated++;
    }

    result.l2Violation = std::sqrt(sumSq / std::max(1, N));
    result.satisfied = (result.numViolated == 0);
    return result;
  }

  // ── Domain type: parametric_2d ──
  if (fc.domain.type == "parametric_2d") {
    if (fc.domain.variables.size() < 2 || fc.domain.samples.size() < 2)
      return result;

    const std::string &var0 = fc.domain.variables[0];
    const std::string &var1 = fc.domain.variables[1];
    int N0 = fc.domain.samples[0];
    int N1 = fc.domain.samples[1];
    result.numSamples = N0 * N1;

    auto range0 = fc.domain.ranges.find(var0);
    auto range1 = fc.domain.ranges.find(var1);
    if (range0 == fc.domain.ranges.end() ||
        range1 == fc.domain.ranges.end())
      return result;

    float lo0 = ExpressionEvaluator::evaluate(range0->second.first, ctx);
    float hi0 = ExpressionEvaluator::evaluate(range0->second.second, ctx);
    float lo1 = ExpressionEvaluator::evaluate(range1->second.first, ctx);
    float hi1 = ExpressionEvaluator::evaluate(range1->second.second, ctx);

    float sumSq = 0.0f;

    for (int i = 0; i < N0; i++) {
      float t0 = (N0 > 1) ? (float)i / (N0 - 1) : 0.0f;
      float val0 = lo0 + t0 * (hi0 - lo0);

      for (int j = 0; j < N1; j++) {
        float t1 = (N1 > 1) ? (float)j / (N1 - 1) : 0.0f;
        float val1 = lo1 + t1 * (hi1 - lo1);

        ExpressionEvaluator::Context localCtx = ctx;
        localCtx.domainVariables[var0] = val0;
        localCtx.domainVariables[var1] = val1;

        float v = evaluateRelation(fc.lhs, fc.op, fc.rhs,
                                   fc.tolerance, localCtx);

        result.maxViolation = std::max(result.maxViolation, v);
        sumSq += v * v;
        if (v > 0)
          result.numViolated++;
      }
    }

    result.l2Violation =
        std::sqrt(sumSq / std::max(1, result.numSamples));
    result.satisfied = (result.numViolated == 0);
    return result;
  }

  // ── Domain type: parametric_3d ──
  if (fc.domain.type == "parametric_3d") {
    if (fc.domain.variables.size() < 3 || fc.domain.samples.size() < 3)
      return result;

    const std::string &v0 = fc.domain.variables[0];
    const std::string &v1 = fc.domain.variables[1];
    const std::string &v2 = fc.domain.variables[2];
    int N0 = fc.domain.samples[0];
    int N1 = fc.domain.samples[1];
    int N2 = fc.domain.samples[2];
    result.numSamples = N0 * N1 * N2;

    auto r0 = fc.domain.ranges.find(v0);
    auto r1 = fc.domain.ranges.find(v1);
    auto r2 = fc.domain.ranges.find(v2);
    if (r0 == fc.domain.ranges.end() || r1 == fc.domain.ranges.end() ||
        r2 == fc.domain.ranges.end())
      return result;

    float lo0 = ExpressionEvaluator::evaluate(r0->second.first, ctx);
    float hi0 = ExpressionEvaluator::evaluate(r0->second.second, ctx);
    float lo1 = ExpressionEvaluator::evaluate(r1->second.first, ctx);
    float hi1 = ExpressionEvaluator::evaluate(r1->second.second, ctx);
    float lo2 = ExpressionEvaluator::evaluate(r2->second.first, ctx);
    float hi2 = ExpressionEvaluator::evaluate(r2->second.second, ctx);

    float sumSq = 0.0f;

    for (int i = 0; i < N0; i++) {
      float val0 = lo0 + ((N0 > 1) ? (float)i / (N0 - 1) : 0.0f) * (hi0 - lo0);
      for (int j = 0; j < N1; j++) {
        float val1 = lo1 + ((N1 > 1) ? (float)j / (N1 - 1) : 0.0f) * (hi1 - lo1);
        for (int k = 0; k < N2; k++) {
          float val2 = lo2 + ((N2 > 1) ? (float)k / (N2 - 1) : 0.0f) * (hi2 - lo2);

          ExpressionEvaluator::Context localCtx = ctx;
          localCtx.domainVariables[v0] = val0;
          localCtx.domainVariables[v1] = val1;
          localCtx.domainVariables[v2] = val2;

          float v = evaluateRelation(fc.lhs, fc.op, fc.rhs,
                                     fc.tolerance, localCtx);
          result.maxViolation = std::max(result.maxViolation, v);
          sumSq += v * v;
          if (v > 0)
            result.numViolated++;
        }
      }
    }

    result.l2Violation =
        std::sqrt(sumSq / std::max(1, result.numSamples));
    result.satisfied = (result.numViolated == 0);
    return result;
  }

  // ── Domain type: surface ──
  // Pour l'instant, on ne supporte pas l'échantillonnage de surface
  // nommée (nécessite un maillage). On retourne un résultat vide.
  if (fc.domain.type == "surface") {
    // TODO: échantillonner la surface nommée
    result.numSamples = 0;
    result.satisfied = true;
    return result;
  }

  return result;
}

// ═══════════════════════════════════════════════════════════════
// evaluate — Évalue toutes les field constraints
// ═══════════════════════════════════════════════════════════════

std::vector<FieldConstraintEvaluator::Violation>
FieldConstraintEvaluator::evaluate(
    const std::vector<FieldConstraint> &constraints,
    const ExpressionEvaluator::Context &ctx) {

  std::vector<Violation> results;
  results.reserve(constraints.size());

  for (const auto &fc : constraints) {
    results.push_back(evaluateOne(fc, ctx));
  }

  return results;
}