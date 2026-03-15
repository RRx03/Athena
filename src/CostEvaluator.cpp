#include "CostEvaluator.hpp"
#include <cmath>
#include <iostream>
#include <simd/simd.h>

static const simd::float3 ORIGIN = simd_make_float3(0, 0, 0);

CostEvaluator::CostBreakdown CostEvaluator::evaluate(
    const ProblemDefinition &problem,
    const ExpressionEvaluator::Context &ctx,
    const std::vector<FieldConstraintEvaluator::Violation> &fieldViolations) {

  CostBreakdown cb;
  cb.total = 0.0f;
  cb.feasible = true;

  for (const auto &c : problem.constraints) {

    // ── minimize : on veut réduire cette grandeur ──
    if (c.op == "minimize") {
      if (!ctx.fieldEvaluator)
        continue;
      float val = ctx.fieldEvaluator(c.quantity, ORIGIN);
      // Normaliser par une échelle raisonnable pour que le terme soit ~O(1)
      // On utilise la valeur elle-même comme échelle
      float scale = std::max(std::abs(val), 1.0f);
      float term = val / scale; // Positif — l'optimiseur minimise
      cb.terms[c.id] = term;
      cb.total += term * (float)c.priority;
      continue;
    }

    // ── maximize : on veut augmenter cette grandeur ──
    if (c.op == "maximize") {
      if (!ctx.fieldEvaluator)
        continue;
      float val = ctx.fieldEvaluator(c.quantity, ORIGIN);
      float scale = std::max(std::abs(val), 1.0f);
      float term = -val / scale; // NÉGATIF — minimiser (-val) = maximiser val
      cb.terms[c.id] = term;
      cb.total += term * (float)c.priority;
      continue;
    }

    // ── Inégalités : >=, <=, <, >, = ──
    float actual = 0.0f;
    if (ctx.fieldEvaluator)
      actual = ctx.fieldEvaluator(c.quantity, ORIGIN);

    float target = 0.0f;
    if (!c.value.empty()) {
      try {
        target = ExpressionEvaluator::resolve(c.value, ctx);
      } catch (...) {
        continue;
      }
    }

    float violation = 0.0f;
    if (c.op == ">=" || c.op == ">") {
      violation = std::max(0.0f, target - actual);
    } else if (c.op == "<=" || c.op == "<") {
      violation = std::max(0.0f, actual - target);
    } else if (c.op == "=" || c.op == "==") {
      violation = std::abs(actual - target);
    }

    float scale = std::max(std::abs(target), 1.0f);
    float normalizedViol = violation / scale;
    float term = normalizedViol * normalizedViol;

    cb.terms[c.id] = term;
    cb.total += term * 100.0f * (float)c.priority;

    if (violation > 0 && c.priority == 1)
      cb.feasible = false;
  }

  // ── Field constraints ──
  for (const auto &fv : fieldViolations) {
    float term = 0.0f;

    int priority = 1;
    for (const auto &fc : problem.fieldConstraints) {
      if (fc.id == fv.constraintId) {
        priority = fc.priority;
        break;
      }
    }

    if (priority == 1) {
      term = fv.maxViolation * fv.maxViolation;
      if (!fv.satisfied)
        cb.feasible = false;
    } else {
      term = fv.l2Violation * fv.l2Violation;
    }

    cb.terms[fv.constraintId] = term;
    cb.total += term * 100.0f * (float)priority;
  }

  if (!cb.feasible)
    cb.total += 1e6f;

  return cb;
}