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

  // ── 1. Contraintes scalaires ──
  for (const auto &c : problem.constraints) {

    // "minimize" → ajouter la valeur normalisée au coût
    if (c.op == "minimize") {
      if (!ctx.fieldEvaluator)
        continue;
      // Évaluer la quantité via le fieldEvaluator (grandeurs globales)
      float val = ctx.fieldEvaluator(c.quantity, ORIGIN);
      float scale = std::max(std::abs(val), 1.0f);
      float term = val / scale;
      cb.terms[c.id] = term;
      cb.total += term * (float)c.priority;
      continue;
    }

    // "maximize" → soustraire (minimiser le négatif)
    if (c.op == "maximize") {
      if (!ctx.fieldEvaluator)
        continue;
      float val = ctx.fieldEvaluator(c.quantity, ORIGIN);
      float scale = std::max(std::abs(val), 1.0f);
      float term = val / scale;
      cb.terms[c.id] = term;
      cb.total += term * (float)c.priority;
      continue;
    }

    // Contraintes d'inégalité : >=, <=, <, >
    // On a besoin d'évaluer la quantité ET la valeur cible
    float actual = 0.0f;
    if (ctx.fieldEvaluator)
      actual = ctx.fieldEvaluator(c.quantity, ORIGIN);

    float target = 0.0f;
    if (!c.value.empty()) {
      try {
        target = ExpressionEvaluator::resolve(c.value, ctx);
      } catch (...) {
        continue; // Expression non résolvable pour l'instant
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

    // Normaliser la violation par la cible
    float scale = std::max(std::abs(target), 1.0f);
    float normalizedViol = violation / scale;

    // Coût = violation² (pénalité quadratique)
    float term = normalizedViol * normalizedViol;
    cb.terms[c.id] = term;
    cb.total += term * 100.0f * (float)c.priority; // Poids fort pour les inégalités

    if (violation > 0 && c.priority == 1)
      cb.feasible = false;
  }

  // ── 2. Field constraints ──
  for (const auto &fv : fieldViolations) {
    float term = 0.0f;

    // Trouver la priorité de cette field constraint
    int priority = 1;
    for (const auto &fc : problem.fieldConstraints) {
      if (fc.id == fv.constraintId) {
        priority = fc.priority;
        break;
      }
    }

    if (priority == 1) {
      // Sécurité : max violation (un seul point suffit à échouer)
      term = fv.maxViolation * fv.maxViolation;
      if (!fv.satisfied)
        cb.feasible = false;
    } else {
      // Performance : L2 normalisée
      term = fv.l2Violation * fv.l2Violation;
    }

    cb.terms[fv.constraintId] = term;
    cb.total += term * 100.0f * (float)priority;
  }

  // ── 3. Évaluation paresseuse ──
  // Si infeasible (P1 violée massivement), on ajoute un gros coût
  // pour guider l'optimiseur vers la faisabilité d'abord
  if (!cb.feasible)
    cb.total += 1e6f;

  return cb;
}