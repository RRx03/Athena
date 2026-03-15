#pragma once
#include <functional>
#include <string>
#include <vector>

// ═══════════════════════════════════════════════════════════════
// Optimizer — Algorithmes d'optimisation sans gradient
//
// Nelder-Mead (simplexe) : robuste, pas de gradient,
// fonctionne bien en dimension < 50.
// ═══════════════════════════════════════════════════════════════

class Optimizer {
public:
  using CostFn = std::function<float(const std::vector<float> &)>;

  struct Result {
    std::vector<float> bestParams;
    float bestCost = 1e30f;
    int iterations = 0;
    bool converged = false;
  };

  // Nelder-Mead simplex
  static Result nelderMead(const CostFn &cost, std::vector<float> initial,
                           const std::vector<float> &lo,
                           const std::vector<float> &hi, int maxIter,
                           float tolerance);

private:
  // Clamp un vecteur dans les bornes
  static void clampToBounds(std::vector<float> &x,
                            const std::vector<float> &lo,
                            const std::vector<float> &hi);
};
