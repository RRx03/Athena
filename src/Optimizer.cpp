#include "Optimizer.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

void Optimizer::clampToBounds(std::vector<float> &x,
                              const std::vector<float> &lo,
                              const std::vector<float> &hi) {
  for (size_t i = 0; i < x.size(); i++)
    x[i] = std::clamp(x[i], lo[i], hi[i]);
}

// ═══════════════════════════════════════════════════════════════
// Nelder-Mead Simplex
//
// Un simplexe en dimension N a N+1 sommets. À chaque itération,
// on remplace le pire sommet par une réflexion, expansion,
// contraction, ou réduction.
//
// Paramètres standard : α=1 (réflexion), γ=2 (expansion),
//                        ρ=0.5 (contraction), σ=0.5 (réduction)
// ═══════════════════════════════════════════════════════════════

Optimizer::Result Optimizer::nelderMead(const CostFn &cost,
                                        std::vector<float> initial,
                                        const std::vector<float> &lo,
                                        const std::vector<float> &hi,
                                        int maxIter, float tolerance) {
  const int N = (int)initial.size();
  if (N == 0)
    return {initial, 0, 0, true};

  const float alpha = 1.0f;  // Réflexion
  const float gamma = 2.0f;  // Expansion
  const float rho = 0.5f;    // Contraction
  const float sigma = 0.5f;  // Réduction

  // ── Initialiser le simplexe ──
  // N+1 sommets : le point initial + N perturbations
  struct Vertex {
    std::vector<float> x;
    float cost;
  };

  std::vector<Vertex> simplex(N + 1);

  // Premier sommet = point initial
  simplex[0].x = initial;
  clampToBounds(simplex[0].x, lo, hi);
  simplex[0].cost = cost(simplex[0].x);

  // N sommets supplémentaires : perturbation de 5% de la plage sur chaque axe
  for (int i = 0; i < N; i++) {
    simplex[i + 1].x = initial;
    float range = hi[i] - lo[i];
    float delta = range * 0.05f;
    if (delta < 1e-10f)
      delta = std::max(std::abs(initial[i]) * 0.05f, 1e-6f);
    simplex[i + 1].x[i] += delta;
    clampToBounds(simplex[i + 1].x, lo, hi);
    simplex[i + 1].cost = cost(simplex[i + 1].x);
  }

  Result result;
  result.iterations = 0;

  for (int iter = 0; iter < maxIter; iter++) {
    result.iterations = iter + 1;

    // ── Trier le simplexe par coût ──
    std::sort(simplex.begin(), simplex.end(),
              [](const Vertex &a, const Vertex &b) {
                return a.cost < b.cost;
              });

    // Meilleur, deuxième pire, pire
    float bestCost = simplex[0].cost;
    float worstCost = simplex[N].cost;
    float secondWorstCost = simplex[N - 1].cost;

    // ── Test de convergence ──
    // L'écart entre le meilleur et le pire est petit
    if (std::abs(worstCost - bestCost) <
        tolerance * (std::abs(bestCost) + 1e-10f)) {
      result.converged = true;
      break;
    }

    // ── Centroïde de tous les sommets sauf le pire ──
    std::vector<float> centroid(N, 0.0f);
    for (int i = 0; i < N; i++) // N premiers (exclut le pire)
      for (int j = 0; j < N; j++)
        centroid[j] += simplex[i].x[j];
    for (int j = 0; j < N; j++)
      centroid[j] /= N;

    // ── Réflexion ──
    std::vector<float> reflected(N);
    for (int j = 0; j < N; j++)
      reflected[j] = centroid[j] + alpha * (centroid[j] - simplex[N].x[j]);
    clampToBounds(reflected, lo, hi);
    float reflectedCost = cost(reflected);

    if (reflectedCost < secondWorstCost && reflectedCost >= bestCost) {
      // Le point réfléchi est meilleur que le 2ème pire mais pas le meilleur
      simplex[N].x = reflected;
      simplex[N].cost = reflectedCost;
      continue;
    }

    // ── Expansion ──
    if (reflectedCost < bestCost) {
      std::vector<float> expanded(N);
      for (int j = 0; j < N; j++)
        expanded[j] = centroid[j] + gamma * (reflected[j] - centroid[j]);
      clampToBounds(expanded, lo, hi);
      float expandedCost = cost(expanded);

      if (expandedCost < reflectedCost) {
        simplex[N].x = expanded;
        simplex[N].cost = expandedCost;
      } else {
        simplex[N].x = reflected;
        simplex[N].cost = reflectedCost;
      }
      continue;
    }

    // ── Contraction ──
    std::vector<float> contracted(N);
    if (reflectedCost < worstCost) {
      // Outside contraction
      for (int j = 0; j < N; j++)
        contracted[j] = centroid[j] + rho * (reflected[j] - centroid[j]);
    } else {
      // Inside contraction
      for (int j = 0; j < N; j++)
        contracted[j] =
            centroid[j] + rho * (simplex[N].x[j] - centroid[j]);
    }
    clampToBounds(contracted, lo, hi);
    float contractedCost = cost(contracted);

    if (contractedCost < worstCost) {
      simplex[N].x = contracted;
      simplex[N].cost = contractedCost;
      continue;
    }

    // ── Réduction (shrink) ──
    // Rapprocher tous les sommets du meilleur
    for (int i = 1; i <= N; i++) {
      for (int j = 0; j < N; j++)
        simplex[i].x[j] =
            simplex[0].x[j] + sigma * (simplex[i].x[j] - simplex[0].x[j]);
      clampToBounds(simplex[i].x, lo, hi);
      simplex[i].cost = cost(simplex[i].x);
    }
  }

  // Trier une dernière fois pour avoir le meilleur en premier
  std::sort(simplex.begin(), simplex.end(),
            [](const Vertex &a, const Vertex &b) {
              return a.cost < b.cost;
            });

  result.bestParams = simplex[0].x;
  result.bestCost = simplex[0].cost;
  return result;
}
