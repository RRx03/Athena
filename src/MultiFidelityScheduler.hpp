#pragma once
#include "Optimizer.hpp"
#include "ProblemDefinition.hpp"
#include <functional>
#include <string>
#include <vector>

class MultiFidelityScheduler {
public:
  struct LevelResult {
    Optimizer::Result optimResult;
    int fidelityLevel;
    std::string physicsModel;
    int samplesUsed;
  };

  // Exécute la boucle multi-fidélité.
  // costAtFidelity(params, samples, physicsModel) → coût scalaire
  using CostAtFidelity =
      std::function<float(const std::vector<float> &params, int samples,
                          const std::string &physics)>;

  static std::vector<LevelResult>
  run(const ProblemDefinition &problem, const std::vector<float> &initial,
      const std::vector<float> &lo, const std::vector<float> &hi,
      const CostAtFidelity &costFn);
};
