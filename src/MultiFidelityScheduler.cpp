#include "MultiFidelityScheduler.hpp"
#include <iostream>

std::vector<MultiFidelityScheduler::LevelResult>
MultiFidelityScheduler::run(const ProblemDefinition &problem,
                            const std::vector<float> &initial,
                            const std::vector<float> &lo,
                            const std::vector<float> &hi,
                            const CostAtFidelity &costFn) {
  std::vector<LevelResult> results;
  const auto &opt = problem.optimization;

  if (opt.mode == "fixed") {
    // Mode fixed : un seul niveau
    int samples = opt.fixedSampleSize;
    int maxIter = opt.fixedMaxIterations;
    std::string physics = "isentropic_1d";

    auto wrappedCost = [&](const std::vector<float> &p) -> float {
      return costFn(p, samples, physics);
    };

    std::cout << "  [Fixed] samples=" << samples << " maxIter=" << maxIter
              << std::endl;
    auto optResult =
        Optimizer::nelderMead(wrappedCost, initial, lo, hi, maxIter, 1e-6f);

    results.push_back({optResult, 0, physics, samples});

  } else if (opt.mode == "dynamic") {
    // Mode dynamic : parcourir les niveaux de fidélité
    std::vector<float> currentParams = initial;

    for (int lvl = 0; lvl < (int)opt.fidelityLevels.size(); lvl++) {
      const auto &fl = opt.fidelityLevels[lvl];

      auto wrappedCost = [&](const std::vector<float> &p) -> float {
        return costFn(p, fl.samples, fl.physicsModel);
      };

      std::cout << "  [Level " << lvl << "] physics=" << fl.physicsModel
                << " samples=" << fl.samples
                << " maxIter=" << fl.maxIterations << std::endl;

      auto optResult = Optimizer::nelderMead(wrappedCost, currentParams, lo,
                                             hi, fl.maxIterations, 1e-6f);

      std::cout << "    → cost=" << optResult.bestCost
                << " iter=" << optResult.iterations
                << " converged=" << optResult.converged << std::endl;

      results.push_back({optResult, lvl, fl.physicsModel, fl.samples});

      // Passer les meilleurs paramètres au niveau suivant
      currentParams = optResult.bestParams;
    }

  } else if (opt.mode == "convergence") {
    // Mode convergence : augmenter les samples tant que les critères
    // ne sont pas satisfaits
    std::vector<float> currentParams = initial;
    int samples = opt.convergenceMinSamples;
    int totalIter = 0;

    while (samples <= opt.convergenceMaxSamples &&
           totalIter < opt.convergenceMaxIterations) {

      auto wrappedCost = [&](const std::vector<float> &p) -> float {
        return costFn(p, samples, "isentropic_1d");
      };

      int iterBudget =
          std::min(200, opt.convergenceMaxIterations - totalIter);

      std::cout << "  [Convergence] samples=" << samples
                << " iterBudget=" << iterBudget << std::endl;

      auto optResult = Optimizer::nelderMead(wrappedCost, currentParams, lo,
                                             hi, iterBudget, 1e-6f);

      totalIter += optResult.iterations;
      results.push_back({optResult, (int)results.size(), "isentropic_1d", samples});
      currentParams = optResult.bestParams;

      std::cout << "    → cost=" << optResult.bestCost
                << " iter=" << optResult.iterations << std::endl;

      // Si convergé, raffiner
      if (optResult.converged || optResult.bestCost < 1e-4f)
        samples = (int)(samples * opt.convergenceSampleGrowth);
      else
        break; // Pas convergé → inutile de raffiner
    }

  } else {
    // Mode inconnu → traiter comme fixed avec les défauts
    auto wrappedCost = [&](const std::vector<float> &p) -> float {
      return costFn(p, 50, "isentropic_1d");
    };
    auto optResult =
        Optimizer::nelderMead(wrappedCost, initial, lo, hi, 500, 1e-6f);
    results.push_back({optResult, 0, "isentropic_1d", 50});
  }

  return results;
}
