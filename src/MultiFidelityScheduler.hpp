#pragma once
#include "Optimizer.hpp"
#include "ProblemDefinition.hpp"
#include <functional>
#include <vector>

class MultiFidelityScheduler {
public:
  struct LevelResult {
    Optimizer::Result optimResult;
    int fidelityLevel;
    std::string physicsModel;
    int samplesUsed;
  };
  static std::vector<LevelResult>
  run(const ProblemDefinition &problem,
      const std::function<float(const std::vector<float> &, int, const std::string &)> &costFn);
};
