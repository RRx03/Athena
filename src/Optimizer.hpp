#pragma once
#include <functional>
#include <vector>

class Optimizer {
public:
  using CostFn = std::function<float(const std::vector<float> &)>;
  struct Result {
    std::vector<float> bestParams;
    float bestCost;
    int iterations;
    bool converged;
  };
  static Result nelderMead(const CostFn &cost, std::vector<float> initial,
                           const std::vector<float> &lo, const std::vector<float> &hi,
                           int maxIter, float tolerance);
  static Result gradientDescent(const CostFn &cost, std::vector<float> initial,
                                const std::vector<float> &lo, const std::vector<float> &hi,
                                int maxIter, float tolerance, float lr);
};
