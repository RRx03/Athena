#pragma once
#include "ProblemDefinition.hpp"
#include <string>

class Solver {
public:
  struct Result {
    bool converged = false;
    int iterations = 0;
    float finalCost = 1e30f;
    std::string geometryFile;
    ProblemDefinition problem;
  };

  static Result solve(const std::string &problemFile,
                      const std::string &geometryFile);
};
