#pragma once
#include "ProblemDefinition.hpp"
#include <string>

class Solver {
public:
  struct Result {
    bool converged;
    int iterations;
    float finalCost;
    std::string geometryFile;
    ProblemDefinition problem;
  };
  static Result solve(const std::string &problemFile,
                      const std::string &geometryFile);
};
