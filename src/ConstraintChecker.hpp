#pragma once
#include "ProblemDefinition.hpp"
#include <string>
#include <vector>

class ConstraintChecker {
public:
  struct Diagnostic {
    std::string errorType;
    std::string message;
    std::vector<std::string> conflicting;
    std::string suggestion;
  };
  static std::vector<Diagnostic> check(const ProblemDefinition &problem);
};
