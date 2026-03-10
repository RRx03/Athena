#pragma once
#include "ExpressionEvaluator.hpp"
#include "ProblemDefinition.hpp"
#include <vector>

class FieldConstraintEvaluator {
public:
  struct Violation {
    std::string constraintId;
    float maxViolation;
    float l2Violation;
    int numSamples;
    int numViolated;
  };
  static std::vector<Violation>
  evaluate(const std::vector<FieldConstraint> &constraints,
           const ExpressionEvaluator::Context &ctx);
};
