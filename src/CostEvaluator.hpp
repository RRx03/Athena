#pragma once
#include "ExpressionEvaluator.hpp"
#include "FieldConstraintEvaluator.hpp"
#include "ProblemDefinition.hpp"
#include <map>

class CostEvaluator {
public:
  struct CostBreakdown {
    float total;
    std::map<std::string, float> terms;
  };
  static CostBreakdown evaluate(const ProblemDefinition &problem,
                                const ExpressionEvaluator::Context &ctx,
                                const std::vector<FieldConstraintEvaluator::Violation> &fv);
};
