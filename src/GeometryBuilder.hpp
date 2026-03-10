#pragma once
#include "ExpressionEvaluator.hpp"
#include "ProblemDefinition.hpp"
#include <nlohmann/json.hpp>

class GeometryBuilder {
public:
  static nlohmann::json build(const ProblemDefinition &problem,
                               const ExpressionEvaluator::Context &ctx);
};
