#pragma once
#include "ExpressionEvaluator.hpp"
#include "ProblemDefinition.hpp"
#include <nlohmann/json.hpp>

class GeometryBuilder {
public:
  // Construit l'arbre CSG JSON pour le Geometric Kernel
  static nlohmann::json build(const ProblemDefinition &problem,
                               const ExpressionEvaluator::Context &ctx);
};
