#pragma once
#include "ExpressionEvaluator.hpp"
#include "ProblemDefinition.hpp"
#include <string>
#include <vector>

class ParameterSpace {
public:
  ParameterSpace(const ProblemDefinition &problem);
  std::vector<float> &values();
  const std::vector<float> &values() const;
  int dimension() const;
  const std::vector<float> &lowerBounds() const;
  const std::vector<float> &upperBounds() const;
  void updateContext(ExpressionEvaluator::Context &ctx) const;
  void computeDerived(ExpressionEvaluator::Context &ctx) const;
  const std::vector<std::string> &variableNames() const;

private:
  std::vector<float> _values, _lo, _hi;
  std::vector<std::string> _names;
  std::vector<Parameter> _derivedParams;
};
