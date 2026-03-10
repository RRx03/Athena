#pragma once
#include "ProblemDefinition.hpp"
#include <nlohmann/json.hpp>
#include <string>

class ProblemParser {
public:
  static ProblemDefinition parseFile(const std::string &filepath);
};
