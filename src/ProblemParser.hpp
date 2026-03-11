#pragma once
#include "ProblemDefinition.hpp"
#include <nlohmann/json.hpp>
#include <string>

// ═══════════════════════════════════════════════════════════════
// ProblemParser — Phase 2
//
// Lit un problem.json et remplit toutes les structs de
// ProblemDefinition. Les expressions ne sont PAS évaluées
// ici — elles sont stockées comme des strings brutes.
// L'évaluation se fait au runtime par ExpressionEvaluator
// quand le contexte est rempli avec les valeurs courantes.
// ═══════════════════════════════════════════════════════════════

class ProblemParser {
public:
  static ProblemDefinition parseFile(const std::string &filepath);
  static ProblemDefinition parse(const nlohmann::json &root);

private:
  using json = nlohmann::json;

  // Helpers : extraire une string depuis un JSON qui peut être string ou number
  static std::string toExprString(const json &val);

  // Sous-parsers pour chaque section
  static FrameConfig parseFrame(const json &j);
  static MaterialProperties parseMaterial(const std::string &name,
                                          const json &j);
  static NamedPoint parseNamedPoint(const std::string &name, const json &j);
  static BoundaryCondition parseBC(const json &j);
  static Distribution parseDistribution(const json &j);
  static Constraint parseConstraint(const json &j);
  static FieldConstraint parseFieldConstraint(const json &j);
  static DomainSpec parseDomain(const json &j);
  static Parameter parseParameter(const std::string &name, const json &j);
  static OptimizationConfig parseOptimization(const json &j);
};
