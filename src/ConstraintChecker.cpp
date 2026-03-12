#include "ConstraintChecker.hpp"
#include <set>

std::vector<ConstraintChecker::Diagnostic>
ConstraintChecker::check(const ProblemDefinition &problem) {
  std::vector<Diagnostic> diagnostics;

  // ── 1. Vérifier que tous les param: référencés existent ──
  // On collecte les noms de paramètres déclarés
  std::set<std::string> declaredParams;
  for (const auto &[name, _] : problem.parameters)
    declaredParams.insert(name);

  // Vérifier les BC geometry params
  for (const auto &bc : problem.boundaryConditions) {
    for (const auto &[key, val] : bc.geometryParams) {
      if (val.find("param:") == 0) {
        std::string paramName = val.substr(6); // après "param:"
        if (declaredParams.find(paramName) == declaredParams.end()) {
          diagnostics.push_back(
              {"UNDEFINED_REFERENCE",
               "BC '" + bc.id + "' référence param:" + paramName +
                   " qui n'est pas déclaré dans 'parameters'",
               {bc.id, paramName},
               "Ajouter '" + paramName + "' dans la section 'parameters'"});
        }
      }
    }
  }

  // ── 2. Vérifier les bornes des variables ──
  for (const auto &[name, param] : problem.parameters) {
    if (param.type == "variable" || param.type == "variable_array") {
      if (param.lo >= param.hi) {
        diagnostics.push_back(
            {"INVALID_BOUNDS",
             "Paramètre '" + name + "' a des bornes invalides: lo=" +
                 std::to_string(param.lo) +
                 " >= hi=" + std::to_string(param.hi),
             {name},
             "Vérifier que bounds[0] < bounds[1]"});
      }
      if (param.type == "variable" &&
          (param.value < param.lo || param.value > param.hi)) {
        diagnostics.push_back(
            {"INITIAL_OUT_OF_BOUNDS",
             "Paramètre '" + name + "' a une valeur initiale (" +
                 std::to_string(param.value) + ") hors des bornes [" +
                 std::to_string(param.lo) + ", " + std::to_string(param.hi) +
                 "]",
             {name},
             "Ajuster la valeur initiale ou élargir les bornes"});
      }
    }
  }

  // ── 3. Compter variables vs contraintes (heuristique) ──
  int numVariables = 0;
  for (const auto &[_, param] : problem.parameters) {
    if (param.type == "variable")
      numVariables++;
    else if (param.type == "variable_array")
      numVariables += param.arraySize;
  }

  int numEquality = 0;
  for (const auto &c : problem.constraints) {
    if (c.op == "=" || c.op == "==")
      numEquality++;
  }
  for (const auto &fc : problem.fieldConstraints) {
    if (fc.op == "=")
      numEquality++;
  }

  if (numVariables == 0) {
    diagnostics.push_back(
        {"UNDERDETERMINED",
         "Aucune variable à optimiser. Tous les paramètres sont fixes ou "
         "dérivés.",
         {},
         "Marquer au moins un paramètre comme 'variable'"});
  }

  // ── 4. Vérifier que les materials référencés existent ──
  for (const auto &c : problem.constraints) {
    if (c.value.find("material:") != std::string::npos) {
      // Extraire le nom du matériau (material:wall.xxx → "wall")
      size_t start = c.value.find("material:") + 9;
      size_t dot = c.value.find('.', start);
      if (dot != std::string::npos) {
        std::string matName = c.value.substr(start, dot - start);
        if (problem.materials.find(matName) == problem.materials.end()) {
          diagnostics.push_back(
              {"UNDEFINED_REFERENCE",
               "Contrainte '" + c.id + "' référence material:" + matName +
                   " qui n'est pas déclaré dans 'materials'",
               {c.id, matName},
               "Ajouter '" + matName + "' dans la section 'materials'"});
        }
      }
    }
  }

  // ── 5. Vérifier que les named_points référencés existent ──
  for (const auto &fc : problem.fieldConstraints) {
    for (const auto &[varName, range] : fc.domain.ranges) {
      for (const auto &bound : {range.first, range.second}) {
        if (bound.find("point:") == 0) {
          size_t dot = bound.find('.');
          std::string pointName =
              (dot != std::string::npos) ? bound.substr(6, dot - 6)
                                         : bound.substr(6);
          if (problem.namedPoints.find(pointName) ==
              problem.namedPoints.end()) {
            diagnostics.push_back(
                {"UNDEFINED_REFERENCE",
                 "Field constraint '" + fc.id + "' référence point:" +
                     pointName + " qui n'est pas déclaré",
                 {fc.id, pointName},
                 "Ajouter '" + pointName +
                     "' dans la section 'named_points'"});
          }
        }
      }
    }
  }

  return diagnostics;
}
