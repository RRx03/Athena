#include "ParameterSpace.hpp"
#include <stdexcept>

ParameterSpace::ParameterSpace(const ProblemDefinition &problem) {
  // Parcourir tous les paramètres et extraire les variables
  for (const auto &[name, param] : problem.parameters) {

    if (param.type == "variable") {
      // Un seul float optimisable
      _names.push_back(name);
      _values.push_back(param.value);
      _lo.push_back(param.lo);
      _hi.push_back(param.hi);

    } else if (param.type == "variable_array") {
      // N floats optimisables
      for (int i = 0; i < param.arraySize; i++) {
        _names.push_back(name + "[" + std::to_string(i) + "]");
        // Valeur initiale : si fournie, utiliser ; sinon, milieu des bornes
        if (i < (int)param.arrayValues.size())
          _values.push_back(param.arrayValues[i]);
        else
          _values.push_back((param.lo + param.hi) * 0.5f);
        _lo.push_back(param.lo);
        _hi.push_back(param.hi);
      }

    } else if (param.type == "derived") {
      // Pas dans le vecteur d'optimisation — recalculé à chaque itération
      _derivedParams.push_back(param);

    }
    // "fixed" → ignoré, pas optimisable
  }
}

std::vector<float> &ParameterSpace::values() { return _values; }
const std::vector<float> &ParameterSpace::values() const { return _values; }
int ParameterSpace::dimension() const { return (int)_values.size(); }
const std::vector<float> &ParameterSpace::lowerBounds() const { return _lo; }
const std::vector<float> &ParameterSpace::upperBounds() const { return _hi; }
const std::vector<std::string> &ParameterSpace::variableNames() const {
  return _names;
}

void ParameterSpace::updateContext(ExpressionEvaluator::Context &ctx) const {
  // Injecter les variables optimisables dans le contexte
  for (size_t i = 0; i < _names.size(); i++) {
    // Pour les arrays, on met "profile_r[0]" dans le contexte
    // mais aussi la version sans index pour les paramètres scalaires
    ctx.parameters[_names[i]] = _values[i];
  }

  // Aussi injecter les paramètres scalaires par leur nom simple
  // (pour que "param:r_exit" fonctionne sans "[0]")
  // On parcourt les noms et si le nom ne contient pas "[", c'est un scalaire
  for (size_t i = 0; i < _names.size(); i++) {
    if (_names[i].find('[') == std::string::npos) {
      ctx.parameters[_names[i]] = _values[i];
    }
  }
}

void ParameterSpace::computeDerived(
    ExpressionEvaluator::Context &ctx) const {
  // Les paramètres dérivés sont recalculés via leur expression
  // Pour l'instant, on supporte les équations nommées (pressure_vessel, etc.)
  // qui seront implémentées dans les PhysicsModules.
  // Ici on évalue les expressions analytiques simples.
  for (const auto &dp : _derivedParams) {
    if (!dp.expression.empty() &&
        ExpressionEvaluator::isExpression(dp.expression)) {
      float val = ExpressionEvaluator::evaluate(dp.expression, ctx);
      ctx.parameters[dp.name] = val;
    }
    // Si l'expression est un nom de module ("pressure_vessel"),
    // elle sera traitée par le PhysicsModule correspondant.
  }
}
