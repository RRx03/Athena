#include "Solver.hpp"
#include <iostream>
#include <stdexcept>

Solver::Result Solver::solve(const std::string &problemFile,
                             const std::string &geometryFile) {
  std::cout << "Athena::solve() — Phase 1 : ExpressionEvaluator operationnel" << std::endl;
  throw std::runtime_error(
      "Solver pas encore implémenté. Lancez avec --test pour vérifier l'ExpressionEvaluator.");
}
