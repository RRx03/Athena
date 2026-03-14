#include "Solver.hpp"
#include "tests.hpp"
#include <iostream>
#include <string>

int main(int argc, char *argv[]) {
  std::cout << "╔══════════════════════════════════════╗" << std::endl;
  std::cout << "║   Athena — Generative Solver v0.1    ║" << std::endl;
  std::cout << "║   Unités : SI (mètres, Pa, K, kg)    ║" << std::endl;
  std::cout << "╚══════════════════════════════════════╝" << std::endl;

  if (argc >= 2 && std::string(argv[1]) == "--test") {
    std::cout << "\n=== MODE TEST ===" << std::endl;
    runTests();
    return 0;
  }

  std::string problemFile = (argc >= 2) ? argv[1] : "examples/nozzle_delaval.json";
  std::string outputFile = (argc >= 3) ? argv[2] : "geometry.json";

  std::cout << "\nProblem : " << problemFile << std::endl;
  std::cout << "Output  : " << outputFile << std::endl;

  try {
    auto result = Solver::solve(problemFile, outputFile);
    std::cout << "\n=== RÉSULTAT ===" << std::endl;
    std::cout << "Convergé   : " << (result.converged ? "oui" : "NON") << std::endl;
    std::cout << "Itérations : " << result.iterations << std::endl;
    std::cout << "Coût final : " << result.finalCost << std::endl;
    return result.converged ? 0 : 1;
  } catch (const std::exception &e) {
    std::cerr << "\n[ERREUR] " << e.what() << std::endl;
    return 2;
  }
}
