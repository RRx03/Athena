#include "Solver.hpp"
#include "ConstraintChecker.hpp"
#include "CoordinateResolver.hpp"
#include "CostEvaluator.hpp"
#include "FieldConstraintEvaluator.hpp"
#include "GeometryBuilder.hpp"
#include "MultiFidelityScheduler.hpp"
#include "Optimizer.hpp"
#include "ParameterSpace.hpp"
#include "ProblemParser.hpp"
#include "physics/IsentropicNozzle.hpp"
#include "physics/PressureVessel.hpp"
#include <fstream>
#include <iostream>

Solver::Result Solver::solve(const std::string &problemFile,
                             const std::string &geometryFile) {
  Result result;
  result.geometryFile = geometryFile;

  // ── 1. Parser le problème ──
  std::cout << "\n[1/7] Parsing " << problemFile << "..." << std::endl;
  auto pd = ProblemParser::parseFile(problemFile);
  result.problem = pd;
  std::cout << "  Problem: " << pd.problemType << " (" << pd.units << ")"
            << std::endl;

  // ── 2. Vérifier la cohérence ──
  std::cout << "[2/7] Vérification des contraintes..." << std::endl;
  auto diags = ConstraintChecker::check(pd);
  for (const auto &d : diags) {
    std::cout << "  [" << d.errorType << "] " << d.message << std::endl;
    if (!d.suggestion.empty())
      std::cout << "    → " << d.suggestion << std::endl;
  }
  // Les warnings ne bloquent pas, seules les erreurs critiques
  // (pas encore implémenté : distinguer warning vs error)

  // ── 3. Résoudre le repère ──
  std::cout << "[3/7] Résolution du repère..." << std::endl;
  auto frame = CoordinateResolver::resolveFrame(pd);
  std::cout << "  Origin: (" << frame.origin.x << ", " << frame.origin.y
            << ", " << frame.origin.z << ")" << std::endl;
  std::cout << "  Axis:   (" << frame.axis.x << ", " << frame.axis.y << ", "
            << frame.axis.z << ")" << std::endl;

  // ── 4. Construire l'espace de paramètres ──
  std::cout << "[4/7] Construction de l'espace de paramètres..." << std::endl;
  ParameterSpace ps(pd);
  std::cout << "  Dimension: " << ps.dimension() << " variables" << std::endl;
  for (size_t i = 0; i < ps.variableNames().size() && i < 10; i++) {
    std::cout << "    " << ps.variableNames()[i] << " = " << ps.values()[i]
              << " [" << ps.lowerBounds()[i] << ", " << ps.upperBounds()[i]
              << "]" << std::endl;
  }
  if (ps.variableNames().size() > 10)
    std::cout << "    ... (" << ps.variableNames().size() - 10 << " de plus)"
              << std::endl;

  // ── 5. Configurer les modules physiques ──
  std::cout << "[5/7] Configuration des modules physiques..." << std::endl;
  IsentropicNozzle nozzle;
  PressureVessel vessel;

  // Contexte de base (paramètres fixes + matériaux)
  ExpressionEvaluator::Context baseCtx;
  // Ajouter les paramètres fixes
  for (const auto &[name, param] : pd.parameters) {
    if (param.type == "fixed")
      baseCtx.parameters[name] = param.value;
  }
  // Ajouter les matériaux
  for (const auto &[matName, mat] : pd.materials)
    for (const auto &[propName, propVal] : mat.properties)
      baseCtx.materialProperties[matName + "." + propName] = propVal;

  nozzle.setup(pd, baseCtx);
  vessel.setup(pd, baseCtx);

  // ── 6. Optimisation ──
  std::cout << "[6/7] Optimisation (" << pd.optimization.mode << " / "
            << pd.optimization.method << ")..." << std::endl;

  // Fonction de coût : params → coût scalaire
  auto costAtFidelity = [&](const std::vector<float> &params, int samples,
                            const std::string &physics) -> float {
    // Mettre à jour le ParameterSpace avec les nouvelles valeurs
    auto &vals = ps.values();
    for (size_t i = 0; i < params.size() && i < vals.size(); i++)
      vals[i] = params[i];

    // Construire le contexte complet
    ExpressionEvaluator::Context ctx = baseCtx;
    ps.updateContext(ctx);

    // Résoudre les named_points
    for (const auto &[name, np] : pd.namedPoints) {
      try {
        float x = ExpressionEvaluator::resolve(np.position[0], ctx);
        float y = ExpressionEvaluator::resolve(np.position[1], ctx);
        float z = ExpressionEvaluator::resolve(np.position[2], ctx);
        ctx.namedPoints[name] = simd::float3{x, y, z};
      } catch (...) {
      }
    }

    // Résoudre la physique
    nozzle.solve(ctx);

    // Brancher le fieldEvaluator sur le nozzle + vessel
    ctx.fieldEvaluator = [&nozzle, &vessel](const std::string &field,
                                             simd::float3 pt) -> float {
      // Le vessel répond aux grandeurs structurelles
      if (field == "wall_thickness" || field == "von_mises" ||
          field == "von_mises_max" || field == "von_mises_stress" ||
          field == "wall_thickness_max")
        return vessel.fieldAt(field, pt);
      // Tout le reste → nozzle
      return nozzle.fieldAt(field, pt);
    };

    // Résoudre le vessel (dépend du nozzle pour pression/rayon)
    vessel.solve(ctx);

    // Injecter les paramètres dérivés
    ctx.parameters["wall_thickness"] = vessel.maxThickness();
    ps.computeDerived(ctx);

    // Évaluer les field constraints
    auto fieldViolations =
        FieldConstraintEvaluator::evaluate(pd.fieldConstraints, ctx);

    // Évaluer le coût
    auto cb = CostEvaluator::evaluate(pd, ctx, fieldViolations);
    return cb.total;
  };

  // Lancer le scheduler multi-fidélité
  auto levelResults = MultiFidelityScheduler::run(
      pd, ps.values(), ps.lowerBounds(), ps.upperBounds(), costAtFidelity);

  // Récupérer le meilleur résultat
  if (!levelResults.empty()) {
    auto &best = levelResults.back();
    result.converged = best.optimResult.converged;
    result.iterations = best.optimResult.iterations;
    result.finalCost = best.optimResult.bestCost;

    // Mettre à jour les paramètres avec le meilleur résultat
    auto &vals = ps.values();
    for (size_t i = 0;
         i < best.optimResult.bestParams.size() && i < vals.size(); i++)
      vals[i] = best.optimResult.bestParams[i];
  }

  // ── 7. Construire la géométrie ──
  std::cout << "[7/7] Construction de la géométrie..." << std::endl;

  // Contexte final
  ExpressionEvaluator::Context finalCtx = baseCtx;
  ps.updateContext(finalCtx);

  // Résoudre la physique une dernière fois pour les grandeurs finales
  nozzle.solve(finalCtx);
  finalCtx.fieldEvaluator = [&nozzle](const std::string &field,
                                       simd::float3 pt) -> float {
    return nozzle.fieldAt(field, pt);
  };
  vessel.solve(finalCtx);
  finalCtx.parameters["wall_thickness"] = vessel.maxThickness();

  // Construire le JSON CSG
  auto geometry = GeometryBuilder::build(pd, finalCtx);

  // Ajouter les métadonnées
  geometry["metadata"] = {
      {"problem", pd.problemType},
      {"converged", result.converged},
      {"iterations", result.iterations},
      {"final_cost", result.finalCost},
      {"thrust", nozzle.thrust()},
      {"specific_impulse", nozzle.isp()},
      {"mass_flow", nozzle.massFlow()},
      {"exit_mach", nozzle.exitMach()},
      {"wall_thickness_max", vessel.maxThickness()},
      {"r_throat", finalCtx.parameters["r_throat"]},
      {"r_exit", finalCtx.parameters["r_exit"]},
      {"L_nozzle", finalCtx.parameters["L_nozzle"]}};

  // Écrire le fichier
  std::ofstream out(geometryFile);
  out << geometry.dump(2);
  out.close();

  std::cout << "\n=== Résultats ===" << std::endl;
  std::cout << "  Poussée    : " << nozzle.thrust() / 1000.0f << " kN"
            << std::endl;
  std::cout << "  Isp        : " << nozzle.isp() << " s" << std::endl;
  std::cout << "  Débit      : " << nozzle.massFlow() << " kg/s" << std::endl;
  std::cout << "  Mach sortie: " << nozzle.exitMach() << std::endl;
  std::cout << "  Épaisseur  : " << vessel.maxThickness() * 1000.0f << " mm"
            << std::endl;
  std::cout << "  Géométrie  : " << geometryFile << std::endl;

  return result;
}
