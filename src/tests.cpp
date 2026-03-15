#include "ExpressionEvaluator.hpp"
#include "ProblemParser.hpp"
#include "CoordinateResolver.hpp"
#include "ParameterSpace.hpp"
#include "ConstraintChecker.hpp"
#include "FieldConstraintEvaluator.hpp"
#include "CostEvaluator.hpp"
#include "Optimizer.hpp"
#include "Solver.hpp"
#include "physics/IsentropicNozzle.hpp"
#include "physics/PressureVessel.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <cassert>
#include <cmath>
#include <iostream>

static int passed = 0, failed = 0;

static void check(const std::string &name, float got, float expected, float eps = 1e-4f) {
  if (std::abs(got - expected) < eps) { passed++; }
  else {
    std::cerr << "  ECHEC: " << name << " — attendu " << expected << " obtenu " << got << std::endl;
    failed++;
  }
}

static void checkStr(const std::string &name, const std::string &got, const std::string &expected) {
  if (got == expected) { passed++; }
  else {
    std::cerr << "  ECHEC: " << name << " — attendu '" << expected << "' obtenu '" << got << "'" << std::endl;
    failed++;
  }
}

static void checkBool(const std::string &name, bool got, bool expected) {
  if (got == expected) { passed++; }
  else {
    std::cerr << "  ECHEC: " << name << " — attendu " << expected << " obtenu " << got << std::endl;
    failed++;
  }
}

static void checkInt(const std::string &name, int got, int expected) {
  if (got == expected) { passed++; }
  else {
    std::cerr << "  ECHEC: " << name << " — attendu " << expected << " obtenu " << got << std::endl;
    failed++;
  }
}

static ExpressionEvaluator::Context makeCtx() {
  ExpressionEvaluator::Context ctx;
  ctx.parameters["r_exit"] = 0.035f;
  ctx.parameters["r_throat"] = 0.015f;
  ctx.parameters["L_nozzle"] = 0.3f;
  ctx.parameters["V_max"] = 500.0f;
  ctx.materialProperties["wall.yield_strength"] = 1035e6f;
  ctx.materialProperties["wall.safety_factor"] = 1.5f;
  ctx.materialProperties["wall.density"] = 8190.0f;
  ctx.materialProperties["propellant.gamma"] = 1.2f;
  ctx.namedPoints["THROAT"] = simd::float3{0.015f, 0.0f, 0.15f};
  ctx.namedPoints["ENDTUYERE"] = simd::float3{0.035f, 0.0f, 0.3f};
  ctx.namedPoints["ORIGIN"] = simd::float3{0.0f, 0.0f, 0.0f};
  ctx.domainVariables["z"] = 0.1f;
  ctx.domainVariables["r"] = 0.02f;
  ctx.fieldEvaluator = [](const std::string &field, simd::float3 p) -> float {
    if (field == "velocity") return 500.0f * (1.0f - p.x * p.x);
    if (field == "pressure") return 7e6f;
    if (field == "mach") return 2.5f;
    return 0.0f;
  };
  return ctx;
}

void runTests() {
  auto ctx = makeCtx();

  std::cout << "\n--- Littéraux ---" << std::endl;
  check("int",   ExpressionEvaluator::resolve("42", ctx), 42.0f);
  check("float", ExpressionEvaluator::resolve("3.14", ctx), 3.14f);
  check("sci",   ExpressionEvaluator::resolve("1e6", ctx), 1e6f);
  check("neg",   ExpressionEvaluator::resolve("-2.5", ctx), -2.5f);

  std::cout << "--- Arithmétique ---" << std::endl;
  check("add",    ExpressionEvaluator::evaluate("expr:2 + 3", ctx), 5.0f);
  check("sub",    ExpressionEvaluator::evaluate("expr:10 - 4", ctx), 6.0f);
  check("mul",    ExpressionEvaluator::evaluate("expr:3 * 7", ctx), 21.0f);
  check("div",    ExpressionEvaluator::evaluate("expr:20 / 4", ctx), 5.0f);
  check("pow",    ExpressionEvaluator::evaluate("expr:2 ^ 10", ctx), 1024.0f);
  check("prio",   ExpressionEvaluator::evaluate("expr:2 + 3 * 4", ctx), 14.0f);
  check("paren",  ExpressionEvaluator::evaluate("expr:(2 + 3) * 4", ctx), 20.0f);
  check("unary",  ExpressionEvaluator::evaluate("expr:-3 + 5", ctx), 2.0f);

  std::cout << "--- Fonctions ---" << std::endl;
  check("sin0",   ExpressionEvaluator::evaluate("expr:sin(0)", ctx), 0.0f);
  check("cos0",   ExpressionEvaluator::evaluate("expr:cos(0)", ctx), 1.0f);
  check("sqrt9",  ExpressionEvaluator::evaluate("expr:sqrt(9)", ctx), 3.0f);
  check("abs-5",  ExpressionEvaluator::evaluate("expr:abs(-5)", ctx), 5.0f);
  check("min37",  ExpressionEvaluator::evaluate("expr:min(3, 7)", ctx), 3.0f);
  check("max37",  ExpressionEvaluator::evaluate("expr:max(3, 7)", ctx), 7.0f);
  check("pow23",  ExpressionEvaluator::evaluate("expr:pow(2, 3)", ctx), 8.0f);
  check("exp0",   ExpressionEvaluator::evaluate("expr:exp(0)", ctx), 1.0f);
  check("log1",   ExpressionEvaluator::evaluate("expr:log(1)", ctx), 0.0f);

  std::cout << "--- param: ---" << std::endl;
  check("r_exit",   ExpressionEvaluator::evaluate("param:r_exit", ctx), 0.035f);
  check("r_throat", ExpressionEvaluator::evaluate("param:r_throat", ctx), 0.015f);
  check("param*2",  ExpressionEvaluator::evaluate("expr:param:r_exit * 2", ctx), 0.07f);

  std::cout << "--- material: ---" << std::endl;
  check("yield",  ExpressionEvaluator::evaluate("material:wall.yield_strength", ctx), 1035e6f);
  check("y/sf",   ExpressionEvaluator::evaluate("expr:material:wall.yield_strength / material:wall.safety_factor", ctx), 690e6f);

  std::cout << "--- point: ---" << std::endl;
  check("throat.x", ExpressionEvaluator::evaluate("point:THROAT.x", ctx), 0.015f);
  check("end.z",    ExpressionEvaluator::evaluate("point:ENDTUYERE.z", ctx), 0.3f);

  std::cout << "--- domain: ---" << std::endl;
  check("dom_z", ExpressionEvaluator::evaluate("domain:z", ctx), 0.1f);
  check("dom_r", ExpressionEvaluator::evaluate("domain:r", ctx), 0.02f);
  check("dom_e", ExpressionEvaluator::evaluate("expr:domain:z * 2 + 1", ctx), 1.2f);

  std::cout << "--- field: ---" << std::endl;
  check("f@pt",   ExpressionEvaluator::evaluate("field:pressure@THROAT", ctx), 7e6f);
  check("f@[0]",  ExpressionEvaluator::evaluate("field:velocity@[0, 0, 0.1]", ctx), 500.0f);
  check("f@[.5]", ExpressionEvaluator::evaluate("field:velocity@[0.5, 0, 0.1]", ctx), 500.0f * 0.75f);

  std::cout << "--- Complexes ---" << std::endl;
  check("area_ratio", ExpressionEvaluator::evaluate("expr:(param:r_exit / param:r_throat) ^ 2", ctx),
        (0.035f / 0.015f) * (0.035f / 0.015f));
  check("mixed", ExpressionEvaluator::evaluate("expr:2 * field:velocity@[0, 0, domain:z]", ctx), 1000.0f);

  // ═══════════════════════════════════════════════════════════
  // Phase 2 — ProblemParser
  // ═══════════════════════════════════════════════════════════
  std::cout << "\n=== PHASE 2 : ProblemParser ===" << std::endl;

  try {
    auto pd = ProblemParser::parseFile("examples/nozzle_delaval.json");

    std::cout << "--- Meta ---" << std::endl;
    checkStr("problemType", pd.problemType, "nozzle_delaval");
    checkStr("units", pd.units, "SI");
    checkStr("frame.mode", pd.frame.mode, "auto");

    std::cout << "--- Materials ---" << std::endl;
    checkBool("has propellant", pd.materials.count("propellant") > 0, true);
    checkBool("has wall", pd.materials.count("wall") > 0, true);
    checkStr("wall name", pd.materials["wall"].name, "Inconel 718");
    check("gamma", pd.materials["propellant"].properties["gamma"], 1.2f);
    check("yield", pd.materials["wall"].properties["yield_strength"], 1035e6f);
    check("SF", pd.materials["wall"].properties["safety_factor"], 1.5f);

    std::cout << "--- Named Points ---" << std::endl;
    checkInt("num points", (int)pd.namedPoints.size(), 3);
    checkStr("INLET type", pd.namedPoints["INLET_CENTER"].type, "fixed");
    checkStr("INLET pos[0]", pd.namedPoints["INLET_CENTER"].position[0], "0");
    checkStr("ENDTUYERE type", pd.namedPoints["ENDTUYERE"].type, "constrained");
    checkStr("ENDTUYERE pos[2]", pd.namedPoints["ENDTUYERE"].position[2], "param:L_nozzle");
    checkStr("THROAT type", pd.namedPoints["THROAT"].type, "variable");

    std::cout << "--- Boundary Conditions ---" << std::endl;
    checkInt("num BCs", (int)pd.boundaryConditions.size(), 2);
    checkStr("BC0 id", pd.boundaryConditions[0].id, "inlet");
    checkStr("BC0 geom type", pd.boundaryConditions[0].geometryType, "Disk2D");
    checkStr("BC0 radius", pd.boundaryConditions[0].geometryParams["radius"], "0.04");
    checkStr("BC0 P type", pd.boundaryConditions[0].distributions["total_pressure"].type, "uniform");
    checkStr("BC0 P value", pd.boundaryConditions[0].distributions["total_pressure"].expression, "7e6");
    checkStr("BC1 id", pd.boundaryConditions[1].id, "outlet");
    checkStr("BC1 radius", pd.boundaryConditions[1].geometryParams["radius"], "param:r_exit");
    checkStr("BC1 anchor", pd.boundaryConditions[1].anchors["concentric_with"], "inlet");

    std::cout << "--- Constraints ---" << std::endl;
    checkInt("num constraints", (int)pd.constraints.size(), 3);
    checkStr("C0 id", pd.constraints[0].id, "thrust");
    checkStr("C0 op", pd.constraints[0].op, ">=");
    checkStr("C0 value", pd.constraints[0].value, "50000.0");
    checkInt("C0 priority", pd.constraints[0].priority, 1);
    checkStr("C2 op", pd.constraints[2].op, "minimize");

    std::cout << "--- Field Constraints ---" << std::endl;
    checkInt("num FC", (int)pd.fieldConstraints.size(), 1);
    checkStr("FC0 id", pd.fieldConstraints[0].id, "monotonic_mach");
    checkStr("FC0 coord", pd.fieldConstraints[0].coordinateSystem, "axisymmetric");
    checkStr("FC0 domain type", pd.fieldConstraints[0].domain.type, "parametric_1d");
    checkInt("FC0 domain vars", (int)pd.fieldConstraints[0].domain.variables.size(), 1);
    checkStr("FC0 domain var[0]", pd.fieldConstraints[0].domain.variables[0], "z");
    checkStr("FC0 lhs", pd.fieldConstraints[0].lhs, "field:mach@[0, z + 0.001]");
    checkStr("FC0 op", pd.fieldConstraints[0].op, ">=");
    checkStr("FC0 rhs", pd.fieldConstraints[0].rhs, "field:mach@[0, z]");
    checkInt("FC0 samples", pd.fieldConstraints[0].domain.samples[0], 30);

    std::cout << "--- Parameters ---" << std::endl;
    checkInt("num params", (int)pd.parameters.size(), 6);
    checkStr("r_exit type", pd.parameters["r_exit"].type, "variable");
    check("r_exit value", pd.parameters["r_exit"].value, 0.035f);
    check("r_exit lo", pd.parameters["r_exit"].lo, 0.01f);
    check("r_exit hi", pd.parameters["r_exit"].hi, 0.15f);
    checkStr("wall type", pd.parameters["wall_thickness"].type, "derived");
    checkStr("wall expr", pd.parameters["wall_thickness"].expression, "pressure_vessel");
    checkStr("profile type", pd.parameters["profile_r"].type, "variable_array");
    checkInt("profile size", pd.parameters["profile_r"].arraySize, 7);

    std::cout << "--- Optimization ---" << std::endl;
    checkStr("opt mode", pd.optimization.mode, "dynamic");
    checkStr("opt method", pd.optimization.method, "nelder_mead");
    checkStr("opt precision", pd.optimization.precision, "standard");
    checkInt("fidelity levels", (int)pd.optimization.fidelityLevels.size(), 2);
    checkInt("level0 samples", pd.optimization.fidelityLevels[0].samples, 10);
    checkInt("level0 maxiter", pd.optimization.fidelityLevels[0].maxIterations, 100);
    checkStr("level0 physics", pd.optimization.fidelityLevels[0].physicsModel, "isentropic_1d");
    checkInt("level1 samples", pd.optimization.fidelityLevels[1].samples, 50);

    std::cout << "\n  Phase 2 : ProblemParser OK" << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "\n  ECHEC Phase 2 : " << e.what() << std::endl;
    failed++;
  }

  // ═══════════════════════════════════════════════════════════
  // Phase 3 — CoordinateResolver
  // ═══════════════════════════════════════════════════════════
  std::cout << "\n=== PHASE 3 : CoordinateResolver ===" << std::endl;

  try {
    auto pd = ProblemParser::parseFile("examples/nozzle_delaval.json");
    auto frame = CoordinateResolver::resolveFrame(pd);

    std::cout << "--- Auto mode ---" << std::endl;
    // Le JSON a axis_alignment: "z", donc l'axe doit être Z
    check("axis.x", frame.axis.x, 0.0f);
    check("axis.y", frame.axis.y, 0.0f);
    check("axis.z", frame.axis.z, 1.0f);
    // Origin doit être (0,0,0) car INLET_CENTER est fixed à [0,0,0]
    check("origin.x", frame.origin.x, 0.0f);
    check("origin.y", frame.origin.y, 0.0f);
    check("origin.z", frame.origin.z, 0.0f);
    // Right = cross(axis, up) doit être non-nul
    float rightLen = simd_length(frame.right);
    checkBool("right non-zero", rightLen > 0.5f, true);

    // Test round-trip : toSolver → toUser = identité
    simd::float3 testPt = {1.0f, 2.0f, 3.0f};
    auto inSolver = CoordinateResolver::toSolverFrame(testPt, frame);
    auto backToUser = CoordinateResolver::toUserFrame(inSolver, frame);
    check("roundtrip.x", backToUser.x, testPt.x, 1e-3f);
    check("roundtrip.y", backToUser.y, testPt.y, 1e-3f);
    check("roundtrip.z", backToUser.z, testPt.z, 1e-3f);

    // Test manual mode
    ProblemDefinition pdManual = pd;
    pdManual.frame.mode = "manual";
    pdManual.frame.origin = {1.0f, 2.0f, 3.0f};
    pdManual.frame.axis = {0, 1, 0};
    pdManual.frame.up = {0, 0, 1};
    auto frameM = CoordinateResolver::resolveFrame(pdManual);
    check("manual origin.x", frameM.origin.x, 1.0f);
    check("manual origin.y", frameM.origin.y, 2.0f);
    check("manual axis.y", frameM.axis.y, 1.0f);

    std::cout << "\n  Phase 3 : CoordinateResolver OK" << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "\n  ECHEC Phase 3 : " << e.what() << std::endl;
    failed++;
  }

  // ═══════════════════════════════════════════════════════════
  // Phase 4 — ParameterSpace + ConstraintChecker
  // ═══════════════════════════════════════════════════════════
  std::cout << "\n=== PHASE 4 : ParameterSpace + ConstraintChecker ===" << std::endl;

  try {
    auto pd = ProblemParser::parseFile("examples/nozzle_delaval.json");

    // ── ParameterSpace ──
    ParameterSpace ps(pd);

    std::cout << "--- ParameterSpace ---" << std::endl;
    // 4 variables scalaires (r_exit, r_throat, L_nozzle, z_throat)
    // + 7 variable_array (profile_r[0..6])
    // = 11 dimensions
    checkInt("dimension", ps.dimension(), 11);
    checkBool("has names", ps.variableNames().size() == 11, true);

    // Vérifier que r_exit est dans le vecteur avec la bonne valeur initiale
    bool foundRexit = false;
    for (size_t i = 0; i < ps.variableNames().size(); i++) {
      if (ps.variableNames()[i] == "r_exit") {
        check("r_exit initial", ps.values()[i], 0.035f);
        check("r_exit lo", ps.lowerBounds()[i], 0.01f);
        check("r_exit hi", ps.upperBounds()[i], 0.15f);
        foundRexit = true;
      }
    }
    checkBool("found r_exit", foundRexit, true);

    // Tester updateContext
    ExpressionEvaluator::Context testCtx;
    ps.updateContext(testCtx);
    checkBool("ctx has r_exit", testCtx.parameters.count("r_exit") > 0, true);
    check("ctx r_exit val", testCtx.parameters["r_exit"], 0.035f);

    // Modifier une valeur et vérifier
    for (size_t i = 0; i < ps.variableNames().size(); i++) {
      if (ps.variableNames()[i] == "r_exit") {
        ps.values()[i] = 0.05f;
      }
    }
    ExpressionEvaluator::Context testCtx2;
    ps.updateContext(testCtx2);
    check("ctx r_exit modified", testCtx2.parameters["r_exit"], 0.05f);

    // ── ConstraintChecker ──
    std::cout << "--- ConstraintChecker ---" << std::endl;
    auto diags = ConstraintChecker::check(pd);
    // Le nozzle_delaval.json est valide, pas de diagnostics attendus
    checkInt("no diagnostics", (int)diags.size(), 0);

    // Tester avec un problème invalide
    ProblemDefinition badPd = pd;
    badPd.parameters["r_exit"].lo = 1.0f;
    badPd.parameters["r_exit"].hi = 0.5f; // lo > hi
    auto badDiags = ConstraintChecker::check(badPd);
    checkBool("detected bad bounds", badDiags.size() > 0, true);
    if (!badDiags.empty())
      checkStr("diag type", badDiags[0].errorType, "INVALID_BOUNDS");

    // Tester sans variables
    ProblemDefinition noVarPd;
    noVarPd.parameters["fixed_only"] = {"fixed_only", "fixed", "", 1.0f, 0, 0, 0, {}};
    auto noVarDiags = ConstraintChecker::check(noVarPd);
    checkBool("detected no vars", noVarDiags.size() > 0, true);

    std::cout << "\n  Phase 4 : ParameterSpace + ConstraintChecker OK" << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "\n  ECHEC Phase 4 : " << e.what() << std::endl;
    failed++;
  }

  // ═══════════════════════════════════════════════════════════
  // Phase 5 — IsentropicNozzle + PressureVessel
  // ═══════════════════════════════════════════════════════════
  std::cout << "\n=== PHASE 5 : PhysicsModules ===" << std::endl;

  try {
    auto pd = ProblemParser::parseFile("examples/nozzle_delaval.json");
    ParameterSpace ps(pd);

    // Construire le contexte
    ExpressionEvaluator::Context physCtx;
    ps.updateContext(physCtx);

    // Ajouter les matériaux au contexte
    for (const auto &[matName, mat] : pd.materials)
      for (const auto &[propName, propVal] : mat.properties)
        physCtx.materialProperties[matName + "." + propName] = propVal;

    // ── IsentropicNozzle ──
    std::cout << "--- IsentropicNozzle ---" << std::endl;
    IsentropicNozzle nozzle;
    nozzle.setup(pd, physCtx);
    nozzle.solve(physCtx);

    // Vérifications physiques fondamentales
    float thrust = nozzle.thrust();
    float isp = nozzle.isp();
    float mdot = nozzle.massFlow();
    float exitMach = nozzle.exitMach();

    std::cout << "  Thrust   = " << thrust / 1000.0f << " kN" << std::endl;
    std::cout << "  Isp      = " << isp << " s" << std::endl;
    std::cout << "  mdot     = " << mdot << " kg/s" << std::endl;
    std::cout << "  M_exit   = " << exitMach << std::endl;

    // Poussée doit être positive
    checkBool("thrust > 0", thrust > 0, true);
    // Isp raisonnable pour LOX/RP-1 (200-350s)
    checkBool("isp range", isp > 100.0f && isp < 500.0f, true);
    // Débit positif
    checkBool("mdot > 0", mdot > 0, true);
    // Mach de sortie > 1 (supersonique)
    checkBool("exit supersonic", exitMach > 1.0f, true);

    // Mach au col = 1.0 (par fieldAt)
    float machThroat = nozzle.fieldAt("mach", {0, 0, physCtx.parameters["z_throat"]});
    check("mach@throat", machThroat, 1.0f, 0.1f);

    // Pression décroît de l'entrée à la sortie
    float pInlet = nozzle.fieldAt("pressure", {0, 0, 0});
    float pExit = nozzle.fieldAt("pressure", {0, 0, physCtx.parameters["L_nozzle"]});
    checkBool("P_inlet > P_exit", pInlet > pExit, true);

    // Vitesse croît de l'entrée à la sortie
    float vInlet = nozzle.fieldAt("velocity", {0, 0, 0});
    float vExit = nozzle.fieldAt("velocity", {0, 0, physCtx.parameters["L_nozzle"]});
    checkBool("V_exit > V_inlet", vExit > vInlet, true);

    // fieldAt pour grandeurs globales
    check("fieldAt thrust", nozzle.fieldAt("thrust", {0, 0, 0}), thrust);
    check("fieldAt isp", nozzle.fieldAt("isp", {0, 0, 0}), isp);

    // ── PressureVessel ──
    std::cout << "--- PressureVessel ---" << std::endl;

    // Brancher le fieldEvaluator sur le module isentropique
    physCtx.fieldEvaluator = [&nozzle](const std::string &field,
                                        simd::float3 pt) -> float {
      return nozzle.fieldAt(field, pt);
    };

    PressureVessel vessel;
    vessel.setup(pd, physCtx);
    vessel.solve(physCtx);

    float maxThk = vessel.maxThickness();
    std::cout << "  Max wall thickness = " << maxThk * 1000.0f << " mm"
              << std::endl;

    // Épaisseur doit être positive et raisonnable (0.1mm - 50mm)
    checkBool("thickness > 0", maxThk > 0.0001f, true);
    checkBool("thickness < 0.05", maxThk < 0.05f, true);

    // Von Mises doit être sous la limite
    float vmMax = vessel.fieldAt("von_mises_max", {0, 0, 0});
    float allowable = physCtx.materialProperties["wall.yield_strength"] /
                      physCtx.materialProperties["wall.safety_factor"];
    std::cout << "  VM max   = " << vmMax / 1e6f << " MPa" << std::endl;
    std::cout << "  Allowable= " << allowable / 1e6f << " MPa" << std::endl;
    // VM devrait être proche de l'allowable (le dimensionnement est juste)
    checkBool("VM reasonable", vmMax > 0 && vmMax < allowable * 1.5f, true);

    std::cout << "\n  Phase 5 : PhysicsModules OK" << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "\n  ECHEC Phase 5 : " << e.what() << std::endl;
    failed++;
  }

  // ═══════════════════════════════════════════════════════════
  // Phase 6 — FieldConstraintEvaluator
  // ═══════════════════════════════════════════════════════════
  std::cout << "\n=== PHASE 6 : FieldConstraintEvaluator ===" << std::endl;

  try {
    auto pd = ProblemParser::parseFile("examples/nozzle_delaval.json");
    ParameterSpace ps(pd);

    // Construire le contexte complet
    ExpressionEvaluator::Context fCtx;
    ps.updateContext(fCtx);
    for (const auto &[matName, mat] : pd.materials)
      for (const auto &[propName, propVal] : mat.properties)
        fCtx.materialProperties[matName + "." + propName] = propVal;

    // Résoudre les named_points qui sont des constantes
    for (const auto &[name, np] : pd.namedPoints) {
      try {
        float x = ExpressionEvaluator::resolve(np.position[0], fCtx);
        float y = ExpressionEvaluator::resolve(np.position[1], fCtx);
        float z = ExpressionEvaluator::resolve(np.position[2], fCtx);
        fCtx.namedPoints[name] = simd::float3{x, y, z};
      } catch (...) {}
    }

    // Brancher le module physique
    IsentropicNozzle nozzle;
    nozzle.setup(pd, fCtx);
    nozzle.solve(fCtx);
    fCtx.fieldEvaluator = [&nozzle](const std::string &field,
                                     simd::float3 pt) -> float {
      return nozzle.fieldAt(field, pt);
    };

    // ── Test 1 : monotonic_mach du JSON (doit être satisfait) ──
    std::cout << "--- monotonic_mach (from JSON) ---" << std::endl;
    auto violations = FieldConstraintEvaluator::evaluate(
        pd.fieldConstraints, fCtx);
    checkInt("num violations", (int)violations.size(), 1);
    if (!violations.empty()) {
      checkStr("fc0 id", violations[0].constraintId, "monotonic_mach");
      checkBool("fc0 satisfied", violations[0].satisfied, true);
      checkInt("fc0 samples", violations[0].numSamples, 30);
      std::cout << "  maxViolation = " << violations[0].maxViolation
                << "  numViolated = " << violations[0].numViolated << std::endl;
    }

    // ── Test 2 : contrainte ponctuelle ──
    std::cout << "--- pointwise constraint ---" << std::endl;
    FieldConstraint pointFC;
    pointFC.id = "test_point";
    pointFC.coordinateSystem = "axisymmetric";
    pointFC.domain.type = "point";
    pointFC.domain.pointLocation = "THROAT";
    pointFC.lhs = "field:mach@THROAT";
    pointFC.op = "=";
    pointFC.rhs = "1.0";
    pointFC.tolerance = 0.1f;

    auto pointV = FieldConstraintEvaluator::evaluateOne(pointFC, fCtx);
    checkBool("point satisfied", pointV.satisfied, true);
    check("point maxViol", pointV.maxViolation, 0.0f, 0.15f);

    // ── Test 3 : contrainte 1D qui doit échouer ──
    // "la pression doit être constante le long de l'axe" — FAUX
    std::cout << "--- failing 1d constraint ---" << std::endl;
    FieldConstraint failFC;
    failFC.id = "test_fail";
    failFC.coordinateSystem = "axisymmetric";
    failFC.domain.type = "parametric_1d";
    failFC.domain.variables = {"z"};
    failFC.domain.ranges["z"] = {"0", "param:L_nozzle"};
    failFC.domain.samples = {20};
    failFC.lhs = "field:pressure@[0, z]";
    failFC.op = "=";
    failFC.rhs = "7e6"; // P0 = 7 MPa partout ? Non, ça décroît.
    failFC.tolerance = 0.01f; // 1% de tolérance

    auto failV = FieldConstraintEvaluator::evaluateOne(failFC, fCtx);
    checkBool("fail not satisfied", failV.satisfied, false);
    checkBool("fail has violations", failV.numViolated > 0, true);
    std::cout << "  maxViolation = " << failV.maxViolation / 1e6f << " MPa"
              << "  numViolated = " << failV.numViolated << "/" << failV.numSamples
              << std::endl;

    // ── Test 4 : contrainte 1D avec >= (Mach doit être > 0 partout) ──
    std::cout << "--- mach > 0 everywhere ---" << std::endl;
    FieldConstraint machPosFC;
    machPosFC.id = "test_mach_pos";
    machPosFC.coordinateSystem = "axisymmetric";
    machPosFC.domain.type = "parametric_1d";
    machPosFC.domain.variables = {"z"};
    machPosFC.domain.ranges["z"] = {"0", "param:L_nozzle"};
    machPosFC.domain.samples = {20};
    machPosFC.lhs = "field:mach@[0, z]";
    machPosFC.op = ">=";
    machPosFC.rhs = "0";
    machPosFC.tolerance = 0.0f;

    auto machPosV = FieldConstraintEvaluator::evaluateOne(machPosFC, fCtx);
    checkBool("mach>0 satisfied", machPosV.satisfied, true);

    std::cout << "\n  Phase 6 : FieldConstraintEvaluator OK" << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "\n  ECHEC Phase 6 : " << e.what() << std::endl;
    failed++;
  }

  // ═══════════════════════════════════════════════════════════
  // Phase 7 — CostEvaluator + Optimizer
  // ═══════════════════════════════════════════════════════════
  std::cout << "\n=== PHASE 7 : CostEvaluator + Optimizer ===" << std::endl;

  try {
    // ── Test Optimizer Nelder-Mead sur Rosenbrock ──
    // min f(x,y) = (1-x)² + 100(y-x²)²
    // Solution : (1, 1) avec f=0
    std::cout << "--- Nelder-Mead (Rosenbrock) ---" << std::endl;
    auto rosenbrock = [](const std::vector<float> &p) -> float {
      float x = p[0], y = p[1];
      return (1 - x) * (1 - x) + 100 * (y - x * x) * (y - x * x);
    };

    auto optResult = Optimizer::nelderMead(
        rosenbrock, {-1.0f, -1.0f}, {-5.0f, -5.0f}, {5.0f, 5.0f}, 1000,
        1e-8f);

    std::cout << "  x=" << optResult.bestParams[0]
              << " y=" << optResult.bestParams[1]
              << " cost=" << optResult.bestCost
              << " iter=" << optResult.iterations << std::endl;

    check("rosenbrock x", optResult.bestParams[0], 1.0f, 0.05f);
    check("rosenbrock y", optResult.bestParams[1], 1.0f, 0.05f);
    checkBool("rosenbrock converged", optResult.bestCost < 0.01f, true);

    // ── Test CostEvaluator ──
    std::cout << "--- CostEvaluator ---" << std::endl;
    auto pd = ProblemParser::parseFile("examples/nozzle_delaval.json");
    ParameterSpace ps(pd);
    ExpressionEvaluator::Context costCtx;
    ps.updateContext(costCtx);
    for (const auto &[matName, mat] : pd.materials)
      for (const auto &[propName, propVal] : mat.properties)
        costCtx.materialProperties[matName + "." + propName] = propVal;
    for (const auto &[name, param] : pd.parameters)
      if (param.type == "fixed")
        costCtx.parameters[name] = param.value;

    IsentropicNozzle nozzleC;
    nozzleC.setup(pd, costCtx);
    nozzleC.solve(costCtx);
    costCtx.fieldEvaluator = [&nozzleC](const std::string &f,
                                         simd::float3 p) -> float {
      return nozzleC.fieldAt(f, p);
    };

    auto fv = FieldConstraintEvaluator::evaluate(pd.fieldConstraints, costCtx);
    auto cb = CostEvaluator::evaluate(pd, costCtx, fv);

    std::cout << "  Total cost = " << cb.total << std::endl;
    for (const auto &[id, val] : cb.terms)
      std::cout << "    " << id << " = " << val << std::endl;
    checkBool("cost > 0", cb.total >= 0, true);
    checkBool("has terms", cb.terms.size() > 0, true);

    std::cout << "\n  Phase 7 : CostEvaluator + Optimizer OK" << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "\n  ECHEC Phase 7 : " << e.what() << std::endl;
    failed++;
  }

  // ═══════════════════════════════════════════════════════════
  // Phase 8 — Full Solver Pipeline
  // ═══════════════════════════════════════════════════════════
  std::cout << "\n=== PHASE 8 : Solver Pipeline ===" << std::endl;

  try {
    auto solverResult = Solver::solve("examples/nozzle_delaval.json",
                                       "test_geometry.json");

    std::cout << "--- Solver result ---" << std::endl;
    std::cout << "  Converged  : " << solverResult.converged << std::endl;
    std::cout << "  Iterations : " << solverResult.iterations << std::endl;
    std::cout << "  Final cost : " << solverResult.finalCost << std::endl;

    checkBool("solver ran", solverResult.iterations > 0, true);
    checkBool("cost finite", std::isfinite(solverResult.finalCost), true);

    // Vérifier que le fichier geometry.json a été écrit
    std::ifstream geoFile("test_geometry.json");
    checkBool("geometry file exists", geoFile.good(), true);
    if (geoFile.good()) {
      auto geoJson = nlohmann::json::parse(geoFile);
      checkBool("has type", geoJson.contains("type"), true);
      checkBool("has metadata", geoJson.contains("metadata"), true);
      if (geoJson.contains("metadata")) {
        checkBool("has thrust", geoJson["metadata"].contains("thrust"), true);
        checkBool("has isp", geoJson["metadata"].contains("specific_impulse"), true);
        float thrust = geoJson["metadata"]["thrust"];
        float isp = geoJson["metadata"]["specific_impulse"];
        std::cout << "  Output thrust : " << thrust / 1000.0f << " kN" << std::endl;
        std::cout << "  Output Isp    : " << isp << " s" << std::endl;
        checkBool("thrust > 0", thrust > 0, true);
        checkBool("isp > 0", isp > 0, true);
      }
    }

    std::cout << "\n  Phase 8 : Solver Pipeline OK" << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "\n  ECHEC Phase 8 : " << e.what() << std::endl;
    failed++;
  }

  std::cout << "\n══════════════════════════════════════" << std::endl;
  std::cout << "  " << passed << " passés, " << failed << " échoués" << std::endl;
  std::cout << "══════════════════════════════════════" << std::endl;
  if (failed > 0) std::cerr << "  DES TESTS ONT ECHOUE !" << std::endl;
  else std::cout << "  Tous les tests passent." << std::endl;
}
