#include "ExpressionEvaluator.hpp"
#include "ProblemParser.hpp"
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

static void checkStr(const std::string &name, const std::string &got,
                     const std::string &expected) {
  if (got == expected) {
    passed++;
  } else {
    std::cerr << "  ECHEC: " << name << " — attendu '" << expected
              << "' obtenu '" << got << "'" << std::endl;
    failed++;
  }
}

static void checkBool(const std::string &name, bool got, bool expected) {
  if (got == expected) {
    passed++;
  } else {
    std::cerr << "  ECHEC: " << name << " — attendu " << expected << " obtenu "
              << got << std::endl;
    failed++;
  }
}

static void checkInt(const std::string &name, int got, int expected) {
  if (got == expected) {
    passed++;
  } else {
    std::cerr << "  ECHEC: " << name << " — attendu " << expected << " obtenu "
              << got << std::endl;
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
    checkStr("ENDTUYERE pos[2]", pd.namedPoints["ENDTUYERE"].position[2],
             "param:L_nozzle");
    checkStr("THROAT type", pd.namedPoints["THROAT"].type, "variable");

    std::cout << "--- Boundary Conditions ---" << std::endl;
    checkInt("num BCs", (int)pd.boundaryConditions.size(), 2);
    checkStr("BC0 id", pd.boundaryConditions[0].id, "inlet");
    checkStr("BC0 geom type", pd.boundaryConditions[0].geometryType, "Disk2D");
    checkStr("BC0 radius", pd.boundaryConditions[0].geometryParams["radius"],
             "0.04");
    checkStr("BC0 P type",
             pd.boundaryConditions[0].distributions["total_pressure"].type,
             "uniform");
    checkStr(
        "BC0 P value",
        pd.boundaryConditions[0].distributions["total_pressure"].expression,
        "7e6");
    checkStr("BC1 id", pd.boundaryConditions[1].id, "outlet");
    checkStr("BC1 radius", pd.boundaryConditions[1].geometryParams["radius"],
             "param:r_exit");
    checkStr("BC1 anchor", pd.boundaryConditions[1].anchors["concentric_with"],
             "inlet");

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
    checkStr("FC0 coord", pd.fieldConstraints[0].coordinateSystem,
             "axisymmetric");
    checkStr("FC0 domain type", pd.fieldConstraints[0].domain.type,
             "parametric_1d");
    checkInt("FC0 domain vars",
             (int)pd.fieldConstraints[0].domain.variables.size(), 1);
    checkStr("FC0 domain var[0]", pd.fieldConstraints[0].domain.variables[0],
             "z");
    checkStr("FC0 lhs", pd.fieldConstraints[0].lhs,
             "field:mach@[0, z + 0.001]");
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
    checkStr("wall expr", pd.parameters["wall_thickness"].expression,
             "pressure_vessel");
    checkStr("profile type", pd.parameters["profile_r"].type, "variable_array");
    checkInt("profile size", pd.parameters["profile_r"].arraySize, 7);

    std::cout << "--- Optimization ---" << std::endl;
    checkStr("opt mode", pd.optimization.mode, "dynamic");
    checkStr("opt method", pd.optimization.method, "nelder_mead");
    checkStr("opt precision", pd.optimization.precision, "standard");
    checkInt("fidelity levels", (int)pd.optimization.fidelityLevels.size(), 2);
    checkInt("level0 samples", pd.optimization.fidelityLevels[0].samples, 10);
    checkInt("level0 maxiter", pd.optimization.fidelityLevels[0].maxIterations,
             100);
    checkStr("level0 physics", pd.optimization.fidelityLevels[0].physicsModel,
             "isentropic_1d");
    checkInt("level1 samples", pd.optimization.fidelityLevels[1].samples, 50);

    std::cout << "\n  Phase 2 : ProblemParser OK" << std::endl;
  } catch (const std::exception &e) {
    std::cerr << "\n  ECHEC Phase 2 : " << e.what() << std::endl;
    failed++;
  }

  std::cout << "\n══════════════════════════════════════" << std::endl;
  std::cout << "  " << passed << " passés, " << failed << " échoués" << std::endl;
  std::cout << "══════════════════════════════════════" << std::endl;
  if (failed > 0) std::cerr << "  DES TESTS ONT ECHOUE !" << std::endl;
  else std::cout << "  Tous les tests passent." << std::endl;
}
