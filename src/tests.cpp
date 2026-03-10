#include "ExpressionEvaluator.hpp"
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

  std::cout << "\n══════════════════════════════════════" << std::endl;
  std::cout << "  " << passed << " passés, " << failed << " échoués" << std::endl;
  std::cout << "══════════════════════════════════════" << std::endl;
  if (failed > 0) std::cerr << "  DES TESTS ONT ECHOUE !" << std::endl;
  else std::cout << "  Tous les tests passent." << std::endl;
}
