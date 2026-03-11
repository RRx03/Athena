#include "ProblemParser.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>

using json = nlohmann::json;

// ═══════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════

// Convertit une valeur JSON (string, int, float) en string d'expression.
// Ex: 0.035 → "0.035", "param:r_exit" → "param:r_exit"
std::string ProblemParser::toExprString(const json &val) {
  if (val.is_string())
    return val.get<std::string>();
  if (val.is_number_float())
    return std::to_string(val.get<float>());
  if (val.is_number_integer())
    return std::to_string(val.get<int>());
  throw std::runtime_error("ProblemParser: valeur JSON ni string ni nombre");
}

// Convertit une valeur JSON en float, accepte string ou nombre.
static float toFloat(const json &val) {
  if (val.is_number())
    return val.get<float>();
  if (val.is_string())
    return std::stof(val.get<std::string>());
  throw std::runtime_error("ProblemParser: impossible de convertir en float");
}

// ═══════════════════════════════════════════════════════════════
// Point d'entrée
// ═══════════════════════════════════════════════════════════════

ProblemDefinition ProblemParser::parseFile(const std::string &filepath) {
  std::ifstream f(filepath);
  if (!f.is_open())
    throw std::runtime_error("ProblemParser: impossible d'ouvrir '" +
                             filepath + "'");
  json root;
  try {
    root = json::parse(f);
  } catch (const json::parse_error &e) {
    throw std::runtime_error("ProblemParser: JSON invalide dans '" +
                             filepath + "': " + e.what());
  }
  return parse(root);
}

ProblemDefinition ProblemParser::parse(const json &root) {
  ProblemDefinition pd;

  // ── Meta ──
  pd.problemType = root.value("problem", "unknown");
  pd.units = root.value("units", "SI");

  // ── Frame ──
  if (root.contains("frame"))
    pd.frame = parseFrame(root["frame"]);
  else
    pd.frame.mode = "auto";

  // ── Materials ──
  if (root.contains("materials")) {
    for (auto &[key, val] : root["materials"].items()) {
      pd.materials[key] = parseMaterial(key, val);
    }
  }

  // ── Named Points ──
  if (root.contains("named_points")) {
    for (auto &[key, val] : root["named_points"].items()) {
      pd.namedPoints[key] = parseNamedPoint(key, val);
    }
  }

  // ── Boundary Conditions ──
  if (root.contains("boundary_conditions")) {
    for (const auto &bc : root["boundary_conditions"]) {
      pd.boundaryConditions.push_back(parseBC(bc));
    }
  }

  // ── Constraints ──
  if (root.contains("constraints")) {
    for (const auto &c : root["constraints"]) {
      pd.constraints.push_back(parseConstraint(c));
    }
  }

  // ── Field Constraints ──
  if (root.contains("field_constraints")) {
    for (const auto &fc : root["field_constraints"]) {
      pd.fieldConstraints.push_back(parseFieldConstraint(fc));
    }
  }

  // ── Parameters ──
  if (root.contains("parameters")) {
    for (auto &[key, val] : root["parameters"].items()) {
      pd.parameters[key] = parseParameter(key, val);
    }
  }

  // ── Optimization ──
  if (root.contains("optimization"))
    pd.optimization = parseOptimization(root["optimization"]);

  return pd;
}

// ═══════════════════════════════════════════════════════════════
// Sous-parsers
// ═══════════════════════════════════════════════════════════════

FrameConfig ProblemParser::parseFrame(const json &j) {
  FrameConfig fc;
  fc.mode = j.value("mode", "auto");
  if (fc.mode == "manual") {
    if (j.contains("origin")) {
      fc.origin = {j["origin"][0], j["origin"][1], j["origin"][2]};
    }
    if (j.contains("axis")) {
      fc.axis = {j["axis"][0], j["axis"][1], j["axis"][2]};
    }
    if (j.contains("up")) {
      fc.up = {j["up"][0], j["up"][1], j["up"][2]};
    }
  }
  return fc;
}

MaterialProperties ProblemParser::parseMaterial(const std::string &name,
                                                const json &j) {
  MaterialProperties mat;
  mat.name = j.value("name", name);
  if (j.contains("properties")) {
    for (auto &[key, val] : j["properties"].items()) {
      mat.properties[key] = toFloat(val);
    }
  }
  return mat;
}

NamedPoint ProblemParser::parseNamedPoint(const std::string &name,
                                          const json &j) {
  NamedPoint np;
  np.name = name;
  np.type = j.value("type", "fixed");
  np.domainVariable = j.value("domain_variable", "");

  if (j.contains("position")) {
    const auto &pos = j["position"];
    np.position[0] = toExprString(pos[0]);
    np.position[1] = toExprString(pos[1]);
    np.position[2] = toExprString(pos[2]);
  }
  return np;
}

BoundaryCondition ProblemParser::parseBC(const json &j) {
  BoundaryCondition bc;
  bc.id = j.value("id", "");
  bc.description = j.value("description", "");

  // Geometry
  if (j.contains("geometry")) {
    const auto &geo = j["geometry"];
    bc.geometryType = geo.value("type", "");
    // Stocker tous les champs de la géométrie comme des expressions string
    for (auto &[key, val] : geo.items()) {
      if (key == "type")
        continue;
      bc.geometryParams[key] = toExprString(val);
    }
  }

  // Distributions
  if (j.contains("distributions")) {
    for (auto &[key, val] : j["distributions"].items()) {
      bc.distributions[key] = parseDistribution(val);
    }
  }

  // Anchors
  if (j.contains("anchors")) {
    for (auto &[key, val] : j["anchors"].items()) {
      bc.anchors[key] = toExprString(val);
    }
  }

  return bc;
}

Distribution ProblemParser::parseDistribution(const json &j) {
  Distribution dist;
  dist.type = j.value("type", "uniform");
  dist.coordinateSystem = j.value("coordinate_system", "");
  dist.symmetry = j.value("symmetry", "");

  if (j.contains("value"))
    dist.expression = toExprString(j["value"]);
  if (j.contains("expression"))
    dist.expression = j["expression"].get<std::string>();
  if (j.contains("tolerance"))
    dist.tolerance = toFloat(j["tolerance"]);

  if (j.contains("variables")) {
    for (const auto &v : j["variables"])
      dist.variables.push_back(v.get<std::string>());
  }

  return dist;
}

Constraint ProblemParser::parseConstraint(const json &j) {
  Constraint c;
  c.id = j.value("id", "");
  c.type = j.value("type", "");
  c.quantity = j.value("quantity", "");
  c.op = j.value("op", "");
  if (j.contains("value"))
    c.value = toExprString(j["value"]);
  c.priority = j.value("priority", 1);
  return c;
}

FieldConstraint ProblemParser::parseFieldConstraint(const json &j) {
  FieldConstraint fc;
  fc.id = j.value("id", "");
  fc.description = j.value("description", "");
  fc.coordinateSystem = j.value("coordinate_system", "axisymmetric");
  fc.tolerance = j.value("tolerance", 0.0f);
  fc.priority = j.value("priority", 1);

  // Domain
  if (j.contains("domain"))
    fc.domain = parseDomain(j["domain"]);

  // Relation
  if (j.contains("relation")) {
    const auto &rel = j["relation"];
    if (rel.is_object()) {
      // Structured format: { "lhs": ..., "op": ..., "rhs": ... }
      fc.lhs = rel.value("lhs", "");
      fc.op = rel.value("op", "=");
      fc.rhs = rel.value("rhs", "");
    } else if (rel.is_string()) {
      // String format: "lhs = rhs" — parse manuellement
      std::string s = rel.get<std::string>();
      // Chercher l'opérateur
      for (const auto &op : {">=", "<=", "="}) {
        auto pos = s.find(op);
        if (pos != std::string::npos) {
          fc.lhs = s.substr(0, pos);
          fc.op = op;
          fc.rhs = s.substr(pos + std::string(op).size());
          // Trim
          while (!fc.lhs.empty() && fc.lhs.back() == ' ')
            fc.lhs.pop_back();
          while (!fc.rhs.empty() && fc.rhs.front() == ' ')
            fc.rhs.erase(fc.rhs.begin());
          break;
        }
      }
    }
  }

  return fc;
}

DomainSpec ProblemParser::parseDomain(const json &j) {
  DomainSpec ds;
  ds.type = j.value("type", "point");

  // Variables
  if (j.contains("variables")) {
    for (const auto &v : j["variables"])
      ds.variables.push_back(v.get<std::string>());
  }

  // Ranges : { "z": [lo, hi], "r": [lo, hi] }
  if (j.contains("ranges")) {
    for (auto &[key, val] : j["ranges"].items()) {
      ds.ranges[key] = {toExprString(val[0]), toExprString(val[1])};
    }
  }

  // Samples
  if (j.contains("samples")) {
    if (j["samples"].is_array()) {
      for (const auto &s : j["samples"])
        ds.samples.push_back(s.get<int>());
    } else {
      ds.samples.push_back(j["samples"].get<int>());
    }
  }

  // Surface type
  ds.surfaceName = j.value("surface", "");
  ds.pointLocation = j.value("location", "");

  return ds;
}

Parameter ProblemParser::parseParameter(const std::string &name,
                                        const json &j) {
  Parameter p;
  p.name = name;
  p.type = j.value("type", "fixed");

  if (j.contains("value"))
    p.value = toFloat(j["value"]);

  if (j.contains("bounds")) {
    p.lo = toFloat(j["bounds"][0]);
    p.hi = toFloat(j["bounds"][1]);
  }

  if (j.contains("expression"))
    p.expression = j["expression"].get<std::string>();

  if (j.contains("size"))
    p.arraySize = j["size"].get<int>();

  // Valeurs initiales pour les arrays
  if (j.contains("initial_values")) {
    for (const auto &v : j["initial_values"])
      p.arrayValues.push_back(toFloat(v));
  }

  return p;
}

OptimizationConfig ProblemParser::parseOptimization(const json &j) {
  OptimizationConfig opt;
  opt.mode = j.value("mode", "fixed");
  opt.method = j.value("method", "nelder_mead");
  opt.precision = j.value("precision", "standard");

  // Fixed mode
  if (j.contains("fixed_params")) {
    opt.fixedMaxIterations =
        j["fixed_params"].value("max_iterations", 500);
    opt.fixedSampleSize =
        j["fixed_params"].value("sample_size", 100);
  }

  // Dynamic mode
  if (j.contains("dynamic_params") &&
      j["dynamic_params"].contains("fidelity_levels")) {
    for (const auto &lvl : j["dynamic_params"]["fidelity_levels"]) {
      OptimizationConfig::FidelityLevel fl;
      fl.samples = lvl.value("samples", 50);
      fl.maxIterations = lvl.value("max_iterations", 100);
      fl.physicsModel = lvl.value("physics", "isentropic_1d");
      opt.fidelityLevels.push_back(fl);
    }
  }

  // Convergence mode
  if (j.contains("convergence_params")) {
    const auto &cp = j["convergence_params"];
    opt.convergenceMaxIterations = cp.value("max_total_iterations", 2000);
    opt.convergenceMinSamples = cp.value("min_samples", 10);
    opt.convergenceMaxSamples = cp.value("max_samples", 500);
    opt.convergenceSampleGrowth = cp.value("sample_growth_factor", 2.0f);
    if (cp.contains("criteria_tolerances")) {
      for (auto &[key, val] : cp["criteria_tolerances"].items())
        opt.criteriaTolerances[key] = toFloat(val);
    }
  }

  return opt;
}
