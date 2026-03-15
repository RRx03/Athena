#include "GeometryBuilder.hpp"
#include <cmath>
#include <vector>

using json = nlohmann::json;

json GeometryBuilder::build(const ProblemDefinition &problem,
                            const ExpressionEvaluator::Context &ctx) {

  auto get = [&](const std::string &name, float def) -> float {
    auto it = ctx.parameters.find(name);
    return (it != ctx.parameters.end()) ? it->second : def;
  };

  float r_throat = get("r_throat", 0.015f);
  float r_exit = get("r_exit", 0.035f);
  float L_nozzle = get("L_nozzle", 0.3f);
  float z_throat = get("z_throat", 0.12f);
  float r_inlet = get("r_inlet", r_exit * 1.2f);

  // Épaisseur de paroi (dérivée ou fixe)
  float wall = get("wall_thickness", 0.005f);

  // Collecter les profile_r
  std::vector<float> profileR;
  for (int i = 0; i < 100; i++) {
    auto it = ctx.parameters.find("profile_r[" + std::to_string(i) + "]");
    if (it == ctx.parameters.end())
      break;
    profileR.push_back(it->second);
  }

  // ── Construire les points du profil interne ──
  json internalPts = json::array();
  json externalPts = json::array();

  if (profileR.empty() ||
      (*std::max_element(profileR.begin(), profileR.end()) -
       *std::min_element(profileR.begin(), profileR.end())) < 0.001f) {
    // Profil conique
    int N = 9; // Nombre de points sur le profil
    for (int i = 0; i <= N; i++) {
      float t = (float)i / N;
      float z = t * L_nozzle;
      float r;
      if (z <= z_throat) {
        float frac = (z_throat > 1e-10f) ? z / z_throat : 0.0f;
        r = r_inlet + frac * (r_throat - r_inlet);
      } else {
        float denom = L_nozzle - z_throat;
        float frac = (denom > 1e-10f) ? (z - z_throat) / denom : 0.0f;
        r = r_throat + frac * (r_exit - r_throat);
      }
      internalPts.push_back({r, z});
      externalPts.push_back({r + wall, z});
    }
  } else {
    // Profil depuis les points de contrôle
    for (int i = 0; i < (int)profileR.size(); i++) {
      float t = (float)i / (profileR.size() - 1);
      float z = t * L_nozzle;
      internalPts.push_back({profileR[i], z});
      externalPts.push_back({profileR[i] + wall, z});
    }
  }

  float maxR = r_inlet + wall + 0.01f;
  float halfL = L_nozzle * 0.5f;

  // ── Arbre CSG ──
  // Intersect(Subtract(ext_spline, int_spline), Box)
  json tree = {
      {"type", "Intersect"},
      {"left",
       {{"type", "Subtract"},
        {"base",
         {{"type", "CompositeSpline2D"},
          {"role", "external_wall"},
          {"points", externalPts},
          {"thickness", wall * 0.5f}}},
        {"subtract",
         {{"type", "CompositeSpline2D"},
          {"role", "internal_wall"},
          {"points", internalPts},
          {"thickness", wall * 0.5f}}}}},
      {"right",
       {{"type", "Box"},
        {"position", {0, 0, halfL}},
        {"bounds", {maxR, maxR, halfL + 0.01f}}}}};

  return tree;
}
