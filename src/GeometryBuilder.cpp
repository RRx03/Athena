#include "GeometryBuilder.hpp"
#include <algorithm>
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
  float wall = get("wall_thickness", 0.005f);
  wall = std::max(wall, 0.0005f);

  // Collecter les profile_r
  std::vector<float> profileR;
  for (int i = 0; i < 100; i++) {
    auto it = ctx.parameters.find("profile_r[" + std::to_string(i) + "]");
    if (it == ctx.parameters.end())
      break;
    profileR.push_back(it->second);
  }

  // Détecter si profileR est un vrai profil optimisé ou juste l'init par défaut
  bool useProfileR = false;
  if (profileR.size() >= 3) {
    float minR = *std::min_element(profileR.begin(), profileR.end());
    float maxR = *std::max_element(profileR.begin(), profileR.end());
    float avgR = (minR + maxR) * 0.5f;
    useProfileR = (maxR - minR) > 0.01f * avgR;
  }

  // ── Construire les points [r, y] ──
  // Convention kernel : Y = axe de révolution, r = distance radiale
  // Les points sont centrés sur y=0
  float yOffset = L_nozzle * 0.5f;
  json internalPts = json::array();
  json externalPts = json::array();
  float maxR = 0.0f;

  int N = 12; // Nombre de points sur le profil

  for (int i = 0; i <= N; i++) {
    float t = (float)i / N;
    float yAxial = t * L_nozzle;
    float r;

    if (useProfileR) {
      float idx_f = t * (profileR.size() - 1);
      int idx_lo = std::clamp((int)idx_f, 0, (int)profileR.size() - 2);
      float frac = idx_f - idx_lo;
      r = profileR[idx_lo] * (1.0f - frac) + profileR[idx_lo + 1] * frac;
    } else {
      // Profil conique convergent-divergent
      if (yAxial <= z_throat) {
        float frac = (z_throat > 1e-10f) ? yAxial / z_throat : 0.0f;
        r = r_inlet + frac * (r_throat - r_inlet);
      } else {
        float denom = L_nozzle - z_throat;
        float frac = (denom > 1e-10f) ? (yAxial - z_throat) / denom : 0.0f;
        r = r_throat + frac * (r_exit - r_throat);
      }
    }

    r = std::max(r, 0.001f);
    float y = yAxial - yOffset;

    internalPts.push_back({r, y});
    externalPts.push_back({r + wall, y});
    maxR = std::max(maxR, r + wall);
  }

  float halfY = L_nozzle * 0.5f + 0.001f;
  maxR += 0.002f;

  // ── Arbre CSG ──
  // Subtract(demi-plan_externe, demi-plan_interne) intersecté par Box
  json tree = {{"type", "Intersect"},
               {"left",
                {{"type", "Subtract"},
                 {"base",
                  {{"type", "CompositeSpline2D"},
                   {"role", "external_wall"},
                   {"points", externalPts},
                   {"thickness", 0.0f}}},
                 {"subtract",
                  {{"type", "CompositeSpline2D"},
                   {"role", "internal_wall"},
                   {"points", internalPts},
                   {"thickness", 0.0f}}}}},
               {"right",
                {{"type", "Box"},
                 {"position", {0, 0, 0}},
                 {"bounds", {maxR, halfY, maxR}}}}};

  return tree;
}