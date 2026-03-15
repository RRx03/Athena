#include "GeometryBuilder.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

using json = nlohmann::json;

// ═══════════════════════════════════════════════════════════════
// GeometryBuilder — Produit un arbre CSG pour le Geometric Kernel
//
// Convention du kernel :
//   - Y est l'axe de révolution (toutes les primitives 2D sont
//     dans le plan (r, y) où r = sqrt(x² + z²))
//   - Les points CompositeSpline2D sont [r, y]
//   - La Box est centrée avec bounds [half_x, half_y, half_z]
//
// Arbre CSG :
//   Intersect(
//     Subtract(
//       CompositeSpline2D(external, thickness=0),  → demi-plan externe
//       CompositeSpline2D(internal, thickness=0)    → demi-plan interne
//     ),
//     Box(bounding_box)
//   )
//
// Le Subtract creuse l'intérieur : tout ce qui est entre le profil
// externe et le profil interne est la paroi.
// ═══════════════════════════════════════════════════════════════

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
  wall = std::max(wall, 0.001f); // Épaisseur min visible

  // Collecter les profile_r
  std::vector<float> profileR;
  for (int i = 0; i < 100; i++) {
    auto it = ctx.parameters.find("profile_r[" + std::to_string(i) + "]");
    if (it == ctx.parameters.end())
      break;
    profileR.push_back(it->second);
  }

  bool useProfileR =
      profileR.size() >= 3 &&
      (*std::max_element(profileR.begin(), profileR.end()) -
       *std::min_element(profileR.begin(), profileR.end())) > 0.001f;

  // ── Construire les points [r, y] ──
  // y = position axiale, centrée sur y=0 (kernel convention)
  float yOffset = L_nozzle * 0.5f;
  json internalPts = json::array();
  json externalPts = json::array();

  int N = 10;
  float maxR = 0.0f;

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
      if (yAxial <= z_throat) {
        float frac = (z_throat > 1e-10f) ? yAxial / z_throat : 0.0f;
        r = r_inlet + frac * (r_throat - r_inlet);
      } else {
        float denom = L_nozzle - z_throat;
        float frac = (denom > 1e-10f) ? (yAxial - z_throat) / denom : 0.0f;
        r = r_throat + frac * (r_exit - r_throat);
      }
    }

    r = std::max(r, 0.002f);
    float y = yAxial - yOffset; // Centrer sur y=0

    internalPts.push_back({r, y});
    externalPts.push_back({r + wall, y});
    maxR = std::max(maxR, r + wall);
  }

  float halfY = L_nozzle * 0.5f + 0.002f;
  maxR += 0.005f;

  // ── Arbre CSG ──
  // Subtract(externe_demiplan, interne_demiplan) → coque
  // Intersect avec Box → couper les bords nets
  json tree = {
      {"type", "Intersect"},
      {"left",
       {{"type", "Subtract"},
        {"base",
         {{"type", "CompositeSpline2D"},
          {"role", "external_wall"},
          {"points", externalPts},
          {"thickness", 0.0f}}}, // thickness=0 → mode demi-plan signé
        {"subtract",
         {{"type", "CompositeSpline2D"},
          {"role", "internal_wall"},
          {"points", internalPts},
          {"thickness", 0.0f}}}}}, // thickness=0 → mode demi-plan signé
      {"right",
       {{"type", "Box"},
        {"position", {0, 0, 0}},            // Centré à l'origine
        {"bounds", {maxR, halfY, maxR}}}}}; // [x, y, z] — y est l'axe

  return tree;
}