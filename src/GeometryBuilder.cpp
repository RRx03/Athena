#include "GeometryBuilder.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

using json = nlohmann::json;

// ═══════════════════════════════════════════════════════════════
// GeometryBuilder — Construit le CSG depuis les paramètres optimisés
//
// IMPORTANT : on utilise TOUJOURS le profil conique défini par
// r_throat, r_exit, z_throat, L_nozzle. Ce sont les paramètres
// que le solveur optimise réellement (via IsentropicNozzle).
//
// Les profile_r[] sont ignorés pour l'instant car ils ne sont
// pas contraints en forme (pas de continuité, pas de monotonie
// dans le convergent/divergent). Ils seront utilisés quand on
// ajoutera des contraintes de lissage B-spline dans le solveur.
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

  // Rayon d'entrée : plus grand que le col et la sortie
  float r_inlet = std::max(r_exit, r_throat) * 1.3f;

  // Épaisseur de paroi — minimum visuel pour le rendu
  float wall_real = get("wall_thickness", 0.005f);
  float wall =
      std::max(wall_real, r_throat * 0.15f); // Au moins 15% du rayon du col

  // Convention kernel : Y = axe de révolution
  float yOffset = L_nozzle * 0.5f;

  // ── Construire le profil convergent-divergent ──
  // Plus de points dans les zones de courbure (col)
  json internalPts = json::array();
  json externalPts = json::array();
  float maxR = 0.0f;

  // Points stratégiques : entrée, convergent, col, divergent, sortie
  struct ProfPt {
    float frac;
  }; // fraction de L_nozzle
  float throatFrac = (L_nozzle > 1e-10f) ? z_throat / L_nozzle : 0.4f;

  // Répartir les points avec plus de densité autour du col
  std::vector<float> fracs;
  int nConv = 5; // Points dans le convergent
  int nDiv = 7;  // Points dans le divergent
  for (int i = 0; i <= nConv; i++)
    fracs.push_back(throatFrac * (float)i / nConv);
  for (int i = 1; i <= nDiv; i++)
    fracs.push_back(throatFrac + (1.0f - throatFrac) * (float)i / nDiv);

  for (float frac : fracs) {
    float yAxial = frac * L_nozzle;
    float r;

    if (yAxial <= z_throat) {
      // Convergent : profil lisse (cosinus pour un col arrondi)
      float t = (z_throat > 1e-10f) ? yAxial / z_throat : 0.0f;
      // Interpolation cosinus pour un col lisse
      float smooth_t = 0.5f * (1.0f - std::cos(t * M_PI));
      r = r_inlet + smooth_t * (r_throat - r_inlet);
    } else {
      // Divergent : profil lisse
      float denom = L_nozzle - z_throat;
      float t = (denom > 1e-10f) ? (yAxial - z_throat) / denom : 0.0f;
      float smooth_t = 0.5f * (1.0f - std::cos(t * M_PI));
      r = r_throat + smooth_t * (r_exit - r_throat);
    }

    r = std::max(r, 0.001f);
    float y = yAxial - yOffset;

    internalPts.push_back({r, y});
    externalPts.push_back({r + wall, y});
    maxR = std::max(maxR, r + wall);
  }

  float halfY = L_nozzle * 0.5f + 0.002f;
  maxR += 0.005f;

  // ── Arbre CSG ──
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