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

  z_throat = std::clamp(z_throat, L_nozzle * 0.1f, L_nozzle * 0.6f);

  float r_inlet = std::max(r_exit, r_throat) * 1.3f;

  float wall_real = get("wall_thickness", 0.005f);
  float wall = std::max(wall_real, r_throat * 0.15f);

  float yOffset = L_nozzle * 0.5f;

  json internalPts = json::array();
  json externalPts = json::array();
  float maxR = 0.0f;

  float throatFrac = (L_nozzle > 1e-10f) ? z_throat / L_nozzle : 0.4f;

  std::vector<float> fracs;
  int nConv = 5;
  int nDiv = 7;
  for (int i = 0; i <= nConv; i++)
    fracs.push_back(throatFrac * (float)i / nConv);
  for (int i = 1; i <= nDiv; i++)
    fracs.push_back(throatFrac + (1.0f - throatFrac) * (float)i / nDiv);

  for (float frac : fracs) {
    float yAxial = frac * L_nozzle;
    float r;

    if (yAxial <= z_throat) {
      float t = (z_throat > 1e-10f) ? yAxial / z_throat : 0.0f;
      float smooth_t = 0.5f * (1.0f - std::cos(t * M_PI));
      r = r_inlet + smooth_t * (r_throat - r_inlet);
    } else {
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