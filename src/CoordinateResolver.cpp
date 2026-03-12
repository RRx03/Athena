#include "CoordinateResolver.hpp"
#include <cmath>
#include <stdexcept>

CoordinateResolver::Frame
CoordinateResolver::resolveFrame(const ProblemDefinition &problem) {
  Frame frame;

  if (problem.frame.mode == "manual") {
    // ── Mode manual : l'utilisateur a défini le repère ──
    frame.origin = {problem.frame.origin[0], problem.frame.origin[1],
                    problem.frame.origin[2]};
    frame.axis = {problem.frame.axis[0], problem.frame.axis[1],
                  problem.frame.axis[2]};
    frame.up = {problem.frame.up[0], problem.frame.up[1],
                problem.frame.up[2]};

    // Normaliser
    frame.axis = simd_normalize(frame.axis);
    frame.up = simd_normalize(frame.up);

    // Orthogonaliser up par rapport à axis (Gram-Schmidt)
    float dot = simd_dot(frame.up, frame.axis);
    frame.up = simd_normalize(frame.up - dot * frame.axis);

    // Déduire right
    frame.right = simd_cross(frame.axis, frame.up);

  } else {
    // ── Mode auto : déduire depuis les anchors ──

    // Chercher un anchor "axis_alignment" pour déterminer l'axe
    simd::float3 detectedAxis = {0, 0, 1}; // Défaut : Z
    for (const auto &bc : problem.boundaryConditions) {
      auto it = bc.anchors.find("axis_alignment");
      if (it != bc.anchors.end()) {
        if (it->second == "x")
          detectedAxis = {1, 0, 0};
        else if (it->second == "y")
          detectedAxis = {0, 1, 0};
        else if (it->second == "z")
          detectedAxis = {0, 0, 1};
        break; // Prendre le premier trouvé
      }
    }

    // Chercher l'origine : le premier point nommé de type "fixed"
    // ou le center de la première BC
    simd::float3 detectedOrigin = {0, 0, 0};
    for (const auto &[name, np] : problem.namedPoints) {
      if (np.type == "fixed") {
        // Les positions sont des expressions — on ne peut évaluer
        // que les constantes littérales ici (avant le contexte complet).
        // On essaie de parser comme float, sinon on garde (0,0,0).
        try {
          detectedOrigin.x = std::stof(np.position[0]);
          detectedOrigin.y = std::stof(np.position[1]);
          detectedOrigin.z = std::stof(np.position[2]);
        } catch (...) {
          // Position contient des expressions → sera résolue plus tard
        }
        break;
      }
    }

    frame.origin = detectedOrigin;
    frame.axis = detectedAxis;

    // Choisir un up perpendiculaire à l'axe
    if (std::abs(detectedAxis.y) < 0.9f)
      frame.up = simd_normalize(
          simd::float3{0, 1, 0} -
          simd_dot(simd::float3{0, 1, 0}, detectedAxis) * detectedAxis);
    else
      frame.up = simd_normalize(
          simd::float3{1, 0, 0} -
          simd_dot(simd::float3{1, 0, 0}, detectedAxis) * detectedAxis);

    frame.right = simd_cross(frame.axis, frame.up);
  }

  return frame;
}

simd::float3 CoordinateResolver::toSolverFrame(simd::float3 p,
                                                const Frame &f) {
  simd::float3 rel = p - f.origin;
  return {simd_dot(rel, f.right), simd_dot(rel, f.up),
          simd_dot(rel, f.axis)};
}

simd::float3 CoordinateResolver::toUserFrame(simd::float3 p,
                                              const Frame &f) {
  return f.origin + p.x * f.right + p.y * f.up + p.z * f.axis;
}
