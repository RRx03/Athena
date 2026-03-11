#pragma once
#include "ProblemDefinition.hpp"
#include <simd/simd.h>

// ═══════════════════════════════════════════════════════════════
// CoordinateResolver — Deux modes
//
// Mode "auto" (défaut) :
//   Le solveur déduit l'origine et l'axe à partir des anchors
//   et des géométries BC (disques concentriques → axe commun).
//
// Mode "manual" :
//   L'utilisateur a cliqué "Set Origin" dans le front-end.
//   Il a choisi un point comme origine et orienté les axes.
//   Le JSON contient un bloc "frame" explicite.
//
// JSON dans problem.json :
//
//   "frame": { "mode": "auto" }
//
//   ou
//
//   "frame": {
//       "mode": "manual",
//       "origin": [0.1, 0.0, 0.0],
//       "axis":   [0, 0, 1],
//       "up":     [0, 1, 0]
//   }
//
// Le "right" est déduit par produit vectoriel (axis × up).
// ═══════════════════════════════════════════════════════════════

class CoordinateResolver {
public:
  struct Frame {
    simd::float3 origin;
    simd::float3 axis;  // Axe principal (ex: axe de révolution)
    simd::float3 up;    // Axe secondaire
    simd::float3 right; // Déduit : cross(axis, up)
  };

  // Résout le frame selon le mode (auto ou manual)
  static Frame resolveFrame(const ProblemDefinition &problem);

  // Transformations
  static simd::float3 toSolverFrame(simd::float3 p, const Frame &f);
  static simd::float3 toUserFrame(simd::float3 p, const Frame &f);
};