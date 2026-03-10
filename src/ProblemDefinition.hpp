#pragma once
#include <array>
#include <map>
#include <string>
#include <vector>

struct MaterialProperties {
  std::string name;
  std::map<std::string, float> properties;
};

struct NamedPoint {
  std::string name;
  std::array<std::string, 3> position;
  std::string type;
  std::string domainVariable;
};

struct Distribution {
  std::string type;
  std::string coordinateSystem;
  std::string symmetry;
  std::vector<std::string> variables;
  std::string expression;
  float tolerance = 0.0f;
};

struct BoundaryCondition {
  std::string id;
  std::string description;
  std::string geometryType;
  std::map<std::string, std::string> geometryParams;
  std::map<std::string, Distribution> distributions;
  std::map<std::string, std::string> anchors;
};

struct Constraint {
  std::string id;
  std::string type;
  std::string quantity;
  std::string op;
  std::string value;
  int priority = 1;
};

struct DomainSpec {
  std::string type;
  std::vector<std::string> variables;
  std::map<std::string, std::pair<std::string, std::string>> ranges;
  std::vector<int> samples;
  std::string surfaceName;
  std::string pointLocation;
};

struct FieldConstraint {
  std::string id;
  std::string description;
  std::string coordinateSystem;
  DomainSpec domain;
  std::string lhs;
  std::string op;
  std::string rhs;
  float tolerance = 0.0f;
  int priority = 1;
};

struct Parameter {
  std::string name;
  std::string type;
  std::string expression;
  float value = 0.0f;
  float lo = 0.0f, hi = 0.0f;
  int arraySize = 0;
  std::vector<float> arrayValues;
};

struct OptimizationConfig {
  std::string mode;
  std::string method;
  std::string precision;
  int fixedMaxIterations = 500;
  int fixedSampleSize = 100;

  struct FidelityLevel {
    int samples;
    int maxIterations;
    std::string physicsModel;
  };
  std::vector<FidelityLevel> fidelityLevels;

  int convergenceMaxIterations = 2000;
  int convergenceMinSamples = 10;
  int convergenceMaxSamples = 500;
  float convergenceSampleGrowth = 2.0f;
  std::map<std::string, float> criteriaTolerances;
};

struct ProblemDefinition {
  std::string problemType;
  std::string units;
  std::map<std::string, MaterialProperties> materials;
  std::map<std::string, NamedPoint> namedPoints;
  std::vector<BoundaryCondition> boundaryConditions;
  std::vector<Constraint> constraints;
  std::vector<FieldConstraint> fieldConstraints;
  std::map<std::string, Parameter> parameters;
  OptimizationConfig optimization;
};
