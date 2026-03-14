#include "ConstraintChecker.hpp"
#include "CoordinateResolver.hpp"
#include "ExpressionEvaluator.hpp"
#include "ParameterSpace.hpp"
#include "ProblemParser.hpp"
#include "physics/IsentropicNozzle.hpp"
#include "physics/PressureVessel.hpp"
#include <cassert>
#include <cmath>
#include <iostream>


static void check(const std::string &name, float got, float expected, float eps = 1e-4f);

static void checkStr(const std::string &name, const std::string &got,
                     const std::string &expected);

static void checkBool(const std::string &name, bool got, bool expected);

static void checkInt(const std::string &name, int got, int expected);

static ExpressionEvaluator::Context makeCtx();

void runTests();