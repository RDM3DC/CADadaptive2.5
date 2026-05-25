#pragma once

#include "adaptivecad/geometry/GeometryAdapter.hpp"

namespace adaptivecad::geometry {

constexpr double kEuclideanPi = 3.141592653589793238462643383279502884;

struct AdaptiveMetric {
    double xx = 1.0;
    double xy = 0.0;
    double xz = 0.0;
    double yy = 1.0;
    double yz = 0.0;
    double zz = 1.0;
};

struct PhaseLiftState {
    double theta_R = 0.0;
    int winding = 0;
    bool branch_parity = false;
};

struct ManufacturingState {
    double stress = 0.0;
    double heat = 0.0;
    double support_risk = 0.0;
    double printability = 1.0;
    double sensor = 0.0;
};

struct AdaptiveCADState {
    Point3D position{};
    Point3D normal{0.0, 0.0, 1.0};
    double curvature = 0.0;
    AdaptiveMetric adaptive_metric{};
    double pi_a = kEuclideanPi;
    double pi_f = kEuclideanPi;
    double path_trust = 0.0;
    double curve_memory = 0.0;
    PhaseLiftState phase{};
    double residual = 0.0;
    ManufacturingState manufacturing{};
};

} // namespace adaptivecad::geometry