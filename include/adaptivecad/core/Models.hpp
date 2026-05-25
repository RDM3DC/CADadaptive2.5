#pragma once

#include <vector>

namespace adaptivecad::core {

struct ConstantResidualModel {
    double epsilon = 0.0;
};

struct PowerLawRadialModel {
    double lambda0 = 1.0;
    double beta = 0.0;
    double r0 = 1.0;
};

struct GaussianDefectModel {
    double epsilon = 0.0;
    double length_scale = 1.0;
};

struct MemoryBranchModel {
    double alpha = 0.0;
    double mu = 0.0;
    double eta0 = 0.0;
    double source_strength = 0.0;
};

struct AngularWeightModel {
    double lambda0 = 1.0;
    std::vector<double> cosine_coefficients;
    std::vector<double> sine_coefficients;
};

enum class ModelFamily {
    ConstantResidual,
    PowerLawRadial,
    GaussianDefect,
    MemoryBranch,
    AngularWeight
};

} // namespace adaptivecad::core
