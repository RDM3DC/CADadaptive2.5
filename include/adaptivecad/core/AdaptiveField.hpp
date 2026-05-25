#pragma once

#include "adaptivecad/core/Models.hpp"

#include <vector>

namespace adaptivecad::core {

class AdaptiveField {
public:
    static AdaptiveField ConstantResidual(double epsilon);
    static AdaptiveField PowerLawRadial(double lambda0, double beta, double r0);
    static AdaptiveField GaussianDefect(double epsilon, double lengthScale);
    static AdaptiveField MemoryBranch(double alpha, double mu, double eta0 = 0.0, double sourceStrength = 0.0);
    static AdaptiveField AngularWeight(
        double lambda0,
        const std::vector<double>& cosineCoefficients = {},
        const std::vector<double>& sineCoefficients = {});

    double pi_f(double r) const;
    double pi_f_at_time(double r, double t) const;
    double pi_f_polar(double r, double theta) const;
    double lambda_f(double r) const;
    double lambda_f_at_time(double r, double t) const;
    double lambda_theta(double theta) const;
    double eta(double r) const;
    double eta_at_time(double r, double t) const;
    double shell_weight(double r) const;
    double area_functional(double R) const;
    double phase_map(double theta, int integrationSteps = 512) const;
    double adaptive_mode_cos(int n, double theta, int integrationSteps = 512) const;
    double adaptive_mode_sin(int n, double theta, int integrationSteps = 512) const;
    double effective_dimension() const;
    bool is_time_dependent() const noexcept;

    ModelFamily family() const noexcept;

private:
    explicit AdaptiveField(ConstantResidualModel constantModel);
    explicit AdaptiveField(PowerLawRadialModel powerLawModel);
    explicit AdaptiveField(GaussianDefectModel gaussianModel);
    explicit AdaptiveField(MemoryBranchModel memoryModel);
    explicit AdaptiveField(AngularWeightModel angularModel);

    ModelFamily family_ = ModelFamily::ConstantResidual;
    ConstantResidualModel constantModel_{};
    PowerLawRadialModel powerLawModel_{};
    GaussianDefectModel gaussianModel_{};
    MemoryBranchModel memoryModel_{};
    AngularWeightModel angularModel_{};
};

} // namespace adaptivecad::core
