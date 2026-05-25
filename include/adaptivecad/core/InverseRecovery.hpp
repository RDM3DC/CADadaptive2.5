#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace adaptivecad::core {

struct AreaSample {
    double radius = 0.0;
    double area = 0.0;
};

struct FitResultPowerLaw {
    double lambda0 = 1.0;
    double beta = 0.0;
    double r0 = 1.0;
    double rms_relative_error = 0.0;
};

struct FitResultConstant {
    double lambda_constant = 1.0;
    double epsilon = 0.0;
    double rms_relative_error = 0.0;
    double sse = 0.0;
};

struct NonlinearFitOptions {
    std::size_t max_iterations = 100;
    double parameter_tolerance = 1e-10;
    double gradient_tolerance = 1e-10;
    double initial_damping = 1e-3;
    bool use_huber = true;
    double huber_delta = 1.5;
};

struct FitResultGaussianNonlinear {
    double epsilon = 0.0;
    double length_scale = 1.0;
    double rms_relative_error = 0.0;
    double sse = 0.0;
    std::size_t iterations = 0;
    bool converged = false;

    double epsilon_stddev = 0.0;
    double length_scale_stddev = 0.0;

    double epsilon_ci95_low = 0.0;
    double epsilon_ci95_high = 0.0;
    double length_scale_ci95_low = 0.0;
    double length_scale_ci95_high = 0.0;

    double covariance_00 = 0.0;
    double covariance_01 = 0.0;
    double covariance_11 = 0.0;
};

enum class FitModelKind {
    Constant,
    PowerLaw,
    Gaussian,
};

struct ModelScore {
    FitModelKind model = FitModelKind::Constant;
    std::size_t parameter_count = 0;
    double sse = 0.0;
    double rms_relative_error = 0.0;
    double aic = 0.0;
    double bic = 0.0;
    bool converged = true;
};

struct ModelComparisonOptions {
    double r0 = 1.0;
    double initial_gaussian_epsilon = 0.05;
    double initial_gaussian_length_scale = 1.2;
    NonlinearFitOptions gaussian_options{};
};

struct ModelComparisonResult {
    FitResultConstant constant_fit;
    FitResultPowerLaw power_law_fit;
    FitResultGaussianNonlinear gaussian_fit;

    ModelScore constant_score;
    ModelScore power_law_score;
    ModelScore gaussian_score;

    FitModelKind best_model = FitModelKind::Constant;
    double best_model_weight = 0.0;
    double constant_weight = 0.0;
    double power_law_weight = 0.0;
    double gaussian_weight = 0.0;
    std::string warning;
};

class InverseRecovery {
public:
    static const char* modelKindName(FitModelKind model);
    static FitResultConstant fitConstantFromArea(const std::vector<AreaSample>& samples);
    static FitResultPowerLaw fitPowerLawFromArea(const std::vector<AreaSample>& samples, double r0 = 1.0);
    static FitResultGaussianNonlinear fitGaussianFromAreaNonlinear(
        const std::vector<AreaSample>& samples,
        double initialEpsilon,
        double initialLengthScale,
        const NonlinearFitOptions& options = {});
    static ModelComparisonResult compareAreaModels(
        const std::vector<AreaSample>& samples,
        const ModelComparisonOptions& options = {});
};

} // namespace adaptivecad::core
