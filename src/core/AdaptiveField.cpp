#include "adaptivecad/core/AdaptiveField.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>

namespace adaptivecad::core {
namespace {

constexpr double kPi = std::numbers::pi_v<double>;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kAreaSingularityTolerance = 1e-12;
constexpr int kMinPhaseIntegrationSteps = 8;

void requirePositiveRadius(double value, const char* functionName) {
    if (value <= 0.0) {
        throw std::invalid_argument(std::string(functionName) + ": radius must be > 0");
    }
}

void requireNonNegativeTime(double value, const char* functionName) {
    if (value < 0.0) {
        throw std::invalid_argument(std::string(functionName) + ": time must be >= 0");
    }
}

double memory_eta(const MemoryBranchModel& memoryModel, double t) {
    requireNonNegativeTime(t, "memory_eta");

    if (memoryModel.mu > 0.0) {
        const double etaInfinity = (memoryModel.alpha * memoryModel.source_strength) / memoryModel.mu;
        return etaInfinity + (memoryModel.eta0 - etaInfinity) * std::exp(-memoryModel.mu * t);
    }

    return memoryModel.eta0 + memoryModel.alpha * memoryModel.source_strength * t;
}

double requirePositiveLambda(double lambdaValue, const char* functionName) {
    if (lambdaValue <= 0.0) {
        throw std::runtime_error(std::string(functionName) + ": resulting lambda is non-positive");
    }
    return lambdaValue;
}

double normalize_angle(double theta) {
    const double wrapped = std::fmod(theta, kTwoPi);
    if (wrapped < 0.0) {
        return wrapped + kTwoPi;
    }
    return wrapped;
}

double evaluate_angular_lambda_raw(const AngularWeightModel& angularModel, double theta) {
    const double wrappedTheta = normalize_angle(theta);

    double value = angularModel.lambda0;
    for (std::size_t i = 0; i < angularModel.cosine_coefficients.size(); ++i) {
        const double harmonic = static_cast<double>(i + 1);
        value += angularModel.cosine_coefficients[i] * std::cos(harmonic * wrappedTheta);
    }
    for (std::size_t i = 0; i < angularModel.sine_coefficients.size(); ++i) {
        const double harmonic = static_cast<double>(i + 1);
        value += angularModel.sine_coefficients[i] * std::sin(harmonic * wrappedTheta);
    }

    return value;
}

double integrate_angular_lambda(const AngularWeightModel& angularModel, double start, double end, int steps) {
    if (steps < kMinPhaseIntegrationSteps) {
        throw std::invalid_argument("integrate_angular_lambda: integration steps must be >= 8");
    }

    if (std::abs(end - start) <= 0.0) {
        return 0.0;
    }

    const double stepSize = (end - start) / static_cast<double>(steps);
    double sum = 0.0;

    for (int i = 0; i <= steps; ++i) {
        const double t = start + stepSize * static_cast<double>(i);
        const double lambda = requirePositiveLambda(evaluate_angular_lambda_raw(angularModel, t), "phase_map");
        const double weight = (i == 0 || i == steps) ? 0.5 : 1.0;
        sum += weight * lambda;
    }

    return sum * stepSize;
}

} // namespace

AdaptiveField AdaptiveField::ConstantResidual(double epsilon) {
    if (epsilon <= -1.0) {
        throw std::invalid_argument("Constant residual requires epsilon > -1");
    }
    return AdaptiveField(ConstantResidualModel{epsilon});
}

AdaptiveField AdaptiveField::PowerLawRadial(double lambda0, double beta, double r0) {
    if (lambda0 <= 0.0) {
        throw std::invalid_argument("Power-law branch requires lambda0 > 0");
    }
    if (r0 <= 0.0) {
        throw std::invalid_argument("Power-law branch requires r0 > 0");
    }
    return AdaptiveField(PowerLawRadialModel{lambda0, beta, r0});
}

AdaptiveField AdaptiveField::GaussianDefect(double epsilon, double lengthScale) {
    if (lengthScale <= 0.0) {
        throw std::invalid_argument("Gaussian defect branch requires lengthScale > 0");
    }
    if (epsilon <= -1.0) {
        throw std::invalid_argument("Gaussian defect branch requires epsilon > -1");
    }
    return AdaptiveField(GaussianDefectModel{epsilon, lengthScale});
}

AdaptiveField AdaptiveField::MemoryBranch(double alpha, double mu, double eta0, double sourceStrength) {
    if (mu < 0.0) {
        throw std::invalid_argument("Memory branch requires mu >= 0");
    }
    if (eta0 <= -1.0) {
        throw std::invalid_argument("Memory branch requires eta0 > -1");
    }
    return AdaptiveField(MemoryBranchModel{alpha, mu, eta0, sourceStrength});
}

AdaptiveField AdaptiveField::AngularWeight(
    double lambda0,
    const std::vector<double>& cosineCoefficients,
    const std::vector<double>& sineCoefficients) {
    if (lambda0 <= 0.0) {
        throw std::invalid_argument("Angular weight branch requires lambda0 > 0");
    }

    AngularWeightModel angularModel;
    angularModel.lambda0 = lambda0;
    angularModel.cosine_coefficients = cosineCoefficients;
    angularModel.sine_coefficients = sineCoefficients;

    for (int i = 0; i < 720; ++i) {
        const double theta = (kTwoPi * static_cast<double>(i)) / 720.0;
        const double lambda = evaluate_angular_lambda_raw(angularModel, theta);
        if (lambda <= 0.0) {
            throw std::invalid_argument("Angular weight branch produced non-positive lambda over [0, 2pi)");
        }
    }

    return AdaptiveField(angularModel);
}

double AdaptiveField::pi_f(double r) const {
    requirePositiveRadius(r, "pi_f");

    switch (family_) {
    case ModelFamily::ConstantResidual:
        return kPi * (1.0 + constantModel_.epsilon);
    case ModelFamily::PowerLawRadial:
        return kPi * powerLawModel_.lambda0 * std::pow(r / powerLawModel_.r0, powerLawModel_.beta);
    case ModelFamily::GaussianDefect: {
        const double q = r / gaussianModel_.length_scale;
        const double lambda = 1.0 + gaussianModel_.epsilon * std::exp(-(q * q));
        return kPi * requirePositiveLambda(lambda, "pi_f");
    }
    case ModelFamily::MemoryBranch:
        return kPi * requirePositiveLambda(1.0 + memoryModel_.eta0, "pi_f");
    case ModelFamily::AngularWeight:
        return kPi * requirePositiveLambda(angularModel_.lambda0, "pi_f");
    }

    throw std::runtime_error("pi_f: unsupported model family");
}

double AdaptiveField::pi_f_at_time(double r, double t) const {
    requirePositiveRadius(r, "pi_f_at_time");

    if (family_ != ModelFamily::MemoryBranch) {
        return pi_f(r);
    }

    const double lambda = 1.0 + memory_eta(memoryModel_, t);
    return kPi * requirePositiveLambda(lambda, "pi_f_at_time");
}

double AdaptiveField::pi_f_polar(double r, double theta) const {
    requirePositiveRadius(r, "pi_f_polar");

    if (family_ == ModelFamily::AngularWeight) {
        return kPi * lambda_theta(theta);
    }

    return pi_f(r);
}

double AdaptiveField::lambda_f(double r) const {
    return pi_f(r) / kPi;
}

double AdaptiveField::lambda_f_at_time(double r, double t) const {
    return pi_f_at_time(r, t) / kPi;
}

double AdaptiveField::lambda_theta(double theta) const {
    if (family_ == ModelFamily::AngularWeight) {
        return requirePositiveLambda(evaluate_angular_lambda_raw(angularModel_, theta), "lambda_theta");
    }

    if (family_ == ModelFamily::ConstantResidual) {
        return requirePositiveLambda(1.0 + constantModel_.epsilon, "lambda_theta");
    }

    if (family_ == ModelFamily::MemoryBranch) {
        return requirePositiveLambda(1.0 + memoryModel_.eta0, "lambda_theta");
    }

    return lambda_f(1.0);
}

double AdaptiveField::eta(double r) const {
    return lambda_f(r) - 1.0;
}

double AdaptiveField::eta_at_time(double r, double t) const {
    return lambda_f_at_time(r, t) - 1.0;
}

double AdaptiveField::shell_weight(double r) const {
    return 2.0 * pi_f(r) * r;
}

double AdaptiveField::area_functional(double R) const {
    requirePositiveRadius(R, "area_functional");

    if (family_ == ModelFamily::ConstantResidual) {
        return kPi * (1.0 + constantModel_.epsilon) * R * R;
    }

    if (family_ == ModelFamily::GaussianDefect) {
        const double q = R / gaussianModel_.length_scale;
        const double gaussianPart =
            kPi * gaussianModel_.epsilon * gaussianModel_.length_scale * gaussianModel_.length_scale *
            (1.0 - std::exp(-(q * q)));
        return kPi * R * R + gaussianPart;
    }

    if (family_ == ModelFamily::MemoryBranch) {
        return kPi * requirePositiveLambda(1.0 + memoryModel_.eta0, "area_functional") * R * R;
    }

    if (family_ == ModelFamily::AngularWeight) {
        return kPi * requirePositiveLambda(angularModel_.lambda0, "area_functional") * R * R;
    }

    const double exponent = powerLawModel_.beta;
    const double denominator = exponent + 2.0;
    if (std::abs(denominator) < kAreaSingularityTolerance) {
        throw std::runtime_error("Power-law area functional is singular for beta = -2");
    }

    const double scale = 2.0 * kPi * powerLawModel_.lambda0 / std::pow(powerLawModel_.r0, exponent);
    return scale * std::pow(R, exponent + 2.0) / denominator;
}

double AdaptiveField::phase_map(double theta, int integrationSteps) const {
    if (family_ != ModelFamily::AngularWeight) {
        throw std::runtime_error("phase_map is only available for AngularWeight fields");
    }
    if (integrationSteps < kMinPhaseIntegrationSteps) {
        throw std::invalid_argument("phase_map requires integrationSteps >= 8");
    }

    const double lambdaPeriodIntegral = integrate_angular_lambda(angularModel_, 0.0, kTwoPi, integrationSteps);
    if (lambdaPeriodIntegral <= 0.0) {
        throw std::runtime_error("phase_map: angular lambda period integral is non-positive");
    }

    double cycles = std::floor(theta / kTwoPi);
    double remainder = theta - cycles * kTwoPi;
    if (remainder < 0.0) {
        cycles -= 1.0;
        remainder += kTwoPi;
    }

    const double remainderIntegral = integrate_angular_lambda(angularModel_, 0.0, remainder, integrationSteps);
    const double fullIntegral = cycles * lambdaPeriodIntegral + remainderIntegral;

    return kTwoPi * fullIntegral / lambdaPeriodIntegral;
}

double AdaptiveField::adaptive_mode_cos(int n, double theta, int integrationSteps) const {
    if (n < 0) {
        throw std::invalid_argument("adaptive_mode_cos requires n >= 0");
    }

    return std::cos(static_cast<double>(n) * phase_map(theta, integrationSteps));
}

double AdaptiveField::adaptive_mode_sin(int n, double theta, int integrationSteps) const {
    if (n < 0) {
        throw std::invalid_argument("adaptive_mode_sin requires n >= 0");
    }

    return std::sin(static_cast<double>(n) * phase_map(theta, integrationSteps));
}

double AdaptiveField::effective_dimension() const {
    if (family_ == ModelFamily::PowerLawRadial) {
        return powerLawModel_.beta + 2.0;
    }

    return 2.0;
}

bool AdaptiveField::is_time_dependent() const noexcept {
    return family_ == ModelFamily::MemoryBranch;
}

ModelFamily AdaptiveField::family() const noexcept {
    return family_;
}

AdaptiveField::AdaptiveField(ConstantResidualModel constantModel)
    : family_(ModelFamily::ConstantResidual),
      constantModel_(constantModel),
            powerLawModel_{},
            gaussianModel_{},
            memoryModel_{},
            angularModel_{} {}

AdaptiveField::AdaptiveField(PowerLawRadialModel powerLawModel)
    : family_(ModelFamily::PowerLawRadial),
      constantModel_{},
            powerLawModel_(powerLawModel),
            gaussianModel_{},
            memoryModel_{},
            angularModel_{} {}

AdaptiveField::AdaptiveField(GaussianDefectModel gaussianModel)
        : family_(ModelFamily::GaussianDefect),
            constantModel_{},
            powerLawModel_{},
            gaussianModel_(gaussianModel),
            memoryModel_{},
            angularModel_{} {}

AdaptiveField::AdaptiveField(MemoryBranchModel memoryModel)
        : family_(ModelFamily::MemoryBranch),
            constantModel_{},
            powerLawModel_{},
            gaussianModel_{},
            memoryModel_(memoryModel),
            angularModel_{} {}

AdaptiveField::AdaptiveField(AngularWeightModel angularModel)
        : family_(ModelFamily::AngularWeight),
            constantModel_{},
            powerLawModel_{},
            gaussianModel_{},
            memoryModel_{},
            angularModel_(std::move(angularModel)) {}

} // namespace adaptivecad::core
