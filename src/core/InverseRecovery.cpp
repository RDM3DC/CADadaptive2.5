#include "adaptivecad/core/InverseRecovery.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

#include <algorithm>

namespace adaptivecad::core {
namespace {

constexpr double kPi = std::numbers::pi_v<double>;
constexpr double kNearZero = 1e-12;
constexpr double kMinPositiveSse = 1e-18;

bool isValidSample(const AreaSample& sample) {
    return sample.radius > 0.0 && sample.area > 0.0;
}

std::size_t valid_sample_count(const std::vector<AreaSample>& samples) {
    return static_cast<std::size_t>(std::count_if(
        samples.begin(), samples.end(), [](const AreaSample& s) { return isValidSample(s); }));
}

double positive_sse_floor(double sse) {
    if (!std::isfinite(sse)) {
        return kMinPositiveSse;
    }
    return std::max(sse, kMinPositiveSse);
}

double information_criterion_aic(std::size_t sampleCount, std::size_t parameterCount, double sse) {
    const double n = static_cast<double>(sampleCount);
    return n * std::log(positive_sse_floor(sse) / n) + 2.0 * static_cast<double>(parameterCount);
}

double information_criterion_bic(std::size_t sampleCount, std::size_t parameterCount, double sse) {
    const double n = static_cast<double>(sampleCount);
    return n * std::log(positive_sse_floor(sse) / n) +
        static_cast<double>(parameterCount) * std::log(std::max(1.0, n));
}

double power_law_area(double radius, const FitResultPowerLaw& fit) {
    const double betaPlusTwo = fit.beta + 2.0;
    if (std::abs(betaPlusTwo) < kNearZero) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return (2.0 * kPi * fit.lambda0 / std::pow(fit.r0, fit.beta)) *
        std::pow(radius, betaPlusTwo) / betaPlusTwo;
}

double power_law_relative_sse(const std::vector<AreaSample>& samples, const FitResultPowerLaw& fit) {
    double sse = 0.0;

    for (const AreaSample& sample : samples) {
        if (!isValidSample(sample)) {
            continue;
        }

        const double modelArea = power_law_area(sample.radius, fit);
        const double residual = (modelArea - sample.area) / sample.area;
        sse += residual * residual;
    }

    return sse;
}

double gaussian_area(double radius, double epsilon, double lengthScale) {
    const double q = radius / lengthScale;
    return kPi * radius * radius + kPi * epsilon * lengthScale * lengthScale * (1.0 - std::exp(-(q * q)));
}

double gaussian_area_derivative_epsilon(double radius, double lengthScale) {
    const double q = radius / lengthScale;
    return kPi * lengthScale * lengthScale * (1.0 - std::exp(-(q * q)));
}

double gaussian_area_derivative_length_scale(double radius, double epsilon, double lengthScale) {
    const double q = radius / lengthScale;
    const double expTerm = std::exp(-(q * q));
    return kPi * epsilon *
        (2.0 * lengthScale * (1.0 - expTerm) - (2.0 * radius * radius / lengthScale) * expTerm);
}

double robust_weight(double residual, const NonlinearFitOptions& options) {
    if (!options.use_huber || options.huber_delta <= 0.0) {
        return 1.0;
    }

    const double absResidual = std::abs(residual);
    if (absResidual <= options.huber_delta) {
        return 1.0;
    }

    return options.huber_delta / absResidual;
}

struct GaussianNormalEquation {
    double h00 = 0.0;
    double h01 = 0.0;
    double h11 = 0.0;
    double g0 = 0.0;
    double g1 = 0.0;
    double weighted_cost = 0.0;
    double unweighted_sse = 0.0;
    std::size_t valid_count = 0;
};

GaussianNormalEquation build_gaussian_normal_equation(
    const std::vector<AreaSample>& samples,
    double epsilon,
    double lengthScale,
    const NonlinearFitOptions& options) {
    GaussianNormalEquation normal{};

    for (const AreaSample& sample : samples) {
        if (!isValidSample(sample)) {
            continue;
        }

        const double modelArea = gaussian_area(sample.radius, epsilon, lengthScale);
        const double residual = (modelArea - sample.area) / sample.area;
        const double jacobian0 = gaussian_area_derivative_epsilon(sample.radius, lengthScale) / sample.area;
        const double jacobian1 =
            gaussian_area_derivative_length_scale(sample.radius, epsilon, lengthScale) / sample.area;

        const double weight = robust_weight(residual, options);
        const double sqrtWeight = std::sqrt(weight);
        const double wr = sqrtWeight * residual;
        const double wj0 = sqrtWeight * jacobian0;
        const double wj1 = sqrtWeight * jacobian1;

        normal.h00 += wj0 * wj0;
        normal.h01 += wj0 * wj1;
        normal.h11 += wj1 * wj1;
        normal.g0 += wj0 * wr;
        normal.g1 += wj1 * wr;
        normal.weighted_cost += 0.5 * wr * wr;
        normal.unweighted_sse += residual * residual;
        ++normal.valid_count;
    }

    return normal;
}

bool solve_2x2(
    double a00,
    double a01,
    double a11,
    double b0,
    double b1,
    double* x0,
    double* x1) {
    const double determinant = a00 * a11 - a01 * a01;
    if (std::abs(determinant) < kNearZero) {
        return false;
    }

    *x0 = (b0 * a11 - a01 * b1) / determinant;
    *x1 = (a00 * b1 - a01 * b0) / determinant;
    return std::isfinite(*x0) && std::isfinite(*x1);
}

} // namespace

const char* InverseRecovery::modelKindName(FitModelKind model) {
    switch (model) {
    case FitModelKind::Constant:
        return "constant";
    case FitModelKind::PowerLaw:
        return "power-law";
    case FitModelKind::Gaussian:
        return "gaussian";
    }

    return "unknown";
}

FitResultConstant InverseRecovery::fitConstantFromArea(const std::vector<AreaSample>& samples) {
    if (samples.size() < 2) {
        throw std::invalid_argument("Need at least two area samples");
    }

    double sumBasis = 0.0;
    double sumBasisSquared = 0.0;
    std::size_t validCount = 0;

    for (const AreaSample& sample : samples) {
        if (!isValidSample(sample)) {
            continue;
        }

        const double basis = (kPi * sample.radius * sample.radius) / sample.area;
        sumBasis += basis;
        sumBasisSquared += basis * basis;
        ++validCount;
    }

    if (validCount < 2) {
        throw std::invalid_argument("Need at least two positive (radius, area) samples");
    }
    if (sumBasisSquared <= kNearZero) {
        throw std::runtime_error("Constant model fit is ill-conditioned");
    }

    const double lambdaConstant = sumBasis / sumBasisSquared;
    if (!std::isfinite(lambdaConstant) || lambdaConstant <= 0.0) {
        throw std::runtime_error("Recovered constant lambda is invalid");
    }

    double sse = 0.0;
    for (const AreaSample& sample : samples) {
        if (!isValidSample(sample)) {
            continue;
        }

        const double modelArea = kPi * lambdaConstant * sample.radius * sample.radius;
        const double relativeError = (modelArea - sample.area) / sample.area;
        sse += relativeError * relativeError;
    }

    const double rmsRelativeError = std::sqrt(sse / static_cast<double>(validCount));
    FitResultConstant result{};
    result.lambda_constant = lambdaConstant;
    result.epsilon = lambdaConstant - 1.0;
    result.rms_relative_error = rmsRelativeError;
    result.sse = sse;
    return result;
}

FitResultPowerLaw InverseRecovery::fitPowerLawFromArea(const std::vector<AreaSample>& samples, double r0) {
    if (samples.size() < 2) {
        throw std::invalid_argument("Need at least two area samples");
    }
    if (r0 <= 0.0) {
        throw std::invalid_argument("r0 must be > 0");
    }

    double sumX = 0.0;
    double sumY = 0.0;
    double sumXX = 0.0;
    double sumXY = 0.0;
    std::size_t validCount = 0;

    for (const AreaSample& sample : samples) {
        if (!isValidSample(sample)) {
            continue;
        }

        const double x = std::log(sample.radius);
        const double y = std::log(sample.area);

        sumX += x;
        sumY += y;
        sumXX += x * x;
        sumXY += x * y;
        ++validCount;
    }

    if (validCount < 2) {
        throw std::invalid_argument("Need at least two positive (radius, area) samples");
    }

    const double n = static_cast<double>(validCount);
    const double denominator = n * sumXX - sumX * sumX;
    if (std::abs(denominator) < kNearZero) {
        throw std::runtime_error("Cannot fit power-law from collinear log-radius samples");
    }

    const double slope = (n * sumXY - sumX * sumY) / denominator;
    const double intercept = (sumY - slope * sumX) / n;

    const double beta = slope - 2.0;
    const double betaPlusTwo = beta + 2.0;
    if (std::abs(betaPlusTwo) < kNearZero) {
        throw std::runtime_error("Recovered beta is too close to -2 for stable normalization");
    }

    const double C = std::exp(intercept);
    const double lambda0 = (C * betaPlusTwo * std::pow(r0, beta)) / (2.0 * kPi);
    if (!std::isfinite(lambda0) || lambda0 <= 0.0) {
        throw std::runtime_error("Recovered lambda0 is invalid");
    }

    double sumRelativeSquaredError = 0.0;
    for (const AreaSample& sample : samples) {
        if (!isValidSample(sample)) {
            continue;
        }

        const double modelArea =
            (2.0 * kPi * lambda0 / std::pow(r0, beta)) * std::pow(sample.radius, betaPlusTwo) / betaPlusTwo;
        const double relativeError = (modelArea - sample.area) / sample.area;
        sumRelativeSquaredError += relativeError * relativeError;
    }

    const double rmsRelativeError = std::sqrt(sumRelativeSquaredError / n);
    return FitResultPowerLaw{lambda0, beta, r0, rmsRelativeError};
}

FitResultGaussianNonlinear InverseRecovery::fitGaussianFromAreaNonlinear(
    const std::vector<AreaSample>& samples,
    double initialEpsilon,
    double initialLengthScale,
    const NonlinearFitOptions& options) {
    if (samples.size() < 3) {
        throw std::invalid_argument("Need at least three area samples for nonlinear Gaussian fit");
    }
    if (initialLengthScale <= 0.0) {
        throw std::invalid_argument("initialLengthScale must be > 0");
    }
    if (options.max_iterations == 0) {
        throw std::invalid_argument("max_iterations must be > 0");
    }
    if (options.initial_damping <= 0.0) {
        throw std::invalid_argument("initial_damping must be > 0");
    }

    double epsilon = initialEpsilon;
    double lengthScale = initialLengthScale;
    double damping = options.initial_damping;

    GaussianNormalEquation normal =
        build_gaussian_normal_equation(samples, epsilon, lengthScale, options);
    if (normal.valid_count < 3) {
        throw std::invalid_argument("Need at least three valid positive (radius, area) samples");
    }

    bool converged = false;
    std::size_t performedIterations = 0;

    for (std::size_t iteration = 0; iteration < options.max_iterations; ++iteration) {
        performedIterations = iteration + 1;

        const double gradientNorm = std::sqrt(normal.g0 * normal.g0 + normal.g1 * normal.g1);
        if (gradientNorm <= options.gradient_tolerance) {
            converged = true;
            break;
        }

        const double a00 = normal.h00 + damping * (normal.h00 + 1.0);
        const double a01 = normal.h01;
        const double a11 = normal.h11 + damping * (normal.h11 + 1.0);

        double deltaEpsilon = 0.0;
        double deltaLengthScale = 0.0;
        if (!solve_2x2(
                a00,
                a01,
                a11,
                -normal.g0,
                -normal.g1,
                &deltaEpsilon,
                &deltaLengthScale)) {
            throw std::runtime_error("Nonlinear Gaussian fit failed: singular normal equation");
        }

        const double stepNorm = std::sqrt(
            deltaEpsilon * deltaEpsilon + deltaLengthScale * deltaLengthScale);
        const double parameterNorm = std::sqrt(
            epsilon * epsilon + lengthScale * lengthScale);
        if (stepNorm <= options.parameter_tolerance * (1.0 + parameterNorm)) {
            converged = true;
            break;
        }

        double candidateEpsilon = epsilon + deltaEpsilon;
        double candidateLengthScale = lengthScale + deltaLengthScale;
        if (candidateLengthScale <= 1e-6) {
            candidateLengthScale = 1e-6;
        }

        const GaussianNormalEquation candidate =
            build_gaussian_normal_equation(samples, candidateEpsilon, candidateLengthScale, options);

        if (candidate.weighted_cost < normal.weighted_cost) {
            epsilon = candidateEpsilon;
            lengthScale = candidateLengthScale;
            normal = candidate;
            damping *= 0.5;
            damping = std::max(damping, 1e-12);
        } else {
            damping *= 2.0;
        }
    }

    const GaussianNormalEquation finalNormal =
        build_gaussian_normal_equation(samples, epsilon, lengthScale, options);

    FitResultGaussianNonlinear result{};
    result.epsilon = epsilon;
    result.length_scale = lengthScale;
    result.sse = finalNormal.unweighted_sse;
    result.rms_relative_error = std::sqrt(finalNormal.unweighted_sse / static_cast<double>(finalNormal.valid_count));
    result.iterations = performedIterations;
    result.converged = converged;

    double inv00 = 0.0;
    double inv01 = 0.0;
    double inv11 = 0.0;
    {
        const double determinant = finalNormal.h00 * finalNormal.h11 - finalNormal.h01 * finalNormal.h01;
        if (std::abs(determinant) > kNearZero) {
            inv00 = finalNormal.h11 / determinant;
            inv01 = -finalNormal.h01 / determinant;
            inv11 = finalNormal.h00 / determinant;
        }
    }

    const std::size_t dof = (finalNormal.valid_count > 2) ? (finalNormal.valid_count - 2) : 1;
    const double sigma2 = finalNormal.unweighted_sse / static_cast<double>(dof);

    result.covariance_00 = sigma2 * inv00;
    result.covariance_01 = sigma2 * inv01;
    result.covariance_11 = sigma2 * inv11;

    result.epsilon_stddev = std::sqrt(std::max(0.0, result.covariance_00));
    result.length_scale_stddev = std::sqrt(std::max(0.0, result.covariance_11));

    const double z95 = 1.96;
    result.epsilon_ci95_low = result.epsilon - z95 * result.epsilon_stddev;
    result.epsilon_ci95_high = result.epsilon + z95 * result.epsilon_stddev;
    result.length_scale_ci95_low = result.length_scale - z95 * result.length_scale_stddev;
    result.length_scale_ci95_high = result.length_scale + z95 * result.length_scale_stddev;

    return result;
}

ModelComparisonResult InverseRecovery::compareAreaModels(
    const std::vector<AreaSample>& samples,
    const ModelComparisonOptions& options) {
    const std::size_t validCount = valid_sample_count(samples);
    if (validCount < 3) {
        throw std::invalid_argument("Need at least three valid positive (radius, area) samples for model comparison");
    }

    ModelComparisonResult result{};
    result.constant_fit = fitConstantFromArea(samples);
    result.power_law_fit = fitPowerLawFromArea(samples, options.r0);
    result.gaussian_fit = fitGaussianFromAreaNonlinear(
        samples,
        options.initial_gaussian_epsilon,
        options.initial_gaussian_length_scale,
        options.gaussian_options);

    result.constant_score.model = FitModelKind::Constant;
    result.constant_score.parameter_count = 1;
    result.constant_score.sse = result.constant_fit.sse;
    result.constant_score.rms_relative_error = result.constant_fit.rms_relative_error;
    result.constant_score.aic =
        information_criterion_aic(validCount, result.constant_score.parameter_count, result.constant_score.sse);
    result.constant_score.bic =
        information_criterion_bic(validCount, result.constant_score.parameter_count, result.constant_score.sse);
    result.constant_score.converged = true;

    result.power_law_score.model = FitModelKind::PowerLaw;
    result.power_law_score.parameter_count = 2;
    result.power_law_score.sse = power_law_relative_sse(samples, result.power_law_fit);
    result.power_law_score.rms_relative_error = result.power_law_fit.rms_relative_error;
    result.power_law_score.aic =
        information_criterion_aic(validCount, result.power_law_score.parameter_count, result.power_law_score.sse);
    result.power_law_score.bic =
        information_criterion_bic(validCount, result.power_law_score.parameter_count, result.power_law_score.sse);
    result.power_law_score.converged = true;

    result.gaussian_score.model = FitModelKind::Gaussian;
    result.gaussian_score.parameter_count = 2;
    result.gaussian_score.sse = result.gaussian_fit.sse;
    result.gaussian_score.rms_relative_error = result.gaussian_fit.rms_relative_error;
    result.gaussian_score.aic =
        information_criterion_aic(validCount, result.gaussian_score.parameter_count, result.gaussian_score.sse);
    result.gaussian_score.bic =
        information_criterion_bic(validCount, result.gaussian_score.parameter_count, result.gaussian_score.sse);
    result.gaussian_score.converged = result.gaussian_fit.converged;

    const std::array<double, 3> aic = {
        result.constant_score.aic,
        result.power_law_score.aic,
        result.gaussian_score.aic,
    };
    const double minAic = *std::min_element(aic.begin(), aic.end());

    std::array<double, 3> weights = {0.0, 0.0, 0.0};
    double weightSum = 0.0;
    for (std::size_t i = 0; i < weights.size(); ++i) {
        const double delta = aic[i] - minAic;
        weights[i] = std::exp(-0.5 * delta);
        weightSum += weights[i];
    }

    if (weightSum <= kNearZero) {
        weights = {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};
    } else {
        for (double& weight : weights) {
            weight /= weightSum;
        }
    }

    result.constant_weight = weights[0];
    result.power_law_weight = weights[1];
    result.gaussian_weight = weights[2];

    std::size_t bestIndex = 0;
    for (std::size_t i = 1; i < weights.size(); ++i) {
        if (weights[i] > weights[bestIndex]) {
            bestIndex = i;
        }
    }

    switch (bestIndex) {
    case 0:
        result.best_model = FitModelKind::Constant;
        result.best_model_weight = result.constant_weight;
        break;
    case 1:
        result.best_model = FitModelKind::PowerLaw;
        result.best_model_weight = result.power_law_weight;
        break;
    default:
        result.best_model = FitModelKind::Gaussian;
        result.best_model_weight = result.gaussian_weight;
        break;
    }

    auto append_warning = [&result](const char* text) {
        if (!result.warning.empty()) {
            result.warning += " ";
        }
        result.warning += text;
    };

    if (!result.gaussian_fit.converged) {
        append_warning("Gaussian optimizer did not converge; model ranking may be unreliable.");
    }
    if (result.best_model_weight < 0.60) {
        append_warning("Best-model confidence is low (Akaike weight < 0.60).");
    }

    return result;
}

} // namespace adaptivecad::core
