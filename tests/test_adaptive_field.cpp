#include "adaptivecad/core/AdaptiveField.hpp"
#include "adaptivecad/core/InverseRecovery.hpp"

#include <cassert>
#include <cmath>
#include <numbers>
#include <vector>

namespace {

bool near(double a, double b, double tol) {
    return std::abs(a - b) <= tol;
}

} // namespace

int main() {
    using adaptivecad::core::AdaptiveField;
    using adaptivecad::core::AreaSample;
    using adaptivecad::core::FitModelKind;
    using adaptivecad::core::FitResultPowerLaw;
    using adaptivecad::core::InverseRecovery;
    using adaptivecad::core::ModelComparisonOptions;

    {
        const AdaptiveField field = AdaptiveField::ConstantResidual(0.10);
        const double pi = std::numbers::pi_v<double>;

        assert(near(field.pi_f(1.0), pi * 1.10, 1e-12));
        assert(near(field.lambda_f(2.0), 1.10, 1e-12));
        assert(near(field.eta(3.0), 0.10, 1e-12));
        assert(near(field.area_functional(2.0), pi * 1.10 * 4.0, 1e-12));
        assert(near(field.effective_dimension(), 2.0, 1e-12));
    }

    {
        const AdaptiveField field = AdaptiveField::PowerLawRadial(1.25, 0.50, 1.0);
        const double expectedPiF = std::numbers::pi_v<double> * 1.25 * std::pow(2.0, 0.50);

        assert(near(field.pi_f(2.0), expectedPiF, 1e-12));
        assert(near(field.effective_dimension(), 2.50, 1e-12));
    }

    {
        const AdaptiveField field = AdaptiveField::GaussianDefect(0.20, 3.0);
        const double q = 2.0 / 3.0;
        const double expectedLambda = 1.0 + 0.20 * std::exp(-(q * q));
        const double expectedArea = std::numbers::pi_v<double> * 4.0 +
            std::numbers::pi_v<double> * 0.20 * 9.0 * (1.0 - std::exp(-(4.0 / 9.0)));

        assert(near(field.lambda_f(2.0), expectedLambda, 1e-12));
        assert(near(field.area_functional(2.0), expectedArea, 1e-12));
        assert(near(field.effective_dimension(), 2.0, 1e-12));
    }

    {
        const AdaptiveField field = AdaptiveField::MemoryBranch(0.40, 0.20, 0.10, 0.50);

        assert(field.is_time_dependent());
        assert(near(field.eta(2.0), 0.10, 1e-12));
        assert(near(field.eta_at_time(2.0, 0.0), 0.10, 1e-12));

        const double etaAtFive = field.eta_at_time(2.0, 5.0);
        assert(etaAtFive > 0.10);
        assert(etaAtFive < 1.0);
    }

    {
        const AdaptiveField angular = AdaptiveField::AngularWeight(1.30);
        const double theta = std::numbers::pi_v<double> / 3.0;

        assert(near(angular.lambda_theta(theta), 1.30, 1e-12));
        assert(near(angular.phase_map(theta), theta, 1e-9));
        assert(near(angular.adaptive_mode_cos(2, theta), std::cos(2.0 * theta), 1e-9));
        assert(near(angular.adaptive_mode_sin(3, theta), std::sin(3.0 * theta), 1e-9));
    }

    {
        const AdaptiveField truth = AdaptiveField::PowerLawRadial(1.15, 0.20, 1.0);
        const std::vector<AreaSample> samples = {
            {1.0, truth.area_functional(1.0)},
            {1.5, truth.area_functional(1.5)},
            {2.0, truth.area_functional(2.0)},
            {2.5, truth.area_functional(2.5)},
            {3.0, truth.area_functional(3.0)}
        };

        const FitResultPowerLaw fit = InverseRecovery::fitPowerLawFromArea(samples, 1.0);
        assert(near(fit.beta, 0.20, 1e-10));
        assert(near(fit.lambda0, 1.15, 1e-10));
        assert(fit.rms_relative_error < 1e-10);
    }

    {
        const AdaptiveField truth = AdaptiveField::GaussianDefect(0.15, 2.40);
        const std::vector<AreaSample> samples = {
            {0.7, truth.area_functional(0.7)},
            {1.0, truth.area_functional(1.0)},
            {1.5, truth.area_functional(1.5)},
            {2.0, truth.area_functional(2.0)},
            {2.7, truth.area_functional(2.7)},
            {3.2, truth.area_functional(3.2)}
        };

        adaptivecad::core::NonlinearFitOptions options;
        options.max_iterations = 200;
        options.use_huber = false;

        const auto fit = InverseRecovery::fitGaussianFromAreaNonlinear(samples, 0.05, 1.2, options);
        assert(fit.converged);
        assert(near(fit.epsilon, 0.15, 1e-6));
        assert(near(fit.length_scale, 2.40, 1e-5));
        assert(fit.rms_relative_error < 1e-9);
        assert(fit.epsilon_stddev >= 0.0);
        assert(fit.length_scale_stddev >= 0.0);
        assert(fit.epsilon_ci95_low <= fit.epsilon && fit.epsilon_ci95_high >= fit.epsilon);
        assert(fit.length_scale_ci95_low <= fit.length_scale && fit.length_scale_ci95_high >= fit.length_scale);
    }

    {
        const AdaptiveField truth = AdaptiveField::ConstantResidual(0.12);
        const std::vector<AreaSample> samples = {
            {0.7, truth.area_functional(0.7)},
            {1.1, truth.area_functional(1.1)},
            {1.8, truth.area_functional(1.8)},
            {2.6, truth.area_functional(2.6)}
        };

        const auto fit = InverseRecovery::fitConstantFromArea(samples);
        assert(near(fit.lambda_constant, 1.12, 1e-10));
        assert(near(fit.epsilon, 0.12, 1e-10));
        assert(fit.rms_relative_error < 1e-10);
    }

    {
        const AdaptiveField truth = AdaptiveField::GaussianDefect(0.15, 2.40);
        const std::vector<AreaSample> samples = {
            {0.7, truth.area_functional(0.7)},
            {1.0, truth.area_functional(1.0)},
            {1.5, truth.area_functional(1.5)},
            {2.0, truth.area_functional(2.0)},
            {2.7, truth.area_functional(2.7)},
            {3.2, truth.area_functional(3.2)}
        };

        ModelComparisonOptions options;
        options.r0 = 1.0;
        options.initial_gaussian_epsilon = 0.05;
        options.initial_gaussian_length_scale = 1.2;
        options.gaussian_options.max_iterations = 200;
        options.gaussian_options.use_huber = false;

        const auto comparison = InverseRecovery::compareAreaModels(samples, options);
        assert(comparison.best_model == FitModelKind::Gaussian);
        assert(comparison.gaussian_weight > comparison.constant_weight);
        assert(comparison.gaussian_weight > comparison.power_law_weight);
        assert(comparison.gaussian_score.aic <= comparison.constant_score.aic);
        assert(comparison.gaussian_score.aic <= comparison.power_law_score.aic);
    }

    {
        const AdaptiveField truth = AdaptiveField::PowerLawRadial(1.20, 0.40, 1.0);
        const std::vector<AreaSample> samples = {
            {0.8, truth.area_functional(0.8)},
            {1.0, truth.area_functional(1.0)},
            {1.4, truth.area_functional(1.4)},
            {2.0, truth.area_functional(2.0)},
            {2.6, truth.area_functional(2.6)},
            {3.1, truth.area_functional(3.1)}
        };

        ModelComparisonOptions options;
        options.r0 = 1.0;
        options.initial_gaussian_epsilon = 0.05;
        options.initial_gaussian_length_scale = 1.2;
        options.gaussian_options.max_iterations = 200;
        options.gaussian_options.use_huber = false;

        const auto comparison = InverseRecovery::compareAreaModels(samples, options);
        assert(comparison.best_model == FitModelKind::PowerLaw);
        assert(comparison.power_law_weight > comparison.constant_weight);
        assert(comparison.power_law_weight > comparison.gaussian_weight);
        assert(comparison.power_law_score.aic <= comparison.constant_score.aic);
        assert(comparison.power_law_score.aic <= comparison.gaussian_score.aic);
    }

    return 0;
}
