#pragma once

#include "adaptivecad/geometry/HistoryAwareState.hpp"

#include <cstddef>
#include <vector>

namespace adaptivecad::geometry {

struct FlatPiCorrection {
    double eta = 0.0;
    AdaptiveMetric adaptive_metric{};
    double pi_f = kEuclideanPi;
};

struct PathCostWeights {
    double residual = 0.0;
    double memory = 0.0;
    double phase = 0.0;
    double support = 0.0;
    double heat = 0.0;
    double stress = 0.0;
    double trust_reward = 0.0;
};

struct AdaptiveGeodesicEdge {
    std::size_t from = 0;
    std::size_t to = 0;
    double euclidean_length = 0.0;
    double residual = 0.0;
    double memory = 0.0;
    int winding_delta = 0;
    bool parity_flip = false;
    double support_penalty = 0.0;
    double heat = 0.0;
    double stress = 0.0;
    double trust = 0.0;
    bool bidirectional = true;
};

struct AdaptiveGeodesicGraph {
    std::vector<AdaptiveCADState> nodes;
    std::vector<AdaptiveGeodesicEdge> edges;
};

struct AdaptiveGeodesicResult {
    bool found = false;
    double cost = 0.0;
    std::vector<std::size_t> node_path;
    std::vector<std::size_t> edge_path;
};

FlatPiCorrection make_flat_pi_correction(double eta);
double power_law_pi_f(double radius, double lambda0, double r0, double beta);
double radial_operator_effective_dimension(double beta) noexcept;
double radial_laplacian_first_derivative_weight(double radius, double beta);

PhaseLiftState advance_phase_lift(const PhaseLiftState& state, double deltaTheta);
double update_curve_memory(double previousMemory, double source, double decay, double dt);
double update_path_trust(double previousTrust, double inputMagnitude, double alpha, double decay, double dt);

double infer_pi_f_from_area_growth(double radius, double areaDerivative);
double infer_pi_f_from_radial_flux(double radius, double radialFlux, double radialGradient);

double adaptive_edge_cost(
    const AdaptiveGeodesicGraph& graph,
    const AdaptiveGeodesicEdge& edge,
    const PathCostWeights& weights);
AdaptiveGeodesicResult compute_adaptive_geodesic(
    const AdaptiveGeodesicGraph& graph,
    std::size_t startNode,
    std::size_t goalNode,
    const PathCostWeights& weights);

AdaptiveGeodesicGraph make_torus_phase_lift_benchmark(std::size_t segmentCount, double majorRadius);
AdaptiveGeodesicGraph make_benchy_manufacturing_benchmark();

} // namespace adaptivecad::geometry