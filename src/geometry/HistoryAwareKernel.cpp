#include "adaptivecad/geometry/HistoryAwareKernel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>

namespace adaptivecad::geometry {
namespace {

constexpr double kTwoPi = 2.0 * kEuclideanPi;

double squared_distance(const Point3D& left, const Point3D& right) {
    const double dx = left.x - right.x;
    const double dy = left.y - right.y;
    const double dz = left.z - right.z;
    return dx * dx + dy * dy + dz * dz;
}

double metric_trace_scale(const AdaptiveMetric& metric) {
    return std::sqrt(std::max(0.0, (metric.xx + metric.yy + metric.zz) / 3.0));
}

double averaged_metric_scale(const AdaptiveCADState& from, const AdaptiveCADState& to) {
    return 0.5 * (metric_trace_scale(from.adaptive_metric) + metric_trace_scale(to.adaptive_metric));
}

double metric_length(
    const AdaptiveCADState& from,
    const AdaptiveCADState& to,
    double fallbackLength) {
    const double positionLength = std::sqrt(squared_distance(from.position, to.position));
    const double euclideanLength = positionLength > 0.0 ? positionLength : fallbackLength;
    return euclideanLength * averaged_metric_scale(from, to);
}

double average_node_value(double left, double right, double edgeValue) {
    return 0.5 * (left + right) + edgeValue;
}

void validate_edge_indices(const AdaptiveGeodesicGraph& graph, const AdaptiveGeodesicEdge& edge) {
    if (edge.from >= graph.nodes.size() || edge.to >= graph.nodes.size()) {
        throw std::out_of_range("adaptive geodesic edge references a node outside the graph");
    }
}

} // namespace

FlatPiCorrection make_flat_pi_correction(double eta) {
    const double metricScale = std::exp(2.0 * eta);

    FlatPiCorrection correction;
    correction.eta = eta;
    correction.adaptive_metric.xx = metricScale;
    correction.adaptive_metric.yy = metricScale;
    correction.adaptive_metric.zz = metricScale;
    correction.pi_f = kEuclideanPi * (1.0 + eta);
    return correction;
}

double power_law_pi_f(double radius, double lambda0, double r0, double beta) {
    if (radius <= 0.0 || lambda0 <= 0.0 || r0 <= 0.0) {
        throw std::invalid_argument("power_law_pi_f requires positive radius, lambda0, and r0");
    }

    return kEuclideanPi * lambda0 * std::pow(radius / r0, beta);
}

double radial_operator_effective_dimension(double beta) noexcept {
    return beta + 2.0;
}

double radial_laplacian_first_derivative_weight(double radius, double beta) {
    if (radius <= 0.0) {
        throw std::invalid_argument("radial_laplacian_first_derivative_weight requires positive radius");
    }

    return (beta + 1.0) / radius;
}

PhaseLiftState advance_phase_lift(const PhaseLiftState& state, double deltaTheta) {
    PhaseLiftState advanced = state;
    double unwrapped = advanced.theta_R + deltaTheta;

    while (unwrapped >= kTwoPi) {
        unwrapped -= kTwoPi;
        ++advanced.winding;
    }
    while (unwrapped < 0.0) {
        unwrapped += kTwoPi;
        --advanced.winding;
    }

    advanced.theta_R = unwrapped;
    advanced.branch_parity = (advanced.winding % 2) != 0;
    return advanced;
}

double update_curve_memory(double previousMemory, double source, double decay, double dt) {
    if (decay < 0.0 || dt < 0.0) {
        throw std::invalid_argument("update_curve_memory requires non-negative decay and dt");
    }

    return previousMemory + dt * (source - decay * previousMemory);
}

double update_path_trust(double previousTrust, double inputMagnitude, double alpha, double decay, double dt) {
    if (alpha < 0.0 || decay < 0.0 || dt < 0.0) {
        throw std::invalid_argument("update_path_trust requires non-negative alpha, decay, and dt");
    }

    const double updated = previousTrust + dt * (alpha * std::abs(inputMagnitude) - decay * previousTrust);
    return std::max(0.0, updated);
}

double infer_pi_f_from_area_growth(double radius, double areaDerivative) {
    if (radius <= 0.0) {
        throw std::invalid_argument("infer_pi_f_from_area_growth requires positive radius");
    }

    return areaDerivative / (2.0 * radius);
}

double infer_pi_f_from_radial_flux(double radius, double radialFlux, double radialGradient) {
    if (radius <= 0.0 || radialGradient == 0.0) {
        throw std::invalid_argument("infer_pi_f_from_radial_flux requires positive radius and non-zero radial gradient");
    }

    return radialFlux / (2.0 * radius * radialGradient);
}

double adaptive_edge_cost(
    const AdaptiveGeodesicGraph& graph,
    const AdaptiveGeodesicEdge& edge,
    const PathCostWeights& weights) {
    validate_edge_indices(graph, edge);

    const AdaptiveCADState& from = graph.nodes[edge.from];
    const AdaptiveCADState& to = graph.nodes[edge.to];
    const double adaptiveLength = metric_length(from, to, edge.euclidean_length);
    const double residualPenalty = weights.residual * average_node_value(from.residual, to.residual, edge.residual);
    const double memoryPenalty = weights.memory * average_node_value(from.curve_memory, to.curve_memory, edge.memory);
    const double supportPenalty = weights.support * average_node_value(
        from.manufacturing.support_risk,
        to.manufacturing.support_risk,
        edge.support_penalty);
    const double heatPenalty = weights.heat * average_node_value(from.manufacturing.heat, to.manufacturing.heat, edge.heat);
    const double stressPenalty = weights.stress * average_node_value(from.manufacturing.stress, to.manufacturing.stress, edge.stress);
    const double phasePenalty = weights.phase * (
        std::abs(edge.winding_delta) + (edge.parity_flip ? 1.0 : 0.0)
        + std::abs(to.phase.winding - from.phase.winding)
        + (to.phase.branch_parity != from.phase.branch_parity ? 1.0 : 0.0));
    const double trustReward = weights.trust_reward * average_node_value(from.path_trust, to.path_trust, edge.trust);

    return std::max(1e-12,
        adaptiveLength + residualPenalty + memoryPenalty + phasePenalty + supportPenalty + heatPenalty + stressPenalty
            - trustReward);
}

AdaptiveGeodesicResult compute_adaptive_geodesic(
    const AdaptiveGeodesicGraph& graph,
    std::size_t startNode,
    std::size_t goalNode,
    const PathCostWeights& weights) {
    if (startNode >= graph.nodes.size() || goalNode >= graph.nodes.size()) {
        throw std::out_of_range("adaptive geodesic start or goal node is outside the graph");
    }

    std::vector<std::vector<std::size_t>> adjacency(graph.nodes.size());
    for (std::size_t edgeIndex = 0; edgeIndex < graph.edges.size(); ++edgeIndex) {
        const AdaptiveGeodesicEdge& edge = graph.edges[edgeIndex];
        validate_edge_indices(graph, edge);
        adjacency[edge.from].push_back(edgeIndex);
        if (edge.bidirectional) {
            adjacency[edge.to].push_back(edgeIndex);
        }
    }

    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> distances(graph.nodes.size(), infinity);
    std::vector<std::size_t> previousNode(graph.nodes.size(), graph.nodes.size());
    std::vector<std::size_t> previousEdge(graph.nodes.size(), graph.edges.size());

    using QueueEntry = std::pair<double, std::size_t>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
    distances[startNode] = 0.0;
    queue.push(QueueEntry{0.0, startNode});

    while (!queue.empty()) {
        const auto [distance, currentNode] = queue.top();
        queue.pop();

        if (distance > distances[currentNode]) {
            continue;
        }
        if (currentNode == goalNode) {
            break;
        }

        for (std::size_t edgeIndex : adjacency[currentNode]) {
            AdaptiveGeodesicEdge edge = graph.edges[edgeIndex];
            std::size_t neighborNode = edge.to;
            if (edge.from != currentNode) {
                neighborNode = edge.from;
                std::swap(edge.from, edge.to);
                edge.winding_delta = -edge.winding_delta;
            }

            const double candidateDistance = distance + adaptive_edge_cost(graph, edge, weights);
            if (candidateDistance + 1e-12 < distances[neighborNode]) {
                distances[neighborNode] = candidateDistance;
                previousNode[neighborNode] = currentNode;
                previousEdge[neighborNode] = edgeIndex;
                queue.push(QueueEntry{candidateDistance, neighborNode});
            }
        }
    }

    AdaptiveGeodesicResult result;
    if (!std::isfinite(distances[goalNode])) {
        return result;
    }

    result.found = true;
    result.cost = distances[goalNode];

    std::vector<std::size_t> reversedNodes;
    std::vector<std::size_t> reversedEdges;
    for (std::size_t node = goalNode; node != startNode; node = previousNode[node]) {
        reversedNodes.push_back(node);
        reversedEdges.push_back(previousEdge[node]);
    }
    reversedNodes.push_back(startNode);

    result.node_path.assign(reversedNodes.rbegin(), reversedNodes.rend());
    result.edge_path.assign(reversedEdges.rbegin(), reversedEdges.rend());
    return result;
}

AdaptiveGeodesicGraph make_torus_phase_lift_benchmark(std::size_t segmentCount, double majorRadius) {
    if (segmentCount < 4 || majorRadius <= 0.0) {
        throw std::invalid_argument("make_torus_phase_lift_benchmark requires at least 4 segments and positive radius");
    }

    AdaptiveGeodesicGraph graph;
    graph.nodes.reserve(segmentCount);
    graph.edges.reserve(segmentCount);

    for (std::size_t index = 0; index < segmentCount; ++index) {
        const double theta = kTwoPi * static_cast<double>(index) / static_cast<double>(segmentCount);
        AdaptiveCADState state;
        state.position = Point3D{majorRadius * std::cos(theta), majorRadius * std::sin(theta), 0.0};
        state.normal = Point3D{std::cos(theta), std::sin(theta), 0.0};
        state.phase.theta_R = theta;
        graph.nodes.push_back(state);
    }

    const double segmentLength = kTwoPi * majorRadius / static_cast<double>(segmentCount);
    for (std::size_t index = 0; index < segmentCount; ++index) {
        AdaptiveGeodesicEdge edge;
        edge.from = index;
        edge.to = (index + 1) % segmentCount;
        edge.euclidean_length = segmentLength;
        edge.winding_delta = edge.to == 0 ? 1 : 0;
        edge.parity_flip = edge.to == 0;
        graph.edges.push_back(edge);
    }

    return graph;
}

AdaptiveGeodesicGraph make_benchy_manufacturing_benchmark() {
    AdaptiveGeodesicGraph graph;
    graph.nodes.resize(5);
    graph.nodes[0].position = Point3D{0.0, 0.0, 0.0};
    graph.nodes[1].position = Point3D{1.0, 0.0, 0.0};
    graph.nodes[2].position = Point3D{2.0, 0.0, 0.0};
    graph.nodes[3].position = Point3D{1.0, 1.0, 0.0};
    graph.nodes[4].position = Point3D{2.0, 1.0, 0.0};

    graph.nodes[1].manufacturing.support_risk = 1.0;
    graph.nodes[1].manufacturing.heat = 1.0;
    graph.nodes[2].manufacturing.support_risk = 1.0;
    graph.nodes[2].manufacturing.heat = 0.8;
    graph.nodes[3].path_trust = 0.5;
    graph.nodes[4].path_trust = 0.6;

    graph.edges = {
        AdaptiveGeodesicEdge{0, 1, 1.0, 0.0, 0.0, 0, false, 0.8, 0.8, 0.0, 0.0, true},
        AdaptiveGeodesicEdge{1, 2, 1.0, 0.0, 0.0, 0, false, 0.7, 0.8, 0.0, 0.0, true},
        AdaptiveGeodesicEdge{0, 3, 1.35, 0.0, 0.0, 0, false, 0.0, 0.1, 0.0, 0.2, true},
        AdaptiveGeodesicEdge{3, 4, 1.0, 0.0, 0.0, 0, false, 0.0, 0.1, 0.0, 0.2, true},
        AdaptiveGeodesicEdge{4, 2, 1.0, 0.0, 0.0, 0, false, 0.1, 0.1, 0.0, 0.2, true}};

    return graph;
}

} // namespace adaptivecad::geometry