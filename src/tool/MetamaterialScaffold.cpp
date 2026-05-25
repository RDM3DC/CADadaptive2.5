#include "adaptivecad/tool/MetamaterialScaffold.hpp"

#include "adaptivecad/core/AdaptiveField.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace adaptivecad::tool {
namespace {

struct GridNode {
    geometry::Point3D point{};
    bool is_left_anchor = false;
    bool is_right_anchor = false;
};

struct GraphArc {
    std::size_t neighbor_index = 0;
    double length = 0.0;
};

struct CandidateStrut {
    std::size_t start_node = 0;
    std::size_t end_node = 0;
    ScaffoldStrutAxis axis = ScaffoldStrutAxis::X;
    double length = 0.0;
    bool removed = false;
};

double squared_distance(const geometry::Point3D& left, const geometry::Point3D& right) {
    const double dx = left.x - right.x;
    const double dy = left.y - right.y;
    const double dz = left.z - right.z;
    return dx * dx + dy * dy + dz * dz;
}

geometry::Point3D subtract_point(const geometry::Point3D& left, const geometry::Point3D& right) {
    return geometry::Point3D{left.x - right.x, left.y - right.y, left.z - right.z};
}

double dot_product(const geometry::Point3D& left, const geometry::Point3D& right) {
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

double point_segment_distance_squared(
    const geometry::Point3D& point,
    const geometry::Point3D& segment_start,
    const geometry::Point3D& segment_end) {
    const geometry::Point3D segment = subtract_point(segment_end, segment_start);
    const geometry::Point3D relative = subtract_point(point, segment_start);
    const double segment_norm_sq = dot_product(segment, segment);
    if (segment_norm_sq <= 0.0) {
        return squared_distance(point, segment_start);
    }

    double projection = dot_product(relative, segment) / segment_norm_sq;
    projection = std::clamp(projection, 0.0, 1.0);
    const geometry::Point3D closest{
        segment_start.x + segment.x * projection,
        segment_start.y + segment.y * projection,
        segment_start.z + segment.z * projection};
    return squared_distance(point, closest);
}

std::size_t grid_index(int x, int y, int z, int node_count_y, int node_count_z) {
    return static_cast<std::size_t>((x * node_count_y + y) * node_count_z + z);
}

double evaluate_directional_lambda(
    const core::AdaptiveField& angular_field,
    const core::AngularWeightModel& angular_model,
    ScaffoldStrutAxis axis) {
    switch (axis) {
    case ScaffoldStrutAxis::X:
        return angular_field.lambda_theta(0.0);
    case ScaffoldStrutAxis::Y:
        return angular_field.lambda_theta(std::numbers::pi_v<double> / 2.0);
    case ScaffoldStrutAxis::Z:
        return angular_model.lambda0;
    }

    return angular_model.lambda0;
}

double compute_directional_gain(
    const core::AdaptiveField& angular_field,
    const MetamaterialScaffoldParameters& parameters,
    ScaffoldStrutAxis axis) {
    const double lambda0 = std::max(parameters.angular_model.lambda0, 1e-9);
    const double directional_lambda = evaluate_directional_lambda(angular_field, parameters.angular_model, axis);
    const double normalized_lambda = directional_lambda / lambda0;
    return std::max(0.1, 1.0 + parameters.angular_gain * (normalized_lambda - 1.0));
}

std::vector<double> compute_shortest_path_distances(
    const std::vector<std::vector<GraphArc>>& adjacency,
    const std::vector<std::size_t>& sources) {
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> distances(adjacency.size(), infinity);

    using QueueEntry = std::pair<double, std::size_t>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
    for (std::size_t source : sources) {
        if (source >= adjacency.size()) {
            continue;
        }
        distances[source] = 0.0;
        queue.push(QueueEntry{0.0, source});
    }

    while (!queue.empty()) {
        const auto [distance, node_index_value] = queue.top();
        queue.pop();
        if (distance > distances[node_index_value]) {
            continue;
        }

        for (const auto& arc : adjacency[node_index_value]) {
            const double candidate_distance = distance + arc.length;
            if (candidate_distance + 1e-12 < distances[arc.neighbor_index]) {
                distances[arc.neighbor_index] = candidate_distance;
                queue.push(QueueEntry{candidate_distance, arc.neighbor_index});
            }
        }
    }

    return distances;
}

SceneBody make_prism_body(
    const geometry::Point3D& start,
    const geometry::Point3D& end,
    ScaffoldStrutAxis axis,
    double thickness,
    double backbone_score,
    std::size_t strut_index) {
    const double half_thickness = thickness * 0.5;
    double min_x = std::min(start.x, end.x);
    double max_x = std::max(start.x, end.x);
    double min_y = std::min(start.y, end.y);
    double max_y = std::max(start.y, end.y);
    double min_z = std::min(start.z, end.z);
    double max_z = std::max(start.z, end.z);

    switch (axis) {
    case ScaffoldStrutAxis::X:
        min_y = start.y - half_thickness;
        max_y = start.y + half_thickness;
        min_z = start.z - half_thickness;
        max_z = start.z + half_thickness;
        break;
    case ScaffoldStrutAxis::Y:
        min_x = start.x - half_thickness;
        max_x = start.x + half_thickness;
        min_z = start.z - half_thickness;
        max_z = start.z + half_thickness;
        break;
    case ScaffoldStrutAxis::Z:
        min_x = start.x - half_thickness;
        max_x = start.x + half_thickness;
        min_y = start.y - half_thickness;
        max_y = start.y + half_thickness;
        break;
    }

    SceneBody body;
    body.name = backbone_score >= 0.95
        ? "Transport Backbone Strut " + std::to_string(strut_index + 1)
        : "Metamaterial Strut " + std::to_string(strut_index + 1);
    body.vertices = {
        geometry::Point3D{min_x, min_y, min_z},
        geometry::Point3D{max_x, min_y, min_z},
        geometry::Point3D{max_x, max_y, min_z},
        geometry::Point3D{min_x, max_y, min_z},
        geometry::Point3D{min_x, min_y, max_z},
        geometry::Point3D{max_x, min_y, max_z},
        geometry::Point3D{max_x, max_y, max_z},
        geometry::Point3D{min_x, max_y, max_z}};
    body.faces = {
        SceneFace{{0, 3, 2, 1}},
        SceneFace{{4, 5, 6, 7}},
        SceneFace{{0, 4, 7, 3}},
        SceneFace{{1, 2, 6, 5}},
        SceneFace{{0, 1, 5, 4}},
        SceneFace{{3, 7, 6, 2}}};
    body.volume_hint = std::max(0.0, max_x - min_x) * std::max(0.0, max_y - min_y) * std::max(0.0, max_z - min_z);
    return body;
}

SceneBody make_connector_body(
    const geometry::Point3D& center,
    double half_extent,
    double backbone_influence,
    std::size_t connector_index) {
    const double min_x = center.x - half_extent;
    const double max_x = center.x + half_extent;
    const double min_y = center.y - half_extent;
    const double max_y = center.y + half_extent;
    const double min_z = center.z - half_extent;
    const double max_z = center.z + half_extent;

    SceneBody body;
    body.name = backbone_influence >= 0.95
        ? "Transport Connector " + std::to_string(connector_index + 1)
        : "Metamaterial Connector " + std::to_string(connector_index + 1);
    body.vertices = {
        geometry::Point3D{min_x, min_y, min_z},
        geometry::Point3D{max_x, min_y, min_z},
        geometry::Point3D{max_x, max_y, min_z},
        geometry::Point3D{min_x, max_y, min_z},
        geometry::Point3D{min_x, min_y, max_z},
        geometry::Point3D{max_x, min_y, max_z},
        geometry::Point3D{max_x, max_y, max_z},
        geometry::Point3D{min_x, max_y, max_z}};
    body.faces = {
        SceneFace{{0, 3, 2, 1}},
        SceneFace{{4, 5, 6, 7}},
        SceneFace{{0, 4, 7, 3}},
        SceneFace{{1, 2, 6, 5}},
        SceneFace{{0, 1, 5, 4}},
        SceneFace{{3, 7, 6, 2}}};
    const double side = half_extent * 2.0;
    body.volume_hint = side * side * side;
    return body;
}

} // namespace

GeneratedMetamaterialScaffold generate_chern_locked_metamaterial_scaffold(
    const MetamaterialScaffoldParameters& parameters) {
    if (parameters.cells_x < 1) {
        throw std::invalid_argument("metamaterial scaffold requires cells_x >= 1");
    }
    if (parameters.cells_y < 0 || parameters.cells_z < 0) {
        throw std::invalid_argument("metamaterial scaffold requires non-negative cell counts");
    }
    if (parameters.spacing <= 0.0) {
        throw std::invalid_argument("metamaterial scaffold spacing must be positive");
    }
    if (parameters.base_thickness <= 0.0 || parameters.minimum_thickness <= 0.0) {
        throw std::invalid_argument("metamaterial scaffold thickness values must be positive");
    }
    if (parameters.damage_sphere.radius < 0.0) {
        throw std::invalid_argument("metamaterial scaffold damage radius cannot be negative");
    }

    const core::AdaptiveField angular_field = core::AdaptiveField::AngularWeight(
        parameters.angular_model.lambda0,
        parameters.angular_model.cosine_coefficients,
        parameters.angular_model.sine_coefficients);

    const int node_count_x = parameters.cells_x + 1;
    const int node_count_y = parameters.cells_y + 1;
    const int node_count_z = parameters.cells_z + 1;
    const geometry::Point3D origin_shift{
        -0.5 * static_cast<double>(parameters.cells_x) * parameters.spacing,
        -0.5 * static_cast<double>(parameters.cells_y) * parameters.spacing,
        -0.5 * static_cast<double>(parameters.cells_z) * parameters.spacing};

    std::vector<GridNode> nodes;
    nodes.reserve(static_cast<std::size_t>(node_count_x * node_count_y * node_count_z));
    std::vector<std::size_t> left_anchors;
    std::vector<std::size_t> right_anchors;

    for (int x = 0; x < node_count_x; ++x) {
        for (int y = 0; y < node_count_y; ++y) {
            for (int z = 0; z < node_count_z; ++z) {
                GridNode node;
                node.point = geometry::Point3D{
                    origin_shift.x + static_cast<double>(x) * parameters.spacing,
                    origin_shift.y + static_cast<double>(y) * parameters.spacing,
                    origin_shift.z + static_cast<double>(z) * parameters.spacing};
                node.is_left_anchor = (x == 0);
                node.is_right_anchor = (x == node_count_x - 1);

                const std::size_t index = nodes.size();
                nodes.push_back(node);
                if (node.is_left_anchor) {
                    left_anchors.push_back(index);
                }
                if (node.is_right_anchor) {
                    right_anchors.push_back(index);
                }
            }
        }
    }

    std::vector<CandidateStrut> candidate_struts;
    candidate_struts.reserve(static_cast<std::size_t>(parameters.cells_x * node_count_y * node_count_z)
        + static_cast<std::size_t>(node_count_x * parameters.cells_y * node_count_z)
        + static_cast<std::size_t>(node_count_x * node_count_y * parameters.cells_z));

    const double damage_radius_sq = parameters.damage_sphere.radius * parameters.damage_sphere.radius;
    auto append_candidate = [&](std::size_t start_node, std::size_t end_node, ScaffoldStrutAxis axis) {
        CandidateStrut strut;
        strut.start_node = start_node;
        strut.end_node = end_node;
        strut.axis = axis;
        strut.length = parameters.spacing;
        if (parameters.damage_sphere.radius > 0.0) {
            const double distance_sq = point_segment_distance_squared(
                parameters.damage_sphere.center,
                nodes[start_node].point,
                nodes[end_node].point);
            strut.removed = distance_sq <= damage_radius_sq;
        }
        candidate_struts.push_back(strut);
    };

    for (int x = 0; x < node_count_x; ++x) {
        for (int y = 0; y < node_count_y; ++y) {
            for (int z = 0; z < node_count_z; ++z) {
                const std::size_t start_node = grid_index(x, y, z, node_count_y, node_count_z);
                if (x + 1 < node_count_x) {
                    append_candidate(start_node, grid_index(x + 1, y, z, node_count_y, node_count_z), ScaffoldStrutAxis::X);
                }
                if (y + 1 < node_count_y) {
                    append_candidate(start_node, grid_index(x, y + 1, z, node_count_y, node_count_z), ScaffoldStrutAxis::Y);
                }
                if (z + 1 < node_count_z) {
                    append_candidate(start_node, grid_index(x, y, z + 1, node_count_y, node_count_z), ScaffoldStrutAxis::Z);
                }
            }
        }
    }

    std::vector<std::vector<GraphArc>> adjacency(nodes.size());
    std::vector<bool> node_has_removed_incident_strut(nodes.size(), false);
    std::size_t removed_strut_count = 0;
    for (const auto& strut : candidate_struts) {
        if (strut.removed) {
            ++removed_strut_count;
            node_has_removed_incident_strut[strut.start_node] = true;
            node_has_removed_incident_strut[strut.end_node] = true;
            continue;
        }

        adjacency[strut.start_node].push_back(GraphArc{strut.end_node, strut.length});
        adjacency[strut.end_node].push_back(GraphArc{strut.start_node, strut.length});
    }

    const std::vector<double> left_distances = compute_shortest_path_distances(adjacency, left_anchors);
    const std::vector<double> right_distances = compute_shortest_path_distances(adjacency, right_anchors);
    const double infinity = std::numeric_limits<double>::infinity();

    double transport_span = infinity;
    for (std::size_t anchor : right_anchors) {
        transport_span = std::min(transport_span, left_distances[anchor]);
    }

    SceneDocument document;
    document.scene_label = parameters.scene_label;
    document.source_kind = SceneSourceKind::GeneratedScaffold;
    document.angular_model = parameters.angular_model;

    GeneratedMetamaterialScaffold result;
    result.node_count = nodes.size();
    result.candidate_strut_count = candidate_struts.size();
    result.removed_strut_count = removed_strut_count;
    result.transport_span = std::isfinite(transport_span) ? transport_span : 0.0;

    const double decay_length = std::max(parameters.minimum_thickness, parameters.backbone_decay_length * parameters.spacing);
    std::vector<std::vector<std::size_t>> adjacent_struts_by_node(nodes.size());
    for (std::size_t index = 0; index < candidate_struts.size(); ++index) {
        const auto& strut = candidate_struts[index];
        if (strut.removed) {
            continue;
        }

        const double start_left = left_distances[strut.start_node];
        const double end_left = left_distances[strut.end_node];
        const double start_right = right_distances[strut.start_node];
        const double end_right = right_distances[strut.end_node];

        double backbone_score = 0.0;
        if (std::isfinite(transport_span)) {
            const double route_a = start_left + strut.length + end_right;
            const double route_b = end_left + strut.length + start_right;
            const double best_route = std::min(route_a, route_b);
            if (std::isfinite(best_route)) {
                const double slack = std::max(0.0, best_route - transport_span);
                backbone_score = std::exp(-slack / decay_length);
            }
        }

        const double directional_gain = compute_directional_gain(angular_field, parameters, strut.axis);
        const double conductance = directional_gain * (1.0 + parameters.backbone_gain * backbone_score);
        const double thickness = std::max(parameters.minimum_thickness, parameters.base_thickness * conductance);

        const geometry::Point3D& start = nodes[strut.start_node].point;
        const geometry::Point3D& end = nodes[strut.end_node].point;
        const std::size_t strutDescriptorIndex = result.struts.size();
        result.struts.push_back(ScaffoldStrutDescriptor{start, end, strut.axis, conductance, backbone_score, thickness});
        adjacent_struts_by_node[strut.start_node].push_back(strutDescriptorIndex);
        adjacent_struts_by_node[strut.end_node].push_back(strutDescriptorIndex);
        document.bodies.push_back(make_prism_body(start, end, strut.axis, thickness, backbone_score, document.bodies.size()));
    }

    if (document.bodies.empty()) {
        throw std::runtime_error("metamaterial scaffold generation produced no surviving struts");
    }

    for (std::size_t nodeIndex = 0; nodeIndex < nodes.size(); ++nodeIndex) {
        const std::vector<std::size_t>& adjacentStruts = adjacent_struts_by_node[nodeIndex];
        if (adjacentStruts.empty()) {
            continue;
        }

        double maxThickness = parameters.minimum_thickness;
        double maxBackbone = 0.0;
        for (std::size_t strutIndex : adjacentStruts) {
            maxThickness = std::max(maxThickness, result.struts[strutIndex].thickness);
            maxBackbone = std::max(maxBackbone, result.struts[strutIndex].backbone_score);
        }

        ScaffoldConnectorDescriptor connector;
        connector.center = nodes[nodeIndex].point;
        connector.half_extent = std::max(parameters.minimum_thickness * 0.5, maxThickness * 0.60);
        connector.adjacent_strut_indices = adjacentStruts;
        connector.backbone_influence = maxBackbone;
        connector.boundary_node = nodes[nodeIndex].is_left_anchor || nodes[nodeIndex].is_right_anchor;
        connector.damage_adjacent = node_has_removed_incident_strut[nodeIndex];

        const std::size_t connectorIndex = result.connectors.size();
        result.connectors.push_back(connector);
        document.bodies.push_back(make_connector_body(
            connector.center,
            connector.half_extent,
            connector.backbone_influence,
            connectorIndex));
    }

    result.scene_document = std::move(document);
    return result;
}

} // namespace adaptivecad::tool