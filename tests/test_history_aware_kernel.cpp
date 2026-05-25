#include "adaptivecad/geometry/BRepKernel.hpp"
#include "adaptivecad/geometry/HistoryAwareKernel.hpp"

#include <cassert>
#include <cmath>
#include <vector>

namespace {

bool near(double left, double right, double tolerance) {
    return std::abs(left - right) <= tolerance;
}

} // namespace

int main() {
    using adaptivecad::geometry::AdaptiveCADState;
    using adaptivecad::geometry::AdaptiveGeodesicEdge;
    using adaptivecad::geometry::AdaptiveGeodesicGraph;
    using adaptivecad::geometry::BRepKernel;
    using adaptivecad::geometry::PathCostWeights;
    using adaptivecad::geometry::PhaseLiftState;
    using adaptivecad::geometry::Point3D;
    using adaptivecad::geometry::TopologyHealingPolicy;
    using adaptivecad::geometry::kEuclideanPi;

    {
        const auto correction = adaptivecad::geometry::make_flat_pi_correction(0.10);
        assert(near(correction.pi_f, kEuclideanPi * 1.10, 1e-12));
        assert(correction.adaptive_metric.xx > 1.0);
        assert(near(adaptivecad::geometry::power_law_pi_f(2.0, 1.2, 1.0, 0.5),
            kEuclideanPi * 1.2 * std::sqrt(2.0), 1e-12));
        assert(near(adaptivecad::geometry::radial_operator_effective_dimension(0.25), 2.25, 1e-12));
        assert(near(adaptivecad::geometry::radial_laplacian_first_derivative_weight(2.0, 0.25), 0.625, 1e-12));
    }

    {
        PhaseLiftState phase;
        phase.theta_R = 1.75 * kEuclideanPi;
        const PhaseLiftState advanced = adaptivecad::geometry::advance_phase_lift(phase, 0.75 * kEuclideanPi);
        assert(advanced.winding == 1);
        assert(advanced.branch_parity);
        assert(near(advanced.theta_R, 0.5 * kEuclideanPi, 1e-12));
    }

    {
        assert(near(adaptivecad::geometry::update_curve_memory(2.0, 3.0, 0.5, 0.25), 2.5, 1e-12));
        assert(near(adaptivecad::geometry::update_path_trust(1.0, -2.0, 0.4, 0.1, 0.5), 1.35, 1e-12));
        assert(near(adaptivecad::geometry::infer_pi_f_from_area_growth(2.0, 4.0 * kEuclideanPi),
            kEuclideanPi, 1e-12));
        assert(near(adaptivecad::geometry::infer_pi_f_from_radial_flux(2.0, 8.0 * kEuclideanPi, 4.0),
            kEuclideanPi / 2.0, 1e-12));
    }

    {
        BRepKernel kernel = BRepKernel::create_preferred(true);
        const auto vertex = kernel.create_vertex(1.0, 2.0, 3.0);
        assert(near(vertex.adaptive_state.position.x, 1.0, 1e-12));
        assert(near(vertex.adaptive_state.pi_a, kEuclideanPi, 1e-12));

        const auto otherVertex = kernel.create_vertex(3.0, 2.0, 3.0);
        const auto edge = kernel.create_edge(vertex, otherVertex);
        assert(near(edge.adaptive_state.position.x, 2.0, 1e-12));

        const auto thirdVertex = kernel.create_vertex(1.0, 4.0, 3.0);
        const auto edgeTwo = kernel.create_edge(otherVertex, thirdVertex);
        const auto edgeThree = kernel.create_edge(thirdVertex, vertex);
        kernel.set_healing_policy(TopologyHealingPolicy::Strict);
        const auto face = kernel.create_face_from_edges({edge, edgeTwo, edgeThree}, 2.0);
        assert(near(face.adaptive_state.pi_f, kEuclideanPi, 1e-12));
    }

    {
        AdaptiveGeodesicGraph graph;
        graph.nodes.resize(4);
        graph.nodes[0].position = Point3D{0.0, 0.0, 0.0};
        graph.nodes[1].position = Point3D{1.0, 0.0, 0.0};
        graph.nodes[2].position = Point3D{2.0, 0.0, 0.0};
        graph.nodes[3].position = Point3D{1.0, 1.0, 0.0};
        graph.nodes[1].residual = 10.0;
        graph.nodes[2].residual = 10.0;
        graph.nodes[3].path_trust = 2.0;
        graph.edges = {
            AdaptiveGeodesicEdge{0, 1, 1.0},
            AdaptiveGeodesicEdge{1, 2, 1.0},
            AdaptiveGeodesicEdge{0, 3, 1.4, 0.0, 0.0, 0, false, 0.0, 0.0, 0.0, 0.5},
            AdaptiveGeodesicEdge{3, 2, 1.4, 0.0, 0.0, 0, false, 0.0, 0.0, 0.0, 0.5}};

        PathCostWeights weights;
        weights.residual = 1.0;
        weights.trust_reward = 0.1;
        const auto result = adaptivecad::geometry::compute_adaptive_geodesic(graph, 0, 2, weights);
        assert(result.found);
        const std::vector<std::size_t> expectedPath = {0, 3, 2};
        assert(result.node_path == expectedPath);
    }

    {
        const auto torusGraph = adaptivecad::geometry::make_torus_phase_lift_benchmark(8, 2.0);
        assert(torusGraph.nodes.size() == 8);
        assert(torusGraph.edges.size() == 8);
        assert(torusGraph.edges.back().winding_delta == 1);
        assert(torusGraph.edges.back().parity_flip);
    }

    {
        const auto benchyGraph = adaptivecad::geometry::make_benchy_manufacturing_benchmark();
        PathCostWeights weights;
        weights.support = 4.0;
        weights.heat = 3.0;
        weights.trust_reward = 0.2;
        const auto result = adaptivecad::geometry::compute_adaptive_geodesic(benchyGraph, 0, 2, weights);
        assert(result.found);
        const std::vector<std::size_t> expectedPath = {0, 3, 4, 2};
        assert(result.node_path == expectedPath);
    }

    return 0;
}