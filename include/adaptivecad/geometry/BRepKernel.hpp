#pragma once

#include "adaptivecad/geometry/BRepEntities.hpp"
#include "adaptivecad/geometry/GeometryAdapter.hpp"

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace adaptivecad::geometry {

enum class TopologyHealingPolicy {
    Strict,
    RepairFirst,
    RepairOnly
};

struct TopologyDiagnostic {
    bool success = true;
    std::string operation;
    std::string backend;
    std::string message;
    TopologyHealingPolicy healing_policy = TopologyHealingPolicy::RepairFirst;
    bool healing_attempted = false;
    bool healing_succeeded = false;
    bool edges_reordered = false;
    bool bridge_edges_created = false;
    bool non_planar_input_detected = false;
    std::vector<EntityId> healing_edge_ids;
    std::vector<EntityId> input_edge_ids;
    std::vector<EntityId> invalid_edge_ids;
    int occ_wire_error_code = -1;
    int occ_face_error_code = -1;
    bool occ_wire_closed = true;
    bool occ_wire_valid = true;
    bool occ_face_valid = true;
};

class BRepKernel {
public:
    explicit BRepKernel(std::unique_ptr<GeometryAdapter> geometryAdapter);

    static BRepKernel create_preferred(bool preferOpenCascade = true);

    const GeometryAdapter& geometry() const;
    std::string backend_name() const;
    bool has_open_cascade_topology() const noexcept;
    const TopologyDiagnostic& last_topology_diagnostic() const noexcept;
    void set_healing_policy(TopologyHealingPolicy policy) noexcept;
    TopologyHealingPolicy healing_policy() const noexcept;
    static const char* healing_policy_name(TopologyHealingPolicy policy) noexcept;

    Vertex create_vertex(double x, double y, double z);
    Edge create_edge(const Vertex& start, const Vertex& end);
    Face create_face_from_edges(const std::vector<Edge>& edges, double areaHint = 0.0);
    Body create_body_from_faces(const std::vector<Face>& faces, double volumeHint = 0.0);
    Body create_torus(
        double majorRadius,
        double minorRadius,
        std::size_t majorSegments = 32,
        std::size_t minorSegments = 16);

    std::size_t vertex_count() const noexcept;
    std::size_t edge_count() const noexcept;
    std::size_t face_count() const noexcept;
    std::size_t body_count() const noexcept;

private:
    const Vertex* find_vertex(EntityId id) const noexcept;
    bool validate_edge_loop_strict(const std::vector<Edge>& inputEdges, std::string* failureReason) const;
    bool reorder_and_repair_edge_loop(const std::vector<Edge>& inputEdges, std::vector<Edge>* outputEdges,
        std::vector<EntityId>* healingEdgeIds, std::string* failureReason);
    bool is_loop_non_planar(const std::vector<Edge>& edges, double tolerance) const;
    void reset_topology_diagnostic(const char* operation);
    bool is_open_cascade_backend() const noexcept;
    EntityId next_id();

    std::unique_ptr<GeometryAdapter> geometryAdapter_;
    EntityId nextId_ = 1;
    std::vector<Vertex> vertices_;
    std::vector<Edge> edges_;
    std::vector<Face> faces_;
    std::vector<Body> bodies_;
    TopologyHealingPolicy healingPolicy_ = TopologyHealingPolicy::RepairFirst;
    TopologyDiagnostic lastTopologyDiagnostic_{};
};

} // namespace adaptivecad::geometry
