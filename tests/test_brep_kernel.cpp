#include "adaptivecad/geometry/BRepKernel.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

bool near(double a, double b, double tol) {
    return std::abs(a - b) <= tol;
}

} // namespace

int main() {
    using adaptivecad::geometry::BRepKernel;
    using adaptivecad::geometry::Edge;
    using adaptivecad::geometry::Face;
    using adaptivecad::geometry::TopologyHealingPolicy;
    using adaptivecad::geometry::Vertex;

    BRepKernel kernel = BRepKernel::create_preferred(true);
    assert(kernel.healing_policy() == TopologyHealingPolicy::RepairFirst);

    const Vertex v1 = kernel.create_vertex(0.0, 0.0, 0.0);
    const Vertex v2 = kernel.create_vertex(1.0, 0.0, 0.0);
    const Vertex v3 = kernel.create_vertex(0.0, 1.0, 0.0);

    assert(v1.id != v2.id);
    assert(v2.id != v3.id);
    assert(kernel.vertex_count() == 3);

    const Edge e1 = kernel.create_edge(v1, v2);
    const Edge e2 = kernel.create_edge(v2, v3);
    const Edge e3 = kernel.create_edge(v3, v1);

    assert(near(e1.length, 1.0, 1e-12));
    assert(kernel.edge_count() == 3);

    const Face face = kernel.create_face_from_edges({e1, e2, e3}, 0.5);
    assert(face.id > 0);
    assert(face.edge_ids.size() == 3);
    assert(near(face.area, 0.5, 1e-12));
    assert(kernel.face_count() == 1);

    {
        const adaptivecad::geometry::TopologyDiagnostic& diagnostic = kernel.last_topology_diagnostic();
        assert(diagnostic.success);
        assert(diagnostic.operation == "create_face_from_edges");
        assert(diagnostic.healing_policy == TopologyHealingPolicy::RepairFirst);
        assert(diagnostic.healing_attempted);
        assert(diagnostic.healing_succeeded);
        assert(!diagnostic.edges_reordered);
        assert(!diagnostic.bridge_edges_created);
        assert(diagnostic.input_edge_ids.size() == 3);
    }

    {
        const Face reorderedFace = kernel.create_face_from_edges({e2, e1, e3}, 0.5);
        assert(reorderedFace.id > 0);

        const adaptivecad::geometry::TopologyDiagnostic& diagnostic = kernel.last_topology_diagnostic();
        assert(diagnostic.success);
        assert(diagnostic.healing_policy == TopologyHealingPolicy::RepairFirst);
        assert(diagnostic.healing_attempted);
        assert(diagnostic.healing_succeeded);
        assert(diagnostic.edges_reordered);
        assert(!diagnostic.bridge_edges_created);
        assert(diagnostic.healing_edge_ids.empty());
    }

    {
        kernel.set_healing_policy(TopologyHealingPolicy::Strict);
        assert(kernel.healing_policy() == TopologyHealingPolicy::Strict);

        bool strictRejected = false;
        try {
            (void)kernel.create_face_from_edges({e2, e1, e3}, 0.5);
        } catch (const std::runtime_error& err) {
            strictRejected = true;
            const std::string errorMessage = err.what();
            assert(errorMessage.find("strict") != std::string::npos);
            const adaptivecad::geometry::TopologyDiagnostic& diagnostic = kernel.last_topology_diagnostic();
            assert(!diagnostic.success);
            assert(diagnostic.healing_policy == TopologyHealingPolicy::Strict);
            assert(!diagnostic.healing_attempted);
        }
        assert(strictRejected);

        const Face strictFace = kernel.create_face_from_edges({e1, e2, e3}, 0.5);
        assert(strictFace.id > 0);
        const adaptivecad::geometry::TopologyDiagnostic& strictDiagnostic = kernel.last_topology_diagnostic();
        assert(strictDiagnostic.success);
        assert(strictDiagnostic.healing_policy == TopologyHealingPolicy::Strict);
        assert(!strictDiagnostic.healing_attempted);
    }

    kernel.set_healing_policy(TopologyHealingPolicy::RepairOnly);
    assert(kernel.healing_policy() == TopologyHealingPolicy::RepairOnly);

    {
        const Vertex v4 = kernel.create_vertex(0.0, 0.0, 1e-7);
        const Edge e4 = kernel.create_edge(v3, v4);

        const Face bridgedFace = kernel.create_face_from_edges({e1, e2, e4}, 0.5);
        assert(bridgedFace.id > 0);
        assert(bridgedFace.edge_ids.size() == 4);

        const adaptivecad::geometry::TopologyDiagnostic& diagnostic = kernel.last_topology_diagnostic();
        assert(diagnostic.success);
        assert(diagnostic.healing_policy == TopologyHealingPolicy::RepairOnly);
        assert(diagnostic.healing_attempted);
        assert(diagnostic.healing_succeeded);
        assert(diagnostic.bridge_edges_created);
        assert(diagnostic.healing_edge_ids.size() == 1);
    }

    kernel.set_healing_policy(TopologyHealingPolicy::RepairFirst);

    const auto body = kernel.create_body_from_faces({face}, 0.0);
    assert(body.id > 0);
    assert(body.face_ids.size() == 1);
    assert(kernel.body_count() == 1);

    const auto torus = kernel.create_torus(2.0, 0.5, 12, 8);
    assert(torus.id > 0);
    assert(torus.volume > 0.0);
    assert(near(torus.volume, 2.0 * std::numbers::pi_v<double> * std::numbers::pi_v<double> * 2.0 * 0.5 * 0.5, 1e-9));
    assert(kernel.body_count() == 2);

    if (kernel.has_open_cascade_topology()) {
        assert(torus.native_handle.is_valid());
        assert(torus.native_handle.type_name == "TopoDS_Shape");
    } else {
        assert(!torus.native_handle.is_valid());
        assert(torus.face_ids.size() == 12 * 8);
    }

    bool invalidTorusRejected = false;
    try {
        (void)kernel.create_torus(1.0, 1.0);
    } catch (const std::invalid_argument&) {
        invalidTorusRejected = true;
    }
    assert(invalidTorusRejected);

    if (kernel.has_open_cascade_topology()) {
        assert(v1.native_handle.is_valid());
        assert(e1.native_handle.is_valid());
        assert(face.native_handle.is_valid());
        assert(body.native_handle.is_valid());

        assert(e1.native_handle.type_name == "TopoDS_Edge");
        assert(face.native_handle.type_name == "TopoDS_Face");
        assert(body.native_handle.type_name == "TopoDS_Compound");

        Edge brokenEdge = e2;
        brokenEdge.native_handle = {};

        bool sawStrictFailure = false;
        try {
            (void)kernel.create_face_from_edges({e1, brokenEdge, e3}, 0.0);
        } catch (const std::runtime_error& err) {
            sawStrictFailure = true;
            const std::string errorMessage = err.what();
            assert(errorMessage.find("missing native TopoDS_Edge handles") != std::string::npos);

            const adaptivecad::geometry::TopologyDiagnostic& diagnostic = kernel.last_topology_diagnostic();
            assert(!diagnostic.success);
            assert(diagnostic.operation == "create_face_from_edges");
            assert(diagnostic.invalid_edge_ids.size() == 1);
            assert(diagnostic.invalid_edge_ids[0] == brokenEdge.id);
        }

        assert(sawStrictFailure);
    } else {
        assert(!v1.native_handle.is_valid());
        assert(!e1.native_handle.is_valid());
        assert(!face.native_handle.is_valid());
        assert(!body.native_handle.is_valid());
    }

    return 0;
}
