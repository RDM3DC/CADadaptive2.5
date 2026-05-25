#include "adaptivecad/geometry/BRepKernel.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#ifdef ADAPTIVECAD_HAS_OPENCASCADE
#include <BRep_Builder.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <ShapeFix_Face.hxx>
#include <ShapeFix_Wire.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS.hxx>
#include <gp_Pnt.hxx>
#endif

namespace adaptivecad::geometry {
namespace {

constexpr double kTopologyHealTolerance = 1e-6;
constexpr double kPlanarityTolerance = 1e-6;
constexpr double kTwoPi = 2.0 * std::numbers::pi_v<double>;

double distance(const Point3D& a, const Point3D& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double torus_volume(double majorRadius, double minorRadius) {
    return 2.0 * std::numbers::pi_v<double> * std::numbers::pi_v<double>
        * majorRadius * minorRadius * minorRadius;
}

double torus_patch_area_hint(double majorRadius, double minorRadius, std::size_t majorSegments, std::size_t minorSegments) {
    return 4.0 * std::numbers::pi_v<double> * std::numbers::pi_v<double> * majorRadius * minorRadius
        / static_cast<double>(majorSegments * minorSegments);
}

Point3D torus_point(double majorRadius, double minorRadius, std::size_t uIndex, std::size_t vIndex,
    std::size_t majorSegments, std::size_t minorSegments) {
    const double u = kTwoPi * static_cast<double>(uIndex) / static_cast<double>(majorSegments);
    const double v = kTwoPi * static_cast<double>(vIndex) / static_cast<double>(minorSegments);
    const double radial = majorRadius + minorRadius * std::cos(v);
    return Point3D{
        radial * std::cos(u),
        radial * std::sin(u),
        minorRadius * std::sin(v)};
}

std::string join_entity_ids(const std::vector<EntityId>& ids) {
    std::ostringstream output;
    for (std::size_t i = 0; i < ids.size(); ++i) {
        if (i > 0) {
            output << ",";
        }
        output << ids[i];
    }
    return output.str();
}

std::string format_topology_diagnostic(const TopologyDiagnostic& diagnostic) {
    std::ostringstream output;
    output << diagnostic.operation << " failed";
    if (!diagnostic.message.empty()) {
        output << ": " << diagnostic.message;
    }
    if (!diagnostic.backend.empty()) {
        output << " | backend=" << diagnostic.backend;
    }
    output << " | healing_policy=" << BRepKernel::healing_policy_name(diagnostic.healing_policy);
    if (!diagnostic.input_edge_ids.empty()) {
        output << " | input_edge_ids=[" << join_entity_ids(diagnostic.input_edge_ids) << "]";
    }
    if (!diagnostic.invalid_edge_ids.empty()) {
        output << " | invalid_edge_ids=[" << join_entity_ids(diagnostic.invalid_edge_ids) << "]";
    }
    if (!diagnostic.healing_edge_ids.empty()) {
        output << " | healing_edge_ids=[" << join_entity_ids(diagnostic.healing_edge_ids) << "]";
    }
    output << " | healing_attempted=" << (diagnostic.healing_attempted ? "true" : "false");
    output << " | healing_succeeded=" << (diagnostic.healing_succeeded ? "true" : "false");
    output << " | edges_reordered=" << (diagnostic.edges_reordered ? "true" : "false");
    output << " | bridge_edges_created=" << (diagnostic.bridge_edges_created ? "true" : "false");
    output << " | non_planar_input_detected=" << (diagnostic.non_planar_input_detected ? "true" : "false");
    if (diagnostic.occ_wire_error_code >= 0) {
        output << " | occ_wire_error_code=" << diagnostic.occ_wire_error_code;
    }
    if (diagnostic.occ_face_error_code >= 0) {
        output << " | occ_face_error_code=" << diagnostic.occ_face_error_code;
    }
    output << " | occ_wire_closed=" << (diagnostic.occ_wire_closed ? "true" : "false");
    output << " | occ_wire_valid=" << (diagnostic.occ_wire_valid ? "true" : "false");
    output << " | occ_face_valid=" << (diagnostic.occ_face_valid ? "true" : "false");
    return output.str();
}

[[noreturn]] void throw_topology_diagnostic_error(const TopologyDiagnostic& diagnostic) {
    throw std::runtime_error(format_topology_diagnostic(diagnostic));
}

#ifdef ADAPTIVECAD_HAS_OPENCASCADE
template <typename T>
NativeTopologyHandle make_occ_handle(const T& value, const char* typeName) {
    NativeTopologyHandle native;
    native.backend = NativeTopologyBackend::OpenCascade;
    native.type_name = typeName;
    native.handle = std::static_pointer_cast<void>(std::make_shared<T>(value));
    return native;
}

template <typename T>
std::shared_ptr<T> extract_occ_handle(const NativeTopologyHandle& native, const char* typeName) {
    if (native.backend != NativeTopologyBackend::OpenCascade || !native.is_valid()) {
        return {};
    }

    if (!native.type_name.empty() && native.type_name != typeName) {
        return {};
    }

    return std::static_pointer_cast<T>(native.handle);
}
#endif

} // namespace

BRepKernel::BRepKernel(std::unique_ptr<GeometryAdapter> geometryAdapter)
    : geometryAdapter_(std::move(geometryAdapter)) {
    if (!geometryAdapter_) {
        throw std::invalid_argument("BRepKernel requires a non-null GeometryAdapter");
    }
    reset_topology_diagnostic("none");
}

BRepKernel BRepKernel::create_preferred(bool preferOpenCascade) {
    return BRepKernel(create_preferred_geometry_adapter(preferOpenCascade));
}

const GeometryAdapter& BRepKernel::geometry() const {
    return *geometryAdapter_;
}

std::string BRepKernel::backend_name() const {
    return geometryAdapter_->backend_name();
}

bool BRepKernel::has_open_cascade_topology() const noexcept {
#ifdef ADAPTIVECAD_HAS_OPENCASCADE
    return is_open_cascade_backend();
#else
    return false;
#endif
}

const TopologyDiagnostic& BRepKernel::last_topology_diagnostic() const noexcept {
    return lastTopologyDiagnostic_;
}

void BRepKernel::set_healing_policy(TopologyHealingPolicy policy) noexcept {
    healingPolicy_ = policy;
}

TopologyHealingPolicy BRepKernel::healing_policy() const noexcept {
    return healingPolicy_;
}

const char* BRepKernel::healing_policy_name(TopologyHealingPolicy policy) noexcept {
    switch (policy) {
    case TopologyHealingPolicy::Strict:
        return "Strict";
    case TopologyHealingPolicy::RepairFirst:
        return "RepairFirst";
    case TopologyHealingPolicy::RepairOnly:
        return "RepairOnly";
    }

    return "Unknown";
}

Vertex BRepKernel::create_vertex(double x, double y, double z) {
    Vertex vertex;
    vertex.id = next_id();
    vertex.point = geometryAdapter_->make_point(x, y, z);
    vertex.adaptive_state.position = vertex.point;

#ifdef ADAPTIVECAD_HAS_OPENCASCADE
    if (is_open_cascade_backend()) {
        BRepBuilderAPI_MakeVertex vertexMaker(gp_Pnt(vertex.point.x, vertex.point.y, vertex.point.z));
        if (vertexMaker.IsDone()) {
            vertex.native_handle = make_occ_handle(vertexMaker.Vertex(), "TopoDS_Vertex");
        }
    }
#endif

    vertices_.push_back(vertex);
    return vertex;
}

Edge BRepKernel::create_edge(const Vertex& start, const Vertex& end) {
    if (start.id == 0 || end.id == 0) {
        throw std::invalid_argument("create_edge requires valid vertices");
    }
    if (start.id == end.id) {
        throw std::invalid_argument("create_edge requires distinct vertices");
    }

    Edge edge;
    edge.id = next_id();
    edge.start_vertex_id = start.id;
    edge.end_vertex_id = end.id;
    edge.length = distance(start.point, end.point);
    edge.adaptive_state.position = Point3D{
        0.5 * (start.point.x + end.point.x),
        0.5 * (start.point.y + end.point.y),
        0.5 * (start.point.z + end.point.z)};

#ifdef ADAPTIVECAD_HAS_OPENCASCADE
    if (is_open_cascade_backend()) {
        const gp_Pnt startPoint(start.point.x, start.point.y, start.point.z);
        const gp_Pnt endPoint(end.point.x, end.point.y, end.point.z);
        BRepBuilderAPI_MakeEdge edgeMaker(startPoint, endPoint);
        if (edgeMaker.IsDone()) {
            edge.native_handle = make_occ_handle(edgeMaker.Edge(), "TopoDS_Edge");
        }
    }
#endif

    edges_.push_back(edge);
    return edge;
}

const Vertex* BRepKernel::find_vertex(EntityId id) const noexcept {
    for (const Vertex& vertex : vertices_) {
        if (vertex.id == id) {
            return &vertex;
        }
    }
    return nullptr;
}

bool BRepKernel::validate_edge_loop_strict(const std::vector<Edge>& inputEdges, std::string* failureReason) const {
    if (failureReason) {
        failureReason->clear();
    }

    if (inputEdges.empty()) {
        if (failureReason) {
            *failureReason = "no edges provided";
        }
        return false;
    }

    for (std::size_t i = 0; i < inputEdges.size(); ++i) {
        const Edge& current = inputEdges[i];
        if (current.id == 0 || current.start_vertex_id == 0 || current.end_vertex_id == 0) {
            if (failureReason) {
                *failureReason = "strict loop contains invalid edge ids or vertex ids";
            }
            return false;
        }

        const std::size_t nextIndex = (i + 1) % inputEdges.size();
        const Edge& next = inputEdges[nextIndex];
        if (current.end_vertex_id != next.start_vertex_id) {
            if (failureReason) {
                *failureReason = "strict loop requires contiguous edge orientation and closure";
            }
            return false;
        }
    }

    return true;
}

bool BRepKernel::reorder_and_repair_edge_loop(const std::vector<Edge>& inputEdges, std::vector<Edge>* outputEdges,
    std::vector<EntityId>* healingEdgeIds, std::string* failureReason) {
    if (!outputEdges || !healingEdgeIds || !failureReason) {
        return false;
    }

    outputEdges->clear();
    healingEdgeIds->clear();
    failureReason->clear();

    if (inputEdges.empty()) {
        *failureReason = "no edges provided";
        return false;
    }

    auto orient_edge_for_connection = [](const Edge& source, bool reverse) {
        Edge oriented = source;
        if (!reverse) {
            return oriented;
        }

        std::swap(oriented.start_vertex_id, oriented.end_vertex_id);

#ifdef ADAPTIVECAD_HAS_OPENCASCADE
        const std::shared_ptr<TopoDS_Edge> occEdge =
            extract_occ_handle<TopoDS_Edge>(source.native_handle, "TopoDS_Edge");
        if (occEdge) {
            const TopoDS_Edge reversedOccEdge = TopoDS::Edge(occEdge->Reversed());
            oriented.native_handle = make_occ_handle(reversedOccEdge, "TopoDS_Edge");
        }
#endif

        return oriented;
    };

    std::vector<Edge> remaining = inputEdges;
    std::vector<Edge> ordered;
    ordered.reserve(inputEdges.size() + 1);

    ordered.push_back(remaining.front());
    remaining.erase(remaining.begin());

    while (!remaining.empty()) {
        const EntityId expectedStart = ordered.back().end_vertex_id;

        bool found = false;
        for (std::size_t i = 0; i < remaining.size(); ++i) {
            const Edge& candidate = remaining[i];
            if (candidate.start_vertex_id == expectedStart) {
                ordered.push_back(orient_edge_for_connection(candidate, false));
                remaining.erase(remaining.begin() + i);
                found = true;
                break;
            }
            if (candidate.end_vertex_id == expectedStart) {
                ordered.push_back(orient_edge_for_connection(candidate, true));
                remaining.erase(remaining.begin() + i);
                found = true;
                break;
            }
        }

        if (!found) {
            *failureReason = "could not connect edges into a continuous loop";
            return false;
        }
    }

    if (ordered.front().start_vertex_id != ordered.back().end_vertex_id) {
        const Vertex* loopStart = find_vertex(ordered.front().start_vertex_id);
        const Vertex* loopEnd = find_vertex(ordered.back().end_vertex_id);

        if (!loopStart || !loopEnd) {
            *failureReason = "could not find loop endpoint vertices for gap healing";
            return false;
        }

        const double closingDistance = distance(loopStart->point, loopEnd->point);
        if (closingDistance > kTopologyHealTolerance) {
            std::ostringstream reason;
            reason << "loop is open and endpoint gap " << closingDistance
                   << " exceeds tolerance " << kTopologyHealTolerance;
            *failureReason = reason.str();
            return false;
        }

        const Edge bridgeEdge = create_edge(*loopEnd, *loopStart);
        healingEdgeIds->push_back(bridgeEdge.id);
        ordered.push_back(bridgeEdge);
    }

    *outputEdges = std::move(ordered);
    return true;
}

bool BRepKernel::is_loop_non_planar(const std::vector<Edge>& edges, double tolerance) const {
    std::unordered_set<EntityId> seenVertexIds;
    std::vector<Point3D> points;
    points.reserve(edges.size() * 2);

    auto collect_vertex = [&](EntityId vertexId) {
        if (vertexId == 0 || seenVertexIds.find(vertexId) != seenVertexIds.end()) {
            return;
        }
        const Vertex* vertex = find_vertex(vertexId);
        if (!vertex) {
            return;
        }
        seenVertexIds.insert(vertexId);
        points.push_back(vertex->point);
    };

    for (const Edge& edge : edges) {
        collect_vertex(edge.start_vertex_id);
        collect_vertex(edge.end_vertex_id);
    }

    if (points.size() < 4) {
        return false;
    }

    const Point3D p0 = points[0];

    Point3D v1{};
    bool hasV1 = false;
    for (std::size_t i = 1; i < points.size(); ++i) {
        const Point3D candidate{points[i].x - p0.x, points[i].y - p0.y, points[i].z - p0.z};
        const double length = std::sqrt(candidate.x * candidate.x + candidate.y * candidate.y + candidate.z * candidate.z);
        if (length > tolerance) {
            v1 = candidate;
            hasV1 = true;
            break;
        }
    }

    if (!hasV1) {
        return false;
    }

    Point3D normal{};
    bool hasNormal = false;
    for (std::size_t i = 1; i < points.size(); ++i) {
        const Point3D v2{points[i].x - p0.x, points[i].y - p0.y, points[i].z - p0.z};
        const Point3D cross{
            v1.y * v2.z - v1.z * v2.y,
            v1.z * v2.x - v1.x * v2.z,
            v1.x * v2.y - v1.y * v2.x};
        const double norm = std::sqrt(cross.x * cross.x + cross.y * cross.y + cross.z * cross.z);
        if (norm > tolerance) {
            normal = cross;
            hasNormal = true;
            break;
        }
    }

    if (!hasNormal) {
        return false;
    }

    const double normalNorm = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
    if (normalNorm <= std::numeric_limits<double>::epsilon()) {
        return false;
    }

    for (const Point3D& point : points) {
        const Point3D diff{point.x - p0.x, point.y - p0.y, point.z - p0.z};
        const double distanceToPlane =
            std::abs(normal.x * diff.x + normal.y * diff.y + normal.z * diff.z) / normalNorm;
        if (distanceToPlane > tolerance) {
            return true;
        }
    }

    return false;
}

Face BRepKernel::create_face_from_edges(const std::vector<Edge>& edges, double areaHint) {
    reset_topology_diagnostic("create_face_from_edges");

    if (edges.size() < 3) {
        lastTopologyDiagnostic_.success = false;
        lastTopologyDiagnostic_.healing_attempted = false;
        lastTopologyDiagnostic_.healing_succeeded = false;
        lastTopologyDiagnostic_.message = "requires at least 3 edges";
        throw std::invalid_argument("create_face_from_edges requires at least 3 edges");
    }

    for (const Edge& edge : edges) {
        lastTopologyDiagnostic_.input_edge_ids.push_back(edge.id);
        if (edge.id == 0) {
            lastTopologyDiagnostic_.success = false;
            lastTopologyDiagnostic_.healing_succeeded = false;
            lastTopologyDiagnostic_.invalid_edge_ids.push_back(edge.id);
            lastTopologyDiagnostic_.message = "received an edge with id = 0";
            throw std::invalid_argument("create_face_from_edges received invalid edge id");
        }
    }

    std::vector<Edge> healedEdges;
    std::vector<EntityId> healingEdgeIds;
    std::string healingFailureReason;

    const bool strictLoopIsValid = validate_edge_loop_strict(edges, &healingFailureReason);
    switch (healingPolicy_) {
    case TopologyHealingPolicy::Strict:
        lastTopologyDiagnostic_.healing_attempted = false;
        if (!strictLoopIsValid) {
            lastTopologyDiagnostic_.success = false;
            lastTopologyDiagnostic_.healing_succeeded = false;
            lastTopologyDiagnostic_.message = healingFailureReason;
            throw_topology_diagnostic_error(lastTopologyDiagnostic_);
        }
        healedEdges = edges;
        lastTopologyDiagnostic_.healing_succeeded = true;
        break;
    case TopologyHealingPolicy::RepairFirst:
        lastTopologyDiagnostic_.healing_attempted = true;
        if (strictLoopIsValid) {
            healedEdges = edges;
            lastTopologyDiagnostic_.healing_succeeded = true;
        } else {
            if (!reorder_and_repair_edge_loop(edges, &healedEdges, &healingEdgeIds, &healingFailureReason)) {
                lastTopologyDiagnostic_.success = false;
                lastTopologyDiagnostic_.healing_succeeded = false;
                lastTopologyDiagnostic_.message = healingFailureReason;
                throw_topology_diagnostic_error(lastTopologyDiagnostic_);
            }
            lastTopologyDiagnostic_.healing_succeeded = true;
        }
        break;
    case TopologyHealingPolicy::RepairOnly:
        lastTopologyDiagnostic_.healing_attempted = true;
        if (!reorder_and_repair_edge_loop(edges, &healedEdges, &healingEdgeIds, &healingFailureReason)) {
            lastTopologyDiagnostic_.success = false;
            lastTopologyDiagnostic_.healing_succeeded = false;
            lastTopologyDiagnostic_.message = healingFailureReason;
            throw_topology_diagnostic_error(lastTopologyDiagnostic_);
        }
        lastTopologyDiagnostic_.healing_succeeded = true;
        break;
    }

    lastTopologyDiagnostic_.bridge_edges_created = !healingEdgeIds.empty();
    lastTopologyDiagnostic_.healing_edge_ids = healingEdgeIds;

    bool reordered = healedEdges.size() != edges.size();
    if (!reordered && healedEdges.size() == edges.size()) {
        for (std::size_t i = 0; i < edges.size(); ++i) {
            if (healedEdges[i].id != edges[i].id ||
                healedEdges[i].start_vertex_id != edges[i].start_vertex_id ||
                healedEdges[i].end_vertex_id != edges[i].end_vertex_id) {
                reordered = true;
                break;
            }
        }
    }
    lastTopologyDiagnostic_.edges_reordered = reordered;
    lastTopologyDiagnostic_.non_planar_input_detected = is_loop_non_planar(healedEdges, kPlanarityTolerance);

    Face face;
    face.id = next_id();
    face.area = areaHint;
    face.edge_ids.reserve(healedEdges.size());
    for (const Edge& edge : healedEdges) {
        face.edge_ids.push_back(edge.id);
    }
    if (!healedEdges.empty()) {
        const Vertex* firstVertex = find_vertex(healedEdges.front().start_vertex_id);
        if (firstVertex) {
            face.adaptive_state.position = firstVertex->point;
        }
    }

#ifdef ADAPTIVECAD_HAS_OPENCASCADE
    if (is_open_cascade_backend()) {
        BRepBuilderAPI_MakeWire wireMaker;
        std::vector<TopoDS_Edge> occEdges;
        occEdges.reserve(healedEdges.size());

        for (const Edge& edge : healedEdges) {
            const std::shared_ptr<TopoDS_Edge> occEdge =
                extract_occ_handle<TopoDS_Edge>(edge.native_handle, "TopoDS_Edge");
            if (occEdge) {
                occEdges.push_back(*occEdge);
            } else {
                lastTopologyDiagnostic_.invalid_edge_ids.push_back(edge.id);
            }
        }

        if (!lastTopologyDiagnostic_.invalid_edge_ids.empty()) {
            lastTopologyDiagnostic_.success = false;
            lastTopologyDiagnostic_.message =
                "missing native TopoDS_Edge handles for one or more edges on OpenCascade backend";
            throw_topology_diagnostic_error(lastTopologyDiagnostic_);
        }

        for (const TopoDS_Edge& occEdge : occEdges) {
            wireMaker.Add(occEdge);
            lastTopologyDiagnostic_.occ_wire_error_code = static_cast<int>(wireMaker.Error());
            if (!wireMaker.IsDone()) {
                lastTopologyDiagnostic_.success = false;
                lastTopologyDiagnostic_.message = "wire construction failed while adding an edge";
                throw_topology_diagnostic_error(lastTopologyDiagnostic_);
            }
        }

        if (!wireMaker.IsDone()) {
            lastTopologyDiagnostic_.success = false;
            lastTopologyDiagnostic_.occ_wire_error_code = static_cast<int>(wireMaker.Error());
            lastTopologyDiagnostic_.message = "wire construction did not complete";
            throw_topology_diagnostic_error(lastTopologyDiagnostic_);
        }

        TopoDS_Wire wire = wireMaker.Wire();
        if (wire.IsNull()) {
            lastTopologyDiagnostic_.success = false;
            lastTopologyDiagnostic_.message = "wire construction produced a null wire";
            throw_topology_diagnostic_error(lastTopologyDiagnostic_);
        }

        auto try_repair_wire = [&](const char* reason) {
            ShapeFix_Wire wireFix;
            wireFix.Load(wire);
            wireFix.SetPrecision(kTopologyHealTolerance);
            wireFix.FixReorder();
            wireFix.FixConnected();
            wireFix.FixClosed();
            wireFix.Perform();

            const TopoDS_Wire repairedWire = wireFix.Wire();
            if (repairedWire.IsNull()) {
                return false;
            }

            wire = repairedWire;
            lastTopologyDiagnostic_.message = reason;
            return true;
        };

        auto evaluate_wire_state = [&]() {
            lastTopologyDiagnostic_.occ_wire_closed = BRep_Tool::IsClosed(wire);
            BRepCheck_Analyzer wireAnalyzer(wire);
            lastTopologyDiagnostic_.occ_wire_valid = wireAnalyzer.IsValid();
            return lastTopologyDiagnostic_.occ_wire_closed && lastTopologyDiagnostic_.occ_wire_valid;
        };

        const bool allowRepair = healingPolicy_ != TopologyHealingPolicy::Strict;
        const bool forceRepair = healingPolicy_ == TopologyHealingPolicy::RepairOnly;

        bool wireReady = evaluate_wire_state();
        if (!wireReady || forceRepair) {
            if (!allowRepair) {
                lastTopologyDiagnostic_.success = false;
                lastTopologyDiagnostic_.message =
                    "wire failed closure/validity checks under strict healing policy";
                throw_topology_diagnostic_error(lastTopologyDiagnostic_);
            }

            if (!try_repair_wire(forceRepair ? "wire repaired by RepairOnly policy" : "wire repaired with ShapeFix_Wire")) {
                lastTopologyDiagnostic_.success = false;
                lastTopologyDiagnostic_.message = "wire failed closure/validity checks and could not be repaired";
                throw_topology_diagnostic_error(lastTopologyDiagnostic_);
            }

            wireReady = evaluate_wire_state();
            if (!wireReady) {
                lastTopologyDiagnostic_.success = false;
                lastTopologyDiagnostic_.message = "wire remains invalid after ShapeFix_Wire repair";
                throw_topology_diagnostic_error(lastTopologyDiagnostic_);
            }
        }

        BRepBuilderAPI_MakeFace faceMaker(wire, true);
        lastTopologyDiagnostic_.occ_face_error_code = static_cast<int>(faceMaker.Error());

        TopoDS_Face occFace;
        if (faceMaker.IsDone()) {
            occFace = faceMaker.Face();
        } else if (allowRepair && lastTopologyDiagnostic_.non_planar_input_detected) {
            BRepBuilderAPI_MakeFace nonPlanarFaceMaker(wire, false);
            lastTopologyDiagnostic_.occ_face_error_code = static_cast<int>(nonPlanarFaceMaker.Error());
            if (nonPlanarFaceMaker.IsDone()) {
                occFace = nonPlanarFaceMaker.Face();
                lastTopologyDiagnostic_.message = "non-planar input repaired using non-planar face construction";
            }
        }

        if (occFace.IsNull()) {
            lastTopologyDiagnostic_.success = false;
            lastTopologyDiagnostic_.message = "face construction failed from wire";
            throw_topology_diagnostic_error(lastTopologyDiagnostic_);
        }

        auto evaluate_face_state = [&]() {
            BRepCheck_Analyzer faceAnalyzer(occFace);
            lastTopologyDiagnostic_.occ_face_valid = faceAnalyzer.IsValid();
            return lastTopologyDiagnostic_.occ_face_valid;
        };

        bool faceReady = evaluate_face_state();
        if (!faceReady || forceRepair) {
            if (!allowRepair) {
                lastTopologyDiagnostic_.success = false;
                lastTopologyDiagnostic_.message = "face failed validity checks under strict healing policy";
                throw_topology_diagnostic_error(lastTopologyDiagnostic_);
            }

            ShapeFix_Face faceFix(occFace);
            faceFix.SetPrecision(kTopologyHealTolerance);
            faceFix.Perform();
            occFace = faceFix.Face();
            faceReady = !occFace.IsNull() && evaluate_face_state();

            if (faceReady) {
                lastTopologyDiagnostic_.message = "face repaired with ShapeFix_Face";
            }
        }

        if (!lastTopologyDiagnostic_.occ_face_valid) {
            lastTopologyDiagnostic_.success = false;
            lastTopologyDiagnostic_.message = "face failed OpenCascade topological validity checks after repair attempts";
            throw_topology_diagnostic_error(lastTopologyDiagnostic_);
        }

        face.native_handle = make_occ_handle(occFace, "TopoDS_Face");

        if (face.area <= 0.0) {
            GProp_GProps props;
            BRepGProp::SurfaceProperties(occFace, props);
            face.area = props.Mass();
            if (face.area <= 0.0) {
                lastTopologyDiagnostic_.success = false;
                lastTopologyDiagnostic_.message = "face area is non-positive after OpenCascade surface evaluation";
                throw_topology_diagnostic_error(lastTopologyDiagnostic_);
            }
        }
    }
#endif

    lastTopologyDiagnostic_.success = true;
    if (lastTopologyDiagnostic_.message.empty()) {
        lastTopologyDiagnostic_.message = "ok";
    }

    faces_.push_back(face);
    return face;
}

Body BRepKernel::create_body_from_faces(const std::vector<Face>& faces, double volumeHint) {
    if (faces.empty()) {
        throw std::invalid_argument("create_body_from_faces requires at least one face");
    }

    Body body;
    body.id = next_id();
    body.volume = volumeHint;
    if (!faces.empty()) {
        body.adaptive_state.position = faces.front().adaptive_state.position;
    }
    body.face_ids.reserve(faces.size());

    for (const Face& face : faces) {
        if (face.id == 0) {
            throw std::invalid_argument("create_body_from_faces received invalid face id");
        }
        body.face_ids.push_back(face.id);
    }

#ifdef ADAPTIVECAD_HAS_OPENCASCADE
    if (is_open_cascade_backend()) {
        BRep_Builder builder;
        TopoDS_Compound compound;
        builder.MakeCompound(compound);

        bool hasNativeFaces = false;
        for (const Face& face : faces) {
            const std::shared_ptr<TopoDS_Face> occFace =
                extract_occ_handle<TopoDS_Face>(face.native_handle, "TopoDS_Face");
            if (occFace) {
                builder.Add(compound, *occFace);
                hasNativeFaces = true;
            }
        }

        if (hasNativeFaces) {
            body.native_handle = make_occ_handle(compound, "TopoDS_Compound");

            if (body.volume <= 0.0) {
                GProp_GProps props;
                BRepGProp::VolumeProperties(compound, props);
                body.volume = props.Mass();
            }
        }
    }
#endif

    bodies_.push_back(body);
    return body;
}

Body BRepKernel::create_torus(
    double majorRadius,
    double minorRadius,
    std::size_t majorSegments,
    std::size_t minorSegments) {
    if (majorRadius <= 0.0 || minorRadius <= 0.0) {
        throw std::invalid_argument("create_torus requires positive major and minor radii");
    }
    if (minorRadius >= majorRadius) {
        throw std::invalid_argument("create_torus requires minor radius smaller than major radius");
    }
    if (majorSegments < 3 || minorSegments < 3) {
        throw std::invalid_argument("create_torus requires at least 3 major and minor segments");
    }

    Body body;
    body.id = next_id();
    body.volume = torus_volume(majorRadius, minorRadius);
    body.adaptive_state.position = Point3D{0.0, 0.0, 0.0};

#ifdef ADAPTIVECAD_HAS_OPENCASCADE
    if (is_open_cascade_backend()) {
        BRepPrimAPI_MakeTorus torusMaker(majorRadius, minorRadius);
        const TopoDS_Shape torusShape = torusMaker.Shape();
        if (torusShape.IsNull()) {
            throw std::runtime_error("create_torus failed to create an OpenCascade torus shape");
        }

        BRepCheck_Analyzer analyzer(torusShape);
        if (!analyzer.IsValid()) {
            throw std::runtime_error("create_torus produced an invalid OpenCascade torus shape");
        }

        body.native_handle = make_occ_handle(torusShape, "TopoDS_Shape");

        GProp_GProps props;
        BRepGProp::VolumeProperties(torusShape, props);
        if (props.Mass() > 0.0) {
            body.volume = props.Mass();
        }

        bodies_.push_back(body);
        return body;
    }
#endif

    std::vector<Vertex> torusVertices;
    torusVertices.reserve(majorSegments * minorSegments);
    for (std::size_t uIndex = 0; uIndex < majorSegments; ++uIndex) {
        for (std::size_t vIndex = 0; vIndex < minorSegments; ++vIndex) {
            const Point3D point = torus_point(majorRadius, minorRadius, uIndex, vIndex, majorSegments, minorSegments);
            torusVertices.push_back(create_vertex(point.x, point.y, point.z));
        }
    }

    auto vertex_at = [&](std::size_t uIndex, std::size_t vIndex) -> const Vertex& {
        return torusVertices[(uIndex % majorSegments) * minorSegments + (vIndex % minorSegments)];
    };

    const double patchArea = torus_patch_area_hint(majorRadius, minorRadius, majorSegments, minorSegments);
    std::vector<Face> torusFaces;
    torusFaces.reserve(majorSegments * minorSegments);
    for (std::size_t uIndex = 0; uIndex < majorSegments; ++uIndex) {
        for (std::size_t vIndex = 0; vIndex < minorSegments; ++vIndex) {
            const Vertex& p00 = vertex_at(uIndex, vIndex);
            const Vertex& p10 = vertex_at(uIndex + 1, vIndex);
            const Vertex& p11 = vertex_at(uIndex + 1, vIndex + 1);
            const Vertex& p01 = vertex_at(uIndex, vIndex + 1);

            const Edge e1 = create_edge(p00, p10);
            const Edge e2 = create_edge(p10, p11);
            const Edge e3 = create_edge(p11, p01);
            const Edge e4 = create_edge(p01, p00);
            torusFaces.push_back(create_face_from_edges({e1, e2, e3, e4}, patchArea));
        }
    }

    return create_body_from_faces(torusFaces, body.volume);
}

std::size_t BRepKernel::vertex_count() const noexcept {
    return vertices_.size();
}

std::size_t BRepKernel::edge_count() const noexcept {
    return edges_.size();
}

std::size_t BRepKernel::face_count() const noexcept {
    return faces_.size();
}

std::size_t BRepKernel::body_count() const noexcept {
    return bodies_.size();
}

void BRepKernel::reset_topology_diagnostic(const char* operation) {
    lastTopologyDiagnostic_ = TopologyDiagnostic{};
    lastTopologyDiagnostic_.operation = operation;
    lastTopologyDiagnostic_.backend = backend_name();
    lastTopologyDiagnostic_.healing_policy = healingPolicy_;
}

bool BRepKernel::is_open_cascade_backend() const noexcept {
#ifdef ADAPTIVECAD_HAS_OPENCASCADE
    return geometryAdapter_ && geometryAdapter_->backend_name() == "OpenCascade";
#else
    return false;
#endif
}

EntityId BRepKernel::next_id() {
    return nextId_++;
}

} // namespace adaptivecad::geometry
