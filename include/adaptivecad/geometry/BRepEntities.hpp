#pragma once

#include "adaptivecad/geometry/GeometryAdapter.hpp"
#include "adaptivecad/geometry/HistoryAwareState.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace adaptivecad::geometry {

using EntityId = std::uint64_t;

enum class NativeTopologyBackend {
    None,
    OpenCascade
};

struct NativeTopologyHandle {
    NativeTopologyBackend backend = NativeTopologyBackend::None;
    std::string type_name;
    std::shared_ptr<void> handle;

    bool is_valid() const noexcept {
        return static_cast<bool>(handle);
    }
};

struct Vertex {
    EntityId id = 0;
    Point3D point{};
    AdaptiveCADState adaptive_state{};
    NativeTopologyHandle native_handle;
};

struct Edge {
    EntityId id = 0;
    EntityId start_vertex_id = 0;
    EntityId end_vertex_id = 0;
    double length = 0.0;
    AdaptiveCADState adaptive_state{};
    NativeTopologyHandle native_handle;
};

struct Face {
    EntityId id = 0;
    std::vector<EntityId> edge_ids;
    double area = 0.0;
    AdaptiveCADState adaptive_state{};
    NativeTopologyHandle native_handle;
};

struct Body {
    EntityId id = 0;
    std::vector<EntityId> face_ids;
    double volume = 0.0;
    AdaptiveCADState adaptive_state{};
    NativeTopologyHandle native_handle;
};

} // namespace adaptivecad::geometry
