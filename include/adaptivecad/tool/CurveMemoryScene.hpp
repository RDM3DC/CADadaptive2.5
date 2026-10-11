#pragma once

#include "adaptivecad/tool/CurveMemoryBank.hpp"
#include "adaptivecad/tool/SceneDocument.hpp"

#include <array>
#include <cstddef>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace adaptivecad::tool {

// Row-major affine matrix. Placement is independent of the stored local points.
struct CurveMemoryAffine {
    std::array<double, 16> values{
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0};

    static CurveMemoryAffine identity() noexcept;
    static CurveMemoryAffine translation(double x, double y, double z);
};

struct CurveMemorySceneStats {
    std::size_t unique_shapes = 0;
    std::size_t instances = 0;
    std::size_t relations = 0;
    std::size_t overrides = 0;
    std::size_t logical_point_bytes = 0;
    // Capacity-based resident storage estimate, not measured process RAM.
    std::size_t resident_bytes = 0;
    // Same scene metadata/topology/instances/edits, using dense shared points.
    // Dense point reads need no decompression cache.
    std::size_t ordinary_shared_bytes = 0;
    // Same CurveMemoryScene/CurveMemoryBank containers with forced raw storage.
    std::size_t raw_policy_bytes = 0;
    // Current storage plus the entire reachable decoded-cache allowance.
    std::size_t budgeted_resident_bytes = 0;
    std::size_t cache_bytes = 0;
    std::size_t cache_limit_bytes = 0;
};

class CurveMemoryScene {
public:
    // Automatic compression must save at least 5% of the complete imported
    // scene's capacity estimate, including topology, metadata and full cache.
    // This gate runs at ingestion; subsequent instances/edits do not reencode.
    static CurveMemoryScene from_document(const SceneDocument& document, std::size_t cacheBlocks = 4,
        CurveStoragePolicy policy = CurveStoragePolicy::Automatic);

    // Expands all instances for existing native BRep, export, session, and UI APIs.
    SceneDocument materialize() const;
    std::size_t instance_count() const noexcept;
    // Stable index of the exact shared shape used by this instance.
    std::size_t shape_id(std::size_t instance) const;
    std::size_t point_count(std::size_t instance) const;
    geometry::Point3D point(std::size_t instance, std::size_t vertex) const;
    // Indexed reads retain caller order and duplicates while decoding each
    // compressed block at most once. Validate all indices and matching span
    // sizes before writing output. Returned coordinates include edits/placement.
    void points(std::size_t instance, std::span<const std::size_t> vertices,
        std::span<geometry::Point3D> output) const;
    // Fully expanded instance; the returned allocation belongs to the caller.
    std::vector<geometry::Point3D> points(std::size_t instance) const;

    // Copies the selected instance's local edits, with an absolute new placement.
    std::size_t add_instance(std::size_t existing, const CurveMemoryAffine& placement, std::string name);
    void set_point(std::size_t instance, std::size_t vertex, geometry::Point3D localPoint);
    void add_relation(std::size_t from, std::size_t to);
    const std::vector<std::pair<std::size_t, std::size_t>>& relations() const noexcept;

    CurveMemorySceneStats stats() const;
    const CurveMemoryBank& bank() const noexcept;
    void clear_cache() const;

private:
    explicit CurveMemoryScene(std::size_t cacheBlocks, CurveStoragePolicy policy);

    struct Shape {
        std::size_t curve = 0;
        std::vector<SceneFace> faces;
        double volume_hint = 0.0;
    };

    struct Instance {
        std::size_t shape = 0;
        std::string name;
        std::string material_name;
        SceneColor color;
        bool has_color = false;
        CurveMemoryAffine placement;
        std::vector<std::pair<std::size_t, geometry::Point3D>> overrides;
    };

    geometry::Point3D local_point(const Instance& instance, std::size_t vertex) const;
    SceneDocument header_;
    CurveMemoryBank bank_;
    std::vector<Shape> shapes_;
    std::vector<Instance> instances_;
    std::vector<std::pair<std::size_t, std::size_t>> relations_;
};

} // namespace adaptivecad::tool
