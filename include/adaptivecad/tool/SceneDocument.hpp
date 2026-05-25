#pragma once

#include "adaptivecad/core/Models.hpp"
#include "adaptivecad/geometry/BRepEntities.hpp"
#include "adaptivecad/geometry/BRepKernel.hpp"

#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace adaptivecad::tool {

enum class SceneSourceKind {
    Demo,
    WavefrontObj,
    StlMesh,
    PlyMesh,
    StepModel,
    GeneratedScaffold,
    GeneratedPrimitive,
    SessionEmbedded
};

struct SceneColor {
    double red = 0.0;
    double green = 0.0;
    double blue = 0.0;
    double alpha = 1.0;
};

struct SceneMaterial {
    std::string name;
    SceneColor diffuse_color;
    bool has_diffuse_color = false;
};

struct SceneFace {
    SceneFace() = default;

    explicit SceneFace(std::vector<std::size_t> indices, double areaHint = 0.0)
        : vertex_indices(std::move(indices)),
          area_hint(areaHint) {}

    std::vector<std::size_t> vertex_indices;
    double area_hint = 0.0;
    std::string material_name;
    SceneColor color;
    bool has_color = false;
};

struct SceneBody {
    std::string name;
    std::vector<geometry::Point3D> vertices;
    std::vector<SceneFace> faces;
    double volume_hint = 0.0;
    std::string material_name;
    SceneColor color;
    bool has_color = false;
};

struct SceneDocument {
    std::string scene_label;
    SceneSourceKind source_kind = SceneSourceKind::Demo;
    std::filesystem::path source_path;
    core::AngularWeightModel angular_model;
    std::vector<SceneMaterial> materials;
    std::vector<SceneBody> bodies;
};

struct SceneFaceOutline {
    geometry::EntityId face_id = 0;
    std::vector<geometry::Point3D> points;
    std::string material_name;
    SceneColor color;
    bool has_color = false;
};

struct SceneBodyInfo {
    geometry::EntityId body_id = 0;
    std::string name;
    std::string material_name;
    SceneColor color;
    bool has_color = false;
};

struct BuiltScene {
    std::vector<geometry::Vertex> vertices;
    std::vector<geometry::Edge> edges;
    std::vector<geometry::Face> faces;
    std::vector<geometry::Body> bodies;
    std::vector<SceneFaceOutline> face_outlines;
    std::vector<SceneBodyInfo> body_infos;
    geometry::TopologyDiagnostic diagnostic;
    std::string backend_name;
    bool has_open_cascade_topology = false;
};

SceneDocument make_demo_scene_document();
SceneDocument make_box_scene_document(double width = 1.0, double depth = 1.0, double height = 1.0);
SceneDocument make_wedge_scene_document(double width = 1.0, double depth = 1.0, double height = 1.0);
SceneDocument make_plane_scene_document(double width = 1.0, double depth = 1.0);
SceneDocument make_torus_scene_document(
    double major_radius = 1.0,
    double minor_radius = 0.25,
    std::size_t major_segments = 32,
    std::size_t minor_segments = 12);
BuiltScene build_scene_geometry(
    const SceneDocument& document,
    geometry::TopologyHealingPolicy policy,
    bool preferOpenCascade = true);
SceneDocument import_wavefront_obj(const std::filesystem::path& sourcePath);
SceneDocument import_ascii_stl(const std::filesystem::path& sourcePath);
SceneDocument import_ascii_ply(const std::filesystem::path& sourcePath);
SceneDocument import_step_model(const std::filesystem::path& sourcePath);
SceneDocument import_scene_document(const std::filesystem::path& sourcePath);
void export_wavefront_obj(const SceneDocument& document, const std::filesystem::path& targetPath);
void export_scene_document(const SceneDocument& document, const std::filesystem::path& targetPath);

const char* scene_source_kind_name(SceneSourceKind kind) noexcept;
SceneSourceKind parse_scene_source_kind(const std::string& value);

} // namespace adaptivecad::tool
