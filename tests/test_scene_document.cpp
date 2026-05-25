#include "adaptivecad/tool/SceneDocument.hpp"
#include "adaptivecad/tool/SessionBundleIO.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace {

bool near(double left, double right, double tolerance = 1.0e-9) {
    return std::fabs(left - right) <= tolerance;
}

std::size_t count_shared_edge_ids(
    const std::vector<adaptivecad::geometry::EntityId>& left,
    const std::vector<adaptivecad::geometry::EntityId>& right) {
    std::size_t sharedCount = 0;
    for (adaptivecad::geometry::EntityId leftId : left) {
        for (adaptivecad::geometry::EntityId rightId : right) {
            if (leftId == rightId) {
                ++sharedCount;
            }
        }
    }
    return sharedCount;
}

void write_u8(std::ostream& out, std::uint8_t value) {
    const char byte = static_cast<char>(value);
    out.write(&byte, 1);
}

void write_u16_le(std::ostream& out, std::uint16_t value) {
    const std::array<char, 2> bytes{
        static_cast<char>(value & 0xFFU),
        static_cast<char>((value >> 8U) & 0xFFU)};
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

void write_u32_le(std::ostream& out, std::uint32_t value) {
    const std::array<char, 4> bytes{
        static_cast<char>(value & 0xFFU),
        static_cast<char>((value >> 8U) & 0xFFU),
        static_cast<char>((value >> 16U) & 0xFFU),
        static_cast<char>((value >> 24U) & 0xFFU)};
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

void write_i32_le(std::ostream& out, std::int32_t value) {
    write_u32_le(out, static_cast<std::uint32_t>(value));
}

void write_f32_le(std::ostream& out, float value) {
    std::uint32_t raw = 0;
    std::memcpy(&raw, &value, sizeof(raw));
    write_u32_le(out, raw);
}

void write_binary_stl_triangle(
    std::ostream& out,
    const std::array<adaptivecad::geometry::Point3D, 3>& points,
    std::uint16_t attribute = 0) {
    write_f32_le(out, 0.0F);
    write_f32_le(out, 0.0F);
    write_f32_le(out, 1.0F);
    for (const auto& point : points) {
        write_f32_le(out, static_cast<float>(point.x));
        write_f32_le(out, static_cast<float>(point.y));
        write_f32_le(out, static_cast<float>(point.z));
    }
    write_u16_le(out, attribute);
}

} // namespace

int main() {
    using adaptivecad::geometry::TopologyHealingPolicy;
    using adaptivecad::tool::BuiltScene;
    using adaptivecad::tool::SceneDocument;

    const SceneDocument demoDocument = adaptivecad::tool::make_demo_scene_document();
    assert(demoDocument.bodies.size() == 1);
    assert(demoDocument.angular_model.lambda0 > 0.0);

    const BuiltScene demoScene = adaptivecad::tool::build_scene_geometry(demoDocument, TopologyHealingPolicy::RepairFirst);
    assert(demoScene.vertices.size() == 3);
    assert(demoScene.edges.size() == 3);
    assert(demoScene.faces.size() == 1);
    assert(demoScene.bodies.size() == 1);
    assert(demoScene.face_outlines.size() == 1);
    assert(demoScene.body_infos.size() == 1);
    assert(demoScene.body_infos.front().name == "Demo Body");

    const BuiltScene strictDemoScene = adaptivecad::tool::build_scene_geometry(demoDocument, TopologyHealingPolicy::Strict);
    assert(strictDemoScene.diagnostic.success);
    assert(!strictDemoScene.diagnostic.healing_attempted);

    const SceneDocument boxDocument = adaptivecad::tool::make_box_scene_document(2.0, 3.0, 4.0);
    assert(boxDocument.source_kind == adaptivecad::tool::SceneSourceKind::GeneratedPrimitive);
    assert(boxDocument.bodies.size() == 1);
    assert(boxDocument.bodies.front().vertices.size() == 8);
    assert(boxDocument.bodies.front().faces.size() == 6);
    const BuiltScene boxScene = adaptivecad::tool::build_scene_geometry(boxDocument, TopologyHealingPolicy::Strict);
    assert(boxScene.diagnostic.success);
    assert(boxScene.vertices.size() == 8);
    assert(boxScene.edges.size() == 12);
    assert(boxScene.faces.size() == 6);

    const SceneDocument wedgeDocument = adaptivecad::tool::make_wedge_scene_document(2.0, 1.5, 1.0);
    assert(wedgeDocument.bodies.front().vertices.size() == 6);
    assert(wedgeDocument.bodies.front().faces.size() == 5);
    const BuiltScene wedgeScene = adaptivecad::tool::build_scene_geometry(wedgeDocument, TopologyHealingPolicy::Strict);
    assert(wedgeScene.diagnostic.success);
    assert(wedgeScene.edges.size() == 9);
    assert(wedgeScene.faces.size() == 5);

    const SceneDocument planeDocument = adaptivecad::tool::make_plane_scene_document(2.0, 1.0);
    assert(planeDocument.bodies.front().vertices.size() == 4);
    assert(planeDocument.bodies.front().faces.size() == 1);
    const BuiltScene planeScene = adaptivecad::tool::build_scene_geometry(planeDocument, TopologyHealingPolicy::Strict);
    assert(planeScene.diagnostic.success);
    assert(planeScene.edges.size() == 4);
    assert(planeScene.faces.size() == 1);

    const SceneDocument torusDocument = adaptivecad::tool::make_torus_scene_document(1.0, 0.25, 8, 6);
    assert(torusDocument.bodies.front().vertices.size() == 48);
    assert(torusDocument.bodies.front().faces.size() == 48);
    const BuiltScene torusScene = adaptivecad::tool::build_scene_geometry(torusDocument, TopologyHealingPolicy::Strict);
    assert(torusScene.diagnostic.success);
    assert(torusScene.edges.size() == 96);
    assert(torusScene.faces.size() == 48);

    const std::filesystem::path tempPath = std::filesystem::temp_directory_path() / "adaptivecad_scene_document_test.obj";
    const std::filesystem::path tempMaterialPath =
        std::filesystem::temp_directory_path() / "adaptivecad_scene_document_test.mtl";
    {
        std::ofstream out(tempMaterialPath, std::ios::binary | std::ios::trunc);
        assert(static_cast<bool>(out));
        out << "newmtl LowerBlue\n";
        out << "Kd 0.1 0.2 0.8\n";
        out << "d 0.75\n";
        out << "newmtl UpperRed\n";
        out << "Kd 0.9 0.2 0.1\n";
    }
    {
        std::ofstream out(tempPath, std::ios::binary | std::ios::trunc);
        assert(static_cast<bool>(out));
        out << "mtllib " << tempMaterialPath.filename().string() << "\n";
        out << "o Lower\n";
        out << "v 0 0 0\n";
        out << "v 1 0 0\n";
        out << "v 0 1 0\n";
        out << "usemtl LowerBlue\n";
        out << "f 1 2 3\n";
        out << "o Upper\n";
        out << "v 0 0 1\n";
        out << "v 1 0 1\n";
        out << "v 0 1 1\n";
        out << "usemtl UpperRed\n";
        out << "f 4 5 6\n";
    }

    const SceneDocument importedDocument = adaptivecad::tool::import_wavefront_obj(tempPath);
    assert(importedDocument.bodies.size() == 2);
    assert(importedDocument.source_path == tempPath);
    assert(importedDocument.materials.size() == 2);
    assert(importedDocument.bodies[0].faces[0].material_name == "LowerBlue");
    assert(importedDocument.bodies[0].faces[0].has_color);
    assert(near(importedDocument.bodies[0].faces[0].color.red, 0.1));
    assert(near(importedDocument.bodies[0].faces[0].color.green, 0.2));
    assert(near(importedDocument.bodies[0].faces[0].color.blue, 0.8));
    assert(near(importedDocument.bodies[0].faces[0].color.alpha, 0.75));
    assert(importedDocument.bodies[0].has_color);

    const SceneDocument importedDocumentViaDispatch = adaptivecad::tool::import_scene_document(tempPath);
    assert(importedDocumentViaDispatch.bodies.size() == 2);

    const BuiltScene importedScene =
        adaptivecad::tool::build_scene_geometry(importedDocument, TopologyHealingPolicy::RepairFirst);
    assert(importedScene.bodies.size() == 2);
    assert(importedScene.edges.size() == 6);
    assert(importedScene.faces.size() == 2);
    assert(importedScene.face_outlines.size() == 2);
    assert(importedScene.face_outlines[0].has_color);
    assert(importedScene.face_outlines[0].material_name == "LowerBlue");
    assert(importedScene.body_infos.size() == 2);
    assert(importedScene.body_infos[0].name == "Lower");
    assert(importedScene.body_infos[1].name == "Upper");
    assert(importedScene.body_infos[0].has_color);

    const BuiltScene strictImportedScene =
        adaptivecad::tool::build_scene_geometry(importedDocument, TopologyHealingPolicy::Strict);
    assert(strictImportedScene.diagnostic.success);
    assert(!strictImportedScene.diagnostic.healing_attempted);

    const std::filesystem::path exportedObjPath =
        std::filesystem::temp_directory_path() / "adaptivecad_scene_document_export_test.obj";
    adaptivecad::tool::export_scene_document(importedDocument, exportedObjPath);
    const SceneDocument exportedObjDocument = adaptivecad::tool::import_scene_document(exportedObjPath);
    assert(exportedObjDocument.bodies.size() == 2);
    assert(exportedObjDocument.bodies[0].name == "Lower");
    assert(exportedObjDocument.bodies[1].name == "Upper");
    assert(exportedObjDocument.bodies[0].vertices.size() == 3);
    assert(exportedObjDocument.bodies[1].vertices.size() == 3);
    assert(exportedObjDocument.bodies[0].faces.size() == 1);
    assert(exportedObjDocument.bodies[1].faces.size() == 1);
    assert(exportedObjDocument.bodies[0].faces[0].material_name == "LowerBlue");
    assert(exportedObjDocument.bodies[0].faces[0].has_color);
    assert(near(exportedObjDocument.bodies[0].faces[0].color.blue, 0.8));

    const std::filesystem::path stlPath = std::filesystem::temp_directory_path() / "adaptivecad_scene_document_test.stl";
    {
        std::ofstream out(stlPath, std::ios::binary | std::ios::trunc);
        assert(static_cast<bool>(out));
        out << "solid stacked\n";
        out << "facet normal 0 0 1\n";
        out << "outer loop\n";
        out << "vertex 0 0 0\n";
        out << "vertex 1 0 0\n";
        out << "vertex 0 1 0\n";
        out << "endloop\n";
        out << "endfacet\n";
        out << "facet normal 0 0 1\n";
        out << "outer loop\n";
        out << "vertex 0 0 1\n";
        out << "vertex 1 0 1\n";
        out << "vertex 0 1 1\n";
        out << "endloop\n";
        out << "endfacet\n";
        out << "endsolid stacked\n";
    }

    const SceneDocument stlDocument = adaptivecad::tool::import_scene_document(stlPath);
    assert(stlDocument.source_kind == adaptivecad::tool::SceneSourceKind::StlMesh);
    assert(stlDocument.bodies.size() == 1);
    assert(stlDocument.bodies.front().faces.size() == 2);

    const std::filesystem::path binaryStlPath =
        std::filesystem::temp_directory_path() / "adaptivecad_scene_document_test_binary.stl";
    {
        std::ofstream out(binaryStlPath, std::ios::binary | std::ios::trunc);
        assert(static_cast<bool>(out));
        std::array<char, 80> header{};
        const std::string label = "AdaptiveCAD binary STL regression";
        std::memcpy(header.data(), label.data(), label.size());
        out.write(header.data(), static_cast<std::streamsize>(header.size()));
        write_u32_le(out, 2);
        constexpr std::uint16_t blueTintAttribute = 0x8000U | (31U << 10U) | 8U;
        write_binary_stl_triangle(out, {
            adaptivecad::geometry::Point3D{0.0, 0.0, 0.0},
            adaptivecad::geometry::Point3D{1.0, 0.0, 0.0},
            adaptivecad::geometry::Point3D{1.0, 1.0, 0.0}},
            blueTintAttribute);
        write_binary_stl_triangle(out, {
            adaptivecad::geometry::Point3D{0.0, 0.0, 0.0},
            adaptivecad::geometry::Point3D{1.0, 1.0, 0.0},
            adaptivecad::geometry::Point3D{0.0, 1.0, 0.0}},
            blueTintAttribute);
    }

    const SceneDocument binaryStlDocument = adaptivecad::tool::import_scene_document(binaryStlPath);
    assert(binaryStlDocument.source_kind == adaptivecad::tool::SceneSourceKind::StlMesh);
    assert(binaryStlDocument.bodies.size() == 1);
    assert(binaryStlDocument.bodies.front().vertices.size() == 4);
    assert(binaryStlDocument.bodies.front().faces.size() == 2);
    assert(binaryStlDocument.bodies.front().faces.front().has_color);
    assert(near(binaryStlDocument.bodies.front().faces.front().color.red, 1.0));
    assert(near(binaryStlDocument.bodies.front().faces.front().color.blue, 8.0 / 31.0));

    const BuiltScene binaryStlScene =
        adaptivecad::tool::build_scene_geometry(binaryStlDocument, TopologyHealingPolicy::Strict);
    assert(binaryStlScene.diagnostic.success);
    assert(binaryStlScene.edges.size() == 5);
    assert(count_shared_edge_ids(binaryStlScene.faces[0].edge_ids, binaryStlScene.faces[1].edge_ids) == 1);

    const std::filesystem::path plyPath = std::filesystem::temp_directory_path() / "adaptivecad_scene_document_test.ply";
    {
        std::ofstream out(plyPath, std::ios::binary | std::ios::trunc);
        assert(static_cast<bool>(out));
        out << "ply\n";
        out << "format ascii 1.0\n";
        out << "element vertex 4\n";
        out << "property float x\n";
        out << "property float y\n";
        out << "property float z\n";
        out << "property uchar red\n";
        out << "property uchar green\n";
        out << "property uchar blue\n";
        out << "element face 2\n";
        out << "property list uchar int vertex_indices\n";
        out << "end_header\n";
        out << "0 0 0 255 0 64\n";
        out << "1 0 0 255 0 64\n";
        out << "1 1 0 255 0 64\n";
        out << "0 1 0 255 0 64\n";
        out << "3 0 1 2\n";
        out << "3 0 2 3\n";
    }

    const SceneDocument plyDocument = adaptivecad::tool::import_scene_document(plyPath);
    assert(plyDocument.source_kind == adaptivecad::tool::SceneSourceKind::PlyMesh);
    assert(plyDocument.bodies.size() == 1);
    assert(plyDocument.bodies.front().vertices.size() == 4);
    assert(plyDocument.bodies.front().faces.size() == 2);
    assert(plyDocument.bodies.front().faces.front().has_color);
    assert(near(plyDocument.bodies.front().faces.front().color.red, 1.0));
    assert(near(plyDocument.bodies.front().faces.front().color.blue, 64.0 / 255.0));

    const BuiltScene plyScene = adaptivecad::tool::build_scene_geometry(plyDocument, TopologyHealingPolicy::RepairFirst);
    assert(plyScene.vertices.size() == 4);
    assert(plyScene.edges.size() == 5);
    assert(plyScene.faces.size() == 2);
    assert(plyScene.face_outlines.front().has_color);
    assert(count_shared_edge_ids(plyScene.faces[0].edge_ids, plyScene.faces[1].edge_ids) == 1);

    const BuiltScene strictPlyScene = adaptivecad::tool::build_scene_geometry(plyDocument, TopologyHealingPolicy::Strict);
    assert(strictPlyScene.diagnostic.success);
    assert(!strictPlyScene.diagnostic.healing_attempted);
    assert(strictPlyScene.edges.size() == 5);
    assert(count_shared_edge_ids(strictPlyScene.faces[0].edge_ids, strictPlyScene.faces[1].edge_ids) == 1);

    const std::filesystem::path binaryPlyPath =
        std::filesystem::temp_directory_path() / "adaptivecad_scene_document_test_binary.ply";
    {
        std::ofstream out(binaryPlyPath, std::ios::binary | std::ios::trunc);
        assert(static_cast<bool>(out));
        out << "ply\n";
        out << "format binary_little_endian 1.0\n";
        out << "element vertex 4\n";
        out << "property float x\n";
        out << "property float y\n";
        out << "property float z\n";
        out << "property uchar red\n";
        out << "element face 2\n";
        out << "property list uchar int vertex_indices\n";
        out << "end_header\n";
        const std::array<adaptivecad::geometry::Point3D, 4> binaryPlyPoints{
            adaptivecad::geometry::Point3D{0.0, 0.0, 0.0},
            adaptivecad::geometry::Point3D{1.0, 0.0, 0.0},
            adaptivecad::geometry::Point3D{1.0, 1.0, 0.0},
            adaptivecad::geometry::Point3D{0.0, 1.0, 0.0}};
        for (const auto& point : binaryPlyPoints) {
            write_f32_le(out, static_cast<float>(point.x));
            write_f32_le(out, static_cast<float>(point.y));
            write_f32_le(out, static_cast<float>(point.z));
            write_u8(out, 255);
        }
        write_u8(out, 3);
        write_i32_le(out, 0);
        write_i32_le(out, 1);
        write_i32_le(out, 2);
        write_u8(out, 3);
        write_i32_le(out, 0);
        write_i32_le(out, 2);
        write_i32_le(out, 3);
    }

    const SceneDocument binaryPlyDocument = adaptivecad::tool::import_scene_document(binaryPlyPath);
    assert(binaryPlyDocument.source_kind == adaptivecad::tool::SceneSourceKind::PlyMesh);
    assert(binaryPlyDocument.bodies.size() == 1);
    assert(binaryPlyDocument.bodies.front().vertices.size() == 4);
    assert(binaryPlyDocument.bodies.front().faces.size() == 2);

    const BuiltScene binaryPlyScene =
        adaptivecad::tool::build_scene_geometry(binaryPlyDocument, TopologyHealingPolicy::Strict);
    assert(binaryPlyScene.diagnostic.success);
    assert(binaryPlyScene.edges.size() == 5);
    assert(count_shared_edge_ids(binaryPlyScene.faces[0].edge_ids, binaryPlyScene.faces[1].edge_ids) == 1);

    SceneDocument windingDocument;
    windingDocument.scene_label = "Winding normalization regression";
    windingDocument.bodies.push_back(adaptivecad::tool::SceneBody{});
    windingDocument.bodies.front().name = "Winding Body";
    windingDocument.bodies.front().vertices = {
        adaptivecad::geometry::Point3D{0.0, 0.0, 0.0},
        adaptivecad::geometry::Point3D{1.0, 0.0, 0.0},
        adaptivecad::geometry::Point3D{1.0, 1.0, 0.0},
        adaptivecad::geometry::Point3D{2.0, 1.0, 0.0}};
    windingDocument.bodies.front().faces = {
        adaptivecad::tool::SceneFace{{0, 1, 2}},
        adaptivecad::tool::SceneFace{{1, 2, 3}}};

    const BuiltScene windingScene = adaptivecad::tool::build_scene_geometry(windingDocument, TopologyHealingPolicy::Strict);
    assert(windingScene.diagnostic.success);
    assert(!windingScene.diagnostic.healing_attempted);
    assert(windingScene.edges.size() == 5);
    assert(count_shared_edge_ids(windingScene.faces[0].edge_ids, windingScene.faces[1].edge_ids) == 1);

    SceneDocument nonManifoldDocument;
    nonManifoldDocument.scene_label = "Non-manifold regression";
    nonManifoldDocument.bodies.push_back(adaptivecad::tool::SceneBody{});
    nonManifoldDocument.bodies.front().name = "Non-manifold Body";
    nonManifoldDocument.bodies.front().vertices = {
        adaptivecad::geometry::Point3D{0.0, 0.0, 0.0},
        adaptivecad::geometry::Point3D{1.0, 0.0, 0.0},
        adaptivecad::geometry::Point3D{0.0, 1.0, 0.0},
        adaptivecad::geometry::Point3D{0.0, -1.0, 0.0},
        adaptivecad::geometry::Point3D{1.0, 1.0, 0.0}};
    nonManifoldDocument.bodies.front().faces = {
        adaptivecad::tool::SceneFace{{0, 1, 2}},
        adaptivecad::tool::SceneFace{{1, 0, 3}},
        adaptivecad::tool::SceneFace{{0, 1, 4}}};

    bool sawNonManifold = false;
    try {
        (void)adaptivecad::tool::build_scene_geometry(nonManifoldDocument, TopologyHealingPolicy::Strict);
    } catch (const std::runtime_error& err) {
        sawNonManifold = true;
        const std::string message = err.what();
        assert(message.find("non-manifold") != std::string::npos);
    }
    assert(sawNonManifold);

    const std::filesystem::path sessionPath = std::filesystem::temp_directory_path() / "adaptivecad_scene_document_test.ini";
    adaptivecad::tool::SessionBundleData sessionBundle;
    sessionBundle.saved_at = "2026-04-15 00:00:00";
    sessionBundle.workspace = std::filesystem::temp_directory_path();
    sessionBundle.report_text_path = sessionPath.parent_path() / "scene_document_test_report.txt";
    sessionBundle.report_json_path = sessionPath.parent_path() / "scene_document_test_report.json";
    sessionBundle.scene_document = importedDocument;
    sessionBundle.dataset.using_embedded_samples = true;
    sessionBundle.dataset.original_csv_path = "measurements.csv";
    sessionBundle.dataset.valid_rows = 3;
    sessionBundle.dataset.rejected_rows = 1;
    sessionBundle.dataset.summary = "session test";
    sessionBundle.dataset.samples = {
        adaptivecad::core::AreaSample{1.0, 3.1},
        adaptivecad::core::AreaSample{1.5, 7.2},
        adaptivecad::core::AreaSample{2.0, 12.8}};
    sessionBundle.view_state.healing_policy = "RepairOnly";
    sessionBundle.view_state.projection = "Right";
    sessionBundle.view_state.zoom = 1.25;
    sessionBundle.view_state.pan_x = 6.0;
    sessionBundle.view_state.pan_y = -2.0;
    sessionBundle.view_state.selection_kind = "face";
    sessionBundle.view_state.selected_id = 42;
    adaptivecad::tool::save_session_bundle_file(sessionPath, sessionBundle);

    const adaptivecad::tool::SessionBundleData loadedBundle = adaptivecad::tool::load_session_bundle_file(sessionPath);
    assert(loadedBundle.schema_version == 2);
    assert(loadedBundle.scene_document.bodies.size() == 2);
    assert(loadedBundle.scene_document.scene_label == importedDocument.scene_label);
    assert(loadedBundle.scene_document.source_kind == importedDocument.source_kind);
    assert(loadedBundle.scene_document.source_path == importedDocument.source_path);
    assert(loadedBundle.scene_document.angular_model.lambda0 == importedDocument.angular_model.lambda0);
    assert(loadedBundle.scene_document.materials.size() == importedDocument.materials.size());
    assert(loadedBundle.scene_document.bodies[0].faces[0].material_name == "LowerBlue");
    assert(loadedBundle.scene_document.bodies[0].faces[0].has_color);
    assert(near(loadedBundle.scene_document.bodies[0].faces[0].color.alpha, 0.75));
    assert(loadedBundle.dataset.samples.size() == 3);
    assert(loadedBundle.view_state.projection == "Right");
    assert(loadedBundle.view_state.selection_kind == "face");
    assert(loadedBundle.view_state.selected_id == 42);

    std::error_code ec;
    std::filesystem::remove(tempPath, ec);
    std::filesystem::remove(tempMaterialPath, ec);
    std::filesystem::remove(exportedObjPath, ec);
    std::filesystem::remove(exportedObjPath.parent_path() / (exportedObjPath.stem().string() + ".mtl"), ec);
    std::filesystem::remove(stlPath, ec);
    std::filesystem::remove(binaryStlPath, ec);
    std::filesystem::remove(plyPath, ec);
    std::filesystem::remove(binaryPlyPath, ec);
    std::filesystem::remove(sessionPath, ec);
    return 0;
}
