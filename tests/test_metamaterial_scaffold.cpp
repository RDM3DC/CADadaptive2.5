#include "adaptivecad/tool/MetamaterialScaffold.hpp"

#include <algorithm>
#include <cassert>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

double max_thickness_for_axis(
    const adaptivecad::tool::GeneratedMetamaterialScaffold& scaffold,
    adaptivecad::tool::ScaffoldStrutAxis axis) {
    double maximum = 0.0;
    for (const auto& strut : scaffold.struts) {
        if (strut.axis == axis) {
            maximum = std::max(maximum, strut.thickness);
        }
    }
    return maximum;
}

double max_backbone_score(const adaptivecad::tool::GeneratedMetamaterialScaffold& scaffold) {
    double maximum = 0.0;
    for (const auto& strut : scaffold.struts) {
        maximum = std::max(maximum, strut.backbone_score);
    }
    return maximum;
}

std::size_t count_damage_adjacent_connectors(const adaptivecad::tool::GeneratedMetamaterialScaffold& scaffold) {
    std::size_t count = 0;
    for (const auto& connector : scaffold.connectors) {
        if (connector.damage_adjacent) {
            ++count;
        }
    }
    return count;
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

} // namespace

int main() {
    using adaptivecad::geometry::TopologyHealingPolicy;
    using adaptivecad::tool::GeneratedMetamaterialScaffold;
    using adaptivecad::tool::MetamaterialScaffoldParameters;
    using adaptivecad::tool::ScaffoldStrutAxis;
    using adaptivecad::tool::SceneDocument;
    using adaptivecad::tool::SceneSourceKind;

    {
        SceneDocument windingDocument;
        windingDocument.scene_label = "Strict winding normalization";
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

        const auto builtScene =
            adaptivecad::tool::build_scene_geometry(windingDocument, TopologyHealingPolicy::Strict);
        assert(builtScene.diagnostic.success);
        assert(!builtScene.diagnostic.healing_attempted);
        assert(builtScene.edges.size() == 5);
        assert(count_shared_edge_ids(builtScene.faces[0].edge_ids, builtScene.faces[1].edge_ids) == 1);
    }

    {
        SceneDocument nonManifoldDocument;
        nonManifoldDocument.scene_label = "Strict non-manifold rejection";
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
    }

    {
        MetamaterialScaffoldParameters parameters;
        parameters.cells_x = 2;
        parameters.cells_y = 1;
        parameters.cells_z = 1;

        const GeneratedMetamaterialScaffold scaffold =
            adaptivecad::tool::generate_chern_locked_metamaterial_scaffold(parameters);
        assert(scaffold.scene_document.source_kind == SceneSourceKind::GeneratedScaffold);
        assert(scaffold.node_count == 12);
        assert(scaffold.candidate_strut_count == 20);
        assert(scaffold.removed_strut_count == 0);
        assert(scaffold.struts.size() == 20);
        assert(scaffold.connectors.size() == scaffold.node_count);
        assert(scaffold.scene_document.bodies.size() == scaffold.struts.size() + scaffold.connectors.size());
        assert(!scaffold.connectors.empty());
        assert(scaffold.connectors.front().boundary_node);
        assert(!scaffold.connectors.front().adjacent_strut_indices.empty());

        const auto built_scene =
            adaptivecad::tool::build_scene_geometry(scaffold.scene_document, TopologyHealingPolicy::Strict);
        assert(built_scene.bodies.size() == scaffold.scene_document.bodies.size());
        assert(built_scene.faces.size() == (scaffold.struts.size() + scaffold.connectors.size()) * 6);
        assert(built_scene.body_infos.size() == scaffold.scene_document.bodies.size());
        assert(built_scene.diagnostic.success);
        assert(!built_scene.diagnostic.healing_attempted);
    }

    {
        MetamaterialScaffoldParameters parameters;
        parameters.cells_x = 2;
        parameters.cells_y = 2;
        parameters.cells_z = 0;
        parameters.angular_model.lambda0 = 1.0;
        parameters.angular_model.cosine_coefficients = {0.30};

        const GeneratedMetamaterialScaffold scaffold =
            adaptivecad::tool::generate_chern_locked_metamaterial_scaffold(parameters);
        const double x_thickness = max_thickness_for_axis(scaffold, ScaffoldStrutAxis::X);
        const double y_thickness = max_thickness_for_axis(scaffold, ScaffoldStrutAxis::Y);

        assert(x_thickness > y_thickness);
        assert(x_thickness > 0.0);
        assert(y_thickness > 0.0);
    }

    {
        MetamaterialScaffoldParameters parameters;
        parameters.cells_x = 3;
        parameters.cells_y = 2;
        parameters.cells_z = 0;
        parameters.damage_sphere.center = adaptivecad::geometry::Point3D{0.0, 0.0, 0.0};
        parameters.damage_sphere.radius = 0.75;

        const GeneratedMetamaterialScaffold scaffold =
            adaptivecad::tool::generate_chern_locked_metamaterial_scaffold(parameters);
        assert(scaffold.removed_strut_count > 0);
        assert(scaffold.struts.size() + scaffold.removed_strut_count == scaffold.candidate_strut_count);
        assert(scaffold.scene_document.bodies.size() == scaffold.struts.size() + scaffold.connectors.size());
        assert(!scaffold.connectors.empty());
        assert(count_damage_adjacent_connectors(scaffold) > 0);
        assert(scaffold.transport_span >= 3.0);
        assert(max_backbone_score(scaffold) > 0.99);

        const auto built_scene =
            adaptivecad::tool::build_scene_geometry(scaffold.scene_document, TopologyHealingPolicy::Strict);
        assert(built_scene.bodies.size() == scaffold.scene_document.bodies.size());
        assert(built_scene.diagnostic.success);
        assert(!built_scene.diagnostic.healing_attempted);
    }

    return 0;
}