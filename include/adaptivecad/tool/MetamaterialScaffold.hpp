#pragma once

#include "adaptivecad/core/Models.hpp"
#include "adaptivecad/tool/SceneDocument.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace adaptivecad::tool {

enum class ScaffoldStrutAxis {
    X,
    Y,
    Z
};

struct ScaffoldDamageSphere {
    geometry::Point3D center{};
    double radius = 0.0;
};

struct MetamaterialScaffoldParameters {
    int cells_x = 4;
    int cells_y = 3;
    int cells_z = 1;
    double spacing = 1.0;
    double base_thickness = 0.16;
    double minimum_thickness = 0.08;
    double angular_gain = 1.0;
    double backbone_gain = 0.85;
    double backbone_decay_length = 1.5;
    ScaffoldDamageSphere damage_sphere{};
    core::AngularWeightModel angular_model{};
    std::string scene_label = "Chern-Locked Metamaterial Scaffold";
};

struct ScaffoldStrutDescriptor {
    geometry::Point3D start{};
    geometry::Point3D end{};
    ScaffoldStrutAxis axis = ScaffoldStrutAxis::X;
    double conductance = 0.0;
    double backbone_score = 0.0;
    double thickness = 0.0;
};

struct ScaffoldConnectorDescriptor {
    geometry::Point3D center{};
    double half_extent = 0.0;
    std::vector<std::size_t> adjacent_strut_indices;
    double backbone_influence = 0.0;
    bool boundary_node = false;
    bool damage_adjacent = false;
};

struct GeneratedMetamaterialScaffold {
    SceneDocument scene_document;
    std::vector<ScaffoldStrutDescriptor> struts;
    std::vector<ScaffoldConnectorDescriptor> connectors;
    std::size_t node_count = 0;
    std::size_t candidate_strut_count = 0;
    std::size_t removed_strut_count = 0;
    double transport_span = 0.0;
};

GeneratedMetamaterialScaffold generate_chern_locked_metamaterial_scaffold(
    const MetamaterialScaffoldParameters& parameters);

} // namespace adaptivecad::tool