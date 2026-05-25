#include "adaptivecad/Version.hpp"
#include "adaptivecad/core/AdaptiveField.hpp"
#include "adaptivecad/core/InverseRecovery.hpp"
#include "adaptivecad/geometry/BRepKernel.hpp"
#include "adaptivecad/geometry/HistoryAwareKernel.hpp"
#include "adaptivecad/tool/MetamaterialScaffold.hpp"
#include "adaptivecad/tool/SceneDocument.hpp"

#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

adaptivecad::geometry::TopologyHealingPolicy parse_healing_policy(const std::string& text) {
    if (text == "strict") {
        return adaptivecad::geometry::TopologyHealingPolicy::Strict;
    }
    if (text == "repair-first" || text == "repairfirst") {
        return adaptivecad::geometry::TopologyHealingPolicy::RepairFirst;
    }
    if (text == "repair-only" || text == "repaironly") {
        return adaptivecad::geometry::TopologyHealingPolicy::RepairOnly;
    }
    throw std::runtime_error("unknown healing policy: " + text);
}

void print_version(std::ostream& output) {
    output << "AdaptiveCAD " << ADAPTIVECAD_VERSION << '\n';
}

void print_usage(std::ostream& output) {
    output << "Usage: adaptivecad [command] [options]\n\n";
    output << "Commands:\n";
    output << "  --help                         Show this help text\n";
    output << "  --version                      Show version information\n";
    output << "  --demo                         Run the built-in kernel demonstration\n";
    output << "  --inspect-geometry <path>      Import OBJ/STL/PLY/STEP and print a topology summary\n\n";
    output << "  --export-geometry <path>       Import geometry and export it as OBJ\n\n";
    output << "  --create-primitive <kind>      Create box, wedge, plane, torus, or scaffold as OBJ\n\n";
    output << "Options for geometry commands:\n";
    output << "  --output <path>                Output path for export/create commands\n";
    output << "  --healing <policy>             strict, repair-first, or repair-only (default: repair-first)\n";
    output << "  --json                         Emit machine-readable JSON for inspect-geometry\n";
    output << "\nOptions for --create-primitive:\n";
    output << "  --width <n> --depth <n>        Box/wedge/plane footprint dimensions\n";
    output << "  --height <n>                   Box or wedge height\n";
    output << "  --major-radius <n>             Torus major radius\n";
    output << "  --minor-radius <n>             Torus minor radius\n";
    output << "  --major-segments <n>           Torus major segment count (>= 3)\n";
    output << "  --minor-segments <n>           Torus minor segment count (>= 3)\n";
    output << "  --cells-x/y/z <n>              Scaffold lattice cell counts\n";
    output << "  --spacing <n>                  Scaffold lattice spacing\n";
    output << "  --damage-radius <n>            Scaffold spherical damage radius\n";
}

const char* read_option_value(int argc, char** argv, int* index, const std::string& option) {
    if (!index || *index + 1 >= argc || !argv[*index + 1]) {
        throw std::runtime_error(option + " requires a value");
    }
    ++(*index);
    return argv[*index];
}

double parse_finite_double(const std::string& option, const char* text) {
    try {
        const std::string valueText = text ? text : "";
        std::size_t consumed = 0;
        const double value = std::stod(valueText, &consumed);
        if (consumed != valueText.size() || !std::isfinite(value)) {
            throw std::runtime_error("invalid");
        }
        return value;
    } catch (const std::exception&) {
        throw std::runtime_error(option + " requires a finite numeric value");
    }
}

double parse_positive_double(const std::string& option, const char* text) {
    const double value = parse_finite_double(option, text);
    if (value <= 0.0) {
        throw std::runtime_error(option + " requires a positive numeric value");
    }
    return value;
}

double parse_nonnegative_double(const std::string& option, const char* text) {
    const double value = parse_finite_double(option, text);
    if (value < 0.0) {
        throw std::runtime_error(option + " requires a zero or positive numeric value");
    }
    return value;
}

int parse_int_at_least(const std::string& option, const char* text, int minimumValue) {
    try {
        const std::string valueText = text ? text : "";
        std::size_t consumed = 0;
        const long value = std::stol(valueText, &consumed);
        if (consumed != valueText.size() || value < minimumValue || value > std::numeric_limits<int>::max()) {
            throw std::runtime_error("invalid");
        }
        return static_cast<int>(value);
    } catch (const std::exception&) {
        throw std::runtime_error(option + " requires an integer >= " + std::to_string(minimumValue));
    }
}

struct PrimitiveCreateOptions {
    double width = 1.5;
    double depth = 1.0;
    double height = 0.75;
    double major_radius = 1.0;
    double minor_radius = 0.25;
    std::size_t major_segments = 32;
    std::size_t minor_segments = 12;
    int cells_x = 3;
    int cells_y = 2;
    int cells_z = 1;
    double spacing = 1.0;
    double damage_radius = 0.45;
};

PrimitiveCreateOptions default_primitive_create_options(const std::string& kind) {
    PrimitiveCreateOptions options;
    if (kind == "wedge") {
        options.height = 0.9;
    } else if (kind == "plane") {
        options.width = 2.0;
        options.depth = 1.25;
    }
    return options;
}

adaptivecad::tool::SceneDocument create_primitive_document(
    const std::string& kind,
    const PrimitiveCreateOptions& options) {
    if (kind == "box") {
        return adaptivecad::tool::make_box_scene_document(options.width, options.depth, options.height);
    }
    if (kind == "wedge") {
        return adaptivecad::tool::make_wedge_scene_document(options.width, options.depth, options.height);
    }
    if (kind == "plane") {
        return adaptivecad::tool::make_plane_scene_document(options.width, options.depth);
    }
    if (kind == "torus") {
        return adaptivecad::tool::make_torus_scene_document(
            options.major_radius,
            options.minor_radius,
            options.major_segments,
            options.minor_segments);
    }
    if (kind == "scaffold") {
        adaptivecad::tool::MetamaterialScaffoldParameters parameters;
        parameters.cells_x = options.cells_x;
        parameters.cells_y = options.cells_y;
        parameters.cells_z = options.cells_z;
        parameters.spacing = options.spacing;
        parameters.angular_model.lambda0 = 1.0;
        parameters.angular_model.cosine_coefficients = {0.20};
        parameters.damage_sphere.center = adaptivecad::geometry::Point3D{0.0, 0.0, 0.0};
        parameters.damage_sphere.radius = options.damage_radius;
        return adaptivecad::tool::generate_chern_locked_metamaterial_scaffold(parameters).scene_document;
    }

    throw std::runtime_error("unknown primitive kind: " + kind);
}

void append_json_string(std::ostream& output, const std::string& value) {
    output << '"';
    for (unsigned char ch : value) {
        switch (ch) {
        case '"':
            output << "\\\"";
            break;
        case '\\':
            output << "\\\\";
            break;
        case '\b':
            output << "\\b";
            break;
        case '\f':
            output << "\\f";
            break;
        case '\n':
            output << "\\n";
            break;
        case '\r':
            output << "\\r";
            break;
        case '\t':
            output << "\\t";
            break;
        default:
            if (ch < 0x20) {
                const char oldFill = output.fill('0');
                output << "\\u" << std::hex << std::setw(4) << static_cast<int>(ch) << std::dec;
                output.fill(oldFill);
            } else {
                output << static_cast<char>(ch);
            }
            break;
        }
    }
    output << '"';
}

void append_json_entity_ids(std::ostream& output, const std::vector<adaptivecad::geometry::EntityId>& ids) {
    output << '[';
    for (std::size_t index = 0; index < ids.size(); ++index) {
        if (index > 0) {
            output << ", ";
        }
        output << ids[index];
    }
    output << ']';
}

void append_json_scene_color(std::ostream& output, const adaptivecad::tool::SceneColor& color) {
    output << "{\"red\": " << color.red
           << ", \"green\": " << color.green
           << ", \"blue\": " << color.blue
           << ", \"alpha\": " << color.alpha << '}';
}

void write_geometry_inspection_json(
    std::ostream& output,
    const std::filesystem::path& sourcePath,
    adaptivecad::geometry::TopologyHealingPolicy healingPolicy,
    const adaptivecad::tool::SceneDocument& document,
    const adaptivecad::tool::BuiltScene& scene) {
    output << "{\n";
    output << "  \"command\": \"inspect-geometry\",\n";
    output << "  \"source\": ";
    append_json_string(output, sourcePath.string());
    output << ",\n  \"label\": ";
    append_json_string(output, document.scene_label);
    output << ",\n  \"source_kind\": ";
    append_json_string(output, adaptivecad::tool::scene_source_kind_name(document.source_kind));
    output << ",\n  \"backend\": ";
    append_json_string(output, scene.backend_name);
    output << ",\n  \"healing_policy\": ";
    append_json_string(output, adaptivecad::geometry::BRepKernel::healing_policy_name(healingPolicy));
    output << ",\n  \"materials\": " << document.materials.size() << ",\n";
    output << "  \"bodies\": " << scene.bodies.size() << ",\n";
    output << "  \"vertices\": " << scene.vertices.size() << ",\n";
    output << "  \"edges\": " << scene.edges.size() << ",\n";
    output << "  \"faces\": " << scene.faces.size() << ",\n";
    output << "  \"diagnostic\": {\n";
    output << "    \"success\": " << (scene.diagnostic.success ? "true" : "false") << ",\n";
    output << "    \"operation\": ";
    append_json_string(output, scene.diagnostic.operation);
    output << ",\n    \"backend\": ";
    append_json_string(output, scene.diagnostic.backend);
    output << ",\n    \"message\": ";
    append_json_string(output, scene.diagnostic.message);
    output << ",\n    \"healing_attempted\": " << (scene.diagnostic.healing_attempted ? "true" : "false") << ",\n";
    output << "    \"healing_succeeded\": " << (scene.diagnostic.healing_succeeded ? "true" : "false") << ",\n";
    output << "    \"invalid_edge_ids\": ";
    append_json_entity_ids(output, scene.diagnostic.invalid_edge_ids);
    output << "\n  },\n";
    output << "  \"body_infos\": [\n";
    for (std::size_t index = 0; index < scene.body_infos.size(); ++index) {
        const auto& bodyInfo = scene.body_infos[index];
        output << "    {\"id\": " << bodyInfo.body_id << ", \"name\": ";
        append_json_string(output, bodyInfo.name);
        output << ", \"material\": ";
        append_json_string(output, bodyInfo.material_name);
        output << ", \"has_color\": " << (bodyInfo.has_color ? "true" : "false");
        if (bodyInfo.has_color) {
            output << ", \"color\": ";
            append_json_scene_color(output, bodyInfo.color);
        }
        output << '}';
        if (index + 1 < scene.body_infos.size()) {
            output << ',';
        }
        output << '\n';
    }
    output << "  ]\n";
    output << "}\n";
}

int inspect_geometry(
    const std::filesystem::path& sourcePath,
    adaptivecad::geometry::TopologyHealingPolicy healingPolicy,
    bool jsonOutput) {
    const adaptivecad::tool::SceneDocument document = adaptivecad::tool::import_scene_document(sourcePath);
    const adaptivecad::tool::BuiltScene scene =
        adaptivecad::tool::build_scene_geometry(document, healingPolicy);

    if (jsonOutput) {
        write_geometry_inspection_json(std::cout, sourcePath, healingPolicy, document, scene);
        return scene.diagnostic.success ? 0 : 1;
    }

    std::cout << "Geometry inspection\n";
    std::cout << "source = " << sourcePath.string() << '\n';
    std::cout << "label = " << document.scene_label << '\n';
    std::cout << "source_kind = " << adaptivecad::tool::scene_source_kind_name(document.source_kind) << '\n';
    std::cout << "backend = " << scene.backend_name << '\n';
    std::cout << "healing_policy = "
              << adaptivecad::geometry::BRepKernel::healing_policy_name(healingPolicy) << '\n';
    std::cout << "materials = " << document.materials.size() << '\n';
    std::cout << "bodies = " << scene.bodies.size() << '\n';
    std::cout << "vertices = " << scene.vertices.size() << '\n';
    std::cout << "edges = " << scene.edges.size() << '\n';
    std::cout << "faces = " << scene.faces.size() << '\n';
    std::cout << "diagnostic_success = " << (scene.diagnostic.success ? "true" : "false") << '\n';
    std::cout << "diagnostic_message = " << scene.diagnostic.message << '\n';
    for (const auto& body : scene.body_infos) {
        std::cout << "body[" << body.body_id << "] = " << body.name << '\n';
        if (!body.material_name.empty()) {
            std::cout << "body[" << body.body_id << "].material = " << body.material_name << '\n';
        }
        if (body.has_color) {
            std::cout << "body[" << body.body_id << "].color = "
                      << body.color.red << ',' << body.color.green << ','
                      << body.color.blue << ',' << body.color.alpha << '\n';
        }
    }

    return scene.diagnostic.success ? 0 : 1;
}

int export_geometry(
    const std::filesystem::path& sourcePath,
    const std::filesystem::path& targetPath,
    adaptivecad::geometry::TopologyHealingPolicy healingPolicy) {
    const adaptivecad::tool::SceneDocument document = adaptivecad::tool::import_scene_document(sourcePath);
    const adaptivecad::tool::BuiltScene scene =
        adaptivecad::tool::build_scene_geometry(document, healingPolicy);
    if (!scene.diagnostic.success) {
        std::cerr << "Geometry validation failed: " << scene.diagnostic.message << '\n';
        return 1;
    }

    adaptivecad::tool::export_scene_document(document, targetPath);
    std::cout << "Exported geometry\n";
    std::cout << "source = " << sourcePath.string() << '\n';
    std::cout << "target = " << targetPath.string() << '\n';
    std::cout << "bodies = " << scene.bodies.size() << '\n';
    std::cout << "vertices = " << scene.vertices.size() << '\n';
    std::cout << "faces = " << scene.faces.size() << '\n';
    return 0;
}

int create_primitive(
    const std::string& kind,
    const std::filesystem::path& targetPath,
    adaptivecad::geometry::TopologyHealingPolicy healingPolicy,
    const PrimitiveCreateOptions& options) {
    const adaptivecad::tool::SceneDocument document = create_primitive_document(kind, options);
    const adaptivecad::tool::BuiltScene scene =
        adaptivecad::tool::build_scene_geometry(document, healingPolicy);
    if (!scene.diagnostic.success) {
        std::cerr << "Primitive validation failed: " << scene.diagnostic.message << '\n';
        return 1;
    }

    adaptivecad::tool::export_scene_document(document, targetPath);
    std::cout << "Created primitive\n";
    std::cout << "kind = " << kind << '\n';
    std::cout << "target = " << targetPath.string() << '\n';
    std::cout << "bodies = " << scene.bodies.size() << '\n';
    std::cout << "vertices = " << scene.vertices.size() << '\n';
    std::cout << "faces = " << scene.faces.size() << '\n';
    return 0;
}

int run_demo() {
    using adaptivecad::core::AdaptiveField;
    using adaptivecad::core::AreaSample;
    using adaptivecad::core::FitResultPowerLaw;
    using adaptivecad::core::InverseRecovery;
    using adaptivecad::core::ModelComparisonOptions;
    using adaptivecad::core::NonlinearFitOptions;
    using adaptivecad::geometry::BRepKernel;
    using adaptivecad::geometry::Edge;
    using adaptivecad::geometry::Face;
    using adaptivecad::geometry::PathCostWeights;
    using adaptivecad::geometry::PhaseLiftState;
    using adaptivecad::geometry::TopologyHealingPolicy;
    using adaptivecad::geometry::Vertex;
    using adaptivecad::tool::MetamaterialScaffoldParameters;

    const AdaptiveField field = AdaptiveField::PowerLawRadial(1.05, 0.25, 1.0);
    const double radius = 2.0;

    std::cout << "AdaptiveCAD CLI demo\n";
    std::cout << "pi_f(" << radius << ") = " << field.pi_f(radius) << '\n';
    std::cout << "lambda_f(" << radius << ") = " << field.lambda_f(radius) << '\n';
    std::cout << "eta(" << radius << ") = " << field.eta(radius) << '\n';
    std::cout << "shell_weight(" << radius << ") = " << field.shell_weight(radius) << '\n';
    std::cout << "effective_dimension = " << field.effective_dimension() << '\n';

    const std::vector<AreaSample> samples = {
        {1.0, field.area_functional(1.0)},
        {1.5, field.area_functional(1.5)},
        {2.0, field.area_functional(2.0)},
        {2.5, field.area_functional(2.5)}
    };

    const FitResultPowerLaw fit = InverseRecovery::fitPowerLawFromArea(samples, 1.0);
    std::cout << "Recovered beta = " << fit.beta << '\n';
    std::cout << "Recovered lambda0 = " << fit.lambda0 << '\n';
    std::cout << "RMS relative fit error = " << fit.rms_relative_error << '\n';

    const AdaptiveField gaussianField = AdaptiveField::GaussianDefect(0.20, 3.0);
    std::cout << "Gaussian lambda_f(2) = " << gaussianField.lambda_f(2.0) << '\n';

    const AdaptiveField memoryField = AdaptiveField::MemoryBranch(0.40, 0.20, 0.10, 0.50);
    std::cout << "Memory eta(t=0) = " << memoryField.eta_at_time(2.0, 0.0) << '\n';
    std::cout << "Memory eta(t=5) = " << memoryField.eta_at_time(2.0, 5.0) << '\n';

    const AdaptiveField angularField = AdaptiveField::AngularWeight(1.10, {0.08}, {0.02});
    const double theta = std::numbers::pi_v<double> / 2.0;
    std::cout << "Angular lambda(theta=pi/2) = " << angularField.lambda_theta(theta) << '\n';
    std::cout << "Angular phase_map(pi/2) = " << angularField.phase_map(theta) << '\n';
    std::cout << "Angular cos_f^(2)(pi/2) = " << angularField.adaptive_mode_cos(2, theta) << '\n';

    const AdaptiveField gaussianTruth = AdaptiveField::GaussianDefect(0.15, 2.40);
    const std::vector<AreaSample> gaussianSamples = {
        {0.7, gaussianTruth.area_functional(0.7)},
        {1.0, gaussianTruth.area_functional(1.0)},
        {1.5, gaussianTruth.area_functional(1.5)},
        {2.0, gaussianTruth.area_functional(2.0)},
        {2.7, gaussianTruth.area_functional(2.7)},
        {3.2, gaussianTruth.area_functional(3.2)}
    };

    NonlinearFitOptions nonlinearOptions;
    nonlinearOptions.max_iterations = 200;
    nonlinearOptions.use_huber = false;

    const auto nonlinearFit =
        InverseRecovery::fitGaussianFromAreaNonlinear(gaussianSamples, 0.05, 1.2, nonlinearOptions);

    ModelComparisonOptions comparisonOptions;
    comparisonOptions.r0 = 1.0;
    comparisonOptions.initial_gaussian_epsilon = 0.05;
    comparisonOptions.initial_gaussian_length_scale = 1.2;
    comparisonOptions.gaussian_options = nonlinearOptions;
    const auto modelComparison = InverseRecovery::compareAreaModels(gaussianSamples, comparisonOptions);

    std::cout << "Gaussian nonlinear fit epsilon = " << nonlinearFit.epsilon
              << " (95% CI: " << nonlinearFit.epsilon_ci95_low << ", " << nonlinearFit.epsilon_ci95_high << ")\n";
    std::cout << "Gaussian nonlinear fit length_scale = " << nonlinearFit.length_scale
              << " (95% CI: " << nonlinearFit.length_scale_ci95_low
              << ", " << nonlinearFit.length_scale_ci95_high << ")\n";
    std::cout << "Gaussian nonlinear converged = " << (nonlinearFit.converged ? "true" : "false") << '\n';
    std::cout << "Model comparison best = "
              << InverseRecovery::modelKindName(modelComparison.best_model)
              << " (weight=" << modelComparison.best_model_weight << ")\n";
    std::cout << "Model score constant: rms=" << modelComparison.constant_score.rms_relative_error
              << " aic=" << modelComparison.constant_score.aic
              << " bic=" << modelComparison.constant_score.bic
              << " weight=" << modelComparison.constant_weight << '\n';
    std::cout << "Model score power-law: rms=" << modelComparison.power_law_score.rms_relative_error
              << " aic=" << modelComparison.power_law_score.aic
              << " bic=" << modelComparison.power_law_score.bic
              << " weight=" << modelComparison.power_law_weight << '\n';
    std::cout << "Model score gaussian: rms=" << modelComparison.gaussian_score.rms_relative_error
              << " aic=" << modelComparison.gaussian_score.aic
              << " bic=" << modelComparison.gaussian_score.bic
              << " weight=" << modelComparison.gaussian_weight << '\n';
    if (!modelComparison.warning.empty()) {
        std::cout << "Model comparison warning = " << modelComparison.warning << '\n';
    }

    const auto flatCorrection = adaptivecad::geometry::make_flat_pi_correction(0.08);
    const PhaseLiftState liftedPhase = adaptivecad::geometry::advance_phase_lift(
        PhaseLiftState{1.75 * std::numbers::pi_v<double>, 0, false},
        0.75 * std::numbers::pi_v<double>);
    const auto benchyBenchmark = adaptivecad::geometry::make_benchy_manufacturing_benchmark();
    PathCostWeights manufacturingWeights;
    manufacturingWeights.support = 4.0;
    manufacturingWeights.heat = 3.0;
    manufacturingWeights.trust_reward = 0.2;
    const auto manufacturingRoute =
        adaptivecad::geometry::compute_adaptive_geodesic(benchyBenchmark, 0, 2, manufacturingWeights);
    const auto torusBenchmark = adaptivecad::geometry::make_torus_phase_lift_benchmark(8, 2.0);

    std::cout << "History-aware flat pi_f(eta=0.08) = " << flatCorrection.pi_f << '\n';
    std::cout << "Phase-Lift crossing: theta_R=" << liftedPhase.theta_R
              << " winding=" << liftedPhase.winding
              << " parity=" << (liftedPhase.branch_parity ? "odd" : "even") << '\n';
    std::cout << "Benchy-style adaptive route found = " << (manufacturingRoute.found ? "true" : "false")
              << " cost=" << manufacturingRoute.cost
              << " node_count=" << manufacturingRoute.node_path.size() << '\n';
    std::cout << "Torus Phase-Lift benchmark: nodes=" << torusBenchmark.nodes.size()
              << " seam_winding_delta=" << torusBenchmark.edges.back().winding_delta << '\n';

    MetamaterialScaffoldParameters scaffoldParameters;
    scaffoldParameters.cells_x = 3;
    scaffoldParameters.cells_y = 2;
    scaffoldParameters.cells_z = 1;
    scaffoldParameters.angular_model.lambda0 = 1.0;
    scaffoldParameters.angular_model.cosine_coefficients = {0.20};
    scaffoldParameters.damage_sphere.center = adaptivecad::geometry::Point3D{0.0, 0.0, 0.0};
    scaffoldParameters.damage_sphere.radius = 0.60;

    const auto scaffold = adaptivecad::tool::generate_chern_locked_metamaterial_scaffold(scaffoldParameters);
    const auto scaffoldScene =
        adaptivecad::tool::build_scene_geometry(scaffold.scene_document, TopologyHealingPolicy::Strict);
    std::cout << "Generated scaffold: nodes=" << scaffold.node_count
              << " candidate_struts=" << scaffold.candidate_strut_count
              << " removed_struts=" << scaffold.removed_strut_count
              << " surviving_struts=" << scaffold.struts.size()
              << " connectors=" << scaffold.connectors.size()
              << " transport_span=" << scaffold.transport_span << '\n';
    std::cout << "Scaffold build: bodies=" << scaffoldScene.bodies.size()
              << " faces=" << scaffoldScene.faces.size()
              << " backend=" << scaffoldScene.backend_name << '\n';

    BRepKernel brep = BRepKernel::create_preferred(true);
    brep.set_healing_policy(TopologyHealingPolicy::RepairFirst);
    const Vertex v1 = brep.create_vertex(0.0, 0.0, 0.0);
    const Vertex v2 = brep.create_vertex(1.0, 0.0, 0.0);
    const Vertex v3 = brep.create_vertex(0.0, 1.0, 0.0);
    const Edge e1 = brep.create_edge(v1, v2);
    const Edge e2 = brep.create_edge(v2, v3);
    const Edge e3 = brep.create_edge(v3, v1);
    const Face f1 = brep.create_face_from_edges({e1, e2, e3}, 0.5);
    const auto body = brep.create_body_from_faces({f1}, 0.0);
    const auto torusBody = brep.create_torus(2.0, 0.35, 24, 12);
    const auto& topologyDiagnostic = brep.last_topology_diagnostic();

    std::cout << "Geometry backend = " << brep.backend_name() << '\n';
    std::cout << "Healing policy = " << BRepKernel::healing_policy_name(brep.healing_policy()) << '\n';
    std::cout << "OpenCascade native topology active = "
              << (brep.has_open_cascade_topology() ? "true" : "false") << '\n';
    std::cout << "Euclidean circle area r=2 = " << brep.geometry().circle_area(2.0) << '\n';
    std::cout << "Created vertex ids = [" << v1.id << ", " << v2.id << ", " << v3.id << "]\n";
    std::cout << "Edge length e1 = " << e1.length << '\n';
    std::cout << "Edge native handle type = "
              << (e1.native_handle.is_valid() ? e1.native_handle.type_name : "none") << '\n';
    std::cout << "Face native handle type = "
              << (f1.native_handle.is_valid() ? f1.native_handle.type_name : "none") << '\n';
    std::cout << "Body native handle type = "
              << (body.native_handle.is_valid() ? body.native_handle.type_name : "none") << '\n';
    std::cout << "Torus body id = " << torusBody.id
              << ", volume = " << torusBody.volume
              << ", native handle type = "
              << (torusBody.native_handle.is_valid() ? torusBody.native_handle.type_name : "faceted fallback")
              << ", face count = " << torusBody.face_ids.size() << '\n';
    std::cout << "Last topology diagnostic: success=" << (topologyDiagnostic.success ? "true" : "false")
              << " op=" << topologyDiagnostic.operation
              << " invalid_edge_count=" << topologyDiagnostic.invalid_edge_ids.size() << '\n';
    std::cout << "Face id = " << f1.id << ", body id = " << body.id << '\n';
    std::cout << "BRep counts: V=" << brep.vertex_count() << " E=" << brep.edge_count() << " F="
              << brep.face_count() << " B=" << brep.body_count() << '\n';

    return 0;
}

} // namespace

int main(int argc, char** argv) {
    try {
        if (argc <= 1) {
            return run_demo();
        }

        const std::string command = argv[1] ? argv[1] : "";
        if (command == "--help" || command == "-h" || command == "help") {
            print_usage(std::cout);
            return 0;
        }
        if (command == "--version" || command == "version") {
            print_version(std::cout);
            return 0;
        }
        if (command == "--demo" || command == "demo") {
            return run_demo();
        }
        if (command == "--inspect-geometry" || command == "inspect-geometry" || command == "--import-geometry") {
            if (argc < 3) {
                throw std::runtime_error("--inspect-geometry requires a geometry file path");
            }

            std::filesystem::path sourcePath = argv[2];
            bool jsonOutput = false;
            adaptivecad::geometry::TopologyHealingPolicy healingPolicy =
                adaptivecad::geometry::TopologyHealingPolicy::RepairFirst;
            for (int index = 3; index < argc; ++index) {
                const std::string option = argv[index] ? argv[index] : "";
                if (option == "--healing") {
                    healingPolicy = parse_healing_policy(read_option_value(argc, argv, &index, option));
                    continue;
                }
                if (option == "--json") {
                    jsonOutput = true;
                    continue;
                }
                throw std::runtime_error("unknown option for --inspect-geometry: " + option);
            }

            return inspect_geometry(sourcePath, healingPolicy, jsonOutput);
        }
        if (command == "--export-geometry" || command == "export-geometry") {
            if (argc < 3) {
                throw std::runtime_error("--export-geometry requires a geometry file path");
            }

            std::filesystem::path sourcePath = argv[2];
            std::filesystem::path targetPath;
            adaptivecad::geometry::TopologyHealingPolicy healingPolicy =
                adaptivecad::geometry::TopologyHealingPolicy::RepairFirst;
            for (int index = 3; index < argc; ++index) {
                const std::string option = argv[index] ? argv[index] : "";
                if (option == "--healing") {
                    if (index + 1 >= argc) {
                        throw std::runtime_error("--healing requires a policy value");
                    }
                    healingPolicy = parse_healing_policy(argv[++index]);
                    continue;
                }
                if (option == "--output" || option == "-o") {
                    if (index + 1 >= argc) {
                        throw std::runtime_error("--output requires a file path");
                    }
                    targetPath = argv[++index];
                    continue;
                }
                throw std::runtime_error("unknown option for --export-geometry: " + option);
            }

            if (targetPath.empty()) {
                targetPath = sourcePath;
                targetPath.replace_extension(".obj");
                if (targetPath == sourcePath) {
                    targetPath = sourcePath.parent_path() / (sourcePath.stem().string() + "_export.obj");
                }
            }

            return export_geometry(sourcePath, targetPath, healingPolicy);
        }
        if (command == "--create-primitive" || command == "create-primitive") {
            if (argc < 3) {
                throw std::runtime_error("--create-primitive requires a primitive kind");
            }

            const std::string kind = argv[2] ? argv[2] : "";
            PrimitiveCreateOptions primitiveOptions = default_primitive_create_options(kind);
            std::filesystem::path targetPath;
            adaptivecad::geometry::TopologyHealingPolicy healingPolicy =
                adaptivecad::geometry::TopologyHealingPolicy::RepairFirst;
            for (int index = 3; index < argc; ++index) {
                const std::string option = argv[index] ? argv[index] : "";
                if (option == "--healing") {
                    healingPolicy = parse_healing_policy(read_option_value(argc, argv, &index, option));
                    continue;
                }
                if (option == "--output" || option == "-o") {
                    targetPath = read_option_value(argc, argv, &index, option);
                    continue;
                }
                if (option == "--width") {
                    primitiveOptions.width = parse_positive_double(option, read_option_value(argc, argv, &index, option));
                    continue;
                }
                if (option == "--depth") {
                    primitiveOptions.depth = parse_positive_double(option, read_option_value(argc, argv, &index, option));
                    continue;
                }
                if (option == "--height") {
                    primitiveOptions.height = parse_positive_double(option, read_option_value(argc, argv, &index, option));
                    continue;
                }
                if (option == "--major-radius") {
                    primitiveOptions.major_radius = parse_positive_double(option, read_option_value(argc, argv, &index, option));
                    continue;
                }
                if (option == "--minor-radius") {
                    primitiveOptions.minor_radius = parse_positive_double(option, read_option_value(argc, argv, &index, option));
                    continue;
                }
                if (option == "--major-segments") {
                    primitiveOptions.major_segments = static_cast<std::size_t>(
                        parse_int_at_least(option, read_option_value(argc, argv, &index, option), 3));
                    continue;
                }
                if (option == "--minor-segments") {
                    primitiveOptions.minor_segments = static_cast<std::size_t>(
                        parse_int_at_least(option, read_option_value(argc, argv, &index, option), 3));
                    continue;
                }
                if (option == "--cells-x") {
                    primitiveOptions.cells_x = parse_int_at_least(option, read_option_value(argc, argv, &index, option), 1);
                    continue;
                }
                if (option == "--cells-y") {
                    primitiveOptions.cells_y = parse_int_at_least(option, read_option_value(argc, argv, &index, option), 0);
                    continue;
                }
                if (option == "--cells-z") {
                    primitiveOptions.cells_z = parse_int_at_least(option, read_option_value(argc, argv, &index, option), 0);
                    continue;
                }
                if (option == "--spacing") {
                    primitiveOptions.spacing = parse_positive_double(option, read_option_value(argc, argv, &index, option));
                    continue;
                }
                if (option == "--damage-radius") {
                    primitiveOptions.damage_radius = parse_nonnegative_double(option, read_option_value(argc, argv, &index, option));
                    continue;
                }
                throw std::runtime_error("unknown option for --create-primitive: " + option);
            }

            if (targetPath.empty()) {
                targetPath = kind + std::string(".obj");
            }

            return create_primitive(kind, targetPath, healingPolicy, primitiveOptions);
        }

        throw std::runtime_error("unknown command: " + command);
    } catch (const std::exception& ex) {
        std::cerr << "adaptivecad: " << ex.what() << "\n\n";
        print_usage(std::cerr);
        return 2;
    }
}
