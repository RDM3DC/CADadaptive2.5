#include "adaptivecad/core/AdaptiveField.hpp"
#include "adaptivecad/core/InverseRecovery.hpp"
#include "adaptivecad/geometry/BRepKernel.hpp"
#include "adaptivecad/tool/MetamaterialScaffold.hpp"
#include "adaptivecad/tool/SceneDocument.hpp"
#include "adaptivecad/tool/SessionBundleIO.hpp"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <commdlg.h>
#include <commctrl.h>
#include <shellapi.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <chrono>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr int kRefreshButtonId = 1001;
constexpr int kCloseButtonId = 1002;
constexpr int kOutputEditId = 1003;
constexpr int kOpenSpecButtonId = 1004;
constexpr int kExportButtonId = 1005;
constexpr int kStatusLabelId = 1006;
constexpr int kHealingPolicyComboId = 1007;
constexpr int kHealingPolicyLabelId = 1008;
constexpr int kImportCsvButtonId = 1009;
constexpr int kRefitButtonId = 1010;
constexpr int kClearDatasetButtonId = 1011;
constexpr int kDatasetGroupId = 1012;
constexpr int kDatasetPathLabelId = 1013;
constexpr int kDatasetValidLabelId = 1014;
constexpr int kDatasetRejectedLabelId = 1015;
constexpr int kDatasetSummaryLabelId = 1016;
constexpr int kPlotGroupId = 1017;
constexpr int kPlotPanelId = 1018;
constexpr int kViewportGroupId = 1019;
constexpr int kViewportPanelId = 1020;
constexpr int kViewportProjectionLabelId = 1021;
constexpr int kViewportProjectionComboId = 1022;
constexpr int kViewportResetButtonId = 1023;
constexpr int kModelTreeGroupId = 1024;
constexpr int kModelTreeViewId = 1025;
constexpr int kSaveSessionButtonId = 1026;
constexpr int kLoadSessionButtonId = 1027;
constexpr int kImportGeometryButtonId = 1028;
constexpr int kAngularGroupId = 1029;
constexpr int kAngularLambdaLabelId = 1030;
constexpr int kAngularLambdaEditId = 1031;
constexpr int kAngularCosLabelId = 1032;
constexpr int kAngularCosEditId = 1033;
constexpr int kAngularSinLabelId = 1034;
constexpr int kAngularSinEditId = 1035;
constexpr int kAngularApplyButtonId = 1036;
constexpr int kAngularRevertButtonId = 1037;
constexpr int kExportGeometryButtonId = 1038;
constexpr int kCreateBoxButtonId = 1039;
constexpr int kCreateWedgeButtonId = 1040;
constexpr int kCreatePlaneButtonId = 1041;
constexpr int kCreateTorusButtonId = 1042;
constexpr int kCreateScaffoldButtonId = 1043;
constexpr int kPrimitiveGroupId = 1044;
constexpr int kPrimitiveTypeComboId = 1045;
constexpr int kPrimitiveParam1LabelId = 1046;
constexpr int kPrimitiveParam1EditId = 1047;
constexpr int kPrimitiveParam2LabelId = 1048;
constexpr int kPrimitiveParam2EditId = 1049;
constexpr int kPrimitiveParam3LabelId = 1050;
constexpr int kPrimitiveParam3EditId = 1051;
constexpr int kPrimitiveParam4LabelId = 1052;
constexpr int kPrimitiveParam4EditId = 1053;
constexpr int kPrimitiveParam5LabelId = 1054;
constexpr int kPrimitiveParam5EditId = 1055;
constexpr int kCreatePrimitiveButtonId = 1056;
constexpr int kPrimitiveTypeLabelId = 1057;
constexpr int kPrimitiveAppendCheckId = 1058;
constexpr int kBodyEditGroupId = 1059;
constexpr int kBodyNameLabelId = 1060;
constexpr int kBodyNameEditId = 1061;
constexpr int kBodyDxLabelId = 1062;
constexpr int kBodyDxEditId = 1063;
constexpr int kBodyDyLabelId = 1064;
constexpr int kBodyDyEditId = 1065;
constexpr int kBodyDzLabelId = 1066;
constexpr int kBodyDzEditId = 1067;
constexpr int kBodyScaleLabelId = 1068;
constexpr int kBodyScaleEditId = 1069;
constexpr int kBodyRotateLabelId = 1070;
constexpr int kBodyRotateEditId = 1071;
constexpr int kApplyBodyTransformButtonId = 1072;
constexpr int kDuplicateBodyButtonId = 1073;
constexpr int kDeleteBodyButtonId = 1074;
constexpr int kRenameBodyButtonId = 1075;

constexpr const char* kSpecFileName = "adaptive_cad_flat_adasccsc.md";
constexpr const char* kPlotWindowClassName = "AdaptiveCAD_UI_PlotPanel";
constexpr const char* kViewportWindowClassName = "AdaptiveCAD_UI_ViewportPanel";

HWND g_outputEdit = nullptr;
HWND g_statusLabel = nullptr;
HWND g_healingPolicyCombo = nullptr;
HWND g_datasetGroup = nullptr;
HWND g_datasetPathLabel = nullptr;
HWND g_datasetValidLabel = nullptr;
HWND g_datasetRejectedLabel = nullptr;
HWND g_datasetSummaryLabel = nullptr;
HWND g_angularGroup = nullptr;
HWND g_angularLambdaEdit = nullptr;
HWND g_angularCosEdit = nullptr;
HWND g_angularSinEdit = nullptr;
HWND g_primitiveGroup = nullptr;
HWND g_primitiveTypeCombo = nullptr;
HWND g_primitiveParam1Edit = nullptr;
HWND g_primitiveParam2Edit = nullptr;
HWND g_primitiveParam3Edit = nullptr;
HWND g_primitiveParam4Edit = nullptr;
HWND g_primitiveParam5Edit = nullptr;
HWND g_primitiveAppendCheck = nullptr;
HWND g_bodyEditGroup = nullptr;
HWND g_bodyNameEdit = nullptr;
HWND g_bodyDxEdit = nullptr;
HWND g_bodyDyEdit = nullptr;
HWND g_bodyDzEdit = nullptr;
HWND g_bodyScaleEdit = nullptr;
HWND g_bodyRotateEdit = nullptr;
HWND g_plotGroup = nullptr;
HWND g_plotPanel = nullptr;
HWND g_modelTreeGroup = nullptr;
HWND g_modelTreeView = nullptr;
HWND g_viewportGroup = nullptr;
HWND g_viewportPanel = nullptr;
HWND g_viewportProjectionCombo = nullptr;
HTREEITEM g_modelTreeRootItem = nullptr;
bool g_treeSelectionSyncInProgress = false;
std::string g_lastOutput;
std::filesystem::path g_workspaceRoot;
adaptivecad::geometry::TopologyHealingPolicy g_healingPolicy =
    adaptivecad::geometry::TopologyHealingPolicy::RepairFirst;
std::vector<adaptivecad::core::AreaSample> g_importedAreaSamples;
std::filesystem::path g_importedCsvPath;
adaptivecad::tool::SceneDocument g_sceneDocument = adaptivecad::tool::make_demo_scene_document();

struct MeasurementDatasetInfo {
    bool using_imported_csv = false;
    std::filesystem::path active_csv_path;
    std::size_t valid_rows = 0;
    std::size_t rejected_rows = 0;
    std::string summary;
};

MeasurementDatasetInfo g_datasetInfo;

struct PlotPoint {
    double x = 0.0;
    double y = 0.0;
};

struct FitPlotState {
    std::vector<PlotPoint> measured_points;
    std::vector<PlotPoint> constant_curve;
    std::vector<PlotPoint> power_law_curve;
    std::vector<PlotPoint> gaussian_curve;
    std::vector<PlotPoint> residual_points;
    adaptivecad::core::FitModelKind best_model = adaptivecad::core::FitModelKind::Gaussian;
    std::string title;
    std::string subtitle;
    std::string residual_caption;
    std::string error_message;
    bool ready = false;
};

FitPlotState g_plotState;

enum class ViewportSelectionKind {
    None,
    Vertex,
    Edge,
    Face,
    Body,
};

enum class ViewportProjectionMode {
    Top,
    Front,
    Right,
    Isometric,
};

struct ViewportPoint {
    int x = 0;
    int y = 0;
};

struct ProjectedGeometryPoint {
    double x = 0.0;
    double y = 0.0;
};

struct GeometryViewportState {
    std::vector<adaptivecad::geometry::Vertex> vertices;
    std::vector<adaptivecad::geometry::Edge> edges;
    std::vector<adaptivecad::geometry::Face> faces;
    std::vector<adaptivecad::geometry::Body> bodies;
    std::vector<adaptivecad::tool::SceneFaceOutline> face_outlines;
    std::vector<adaptivecad::tool::SceneBodyInfo> body_infos;
    adaptivecad::geometry::TopologyDiagnostic diagnostic;
    std::string backend_name;
    std::string scene_label;
    std::string scene_source_kind;
    std::string scene_source_path;
    std::string subtitle;
    std::string diagnostic_summary;
    ViewportSelectionKind selection_kind = ViewportSelectionKind::None;
    adaptivecad::geometry::EntityId selected_id = 0;
    bool has_open_cascade_topology = false;
    bool ready = false;
    std::string error_message;
};

GeometryViewportState g_geometryViewportState;

struct GeometryViewportViewState {
    ViewportProjectionMode projection = ViewportProjectionMode::Isometric;
    double zoom = 1.0;
    double pan_x = 0.0;
    double pan_y = 0.0;
    bool panning = false;
    POINT last_pan_point{};
};

GeometryViewportViewState g_geometryViewportViewState;

struct ActiveFitAnalysisState {
    double pi_f_at_2 = 0.0;
    double lambda_f_at_2 = 0.0;
    double eta_at_2 = 0.0;
    double shell_weight_at_2 = 0.0;
    double effective_dimension = 0.0;
    adaptivecad::core::FitResultPowerLaw power_law_demo_fit;
    adaptivecad::core::AngularWeightModel angular_model;
    double angular_lambda_pi_over_2 = 0.0;
    double angular_phase_map_pi_over_2 = 0.0;
    double angular_cos_mode_2_pi_over_2 = 0.0;
    std::vector<adaptivecad::core::AreaSample> measurement_samples;
    std::string measurement_source;
    adaptivecad::core::FitResultGaussianNonlinear nonlinear_fit;
    adaptivecad::core::ModelComparisonResult model_comparison;
    bool ready = false;
};

ActiveFitAnalysisState g_activeFitAnalysisState;

std::string format_plot_number(double value, int precision);
std::string trim_copy(const std::string& input);
bool try_parse_double(const std::string& text, double* value);
std::string viewport_selection_label();
void invalidate_viewport();
void reset_viewport_interaction();
void refresh_output(HWND ownerWindow);
void update_angular_branch_controls();
void apply_angular_branch_controls(HWND ownerWindow);
void revert_angular_branch_controls();
void update_primitive_parameter_controls();
void update_body_edit_controls();

int healing_policy_to_combo_index(adaptivecad::geometry::TopologyHealingPolicy policy) {
    switch (policy) {
    case adaptivecad::geometry::TopologyHealingPolicy::Strict:
        return 0;
    case adaptivecad::geometry::TopologyHealingPolicy::RepairFirst:
        return 1;
    case adaptivecad::geometry::TopologyHealingPolicy::RepairOnly:
        return 2;
    }

    return 1;
}

adaptivecad::geometry::TopologyHealingPolicy combo_index_to_healing_policy(int index) {
    switch (index) {
    case 0:
        return adaptivecad::geometry::TopologyHealingPolicy::Strict;
    case 1:
        return adaptivecad::geometry::TopologyHealingPolicy::RepairFirst;
    case 2:
        return adaptivecad::geometry::TopologyHealingPolicy::RepairOnly;
    default:
        return adaptivecad::geometry::TopologyHealingPolicy::RepairFirst;
    }
}

int viewport_projection_to_combo_index(ViewportProjectionMode projection) {
    switch (projection) {
    case ViewportProjectionMode::Top:
        return 0;
    case ViewportProjectionMode::Front:
        return 1;
    case ViewportProjectionMode::Right:
        return 2;
    case ViewportProjectionMode::Isometric:
        return 3;
    }

    return 3;
}

ViewportProjectionMode combo_index_to_viewport_projection(int index) {
    switch (index) {
    case 0:
        return ViewportProjectionMode::Top;
    case 1:
        return ViewportProjectionMode::Front;
    case 2:
        return ViewportProjectionMode::Right;
    case 3:
        return ViewportProjectionMode::Isometric;
    default:
        return ViewportProjectionMode::Isometric;
    }
}

const char* viewport_projection_name(ViewportProjectionMode projection) {
    switch (projection) {
    case ViewportProjectionMode::Top:
        return "Top";
    case ViewportProjectionMode::Front:
        return "Front";
    case ViewportProjectionMode::Right:
        return "Right";
    case ViewportProjectionMode::Isometric:
        return "Isometric";
    }

    return "Isometric";
}

void initialize_healing_policy_combo() {
    if (!g_healingPolicyCombo) {
        return;
    }

    SendMessageA(g_healingPolicyCombo, CB_RESETCONTENT, 0, 0);
    SendMessageA(g_healingPolicyCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>("Strict"));
    SendMessageA(g_healingPolicyCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>("RepairFirst"));
    SendMessageA(g_healingPolicyCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>("RepairOnly"));
    SendMessageA(
        g_healingPolicyCombo,
        CB_SETCURSEL,
        static_cast<WPARAM>(healing_policy_to_combo_index(g_healingPolicy)),
        0);
}

void initialize_viewport_projection_combo() {
    if (!g_viewportProjectionCombo) {
        return;
    }

    SendMessageA(g_viewportProjectionCombo, CB_RESETCONTENT, 0, 0);
    SendMessageA(g_viewportProjectionCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>("Top"));
    SendMessageA(g_viewportProjectionCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>("Front"));
    SendMessageA(g_viewportProjectionCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>("Right"));
    SendMessageA(g_viewportProjectionCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>("Isometric"));
    SendMessageA(
        g_viewportProjectionCombo,
        CB_SETCURSEL,
        static_cast<WPARAM>(viewport_projection_to_combo_index(g_geometryViewportViewState.projection)),
        0);
}

enum class PrimitiveControlKind {
    Box,
    Wedge,
    Plane,
    Torus,
    Scaffold
};

PrimitiveControlKind combo_index_to_primitive_kind(int index) {
    switch (index) {
    case 1:
        return PrimitiveControlKind::Wedge;
    case 2:
        return PrimitiveControlKind::Plane;
    case 3:
        return PrimitiveControlKind::Torus;
    case 4:
        return PrimitiveControlKind::Scaffold;
    case 0:
    default:
        return PrimitiveControlKind::Box;
    }
}

PrimitiveControlKind selected_primitive_kind() {
    if (!g_primitiveTypeCombo) {
        return PrimitiveControlKind::Box;
    }

    const LRESULT selection = SendMessageA(g_primitiveTypeCombo, CB_GETCURSEL, 0, 0);
    return combo_index_to_primitive_kind(static_cast<int>(selection));
}

void initialize_primitive_type_combo() {
    if (!g_primitiveTypeCombo) {
        return;
    }

    SendMessageA(g_primitiveTypeCombo, CB_RESETCONTENT, 0, 0);
    SendMessageA(g_primitiveTypeCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>("Box"));
    SendMessageA(g_primitiveTypeCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>("Wedge"));
    SendMessageA(g_primitiveTypeCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>("Plane"));
    SendMessageA(g_primitiveTypeCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>("Torus"));
    SendMessageA(g_primitiveTypeCombo, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>("Scaffold"));
    SendMessageA(g_primitiveTypeCombo, CB_SETCURSEL, 0, 0);
    update_primitive_parameter_controls();
}

void reset_viewport_camera() {
    g_geometryViewportViewState.zoom = 1.0;
    g_geometryViewportViewState.pan_x = 0.0;
    g_geometryViewportViewState.pan_y = 0.0;
    g_geometryViewportViewState.panning = false;
    g_geometryViewportViewState.last_pan_point = POINT{0, 0};
}

bool path_exists(const std::filesystem::path& path) {
    std::error_code ec;
    return std::filesystem::exists(path, ec);
}

std::filesystem::path detect_workspace_root() {
    char modulePath[MAX_PATH] = {};
    const DWORD pathLength = GetModuleFileNameA(nullptr, modulePath, MAX_PATH);

    if (pathLength == 0 || pathLength >= MAX_PATH) {
        return std::filesystem::current_path();
    }

    const std::filesystem::path executablePath(modulePath);
    const std::filesystem::path executableDir = executablePath.parent_path();
    const std::filesystem::path specInExecutableDir = executableDir / kSpecFileName;
    const std::filesystem::path specInParentDir = executableDir.parent_path() / kSpecFileName;

    if (path_exists(specInExecutableDir)) {
        return executableDir;
    }
    if (path_exists(specInParentDir)) {
        return executableDir.parent_path();
    }

    return std::filesystem::current_path();
}

std::string make_timestamp(const char* format) {
    const auto now = std::chrono::system_clock::now();
    const std::time_t nowTime = std::chrono::system_clock::to_time_t(now);

    std::tm localTime = {};
#if defined(_WIN32)
    localtime_s(&localTime, &nowTime);
#else
    localtime_r(&nowTime, &localTime);
#endif

    std::ostringstream out;
    out << std::put_time(&localTime, format);
    return out.str();
}

void set_status_text(const std::string& text) {
    if (!g_statusLabel) {
        return;
    }
    SetWindowTextA(g_statusLabel, text.c_str());
}

std::vector<adaptivecad::core::AreaSample> build_synthetic_gaussian_samples() {
    using adaptivecad::core::AdaptiveField;
    using adaptivecad::core::AreaSample;

    const AdaptiveField gaussianTruth = AdaptiveField::GaussianDefect(0.15, 2.40);
    return {
        AreaSample{0.7, gaussianTruth.area_functional(0.7)},
        AreaSample{1.0, gaussianTruth.area_functional(1.0)},
        AreaSample{1.5, gaussianTruth.area_functional(1.5)},
        AreaSample{2.0, gaussianTruth.area_functional(2.0)},
        AreaSample{2.7, gaussianTruth.area_functional(2.7)},
        AreaSample{3.2, gaussianTruth.area_functional(3.2)}};
}

void set_dataset_info_to_synthetic() {
    const std::vector<adaptivecad::core::AreaSample> synthetic = build_synthetic_gaussian_samples();

    g_datasetInfo.using_imported_csv = false;
    g_datasetInfo.active_csv_path.clear();
    g_datasetInfo.valid_rows = synthetic.size();
    g_datasetInfo.rejected_rows = 0;
    g_datasetInfo.summary = "No CSV imported. Using synthetic demo dataset for nonlinear fit.";
}

void set_dataset_info_import_success(
    const std::filesystem::path& csvPath,
    std::size_t validRows,
    std::size_t rejectedRows,
    const std::string& summary) {
    g_datasetInfo.using_imported_csv = true;
    g_datasetInfo.active_csv_path = csvPath;
    g_datasetInfo.valid_rows = validRows;
    g_datasetInfo.rejected_rows = rejectedRows;
    g_datasetInfo.summary = summary;
}

void set_dataset_info_import_failure(const std::string& failureSummary) {
    if (g_datasetInfo.summary.empty()) {
        g_datasetInfo.summary = failureSummary;
        return;
    }

    g_datasetInfo.summary = failureSummary + " Active dataset unchanged.";
}

void update_dataset_panel_controls() {
    if (g_datasetPathLabel) {
        const std::string pathText = g_datasetInfo.using_imported_csv
            ? ("Active CSV path: " + g_datasetInfo.active_csv_path.string())
            : "Active CSV path: (none; synthetic dataset active)";
        SetWindowTextA(g_datasetPathLabel, pathText.c_str());
    }

    if (g_datasetValidLabel) {
        const std::string validText = "Valid rows: " + std::to_string(g_datasetInfo.valid_rows);
        SetWindowTextA(g_datasetValidLabel, validText.c_str());
    }

    if (g_datasetRejectedLabel) {
        const std::string rejectedText = "Rejected rows: " + std::to_string(g_datasetInfo.rejected_rows);
        SetWindowTextA(g_datasetRejectedLabel, rejectedText.c_str());
    }

    if (g_datasetSummaryLabel) {
        SetWindowTextA(g_datasetSummaryLabel, ("Summary: " + g_datasetInfo.summary).c_str());
    }
}

std::vector<adaptivecad::core::AreaSample> build_active_measurement_samples() {
    if (!g_importedAreaSamples.empty()) {
        return g_importedAreaSamples;
    }

    return build_synthetic_gaussian_samples();
}

std::string active_measurement_source() {
    if (g_datasetInfo.using_imported_csv) {
        return std::string("CSV: ") + g_datasetInfo.active_csv_path.string();
    }

    return "synthetic demo dataset";
}

ActiveFitAnalysisState build_active_fit_analysis_state() {
    using adaptivecad::core::AdaptiveField;
    using adaptivecad::core::AreaSample;
    using adaptivecad::core::InverseRecovery;
    using adaptivecad::core::ModelComparisonOptions;
    using adaptivecad::core::NonlinearFitOptions;

    ActiveFitAnalysisState state{};

    const AdaptiveField field = AdaptiveField::PowerLawRadial(1.05, 0.25, 1.0);
    state.pi_f_at_2 = field.pi_f(2.0);
    state.lambda_f_at_2 = field.lambda_f(2.0);
    state.eta_at_2 = field.eta(2.0);
    state.shell_weight_at_2 = field.shell_weight(2.0);
    state.effective_dimension = field.effective_dimension();

    const std::vector<AreaSample> demoSamples = {
        {1.0, field.area_functional(1.0)},
        {1.5, field.area_functional(1.5)},
        {2.0, field.area_functional(2.0)},
        {2.5, field.area_functional(2.5)}};
    state.power_law_demo_fit = InverseRecovery::fitPowerLawFromArea(demoSamples, 1.0);

    state.angular_model = g_sceneDocument.angular_model;
    const AdaptiveField angularField = AdaptiveField::AngularWeight(
        state.angular_model.lambda0,
        state.angular_model.cosine_coefficients,
        state.angular_model.sine_coefficients);
    const double theta = std::numbers::pi_v<double> / 2.0;
    state.angular_lambda_pi_over_2 = angularField.lambda_theta(theta);
    state.angular_phase_map_pi_over_2 = angularField.phase_map(theta);
    state.angular_cos_mode_2_pi_over_2 = angularField.adaptive_mode_cos(2, theta);

    state.measurement_samples = build_active_measurement_samples();
    state.measurement_source = active_measurement_source();

    NonlinearFitOptions nonlinearOptions;
    nonlinearOptions.max_iterations = 200;
    nonlinearOptions.use_huber = false;

    state.nonlinear_fit =
        InverseRecovery::fitGaussianFromAreaNonlinear(state.measurement_samples, 0.05, 1.2, nonlinearOptions);

    ModelComparisonOptions comparisonOptions;
    comparisonOptions.r0 = 1.0;
    comparisonOptions.initial_gaussian_epsilon = 0.05;
    comparisonOptions.initial_gaussian_length_scale = 1.2;
    comparisonOptions.gaussian_options = nonlinearOptions;
    state.model_comparison = InverseRecovery::compareAreaModels(state.measurement_samples, comparisonOptions);
    state.ready = true;
    return state;
}

const char* viewport_selection_kind_name(ViewportSelectionKind kind) {
    switch (kind) {
    case ViewportSelectionKind::None:
        return "none";
    case ViewportSelectionKind::Vertex:
        return "vertex";
    case ViewportSelectionKind::Edge:
        return "edge";
    case ViewportSelectionKind::Face:
        return "face";
    case ViewportSelectionKind::Body:
        return "body";
    }

    return "none";
}

double constant_model_area(double radius, const adaptivecad::core::FitResultConstant& fit) {
    const double pi = std::numbers::pi_v<double>;
    return pi * fit.lambda_constant * radius * radius;
}

double power_law_model_area(double radius, const adaptivecad::core::FitResultPowerLaw& fit) {
    const double pi = std::numbers::pi_v<double>;
    const double exponent = fit.beta + 2.0;
    return (2.0 * pi * fit.lambda0 / std::pow(fit.r0, fit.beta)) * std::pow(radius, exponent) / exponent;
}

double gaussian_model_area(double radius, const adaptivecad::core::FitResultGaussianNonlinear& fit) {
    const double pi = std::numbers::pi_v<double>;
    const double q = radius / fit.length_scale;
    return pi * radius * radius +
        pi * fit.epsilon * fit.length_scale * fit.length_scale * (1.0 - std::exp(-(q * q)));
}

double best_model_area(double radius, const adaptivecad::core::ModelComparisonResult& comparison) {
    switch (comparison.best_model) {
    case adaptivecad::core::FitModelKind::Constant:
        return constant_model_area(radius, comparison.constant_fit);
    case adaptivecad::core::FitModelKind::PowerLaw:
        return power_law_model_area(radius, comparison.power_law_fit);
    case adaptivecad::core::FitModelKind::Gaussian:
        return gaussian_model_area(radius, comparison.gaussian_fit);
    }

    return gaussian_model_area(radius, comparison.gaussian_fit);
}

COLORREF model_color(adaptivecad::core::FitModelKind model) {
    switch (model) {
    case adaptivecad::core::FitModelKind::Constant:
        return RGB(118, 118, 118);
    case adaptivecad::core::FitModelKind::PowerLaw:
        return RGB(194, 68, 44);
    case adaptivecad::core::FitModelKind::Gaussian:
        return RGB(28, 110, 164);
    }

    return RGB(28, 110, 164);
}

int scene_color_component_to_byte(double value) {
    const double clamped = std::max(0.0, std::min(1.0, value));
    return static_cast<int>(std::lround(clamped * 255.0));
}

COLORREF scene_color_to_colorref(const adaptivecad::tool::SceneColor& color, double blendWithWhite = 0.18) {
    const auto blend = [blendWithWhite](double value) {
        return value * (1.0 - blendWithWhite) + blendWithWhite;
    };
    return RGB(
        scene_color_component_to_byte(blend(color.red)),
        scene_color_component_to_byte(blend(color.green)),
        scene_color_component_to_byte(blend(color.blue)));
}

std::string format_plot_number(double value, int precision = 2) {
    std::ostringstream out;
    const double magnitude = std::abs(value);
    if (magnitude >= 1000.0 || (magnitude > 0.0 && magnitude < 0.01)) {
        out << std::scientific << std::setprecision(2) << value;
    } else {
        out << std::fixed << std::setprecision(precision) << value;
    }
    return out.str();
}

void set_plot_error_state(const std::string& message) {
    g_plotState = {};
    g_plotState.error_message = message;
    g_plotState.ready = false;
    if (g_plotPanel) {
        InvalidateRect(g_plotPanel, nullptr, TRUE);
    }
}

void set_geometry_error_state(const std::string& message) {
    g_geometryViewportState = {};
    g_geometryViewportState.error_message = message;
    g_geometryViewportState.ready = false;
    if (g_viewportPanel) {
        InvalidateRect(g_viewportPanel, nullptr, TRUE);
    }
}

void extend_range(double value, double* minValue, double* maxValue) {
    if (!std::isfinite(value) || !minValue || !maxValue) {
        return;
    }

    *minValue = std::min(*minValue, value);
    *maxValue = std::max(*maxValue, value);
}

const adaptivecad::geometry::Vertex* find_viewport_vertex(adaptivecad::geometry::EntityId id) {
    for (const auto& vertex : g_geometryViewportState.vertices) {
        if (vertex.id == id) {
            return &vertex;
        }
    }
    return nullptr;
}

const adaptivecad::geometry::Edge* find_viewport_edge(adaptivecad::geometry::EntityId id) {
    for (const auto& edge : g_geometryViewportState.edges) {
        if (edge.id == id) {
            return &edge;
        }
    }
    return nullptr;
}

const adaptivecad::geometry::Face* find_viewport_face(adaptivecad::geometry::EntityId id) {
    for (const auto& face : g_geometryViewportState.faces) {
        if (face.id == id) {
            return &face;
        }
    }
    return nullptr;
}

const adaptivecad::geometry::Body* find_viewport_body(adaptivecad::geometry::EntityId id) {
    for (const auto& body : g_geometryViewportState.bodies) {
        if (body.id == id) {
            return &body;
        }
    }
    return nullptr;
}

LPARAM encode_model_tree_payload(ViewportSelectionKind kind, adaptivecad::geometry::EntityId id) {
    const std::uint64_t kindValue = static_cast<std::uint64_t>(kind) & 0xffULL;
    const std::uint64_t idValue = static_cast<std::uint64_t>(id) & 0x00ffffffffffffffULL;
    return static_cast<LPARAM>((kindValue << 56) | idValue);
}

ViewportSelectionKind decode_model_tree_payload_kind(LPARAM payload) {
    const std::uint64_t value = static_cast<std::uint64_t>(payload);
    switch ((value >> 56) & 0xffULL) {
    case 1:
        return ViewportSelectionKind::Vertex;
    case 2:
        return ViewportSelectionKind::Edge;
    case 3:
        return ViewportSelectionKind::Face;
    case 4:
        return ViewportSelectionKind::Body;
    default:
        return ViewportSelectionKind::None;
    }
}

adaptivecad::geometry::EntityId decode_model_tree_payload_id(LPARAM payload) {
    const std::uint64_t value = static_cast<std::uint64_t>(payload);
    return static_cast<adaptivecad::geometry::EntityId>(value & 0x00ffffffffffffffULL);
}

bool geometry_selection_exists(ViewportSelectionKind kind, adaptivecad::geometry::EntityId id) {
    if (id == 0) {
        return kind == ViewportSelectionKind::None;
    }

    switch (kind) {
    case ViewportSelectionKind::Vertex:
        return find_viewport_vertex(id) != nullptr;
    case ViewportSelectionKind::Edge:
        return find_viewport_edge(id) != nullptr;
    case ViewportSelectionKind::Face:
        return find_viewport_face(id) != nullptr;
    case ViewportSelectionKind::Body:
        for (const auto& body : g_geometryViewportState.bodies) {
            if (body.id == id) {
                return true;
            }
        }
        return false;
    case ViewportSelectionKind::None:
        return false;
    }

    return false;
}

std::string model_tree_vertex_label(const adaptivecad::geometry::Vertex& vertex) {
    std::ostringstream out;
    out << "Vertex v" << vertex.id << " ("
        << format_plot_number(vertex.point.x, 2) << ", "
        << format_plot_number(vertex.point.y, 2) << ", "
        << format_plot_number(vertex.point.z, 2) << ")";
    return out.str();
}

std::string model_tree_edge_label(const adaptivecad::geometry::Edge& edge) {
    std::ostringstream out;
    out << "Edge e" << edge.id << " [v" << edge.start_vertex_id << " -> v" << edge.end_vertex_id
        << "] len=" << format_plot_number(edge.length, 3);
    return out.str();
}

std::string model_tree_face_label(const adaptivecad::geometry::Face& face) {
    std::ostringstream out;
    out << "Face f" << face.id << " area=" << format_plot_number(face.area, 3);
    return out.str();
}

std::string format_double_list(const std::vector<double>& values) {
    if (values.empty()) {
        return "(none)";
    }

    std::ostringstream out;
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            out << ", ";
        }
        out << format_plot_number(values[index], 4);
    }
    return out.str();
}

std::string active_scene_source_label() {
    if (!g_sceneDocument.source_path.empty()) {
        return std::string(adaptivecad::tool::scene_source_kind_name(g_sceneDocument.source_kind)) +
            ": " + g_sceneDocument.source_path.string();
    }
    return adaptivecad::tool::scene_source_kind_name(g_sceneDocument.source_kind);
}

std::string format_double_list_for_edit(const std::vector<double>& values) {
    std::ostringstream out;
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            out << ',';
        }
        out << std::setprecision(15) << values[index];
    }
    return out.str();
}

std::string read_window_text(HWND window) {
    if (!window) {
        return {};
    }

    const int length = GetWindowTextLengthA(window);
    std::string text(static_cast<std::size_t>(std::max(length, 0)) + 1, '\0');
    if (!text.empty()) {
        GetWindowTextA(window, text.data(), length + 1);
        if (!text.empty() && text.back() == '\0') {
            text.pop_back();
        }
    }
    return text;
}

HWND primitive_controls_parent() {
    return g_primitiveGroup ? GetParent(g_primitiveGroup) : nullptr;
}

void set_primitive_field(int labelId, HWND edit, const char* label, const char* value, bool enabled) {
    HWND parent = primitive_controls_parent();
    HWND labelWindow = parent ? GetDlgItem(parent, labelId) : nullptr;
    if (labelWindow) {
        SetWindowTextA(labelWindow, label);
        EnableWindow(labelWindow, enabled ? TRUE : FALSE);
    }
    if (edit) {
        SetWindowTextA(edit, value);
        EnableWindow(edit, enabled ? TRUE : FALSE);
    }
}

void update_primitive_parameter_controls() {
    switch (selected_primitive_kind()) {
    case PrimitiveControlKind::Box:
        set_primitive_field(kPrimitiveParam1LabelId, g_primitiveParam1Edit, "Width:", "1.5", true);
        set_primitive_field(kPrimitiveParam2LabelId, g_primitiveParam2Edit, "Depth:", "1.0", true);
        set_primitive_field(kPrimitiveParam3LabelId, g_primitiveParam3Edit, "Height:", "0.75", true);
        set_primitive_field(kPrimitiveParam4LabelId, g_primitiveParam4Edit, "", "", false);
        set_primitive_field(kPrimitiveParam5LabelId, g_primitiveParam5Edit, "", "", false);
        break;
    case PrimitiveControlKind::Wedge:
        set_primitive_field(kPrimitiveParam1LabelId, g_primitiveParam1Edit, "Width:", "1.5", true);
        set_primitive_field(kPrimitiveParam2LabelId, g_primitiveParam2Edit, "Depth:", "1.0", true);
        set_primitive_field(kPrimitiveParam3LabelId, g_primitiveParam3Edit, "Height:", "0.9", true);
        set_primitive_field(kPrimitiveParam4LabelId, g_primitiveParam4Edit, "", "", false);
        set_primitive_field(kPrimitiveParam5LabelId, g_primitiveParam5Edit, "", "", false);
        break;
    case PrimitiveControlKind::Plane:
        set_primitive_field(kPrimitiveParam1LabelId, g_primitiveParam1Edit, "Width:", "2.0", true);
        set_primitive_field(kPrimitiveParam2LabelId, g_primitiveParam2Edit, "Depth:", "1.25", true);
        set_primitive_field(kPrimitiveParam3LabelId, g_primitiveParam3Edit, "", "", false);
        set_primitive_field(kPrimitiveParam4LabelId, g_primitiveParam4Edit, "", "", false);
        set_primitive_field(kPrimitiveParam5LabelId, g_primitiveParam5Edit, "", "", false);
        break;
    case PrimitiveControlKind::Torus:
        set_primitive_field(kPrimitiveParam1LabelId, g_primitiveParam1Edit, "Major R:", "1.0", true);
        set_primitive_field(kPrimitiveParam2LabelId, g_primitiveParam2Edit, "Minor R:", "0.25", true);
        set_primitive_field(kPrimitiveParam3LabelId, g_primitiveParam3Edit, "Major Seg:", "32", true);
        set_primitive_field(kPrimitiveParam4LabelId, g_primitiveParam4Edit, "Minor Seg:", "12", true);
        set_primitive_field(kPrimitiveParam5LabelId, g_primitiveParam5Edit, "", "", false);
        break;
    case PrimitiveControlKind::Scaffold:
        set_primitive_field(kPrimitiveParam1LabelId, g_primitiveParam1Edit, "Cells X:", "3", true);
        set_primitive_field(kPrimitiveParam2LabelId, g_primitiveParam2Edit, "Cells Y:", "2", true);
        set_primitive_field(kPrimitiveParam3LabelId, g_primitiveParam3Edit, "Cells Z:", "1", true);
        set_primitive_field(kPrimitiveParam4LabelId, g_primitiveParam4Edit, "Spacing:", "1.0", true);
        set_primitive_field(kPrimitiveParam5LabelId, g_primitiveParam5Edit, "Damage R:", "0.45", true);
        break;
    }
}

std::vector<double> parse_double_list_text(const std::string& text, const char* fieldName) {
    std::vector<double> values;
    std::istringstream input(text);
    std::string token;
    while (std::getline(input, token, ',')) {
        const std::string trimmed = trim_copy(token);
        if (trimmed.empty()) {
            continue;
        }

        double value = 0.0;
        if (!try_parse_double(trimmed, &value)) {
            throw std::runtime_error(std::string("Invalid value in ") + fieldName + " coefficient list");
        }
        values.push_back(value);
    }
    return values;
}

void update_angular_branch_controls() {
    if (g_angularLambdaEdit) {
        const std::string lambdaText = format_plot_number(g_sceneDocument.angular_model.lambda0, 6);
        SetWindowTextA(g_angularLambdaEdit, lambdaText.c_str());
    }
    if (g_angularCosEdit) {
        const std::string cosineText = format_double_list_for_edit(g_sceneDocument.angular_model.cosine_coefficients);
        SetWindowTextA(g_angularCosEdit, cosineText.c_str());
    }
    if (g_angularSinEdit) {
        const std::string sineText = format_double_list_for_edit(g_sceneDocument.angular_model.sine_coefficients);
        SetWindowTextA(g_angularSinEdit, sineText.c_str());
    }
}

void revert_angular_branch_controls() {
    update_angular_branch_controls();
    set_status_text("Status: angular branch edits reverted");
}

void apply_angular_branch_controls(HWND ownerWindow) {
    try {
        const std::string lambdaText = read_window_text(g_angularLambdaEdit);
        double lambda0 = 0.0;
        if (!try_parse_double(lambdaText, &lambda0) || lambda0 <= 0.0) {
            throw std::runtime_error("Angular lambda0 must be a positive number");
        }

        adaptivecad::core::AngularWeightModel angularModel;
        angularModel.lambda0 = lambda0;
        angularModel.cosine_coefficients = parse_double_list_text(read_window_text(g_angularCosEdit), "cosine");
        angularModel.sine_coefficients = parse_double_list_text(read_window_text(g_angularSinEdit), "sine");

        (void)adaptivecad::core::AdaptiveField::AngularWeight(
            angularModel.lambda0,
            angularModel.cosine_coefficients,
            angularModel.sine_coefficients);

        g_sceneDocument.angular_model = std::move(angularModel);
        refresh_output(ownerWindow);
        set_status_text("Status: angular branch updated");
    } catch (const std::exception& ex) {
        set_status_text("Status: angular branch update failed");
        MessageBoxA(ownerWindow, ex.what(), "AdaptiveCAD UI", MB_ICONERROR);
    }
}

std::string find_viewport_body_name(adaptivecad::geometry::EntityId bodyId) {
    for (const auto& bodyInfo : g_geometryViewportState.body_infos) {
        if (bodyInfo.body_id == bodyId && !bodyInfo.name.empty()) {
            return bodyInfo.name;
        }
    }

    return "Body b" + std::to_string(bodyId);
}

std::string model_tree_body_label(const adaptivecad::geometry::Body& body) {
    std::ostringstream out;
    out << find_viewport_body_name(body.id) << " [b" << body.id << "] volume=" << format_plot_number(body.volume, 3);
    return out.str();
}

HTREEITEM insert_model_tree_item(HWND treeView, HTREEITEM parent, const std::string& label, LPARAM payload) {
    TVINSERTSTRUCTA item{};
    item.hParent = parent ? parent : TVI_ROOT;
    item.hInsertAfter = TVI_LAST;
    item.item.mask = TVIF_TEXT | TVIF_PARAM;
    item.item.pszText = const_cast<LPSTR>(label.c_str());
    item.item.lParam = payload;
    return reinterpret_cast<HTREEITEM>(SendMessageA(treeView, TVM_INSERTITEMA, 0, reinterpret_cast<LPARAM>(&item)));
}

HTREEITEM find_model_tree_item_recursive(
    HWND treeView,
    HTREEITEM item,
    ViewportSelectionKind kind,
    adaptivecad::geometry::EntityId id) {
    while (item) {
        TVITEMA treeItem{};
        treeItem.mask = TVIF_PARAM;
        treeItem.hItem = item;
        if (SendMessageA(treeView, TVM_GETITEMA, 0, reinterpret_cast<LPARAM>(&treeItem)) != FALSE) {
            if (decode_model_tree_payload_kind(treeItem.lParam) == kind && decode_model_tree_payload_id(treeItem.lParam) == id) {
                return item;
            }
        }

        const HTREEITEM child = TreeView_GetChild(treeView, item);
        if (child) {
            const HTREEITEM nested = find_model_tree_item_recursive(treeView, child, kind, id);
            if (nested) {
                return nested;
            }
        }

        item = TreeView_GetNextSibling(treeView, item);
    }

    return nullptr;
}

void sync_model_tree_selection_from_geometry() {
    if (!g_modelTreeView) {
        return;
    }

    HTREEITEM targetItem = g_modelTreeRootItem;
    if (g_geometryViewportState.selection_kind != ViewportSelectionKind::None && g_geometryViewportState.selected_id != 0) {
        targetItem = find_model_tree_item_recursive(
            g_modelTreeView,
            g_modelTreeRootItem,
            g_geometryViewportState.selection_kind,
            g_geometryViewportState.selected_id);
        if (!targetItem) {
            targetItem = g_modelTreeRootItem;
        }
    }

    g_treeSelectionSyncInProgress = true;
    TreeView_SelectItem(g_modelTreeView, targetItem);
    g_treeSelectionSyncInProgress = false;
}

void populate_model_tree() {
    if (!g_modelTreeView) {
        return;
    }

    g_treeSelectionSyncInProgress = true;
    SendMessageA(g_modelTreeView, TVM_DELETEITEM, 0, reinterpret_cast<LPARAM>(TVI_ROOT));
    g_modelTreeRootItem = nullptr;

    if (!g_geometryViewportState.ready) {
        g_modelTreeRootItem = insert_model_tree_item(g_modelTreeView, nullptr, "Geometry Scene Unavailable", 0);
        TreeView_Expand(g_modelTreeView, g_modelTreeRootItem, TVE_EXPAND);
        TreeView_SelectItem(g_modelTreeView, g_modelTreeRootItem);
        g_treeSelectionSyncInProgress = false;
        return;
    }

    const std::string rootLabel = std::string("Geometry Scene | ") + g_geometryViewportState.scene_label;
    g_modelTreeRootItem = insert_model_tree_item(g_modelTreeView, nullptr, rootLabel, 0);

    HTREEITEM bodiesGroup = insert_model_tree_item(g_modelTreeView, g_modelTreeRootItem, "Bodies", 0);
    for (const auto& body : g_geometryViewportState.bodies) {
        HTREEITEM bodyItem = insert_model_tree_item(
            g_modelTreeView,
            bodiesGroup,
            model_tree_body_label(body),
            encode_model_tree_payload(ViewportSelectionKind::Body, body.id));
        for (adaptivecad::geometry::EntityId faceId : body.face_ids) {
            const auto* face = find_viewport_face(faceId);
            if (!face) {
                continue;
            }

            HTREEITEM faceItem = insert_model_tree_item(
                g_modelTreeView,
                bodyItem,
                model_tree_face_label(*face),
                encode_model_tree_payload(ViewportSelectionKind::Face, face->id));
            for (adaptivecad::geometry::EntityId edgeId : face->edge_ids) {
                const auto* edge = find_viewport_edge(edgeId);
                if (!edge) {
                    continue;
                }

                insert_model_tree_item(
                    g_modelTreeView,
                    faceItem,
                    model_tree_edge_label(*edge),
                    encode_model_tree_payload(ViewportSelectionKind::Edge, edge->id));
            }
        }
    }

    HTREEITEM verticesGroup = insert_model_tree_item(g_modelTreeView, g_modelTreeRootItem, "Vertices", 0);
    for (const auto& vertex : g_geometryViewportState.vertices) {
        insert_model_tree_item(
            g_modelTreeView,
            verticesGroup,
            model_tree_vertex_label(vertex),
            encode_model_tree_payload(ViewportSelectionKind::Vertex, vertex.id));
    }

    HTREEITEM diagnosticsGroup = insert_model_tree_item(g_modelTreeView, g_modelTreeRootItem, "Diagnostics", 0);
    insert_model_tree_item(
        g_modelTreeView,
        diagnosticsGroup,
        std::string("Scene Source: ") + active_scene_source_label(),
        0);
    insert_model_tree_item(
        g_modelTreeView,
        diagnosticsGroup,
        std::string("Success: ") + (g_geometryViewportState.diagnostic.success ? "true" : "false"),
        0);
    insert_model_tree_item(
        g_modelTreeView,
        diagnosticsGroup,
        std::string("Operation: ") + g_geometryViewportState.diagnostic.operation,
        0);
    insert_model_tree_item(
        g_modelTreeView,
        diagnosticsGroup,
        std::string("Healing: ") + (g_geometryViewportState.diagnostic.healing_succeeded ? "true" : "false"),
        0);

    TreeView_Expand(g_modelTreeView, g_modelTreeRootItem, TVE_EXPAND);
    TreeView_Expand(g_modelTreeView, bodiesGroup, TVE_EXPAND);
    TreeView_Expand(g_modelTreeView, verticesGroup, TVE_EXPAND);
    TreeView_Expand(g_modelTreeView, diagnosticsGroup, TVE_EXPAND);
    g_treeSelectionSyncInProgress = false;
    sync_model_tree_selection_from_geometry();
}

void apply_geometry_selection(
    ViewportSelectionKind kind,
    adaptivecad::geometry::EntityId id,
    const std::string& clearedStatus) {
    g_geometryViewportState.selection_kind = kind;
    g_geometryViewportState.selected_id = id;
    invalidate_viewport();
    sync_model_tree_selection_from_geometry();
    update_body_edit_controls();

    if (kind == ViewportSelectionKind::None || id == 0) {
        set_status_text(clearedStatus);
        return;
    }

    set_status_text(std::string("Status: ") + viewport_selection_label());
}

std::string viewport_selection_label() {
    using adaptivecad::geometry::EntityId;

    const EntityId selectedId = g_geometryViewportState.selected_id;
    switch (g_geometryViewportState.selection_kind) {
    case ViewportSelectionKind::Vertex: {
        const auto* vertex = find_viewport_vertex(selectedId);
        if (!vertex) {
            break;
        }
        std::ostringstream out;
        out << "Selected vertex " << vertex->id << " @ ("
            << format_plot_number(vertex->point.x, 2) << ", "
            << format_plot_number(vertex->point.y, 2) << ", "
            << format_plot_number(vertex->point.z, 2) << ")";
        return out.str();
    }
    case ViewportSelectionKind::Edge: {
        const auto* edge = find_viewport_edge(selectedId);
        if (!edge) {
            break;
        }
        std::ostringstream out;
        out << "Selected edge " << edge->id << " [v" << edge->start_vertex_id
            << " -> v" << edge->end_vertex_id << "] length=" << format_plot_number(edge->length, 3);
        return out.str();
    }
    case ViewportSelectionKind::Face: {
        std::ostringstream out;
        out << "Selected face " << selectedId;
        return out.str();
    }
    case ViewportSelectionKind::Body: {
        const auto* body = find_viewport_body(selectedId);
        if (!body) {
            break;
        }
        std::ostringstream out;
        out << "Selected " << find_viewport_body_name(body->id) << " [b" << body->id << "]";
        return out.str();
    }
    case ViewportSelectionKind::None:
        break;
    }

    return "Click a vertex, edge, or face in the viewport to inspect it.";
}

GeometryViewportState build_geometry_viewport_state() {
    GeometryViewportState state{};

    const adaptivecad::tool::BuiltScene scene = adaptivecad::tool::build_scene_geometry(g_sceneDocument, g_healingPolicy, true);
    state.vertices = scene.vertices;
    state.edges = scene.edges;
    state.faces = scene.faces;
    state.bodies = scene.bodies;
    state.face_outlines = scene.face_outlines;
    state.body_infos = scene.body_infos;
    state.diagnostic = scene.diagnostic;
    state.backend_name = scene.backend_name;
    state.has_open_cascade_topology = scene.has_open_cascade_topology;
    state.scene_label = g_sceneDocument.scene_label.empty() ? "Geometry Scene" : g_sceneDocument.scene_label;
    state.scene_source_kind = adaptivecad::tool::scene_source_kind_name(g_sceneDocument.source_kind);
    state.scene_source_path = g_sceneDocument.source_path.string();

    state.subtitle = std::string("Scene: ") + state.scene_label +
        " | Bodies: " + std::to_string(state.bodies.size()) +
        " | Backend: " + state.backend_name +
        " | Policy: " + adaptivecad::geometry::BRepKernel::healing_policy_name(g_healingPolicy);

    std::ostringstream diagnostic;
    diagnostic << "Diag: success=" << (state.diagnostic.success ? "true" : "false")
               << " op=" << state.diagnostic.operation
               << " heal=" << (state.diagnostic.healing_succeeded ? "true" : "false");
    state.diagnostic_summary = diagnostic.str();
    state.ready = true;
    return state;
}

RECT geometry_viewport_draw_rect(const RECT& clientRect) {
    RECT drawRect = clientRect;
    drawRect.left += 12;
    drawRect.top += 44;
    drawRect.right -= 12;
    drawRect.bottom -= 40;
    return drawRect;
}

ProjectedGeometryPoint project_geometry_world_point(const adaptivecad::geometry::Point3D& point) {
    switch (g_geometryViewportViewState.projection) {
    case ViewportProjectionMode::Top:
        return {point.x, point.y};
    case ViewportProjectionMode::Front:
        return {point.x, point.z};
    case ViewportProjectionMode::Right:
        return {point.y, point.z};
    case ViewportProjectionMode::Isometric: {
        const double isoX = 0.8660254037844386 * (point.x - point.y);
        const double isoY = 0.5 * (point.x + point.y) - point.z;
        return {isoX, isoY};
    }
    }

    return {point.x, point.y};
}

void projected_geometry_bounds(double* minX, double* maxX, double* minY, double* maxY) {
    if (!minX || !maxX || !minY || !maxY) {
        return;
    }

    *minX = std::numeric_limits<double>::max();
    *maxX = -std::numeric_limits<double>::max();
    *minY = std::numeric_limits<double>::max();
    *maxY = -std::numeric_limits<double>::max();

    for (const auto& vertex : g_geometryViewportState.vertices) {
        const ProjectedGeometryPoint projected = project_geometry_world_point(vertex.point);
        *minX = std::min(*minX, projected.x);
        *maxX = std::max(*maxX, projected.x);
        *minY = std::min(*minY, projected.y);
        *maxY = std::max(*maxY, projected.y);
    }
}

ViewportPoint project_geometry_point(const adaptivecad::geometry::Point3D& point, const RECT& rect) {
    double minX = 0.0;
    double maxX = 0.0;
    double minY = 0.0;
    double maxY = 0.0;
    projected_geometry_bounds(&minX, &maxX, &minY, &maxY);

    if (!std::isfinite(minX) || !std::isfinite(maxX) || !std::isfinite(minY) || !std::isfinite(maxY)) {
        return {rect.left, rect.top};
    }

    const ProjectedGeometryPoint projectedPoint = project_geometry_world_point(point);
    const double width = std::max(1e-6, maxX - minX);
    const double height = std::max(1e-6, maxY - minY);
    const double pad = 24.0;
    const double availWidth = std::max(20.0, static_cast<double>(rect.right - rect.left) - 2.0 * pad);
    const double availHeight = std::max(20.0, static_cast<double>(rect.bottom - rect.top) - 2.0 * pad);
    const double scale = std::min(availWidth / width, availHeight / height) * g_geometryViewportViewState.zoom;
    const double offsetX = rect.left + pad + 0.5 * (availWidth - width * scale);
    const double offsetY = rect.top + pad + 0.5 * (availHeight - height * scale);

    const int x = static_cast<int>(std::lround(
        offsetX + (projectedPoint.x - minX) * scale + g_geometryViewportViewState.pan_x));
    const int y = static_cast<int>(std::lround(
        rect.bottom - (offsetY + (projectedPoint.y - minY) * scale) + g_geometryViewportViewState.pan_y));
    return {x, y};
}

double point_to_segment_distance(
    double px,
    double py,
    double ax,
    double ay,
    double bx,
    double by) {
    const double vx = bx - ax;
    const double vy = by - ay;
    const double wx = px - ax;
    const double wy = py - ay;
    const double segmentLength2 = vx * vx + vy * vy;
    if (segmentLength2 <= 1e-12) {
        const double dx = px - ax;
        const double dy = py - ay;
        return std::sqrt(dx * dx + dy * dy);
    }

    double t = (wx * vx + wy * vy) / segmentLength2;
    t = std::clamp(t, 0.0, 1.0);
    const double cx = ax + t * vx;
    const double cy = ay + t * vy;
    const double dx = px - cx;
    const double dy = py - cy;
    return std::sqrt(dx * dx + dy * dy);
}

bool point_in_polygon(const std::vector<ViewportPoint>& polygon, int x, int y) {
    bool inside = false;
    if (polygon.size() < 3) {
        return false;
    }

    for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const auto& pi = polygon[i];
        const auto& pj = polygon[j];
        const bool intersects = ((pi.y > y) != (pj.y > y)) &&
            (x < (pj.x - pi.x) * static_cast<double>(y - pi.y) / static_cast<double>(pj.y - pi.y + (pj.y == pi.y ? 1 : 0)) + pi.x);
        if (intersects) {
            inside = !inside;
        }
    }

    return inside;
}

bool face_belongs_to_selected_body(adaptivecad::geometry::EntityId faceId) {
    if (g_geometryViewportState.selection_kind != ViewportSelectionKind::Body ||
        g_geometryViewportState.selected_id == 0) {
        return false;
    }

    const auto* body = find_viewport_body(g_geometryViewportState.selected_id);
    if (!body) {
        return false;
    }
    return std::find(body->face_ids.begin(), body->face_ids.end(), faceId) != body->face_ids.end();
}

void invalidate_viewport() {
    if (g_viewportPanel) {
        InvalidateRect(g_viewportPanel, nullptr, TRUE);
    }
}

void zoom_viewport(double factor) {
    g_geometryViewportViewState.zoom = std::clamp(g_geometryViewportViewState.zoom * factor, 0.25, 8.0);
    invalidate_viewport();
}

void set_viewport_projection_mode(ViewportProjectionMode projection) {
    g_geometryViewportViewState.projection = projection;
    reset_viewport_camera();
    initialize_viewport_projection_combo();
    invalidate_viewport();
    set_status_text(std::string("Status: viewport projection set to ") + viewport_projection_name(projection));
}

void reset_viewport_interaction() {
    reset_viewport_camera();
    invalidate_viewport();
    set_status_text("Status: viewport view reset");
}

void handle_viewport_click(HWND hwnd, int x, int y) {
    if (!g_geometryViewportState.ready || !g_viewportPanel) {
        return;
    }

    RECT clientRect{};
    GetClientRect(hwnd, &clientRect);
    const RECT drawRect = geometry_viewport_draw_rect(clientRect);

    double bestVertexDistance = std::numeric_limits<double>::max();
    adaptivecad::geometry::EntityId vertexId = 0;
    for (const auto& vertex : g_geometryViewportState.vertices) {
        const ViewportPoint projected = project_geometry_point(vertex.point, drawRect);
        const double dx = static_cast<double>(x - projected.x);
        const double dy = static_cast<double>(y - projected.y);
        const double distance = std::sqrt(dx * dx + dy * dy);
        if (distance < bestVertexDistance) {
            bestVertexDistance = distance;
            vertexId = vertex.id;
        }
    }

    if (bestVertexDistance <= 12.0) {
        apply_geometry_selection(ViewportSelectionKind::Vertex, vertexId, "Status: viewport selection cleared");
        return;
    }

    double bestEdgeDistance = std::numeric_limits<double>::max();
    adaptivecad::geometry::EntityId edgeId = 0;
    for (const auto& edge : g_geometryViewportState.edges) {
        const auto* start = find_viewport_vertex(edge.start_vertex_id);
        const auto* end = find_viewport_vertex(edge.end_vertex_id);
        if (!start || !end) {
            continue;
        }

        const ViewportPoint a = project_geometry_point(start->point, drawRect);
        const ViewportPoint b = project_geometry_point(end->point, drawRect);
        const double distance = point_to_segment_distance(
            static_cast<double>(x), static_cast<double>(y),
            static_cast<double>(a.x), static_cast<double>(a.y),
            static_cast<double>(b.x), static_cast<double>(b.y));
        if (distance < bestEdgeDistance) {
            bestEdgeDistance = distance;
            edgeId = edge.id;
        }
    }

    if (bestEdgeDistance <= 10.0) {
        apply_geometry_selection(ViewportSelectionKind::Edge, edgeId, "Status: viewport selection cleared");
        return;
    }

    for (const auto& faceOutline : g_geometryViewportState.face_outlines) {
        std::vector<ViewportPoint> polygon;
        polygon.reserve(faceOutline.points.size());
        for (const auto& point : faceOutline.points) {
            polygon.push_back(project_geometry_point(point, drawRect));
        }

        if (polygon.size() >= 3 && point_in_polygon(polygon, x, y)) {
            apply_geometry_selection(
                ViewportSelectionKind::Face,
                faceOutline.face_id,
                "Status: viewport selection cleared");
            return;
        }
    }

    apply_geometry_selection(ViewportSelectionKind::None, 0, "Status: viewport selection cleared");
}

void draw_geometry_viewport(HDC hdc, const RECT& clientRect) {
    HBRUSH backgroundBrush = CreateSolidBrush(RGB(247, 247, 244));
    FillRect(hdc, &clientRect, backgroundBrush);
    DeleteObject(backgroundBrush);

    SetBkMode(hdc, TRANSPARENT);
    SetTextColor(hdc, RGB(28, 28, 28));

    RECT titleRect = {clientRect.left + 10, clientRect.top + 8, clientRect.right - 10, clientRect.top + 24};
    DrawTextA(hdc, "Geometry Viewport", -1, &titleRect, DT_LEFT | DT_SINGLELINE | DT_END_ELLIPSIS);

    RECT subtitleRect = {clientRect.left + 10, clientRect.top + 24, clientRect.right - 10, clientRect.top + 40};
    std::string subtitle = g_geometryViewportState.ready
        ? g_geometryViewportState.subtitle
        : g_geometryViewportState.error_message;
    if (g_geometryViewportState.ready) {
        subtitle += std::string(" | View: ") + viewport_projection_name(g_geometryViewportViewState.projection) +
            " | Zoom: " + format_plot_number(g_geometryViewportViewState.zoom, 2);
    }
    DrawTextA(hdc, subtitle.c_str(), -1, &subtitleRect, DT_LEFT | DT_SINGLELINE | DT_END_ELLIPSIS);

    const RECT drawRect = geometry_viewport_draw_rect(clientRect);

    HBRUSH canvasBrush = CreateSolidBrush(RGB(255, 255, 255));
    FillRect(hdc, &drawRect, canvasBrush);
    DeleteObject(canvasBrush);
    FrameRect(hdc, &drawRect, reinterpret_cast<HBRUSH>(GetStockObject(GRAY_BRUSH)));

    if (!g_geometryViewportState.ready) {
        RECT messageRect = drawRect;
        DrawTextA(hdc, "Geometry scene unavailable.", -1, &messageRect, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
        return;
    }

    for (const auto& faceOutline : g_geometryViewportState.face_outlines) {
        std::vector<ViewportPoint> polygon;
        polygon.reserve(faceOutline.points.size());
        for (const auto& point : faceOutline.points) {
            polygon.push_back(project_geometry_point(point, drawRect));
        }

        if (polygon.size() < 3) {
            continue;
        }

        std::vector<POINT> facePoints;
        facePoints.reserve(polygon.size());
        for (const auto& point : polygon) {
            facePoints.push_back(POINT{point.x, point.y});
        }

        const bool faceSelected = g_geometryViewportState.selection_kind == ViewportSelectionKind::Face &&
            g_geometryViewportState.selected_id == faceOutline.face_id;
        const bool bodySelected = face_belongs_to_selected_body(faceOutline.face_id);
        const COLORREF faceFill = faceOutline.has_color
            ? scene_color_to_colorref(faceOutline.color)
            : RGB(223, 234, 244);
        HBRUSH faceBrush = CreateSolidBrush((faceSelected || bodySelected) ? RGB(200, 221, 246) : faceFill);
        HPEN facePen = CreatePen(
            PS_SOLID,
            (faceSelected || bodySelected) ? 2 : 1,
            (faceSelected || bodySelected) ? RGB(30, 95, 163) : RGB(125, 150, 175));
        HGDIOBJ oldBrush = SelectObject(hdc, faceBrush);
        HGDIOBJ oldPen = SelectObject(hdc, facePen);
        Polygon(hdc, facePoints.data(), static_cast<int>(facePoints.size()));
        SelectObject(hdc, oldPen);
        SelectObject(hdc, oldBrush);
        DeleteObject(faceBrush);
        DeleteObject(facePen);
    }

    for (const auto& edge : g_geometryViewportState.edges) {
        const auto* start = find_viewport_vertex(edge.start_vertex_id);
        const auto* end = find_viewport_vertex(edge.end_vertex_id);
        if (!start || !end) {
            continue;
        }

        const ViewportPoint a = project_geometry_point(start->point, drawRect);
        const ViewportPoint b = project_geometry_point(end->point, drawRect);
        const bool selected = g_geometryViewportState.selection_kind == ViewportSelectionKind::Edge &&
            g_geometryViewportState.selected_id == edge.id;
        HPEN edgePen = CreatePen(PS_SOLID, selected ? 3 : 2, selected ? RGB(194, 68, 44) : RGB(58, 58, 58));
        HGDIOBJ oldPen = SelectObject(hdc, edgePen);
        MoveToEx(hdc, a.x, a.y, nullptr);
        LineTo(hdc, b.x, b.y);
        SelectObject(hdc, oldPen);
        DeleteObject(edgePen);

        const int labelX = (a.x + b.x) / 2 + 6;
        const int labelY = (a.y + b.y) / 2 - 12;
        const std::string label = "e" + std::to_string(edge.id);
        TextOutA(hdc, labelX, labelY, label.c_str(), static_cast<int>(label.size()));
    }

    for (const auto& vertex : g_geometryViewportState.vertices) {
        const ViewportPoint p = project_geometry_point(vertex.point, drawRect);
        const bool selected = g_geometryViewportState.selection_kind == ViewportSelectionKind::Vertex &&
            g_geometryViewportState.selected_id == vertex.id;
        HPEN vertexPen = CreatePen(PS_SOLID, 1, selected ? RGB(204, 114, 30) : RGB(24, 24, 24));
        HBRUSH vertexBrush = CreateSolidBrush(selected ? RGB(250, 196, 110) : RGB(24, 24, 24));
        HGDIOBJ oldPen = SelectObject(hdc, vertexPen);
        HGDIOBJ oldBrush = SelectObject(hdc, vertexBrush);
        Ellipse(hdc, p.x - 5, p.y - 5, p.x + 6, p.y + 6);
        SelectObject(hdc, oldBrush);
        SelectObject(hdc, oldPen);
        DeleteObject(vertexBrush);
        DeleteObject(vertexPen);

        const std::string label = "v" + std::to_string(vertex.id);
        TextOutA(hdc, p.x + 8, p.y - 8, label.c_str(), static_cast<int>(label.size()));
    }

    RECT footerRect = {clientRect.left + 10, clientRect.bottom - 34, clientRect.right - 10, clientRect.bottom - 18};
    DrawTextA(hdc, viewport_selection_label().c_str(), -1, &footerRect, DT_LEFT | DT_SINGLELINE | DT_END_ELLIPSIS);
    RECT diagnosticRect = {clientRect.left + 10, clientRect.bottom - 18, clientRect.right - 10, clientRect.bottom - 2};
    DrawTextA(hdc, g_geometryViewportState.diagnostic_summary.c_str(), -1, &diagnosticRect, DT_LEFT | DT_SINGLELINE | DT_END_ELLIPSIS);
}

LRESULT CALLBACK viewport_panel_proc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_ERASEBKGND:
        return 1;
    case WM_RBUTTONDOWN:
        g_geometryViewportViewState.panning = true;
        g_geometryViewportViewState.last_pan_point = POINT{
            static_cast<LONG>(static_cast<int>(static_cast<short>(LOWORD(lParam)))),
            static_cast<LONG>(static_cast<int>(static_cast<short>(HIWORD(lParam))))};
        SetCapture(hwnd);
        set_status_text("Status: viewport pan active");
        return 0;
    case WM_RBUTTONUP:
        if (g_geometryViewportViewState.panning) {
            g_geometryViewportViewState.panning = false;
            ReleaseCapture();
            set_status_text("Status: viewport pan complete");
        }
        return 0;
    case WM_MOUSEMOVE:
        if (g_geometryViewportViewState.panning) {
            const POINT currentPoint{
                static_cast<LONG>(static_cast<int>(static_cast<short>(LOWORD(lParam)))),
                static_cast<LONG>(static_cast<int>(static_cast<short>(HIWORD(lParam))))};
            g_geometryViewportViewState.pan_x +=
                static_cast<double>(currentPoint.x - g_geometryViewportViewState.last_pan_point.x);
            g_geometryViewportViewState.pan_y +=
                static_cast<double>(currentPoint.y - g_geometryViewportViewState.last_pan_point.y);
            g_geometryViewportViewState.last_pan_point = currentPoint;
            InvalidateRect(hwnd, nullptr, TRUE);
        }
        return 0;
    case WM_MOUSEWHEEL: {
        const int delta = static_cast<int>(static_cast<short>(HIWORD(wParam)));
        zoom_viewport(delta > 0 ? 1.1 : (1.0 / 1.1));
        set_status_text(std::string("Status: viewport zoom = ") +
            format_plot_number(g_geometryViewportViewState.zoom, 2));
        return 0;
    }
    case WM_LBUTTONDOWN:
        handle_viewport_click(
            hwnd,
            static_cast<int>(static_cast<short>(LOWORD(lParam))),
            static_cast<int>(static_cast<short>(HIWORD(lParam))));
        SetFocus(hwnd);
        return 0;
    case WM_PAINT: {
        PAINTSTRUCT paint{};
        HDC windowDc = BeginPaint(hwnd, &paint);
        RECT clientRect{};
        GetClientRect(hwnd, &clientRect);
        const int width = clientRect.right - clientRect.left;
        const int height = clientRect.bottom - clientRect.top;
        if (width > 0 && height > 0) {
            HDC memoryDc = CreateCompatibleDC(windowDc);
            HBITMAP bitmap = CreateCompatibleBitmap(windowDc, width, height);
            HGDIOBJ oldBitmap = SelectObject(memoryDc, bitmap);
            draw_geometry_viewport(memoryDc, clientRect);
            BitBlt(windowDc, 0, 0, width, height, memoryDc, 0, 0, SRCCOPY);
            SelectObject(memoryDc, oldBitmap);
            DeleteObject(bitmap);
            DeleteDC(memoryDc);
        }
        EndPaint(hwnd, &paint);
        return 0;
    }
    default:
        break;
    }

    return DefWindowProcA(hwnd, msg, wParam, lParam);
}

FitPlotState build_fit_plot_state(const ActiveFitAnalysisState& analysis) {
    using adaptivecad::core::AreaSample;
    using adaptivecad::core::InverseRecovery;

    if (analysis.measurement_samples.size() < 3) {
        throw std::runtime_error("Plot requires at least three samples");
    }

    double minRadius = std::numeric_limits<double>::max();
    double maxRadius = 0.0;
    for (const AreaSample& sample : analysis.measurement_samples) {
        extend_range(sample.radius, &minRadius, &maxRadius);
    }

    const double curveStart = 0.0;
    const double curveEnd = std::max(maxRadius * 1.08, maxRadius + 0.25);
    const int curvePointCount = 96;

    FitPlotState state{};
    state.best_model = analysis.model_comparison.best_model;
    state.title = "Measured Points and Model Fits";

    {
        std::ostringstream subtitle;
        subtitle << "Source: " << analysis.measurement_source
                 << " | Best: " << InverseRecovery::modelKindName(analysis.model_comparison.best_model)
                 << " (weight=" << std::fixed << std::setprecision(2)
                 << analysis.model_comparison.best_model_weight << ")";
        state.subtitle = subtitle.str();
    }

    state.residual_caption = std::string("Residuals (% error) | best: ") +
        InverseRecovery::modelKindName(analysis.model_comparison.best_model);
    if (!analysis.model_comparison.warning.empty()) {
        state.residual_caption += " | " + analysis.model_comparison.warning;
    }

    state.measured_points.reserve(analysis.measurement_samples.size());
    state.residual_points.reserve(analysis.measurement_samples.size());
    for (const AreaSample& sample : analysis.measurement_samples) {
        state.measured_points.push_back(PlotPoint{sample.radius, sample.area});

        const double fittedArea = best_model_area(sample.radius, analysis.model_comparison);
        const double relativeResidualPercent = 100.0 * (fittedArea - sample.area) / sample.area;
        state.residual_points.push_back(PlotPoint{sample.radius, relativeResidualPercent});
    }

    state.constant_curve.reserve(curvePointCount);
    state.power_law_curve.reserve(curvePointCount);
    state.gaussian_curve.reserve(curvePointCount);
    for (int index = 0; index < curvePointCount; ++index) {
        const double t = (curvePointCount == 1)
            ? 0.0
            : static_cast<double>(index) / static_cast<double>(curvePointCount - 1);
        const double radius = curveStart + (curveEnd - curveStart) * t;
        state.constant_curve.push_back(
            PlotPoint{radius, constant_model_area(radius, analysis.model_comparison.constant_fit)});
        state.power_law_curve.push_back(
            PlotPoint{radius, power_law_model_area(radius, analysis.model_comparison.power_law_fit)});
        state.gaussian_curve.push_back(
            PlotPoint{radius, gaussian_model_area(radius, analysis.model_comparison.gaussian_fit)});
    }

    state.ready = true;
    return state;
}

int map_plot_x(double value, const RECT& rect, double minValue, double maxValue) {
    if (maxValue - minValue <= 1e-12) {
        return rect.left + (rect.right - rect.left) / 2;
    }

    const double t = (value - minValue) / (maxValue - minValue);
    return rect.left + static_cast<int>(std::lround(t * static_cast<double>(rect.right - rect.left)));
}

int map_plot_y(double value, const RECT& rect, double minValue, double maxValue) {
    if (maxValue - minValue <= 1e-12) {
        return rect.top + (rect.bottom - rect.top) / 2;
    }

    const double t = (value - minValue) / (maxValue - minValue);
    return rect.bottom - static_cast<int>(std::lround(t * static_cast<double>(rect.bottom - rect.top)));
}

void draw_chart_grid(HDC hdc, const RECT& rect, int columns, int rows) {
    HPEN gridPen = CreatePen(PS_SOLID, 1, RGB(228, 228, 228));
    HGDIOBJ oldPen = SelectObject(hdc, gridPen);

    for (int column = 0; column <= columns; ++column) {
        const int x = rect.left + (rect.right - rect.left) * column / std::max(1, columns);
        MoveToEx(hdc, x, rect.top, nullptr);
        LineTo(hdc, x, rect.bottom);
    }

    for (int row = 0; row <= rows; ++row) {
        const int y = rect.top + (rect.bottom - rect.top) * row / std::max(1, rows);
        MoveToEx(hdc, rect.left, y, nullptr);
        LineTo(hdc, rect.right, y);
    }

    SelectObject(hdc, oldPen);
    DeleteObject(gridPen);
}

void draw_curve(
    HDC hdc,
    const RECT& rect,
    const std::vector<PlotPoint>& points,
    double xMin,
    double xMax,
    double yMin,
    double yMax,
    COLORREF color,
    int width,
    int penStyle = PS_SOLID) {
    if (points.size() < 2) {
        return;
    }

    HPEN pen = CreatePen(penStyle, width, color);
    HGDIOBJ oldPen = SelectObject(hdc, pen);

    bool started = false;
    for (const PlotPoint& point : points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
            started = false;
            continue;
        }

        const int x = map_plot_x(point.x, rect, xMin, xMax);
        const int y = map_plot_y(point.y, rect, yMin, yMax);
        if (!started) {
            MoveToEx(hdc, x, y, nullptr);
            started = true;
        } else {
            LineTo(hdc, x, y);
        }
    }

    SelectObject(hdc, oldPen);
    DeleteObject(pen);
}

void draw_points(
    HDC hdc,
    const RECT& rect,
    const std::vector<PlotPoint>& points,
    double xMin,
    double xMax,
    double yMin,
    double yMax,
    COLORREF color) {
    HPEN pen = CreatePen(PS_SOLID, 1, color);
    HBRUSH brush = CreateSolidBrush(color);
    HGDIOBJ oldPen = SelectObject(hdc, pen);
    HGDIOBJ oldBrush = SelectObject(hdc, brush);

    for (const PlotPoint& point : points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
            continue;
        }

        const int x = map_plot_x(point.x, rect, xMin, xMax);
        const int y = map_plot_y(point.y, rect, yMin, yMax);
        Ellipse(hdc, x - 3, y - 3, x + 4, y + 4);
    }

    SelectObject(hdc, oldBrush);
    SelectObject(hdc, oldPen);
    DeleteObject(brush);
    DeleteObject(pen);
}

void draw_residuals(
    HDC hdc,
    const RECT& rect,
    const std::vector<PlotPoint>& points,
    double xMin,
    double xMax,
    double yMin,
    double yMax,
    COLORREF color) {
    const int zeroY = map_plot_y(0.0, rect, yMin, yMax);
    HPEN zeroPen = CreatePen(PS_SOLID, 1, RGB(100, 100, 100));
    HGDIOBJ oldPen = SelectObject(hdc, zeroPen);
    MoveToEx(hdc, rect.left, zeroY, nullptr);
    LineTo(hdc, rect.right, zeroY);
    SelectObject(hdc, oldPen);
    DeleteObject(zeroPen);

    HPEN barPen = CreatePen(PS_SOLID, 2, color);
    HBRUSH brush = CreateSolidBrush(color);
    oldPen = SelectObject(hdc, barPen);
    HGDIOBJ oldBrush = SelectObject(hdc, brush);

    for (const PlotPoint& point : points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
            continue;
        }

        const int x = map_plot_x(point.x, rect, xMin, xMax);
        const int y = map_plot_y(point.y, rect, yMin, yMax);
        MoveToEx(hdc, x, zeroY, nullptr);
        LineTo(hdc, x, y);
        Ellipse(hdc, x - 3, y - 3, x + 4, y + 4);
    }

    SelectObject(hdc, oldBrush);
    SelectObject(hdc, oldPen);
    DeleteObject(brush);
    DeleteObject(barPen);
}

void draw_legend_item(HDC hdc, int x, int y, COLORREF color, const char* label, bool drawPoint) {
    HPEN pen = CreatePen(PS_SOLID, drawPoint ? 1 : 2, color);
    HBRUSH brush = CreateSolidBrush(color);
    HGDIOBJ oldPen = SelectObject(hdc, pen);
    HGDIOBJ oldBrush = SelectObject(hdc, brush);

    if (drawPoint) {
        Ellipse(hdc, x, y - 4, x + 8, y + 4);
    } else {
        MoveToEx(hdc, x, y, nullptr);
        LineTo(hdc, x + 12, y);
    }

    SelectObject(hdc, oldBrush);
    SelectObject(hdc, oldPen);
    DeleteObject(brush);
    DeleteObject(pen);

    TextOutA(hdc, x + 18, y - 8, label, static_cast<int>(std::strlen(label)));
}

void draw_fit_plot(HDC hdc, const RECT& clientRect) {
    HBRUSH backgroundBrush = CreateSolidBrush(RGB(248, 248, 246));
    FillRect(hdc, &clientRect, backgroundBrush);
    DeleteObject(backgroundBrush);

    SetBkMode(hdc, TRANSPARENT);
    SetTextColor(hdc, RGB(32, 32, 32));

    RECT inner = clientRect;
    inner.left += 10;
    inner.top += 8;
    inner.right -= 10;
    inner.bottom -= 8;

    if (!g_plotState.ready) {
        RECT messageRect = inner;
        std::string message = g_plotState.error_message.empty()
            ? "Refresh to compute measured points, fitted curves, and residuals."
            : g_plotState.error_message;
        DrawTextA(hdc, message.c_str(), -1, &messageRect, DT_CENTER | DT_VCENTER | DT_WORDBREAK);
        return;
    }

    RECT titleRect = {inner.left, inner.top, inner.right, inner.top + 18};
    DrawTextA(hdc, g_plotState.title.c_str(), -1, &titleRect, DT_LEFT | DT_SINGLELINE | DT_END_ELLIPSIS);

    RECT subtitleRect = {inner.left, inner.top + 18, inner.right, inner.top + 34};
    SetTextColor(hdc, RGB(78, 78, 78));
    DrawTextA(hdc, g_plotState.subtitle.c_str(), -1, &subtitleRect, DT_LEFT | DT_SINGLELINE | DT_END_ELLIPSIS);
    SetTextColor(hdc, RGB(32, 32, 32));

    const int legendY = inner.top + 46;
    draw_legend_item(hdc, inner.left, legendY, RGB(24, 24, 24), "Measured", true);
    draw_legend_item(hdc, inner.left + 96, legendY, model_color(adaptivecad::core::FitModelKind::Constant), "Constant", false);
    draw_legend_item(hdc, inner.left + 198, legendY, model_color(adaptivecad::core::FitModelKind::PowerLaw), "Power-law", false);
    draw_legend_item(hdc, inner.left + 314, legendY, model_color(adaptivecad::core::FitModelKind::Gaussian), "Gaussian", false);
    draw_legend_item(hdc, inner.left + 424, legendY, RGB(216, 132, 42), "Residuals", false);

    const int chartLeft = inner.left + 34;
    const int chartRight = inner.right - 8;
    const int chartTop = inner.top + 74;
    const int chartBottom = inner.bottom - 22;
    const int chartGap = 26;
    int availableChartHeight = chartBottom - chartTop;
    if (availableChartHeight < 120) {
        availableChartHeight = 120;
    }

    int upperChartHeight = static_cast<int>(std::lround(static_cast<double>(availableChartHeight - chartGap) * 0.60));
    upperChartHeight = std::max(84, upperChartHeight);
    int lowerChartHeight = availableChartHeight - chartGap - upperChartHeight;
    if (lowerChartHeight < 54) {
        lowerChartHeight = 54;
        upperChartHeight = std::max(72, availableChartHeight - chartGap - lowerChartHeight);
    }

    RECT upperChart = {chartLeft, chartTop, chartRight, chartTop + upperChartHeight};
    RECT lowerChart = {chartLeft, upperChart.bottom + chartGap, chartRight, chartBottom};

    HBRUSH chartBrush = CreateSolidBrush(RGB(255, 255, 255));
    FillRect(hdc, &upperChart, chartBrush);
    FillRect(hdc, &lowerChart, chartBrush);
    DeleteObject(chartBrush);
    FrameRect(hdc, &upperChart, reinterpret_cast<HBRUSH>(GetStockObject(GRAY_BRUSH)));
    FrameRect(hdc, &lowerChart, reinterpret_cast<HBRUSH>(GetStockObject(GRAY_BRUSH)));

    double xMin = std::numeric_limits<double>::max();
    double xMax = 0.0;
    double yMax = 0.0;
    for (const PlotPoint& point : g_plotState.measured_points) {
        xMin = std::min(xMin, point.x);
        xMax = std::max(xMax, point.x);
        yMax = std::max(yMax, point.y);
    }
    for (const PlotPoint& point : g_plotState.constant_curve) {
        xMin = std::min(xMin, point.x);
        xMax = std::max(xMax, point.x);
        yMax = std::max(yMax, point.y);
    }
    for (const PlotPoint& point : g_plotState.power_law_curve) {
        xMin = std::min(xMin, point.x);
        xMax = std::max(xMax, point.x);
        yMax = std::max(yMax, point.y);
    }
    for (const PlotPoint& point : g_plotState.gaussian_curve) {
        xMin = std::min(xMin, point.x);
        xMax = std::max(xMax, point.x);
        yMax = std::max(yMax, point.y);
    }

    xMin = std::min(0.0, xMin);
    xMax = std::max(xMax, 1.0);
    yMax = std::max(yMax * 1.08, 1.0);

    double residualMaxAbs = 1.0;
    for (const PlotPoint& point : g_plotState.residual_points) {
        residualMaxAbs = std::max(residualMaxAbs, std::abs(point.y));
    }
    const double residualMin = -1.15 * residualMaxAbs;
    const double residualMax = 1.15 * residualMaxAbs;

    draw_chart_grid(hdc, upperChart, 5, 4);
    draw_chart_grid(hdc, lowerChart, 5, 4);

    draw_curve(
        hdc,
        upperChart,
        g_plotState.constant_curve,
        xMin,
        xMax,
        0.0,
        yMax,
        model_color(adaptivecad::core::FitModelKind::Constant),
        1,
        PS_DOT);
    draw_curve(
        hdc,
        upperChart,
        g_plotState.power_law_curve,
        xMin,
        xMax,
        0.0,
        yMax,
        model_color(adaptivecad::core::FitModelKind::PowerLaw),
        2);
    draw_curve(
        hdc,
        upperChart,
        g_plotState.gaussian_curve,
        xMin,
        xMax,
        0.0,
        yMax,
        model_color(adaptivecad::core::FitModelKind::Gaussian),
        2);
    draw_points(hdc, upperChart, g_plotState.measured_points, xMin, xMax, 0.0, yMax, RGB(24, 24, 24));
    draw_residuals(hdc, lowerChart, g_plotState.residual_points, xMin, xMax, residualMin, residualMax, RGB(216, 132, 42));

    TextOutA(hdc, upperChart.left, upperChart.top - 18, "Measured / fitted area", 21);
    TextOutA(
        hdc,
        lowerChart.left,
        lowerChart.top - 18,
        g_plotState.residual_caption.c_str(),
        static_cast<int>(g_plotState.residual_caption.size()));

    const std::string xLeft = format_plot_number(xMin, 2);
    const std::string xMid = format_plot_number((xMin + xMax) * 0.5, 2);
    const std::string xRight = format_plot_number(xMax, 2);
    TextOutA(hdc, lowerChart.left - 4, lowerChart.bottom + 4, xLeft.c_str(), static_cast<int>(xLeft.size()));
    TextOutA(
        hdc,
        (lowerChart.left + lowerChart.right) / 2 - 18,
        lowerChart.bottom + 4,
        xMid.c_str(),
        static_cast<int>(xMid.size()));
    TextOutA(
        hdc,
        lowerChart.right - 42,
        lowerChart.bottom + 4,
        xRight.c_str(),
        static_cast<int>(xRight.size()));
    TextOutA(hdc, lowerChart.right - 28, lowerChart.bottom + 20, "r", 1);

    const std::string areaTop = format_plot_number(yMax, 2);
    const std::string areaBottom = format_plot_number(0.0, 2);
    TextOutA(hdc, upperChart.left - 30, upperChart.top - 6, areaTop.c_str(), static_cast<int>(areaTop.size()));
    TextOutA(
        hdc,
        upperChart.left - 24,
        upperChart.bottom - 8,
        areaBottom.c_str(),
        static_cast<int>(areaBottom.size()));

    const std::string residualTop = format_plot_number(residualMax, 1);
    const std::string residualZero = format_plot_number(0.0, 1);
    const std::string residualBottom = format_plot_number(residualMin, 1);
    TextOutA(
        hdc,
        lowerChart.left - 36,
        lowerChart.top - 6,
        residualTop.c_str(),
        static_cast<int>(residualTop.size()));
    TextOutA(
        hdc,
        lowerChart.left - 24,
        (lowerChart.top + lowerChart.bottom) / 2 - 8,
        residualZero.c_str(),
        static_cast<int>(residualZero.size()));
    TextOutA(
        hdc,
        lowerChart.left - 40,
        lowerChart.bottom - 8,
        residualBottom.c_str(),
        static_cast<int>(residualBottom.size()));
}

LRESULT CALLBACK plot_panel_proc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_ERASEBKGND:
        return 1;
    case WM_PAINT: {
        PAINTSTRUCT paint{};
        HDC windowDc = BeginPaint(hwnd, &paint);

        RECT clientRect{};
        GetClientRect(hwnd, &clientRect);
        const int width = clientRect.right - clientRect.left;
        const int height = clientRect.bottom - clientRect.top;
        if (width > 0 && height > 0) {
            HDC memoryDc = CreateCompatibleDC(windowDc);
            HBITMAP bitmap = CreateCompatibleBitmap(windowDc, width, height);
            HGDIOBJ oldBitmap = SelectObject(memoryDc, bitmap);

            draw_fit_plot(memoryDc, clientRect);
            BitBlt(windowDc, 0, 0, width, height, memoryDc, 0, 0, SRCCOPY);

            SelectObject(memoryDc, oldBitmap);
            DeleteObject(bitmap);
            DeleteDC(memoryDc);
        }

        EndPaint(hwnd, &paint);
        return 0;
    }
    default:
        break;
    }

    return DefWindowProcA(hwnd, msg, wParam, lParam);
}

std::string trim_copy(const std::string& input) {
    std::size_t first = 0;
    while (first < input.size() && std::isspace(static_cast<unsigned char>(input[first])) != 0) {
        ++first;
    }

    std::size_t last = input.size();
    while (last > first && std::isspace(static_cast<unsigned char>(input[last - 1])) != 0) {
        --last;
    }

    return input.substr(first, last - first);
}

bool try_parse_double(const std::string& text, double* value) {
    if (!value) {
        return false;
    }

    const std::string trimmed = trim_copy(text);
    if (trimmed.empty()) {
        return false;
    }

    char* endPointer = nullptr;
    const double parsed = std::strtod(trimmed.c_str(), &endPointer);
    if (endPointer == trimmed.c_str()) {
        return false;
    }

    while (*endPointer != '\0') {
        if (std::isspace(static_cast<unsigned char>(*endPointer)) == 0) {
            return false;
        }
        ++endPointer;
    }

    *value = parsed;
    return std::isfinite(parsed);
}

std::string normalize_csv_line(const std::string& line) {
    std::string normalized = line;
    for (char& ch : normalized) {
        if (ch == ';' || ch == '\t') {
            ch = ',';
        }
    }
    return normalized;
}

std::string lowercase_copy(const std::string& value) {
    std::string lowered = value;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return lowered;
}

adaptivecad::geometry::TopologyHealingPolicy parse_healing_policy_string(const std::string& value) {
    const std::string lowered = lowercase_copy(trim_copy(value));
    if (lowered == "strict") {
        return adaptivecad::geometry::TopologyHealingPolicy::Strict;
    }
    if (lowered == "repaironly" || lowered == "repair_only") {
        return adaptivecad::geometry::TopologyHealingPolicy::RepairOnly;
    }
    return adaptivecad::geometry::TopologyHealingPolicy::RepairFirst;
}

void set_active_scene_document(
    HWND ownerWindow,
    adaptivecad::tool::SceneDocument document,
    const std::string& statusText) {
    document.angular_model = g_sceneDocument.angular_model;
    g_sceneDocument = std::move(document);
    reset_viewport_interaction();
    refresh_output(ownerWindow);
    set_status_text(statusText);
}

void import_geometry_document(HWND ownerWindow, const std::filesystem::path& requestedPath = {}) {
    std::filesystem::path scenePath = requestedPath;
    if (scenePath.empty()) {
        char filePathBuffer[MAX_PATH] = {};
        OPENFILENAMEA ofn{};
        ofn.lStructSize = sizeof(ofn);
        ofn.hwndOwner = ownerWindow;
        ofn.lpstrFile = filePathBuffer;
        ofn.nMaxFile = MAX_PATH;
        ofn.lpstrFilter =
            "Geometry files (*.obj;*.stl;*.ply;*.step;*.stp)\0*.obj;*.stl;*.ply;*.step;*.stp\0"
            "Wavefront OBJ (*.obj)\0*.obj\0"
            "ASCII STL (*.stl)\0*.stl\0"
            "ASCII PLY (*.ply)\0*.ply\0"
            "STEP (*.step;*.stp)\0*.step;*.stp\0"
            "All files (*.*)\0*.*\0";
        ofn.nFilterIndex = 1;
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;
        ofn.lpstrTitle = "Import Mesh Geometry";

        if (!GetOpenFileNameA(&ofn)) {
            return;
        }

        scenePath = std::filesystem::path(filePathBuffer);
    }

    try {
        adaptivecad::tool::SceneDocument importedScene = adaptivecad::tool::import_scene_document(scenePath);
        set_active_scene_document(
            ownerWindow,
            std::move(importedScene),
            std::string("Status: imported geometry scene from ") + scenePath.string());
    } catch (const std::exception& ex) {
        const std::string message = std::string("Geometry import failed: ") + ex.what();
        set_status_text("Status: geometry import failed");
        MessageBoxA(ownerWindow, message.c_str(), "AdaptiveCAD UI", MB_ICONERROR);
    }
}

void create_box_scene(HWND ownerWindow) {
    set_active_scene_document(
        ownerWindow,
        adaptivecad::tool::make_box_scene_document(1.5, 1.0, 0.75),
        "Status: created box primitive");
}

void create_wedge_scene(HWND ownerWindow) {
    set_active_scene_document(
        ownerWindow,
        adaptivecad::tool::make_wedge_scene_document(1.5, 1.0, 0.9),
        "Status: created wedge primitive");
}

void create_plane_scene(HWND ownerWindow) {
    set_active_scene_document(
        ownerWindow,
        adaptivecad::tool::make_plane_scene_document(2.0, 1.25),
        "Status: created plane primitive");
}

void create_torus_scene(HWND ownerWindow) {
    set_active_scene_document(
        ownerWindow,
        adaptivecad::tool::make_torus_scene_document(1.0, 0.25, 32, 12),
        "Status: created torus primitive");
}

void create_scaffold_scene(HWND ownerWindow) {
    adaptivecad::tool::MetamaterialScaffoldParameters parameters;
    parameters.cells_x = 3;
    parameters.cells_y = 2;
    parameters.cells_z = 1;
    parameters.angular_model = g_sceneDocument.angular_model;
    parameters.damage_sphere.center = adaptivecad::geometry::Point3D{0.0, 0.0, 0.0};
    parameters.damage_sphere.radius = 0.45;

    const adaptivecad::tool::GeneratedMetamaterialScaffold scaffold =
        adaptivecad::tool::generate_chern_locked_metamaterial_scaffold(parameters);
    set_active_scene_document(
        ownerWindow,
        scaffold.scene_document,
        "Status: created metamaterial scaffold");
}

bool primitive_append_mode_enabled() {
    return g_primitiveAppendCheck &&
        SendMessageA(g_primitiveAppendCheck, BM_GETCHECK, 0, 0) == BST_CHECKED;
}

void append_scene_document_bodies(const adaptivecad::tool::SceneDocument& document) {
    if (g_sceneDocument.bodies.empty()) {
        g_sceneDocument = document;
        return;
    }

    const std::size_t existingCount = g_sceneDocument.bodies.size();
    for (std::size_t index = 0; index < document.bodies.size(); ++index) {
        adaptivecad::tool::SceneBody body = document.bodies[index];
        if (body.name.empty()) {
            body.name = "Body " + std::to_string(existingCount + index + 1);
        }
        g_sceneDocument.bodies.push_back(std::move(body));
    }

    g_sceneDocument.scene_label = "Composed Scene";
    g_sceneDocument.source_kind = adaptivecad::tool::SceneSourceKind::GeneratedPrimitive;
    g_sceneDocument.source_path.clear();
}

void apply_created_scene_document(
    HWND ownerWindow,
    adaptivecad::tool::SceneDocument document,
    const std::string& replaceStatus,
    const std::string& appendStatus) {
    if (primitive_append_mode_enabled()) {
        document.angular_model = g_sceneDocument.angular_model;
        append_scene_document_bodies(document);
        refresh_output(ownerWindow);
        set_status_text(appendStatus);
        return;
    }

    set_active_scene_document(ownerWindow, std::move(document), replaceStatus);
}

double read_positive_primitive_double(HWND edit, const char* fieldName) {
    double value = 0.0;
    if (!try_parse_double(read_window_text(edit), &value) || value <= 0.0) {
        throw std::runtime_error(std::string(fieldName) + " must be a positive number");
    }
    return value;
}

double read_nonnegative_primitive_double(HWND edit, const char* fieldName) {
    double value = 0.0;
    if (!try_parse_double(read_window_text(edit), &value) || value < 0.0) {
        throw std::runtime_error(std::string(fieldName) + " must be zero or positive");
    }
    return value;
}

int read_primitive_int(HWND edit, const char* fieldName, int minimumValue) {
    const std::string text = trim_copy(read_window_text(edit));
    try {
        std::size_t consumed = 0;
        const long value = std::stol(text, &consumed);
        if (consumed != text.size() || value < minimumValue || value > std::numeric_limits<int>::max()) {
            throw std::runtime_error("invalid");
        }
        return static_cast<int>(value);
    } catch (const std::exception&) {
        throw std::runtime_error(std::string(fieldName) + " must be an integer >= " + std::to_string(minimumValue));
    }
}

void create_primitive_from_controls(HWND ownerWindow) {
    try {
        adaptivecad::tool::SceneDocument document;
        std::string statusText;

        switch (selected_primitive_kind()) {
        case PrimitiveControlKind::Box:
            document = adaptivecad::tool::make_box_scene_document(
                read_positive_primitive_double(g_primitiveParam1Edit, "Width"),
                read_positive_primitive_double(g_primitiveParam2Edit, "Depth"),
                read_positive_primitive_double(g_primitiveParam3Edit, "Height"));
            statusText = "Status: created parameterized box";
            break;
        case PrimitiveControlKind::Wedge:
            document = adaptivecad::tool::make_wedge_scene_document(
                read_positive_primitive_double(g_primitiveParam1Edit, "Width"),
                read_positive_primitive_double(g_primitiveParam2Edit, "Depth"),
                read_positive_primitive_double(g_primitiveParam3Edit, "Height"));
            statusText = "Status: created parameterized wedge";
            break;
        case PrimitiveControlKind::Plane:
            document = adaptivecad::tool::make_plane_scene_document(
                read_positive_primitive_double(g_primitiveParam1Edit, "Width"),
                read_positive_primitive_double(g_primitiveParam2Edit, "Depth"));
            statusText = "Status: created parameterized plane";
            break;
        case PrimitiveControlKind::Torus:
            document = adaptivecad::tool::make_torus_scene_document(
                read_positive_primitive_double(g_primitiveParam1Edit, "Major radius"),
                read_positive_primitive_double(g_primitiveParam2Edit, "Minor radius"),
                static_cast<std::size_t>(read_primitive_int(g_primitiveParam3Edit, "Major segments", 3)),
                static_cast<std::size_t>(read_primitive_int(g_primitiveParam4Edit, "Minor segments", 3)));
            statusText = "Status: created parameterized torus";
            break;
        case PrimitiveControlKind::Scaffold: {
            adaptivecad::tool::MetamaterialScaffoldParameters parameters;
            parameters.cells_x = read_primitive_int(g_primitiveParam1Edit, "Cells X", 1);
            parameters.cells_y = read_primitive_int(g_primitiveParam2Edit, "Cells Y", 0);
            parameters.cells_z = read_primitive_int(g_primitiveParam3Edit, "Cells Z", 0);
            parameters.spacing = read_positive_primitive_double(g_primitiveParam4Edit, "Spacing");
            parameters.damage_sphere.center = adaptivecad::geometry::Point3D{0.0, 0.0, 0.0};
            parameters.damage_sphere.radius = read_nonnegative_primitive_double(g_primitiveParam5Edit, "Damage radius");
            parameters.angular_model = g_sceneDocument.angular_model;
            document = adaptivecad::tool::generate_chern_locked_metamaterial_scaffold(parameters).scene_document;
            statusText = "Status: created parameterized metamaterial scaffold";
            break;
        }
        }

        apply_created_scene_document(
            ownerWindow,
            std::move(document),
            statusText,
            "Status: added primitive to scene");
    } catch (const std::exception& ex) {
        const std::string message = std::string("Primitive creation failed: ") + ex.what();
        set_status_text("Status: primitive creation failed");
        MessageBoxA(ownerWindow, message.c_str(), "AdaptiveCAD UI", MB_ICONERROR);
    }
}

bool selected_scene_body_index(std::size_t* bodyIndexOut) {
    if (!bodyIndexOut || !g_geometryViewportState.ready) {
        return false;
    }

    const ViewportSelectionKind kind = g_geometryViewportState.selection_kind;
    const adaptivecad::geometry::EntityId id = g_geometryViewportState.selected_id;
    if (kind == ViewportSelectionKind::None || id == 0) {
        return false;
    }

    for (std::size_t bodyIndex = 0; bodyIndex < g_geometryViewportState.bodies.size(); ++bodyIndex) {
        const auto& body = g_geometryViewportState.bodies[bodyIndex];
        bool matches = kind == ViewportSelectionKind::Body && body.id == id;

        for (adaptivecad::geometry::EntityId faceId : body.face_ids) {
            const auto* face = find_viewport_face(faceId);
            if (!face) {
                continue;
            }

            matches = matches || (kind == ViewportSelectionKind::Face && face->id == id);
            for (adaptivecad::geometry::EntityId edgeId : face->edge_ids) {
                const auto* edge = find_viewport_edge(edgeId);
                if (!edge) {
                    continue;
                }

                matches = matches || (kind == ViewportSelectionKind::Edge && edge->id == id);
                matches = matches || (kind == ViewportSelectionKind::Vertex &&
                    (edge->start_vertex_id == id || edge->end_vertex_id == id));
            }
        }

        if (matches && bodyIndex < g_sceneDocument.bodies.size()) {
            *bodyIndexOut = bodyIndex;
            return true;
        }
    }

    return false;
}

adaptivecad::geometry::Point3D scene_body_centroid(const adaptivecad::tool::SceneBody& body) {
    adaptivecad::geometry::Point3D center{};
    if (body.vertices.empty()) {
        return center;
    }

    for (const auto& point : body.vertices) {
        center.x += point.x;
        center.y += point.y;
        center.z += point.z;
    }
    const double invCount = 1.0 / static_cast<double>(body.vertices.size());
    center.x *= invCount;
    center.y *= invCount;
    center.z *= invCount;
    return center;
}

void translate_scene_body(adaptivecad::tool::SceneBody* body, double dx, double dy, double dz) {
    if (!body) {
        return;
    }

    for (auto& point : body->vertices) {
        point.x += dx;
        point.y += dy;
        point.z += dz;
    }
}

void transform_scene_body(
    adaptivecad::tool::SceneBody* body,
    double dx,
    double dy,
    double dz,
    double scale,
    double rotateDegreesZ) {
    if (!body) {
        return;
    }
    if (scale <= 0.0) {
        throw std::runtime_error("Scale must be a positive number");
    }

    const auto center = scene_body_centroid(*body);
    const double radians = rotateDegreesZ * std::numbers::pi_v<double> / 180.0;
    const double c = std::cos(radians);
    const double s = std::sin(radians);

    for (auto& point : body->vertices) {
        const double localX = (point.x - center.x) * scale;
        const double localY = (point.y - center.y) * scale;
        const double localZ = (point.z - center.z) * scale;
        point.x = center.x + localX * c - localY * s + dx;
        point.y = center.y + localX * s + localY * c + dy;
        point.z = center.z + localZ + dz;
    }

    body->volume_hint = std::max(0.0, body->volume_hint * scale * scale * scale);
}

void update_body_edit_controls() {
    if (g_bodyDxEdit) {
        SetWindowTextA(g_bodyDxEdit, "0");
    }
    if (g_bodyDyEdit) {
        SetWindowTextA(g_bodyDyEdit, "0");
    }
    if (g_bodyDzEdit) {
        SetWindowTextA(g_bodyDzEdit, "0");
    }
    if (g_bodyScaleEdit) {
        SetWindowTextA(g_bodyScaleEdit, "1");
    }
    if (g_bodyRotateEdit) {
        SetWindowTextA(g_bodyRotateEdit, "0");
    }

    std::size_t bodyIndex = 0;
    if (selected_scene_body_index(&bodyIndex) && bodyIndex < g_sceneDocument.bodies.size()) {
        if (g_bodyNameEdit) {
            SetWindowTextA(g_bodyNameEdit, g_sceneDocument.bodies[bodyIndex].name.c_str());
        }
        return;
    }

    if (g_bodyNameEdit) {
        SetWindowTextA(g_bodyNameEdit, "");
    }
}

void apply_selected_body_transform(HWND ownerWindow) {
    try {
        std::size_t bodyIndex = 0;
        if (!selected_scene_body_index(&bodyIndex)) {
            throw std::runtime_error("Select a body, face, edge, or vertex first");
        }

        double moveX = 0.0;
        double moveY = 0.0;
        double moveZ = 0.0;
        double scale = 1.0;
        double rotateZ = 0.0;
        if (!try_parse_double(read_window_text(g_bodyDxEdit), &moveX) ||
            !try_parse_double(read_window_text(g_bodyDyEdit), &moveY) ||
            !try_parse_double(read_window_text(g_bodyDzEdit), &moveZ) ||
            !try_parse_double(read_window_text(g_bodyScaleEdit), &scale) ||
            !try_parse_double(read_window_text(g_bodyRotateEdit), &rotateZ)) {
            throw std::runtime_error("Transform fields must be numeric");
        }

        transform_scene_body(&g_sceneDocument.bodies[bodyIndex], moveX, moveY, moveZ, scale, rotateZ);
        apply_geometry_selection(ViewportSelectionKind::None, 0, "Status: body transformed");
        refresh_output(ownerWindow);
        set_status_text("Status: body transformed");
    } catch (const std::exception& ex) {
        const std::string message = std::string("Transform failed: ") + ex.what();
        set_status_text("Status: transform failed");
        MessageBoxA(ownerWindow, message.c_str(), "AdaptiveCAD UI", MB_ICONERROR);
    }
}

void duplicate_selected_body(HWND ownerWindow) {
    try {
        std::size_t bodyIndex = 0;
        if (!selected_scene_body_index(&bodyIndex)) {
            throw std::runtime_error("Select a body, face, edge, or vertex first");
        }

        adaptivecad::tool::SceneBody copy = g_sceneDocument.bodies[bodyIndex];
        copy.name = copy.name.empty() ? "Body Copy" : copy.name + " Copy";
        translate_scene_body(&copy, 0.35, 0.35, 0.0);
        g_sceneDocument.bodies.push_back(std::move(copy));
        g_sceneDocument.scene_label = "Composed Scene";
        g_sceneDocument.source_kind = adaptivecad::tool::SceneSourceKind::GeneratedPrimitive;
        g_sceneDocument.source_path.clear();
        apply_geometry_selection(ViewportSelectionKind::None, 0, "Status: body duplicated");
        refresh_output(ownerWindow);
        set_status_text("Status: body duplicated");
    } catch (const std::exception& ex) {
        const std::string message = std::string("Duplicate failed: ") + ex.what();
        set_status_text("Status: duplicate failed");
        MessageBoxA(ownerWindow, message.c_str(), "AdaptiveCAD UI", MB_ICONERROR);
    }
}

void delete_selected_body(HWND ownerWindow) {
    try {
        std::size_t bodyIndex = 0;
        if (!selected_scene_body_index(&bodyIndex)) {
            throw std::runtime_error("Select a body, face, edge, or vertex first");
        }
        if (g_sceneDocument.bodies.size() <= 1) {
            throw std::runtime_error("Cannot delete the last body in the scene");
        }

        g_sceneDocument.bodies.erase(g_sceneDocument.bodies.begin() + static_cast<std::ptrdiff_t>(bodyIndex));
        apply_geometry_selection(ViewportSelectionKind::None, 0, "Status: body deleted");
        refresh_output(ownerWindow);
        set_status_text("Status: body deleted");
    } catch (const std::exception& ex) {
        const std::string message = std::string("Delete failed: ") + ex.what();
        set_status_text("Status: delete failed");
        MessageBoxA(ownerWindow, message.c_str(), "AdaptiveCAD UI", MB_ICONERROR);
    }
}

void rename_selected_body(HWND ownerWindow) {
    try {
        std::size_t bodyIndex = 0;
        if (!selected_scene_body_index(&bodyIndex)) {
            throw std::runtime_error("Select a body, face, edge, or vertex first");
        }

        const std::string name = trim_copy(read_window_text(g_bodyNameEdit));
        if (name.empty()) {
            throw std::runtime_error("Body name cannot be empty");
        }

        g_sceneDocument.bodies[bodyIndex].name = name;
        refresh_output(ownerWindow);
        set_status_text("Status: body renamed");
    } catch (const std::exception& ex) {
        const std::string message = std::string("Rename failed: ") + ex.what();
        set_status_text("Status: rename failed");
        MessageBoxA(ownerWindow, message.c_str(), "AdaptiveCAD UI", MB_ICONERROR);
    }
}

std::string default_export_geometry_filename() {
    std::string stem = g_sceneDocument.scene_label.empty()
        ? std::string("adaptivecad_scene")
        : std::filesystem::path(g_sceneDocument.scene_label).stem().string();
    stem = trim_copy(stem);
    if (stem.empty()) {
        stem = "adaptivecad_scene";
    }

    for (char& ch : stem) {
        const unsigned char uch = static_cast<unsigned char>(ch);
        if (std::isalnum(uch) == 0 && ch != '_' && ch != '-' && ch != '.') {
            ch = '_';
        }
    }

    return stem + ".obj";
}

void export_geometry_document(HWND ownerWindow) {
    try {
        if (g_sceneDocument.bodies.empty()) {
            throw std::runtime_error("No geometry scene is available to export");
        }

        char filePathBuffer[MAX_PATH] = {};
        const std::string defaultFileName = default_export_geometry_filename();
        const std::size_t copyCount = std::min(defaultFileName.size(), static_cast<std::size_t>(MAX_PATH - 1));
        std::copy_n(defaultFileName.data(), copyCount, filePathBuffer);
        const std::string initialDir = g_workspaceRoot.string();

        OPENFILENAMEA ofn{};
        ofn.lStructSize = sizeof(ofn);
        ofn.hwndOwner = ownerWindow;
        ofn.lpstrFile = filePathBuffer;
        ofn.nMaxFile = MAX_PATH;
        ofn.lpstrFilter = "Wavefront OBJ (*.obj)\0*.obj\0All files (*.*)\0*.*\0";
        ofn.nFilterIndex = 1;
        ofn.lpstrInitialDir = initialDir.c_str();
        ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT | OFN_HIDEREADONLY;
        ofn.lpstrTitle = "Export Geometry As OBJ";

        if (!GetSaveFileNameA(&ofn)) {
            return;
        }

        std::filesystem::path exportPath(filePathBuffer);
        if (exportPath.extension().empty()) {
            exportPath.replace_extension(".obj");
        }

        adaptivecad::tool::export_scene_document(g_sceneDocument, exportPath);
        set_status_text(std::string("Status: exported geometry to ") + exportPath.string());
    } catch (const std::exception& ex) {
        const std::string message = std::string("Geometry export failed: ") + ex.what();
        set_status_text("Status: geometry export failed");
        MessageBoxA(ownerWindow, message.c_str(), "AdaptiveCAD UI", MB_ICONERROR);
    }
}

ViewportProjectionMode parse_viewport_projection_string(const std::string& value) {
    const std::string lowered = lowercase_copy(trim_copy(value));
    if (lowered == "top") {
        return ViewportProjectionMode::Top;
    }
    if (lowered == "front") {
        return ViewportProjectionMode::Front;
    }
    if (lowered == "right") {
        return ViewportProjectionMode::Right;
    }
    return ViewportProjectionMode::Isometric;
}

ViewportSelectionKind parse_viewport_selection_kind_string(const std::string& value) {
    const std::string lowered = lowercase_copy(trim_copy(value));
    if (lowered == "vertex") {
        return ViewportSelectionKind::Vertex;
    }
    if (lowered == "edge") {
        return ViewportSelectionKind::Edge;
    }
    if (lowered == "face") {
        return ViewportSelectionKind::Face;
    }
    if (lowered == "body") {
        return ViewportSelectionKind::Body;
    }
    return ViewportSelectionKind::None;
}

struct ParsedCsvDataset {
    std::vector<adaptivecad::core::AreaSample> samples;
    std::size_t rejected_rows = 0;
    std::string summary;
};

ParsedCsvDataset parse_area_samples_csv(const std::filesystem::path& csvPath) {
    using adaptivecad::core::AreaSample;

    std::ifstream input(csvPath, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Could not open CSV file");
    }

    std::vector<AreaSample> samples;
    std::string line;
    std::size_t lineNumber = 0;
    std::size_t parseFailures = 0;

    while (std::getline(input, line)) {
        ++lineNumber;

        const std::string trimmedLine = trim_copy(line);
        if (trimmedLine.empty() || trimmedLine[0] == '#') {
            continue;
        }

        const std::string normalized = normalize_csv_line(trimmedLine);
        std::stringstream lineStream(normalized);
        std::string firstToken;
        std::string secondToken;

        if (!std::getline(lineStream, firstToken, ',') || !std::getline(lineStream, secondToken, ',')) {
            ++parseFailures;
            continue;
        }

        double radius = 0.0;
        double area = 0.0;
        if (!try_parse_double(firstToken, &radius) || !try_parse_double(secondToken, &area)) {
            ++parseFailures;
            continue;
        }

        if (radius <= 0.0 || area <= 0.0) {
            throw std::runtime_error(
                "CSV contains non-positive radius or area at line " + std::to_string(lineNumber));
        }

        samples.push_back(AreaSample{radius, area});
    }

    if (samples.size() < 3) {
        throw std::runtime_error(
            "CSV import requires at least 3 valid rows with positive radius and area values");
    }

    if (parseFailures > 0 && samples.empty()) {
        throw std::runtime_error("CSV parse failed: no numeric radius/area rows were found");
    }

    ParsedCsvDataset dataset;
    dataset.samples = std::move(samples);
    dataset.rejected_rows = parseFailures;
    dataset.summary =
        "Imported " + std::to_string(dataset.samples.size()) + " rows" +
        (parseFailures > 0 ? (", rejected " + std::to_string(parseFailures) + " malformed rows") : "");
    return dataset;
}

std::string build_summary_text(const ActiveFitAnalysisState& analysis, const GeometryViewportState& geometryState) {
    using adaptivecad::core::InverseRecovery;
    using adaptivecad::geometry::BRepKernel;

    const auto& nonlinearFit = analysis.nonlinear_fit;
    const auto& modelComparison = analysis.model_comparison;
    const auto& diag = geometryState.diagnostic;
    const adaptivecad::geometry::EntityId faceId = geometryState.faces.empty() ? 0 : geometryState.faces.front().id;
    const adaptivecad::geometry::EntityId bodyId = geometryState.bodies.empty() ? 0 : geometryState.bodies.front().id;

    std::ostringstream out;
    out << "AdaptiveCAD UI Prototype\r\n";
    out << "======================\r\n\r\n";
    out << "Generated at = " << make_timestamp("%Y-%m-%d %H:%M:%S") << "\r\n";
    out << "Workspace = " << g_workspaceRoot.string() << "\r\n\r\n";
    out << "pi_f(2) = " << analysis.pi_f_at_2 << "\r\n";
    out << "lambda_f(2) = " << analysis.lambda_f_at_2 << "\r\n";
    out << "eta(2) = " << analysis.eta_at_2 << "\r\n";
    out << "shell_weight(2) = " << analysis.shell_weight_at_2 << "\r\n";
    out << "effective_dimension = " << analysis.effective_dimension << "\r\n\r\n";

    out << "Recovered beta = " << analysis.power_law_demo_fit.beta << "\r\n";
    out << "Recovered lambda0 = " << analysis.power_law_demo_fit.lambda0 << "\r\n";
    out << "RMS relative fit error = " << analysis.power_law_demo_fit.rms_relative_error << "\r\n\r\n";

    out << "Angular lambda0 = " << analysis.angular_model.lambda0 << "\r\n";
    out << "Angular cosine coefficients = " << format_double_list(analysis.angular_model.cosine_coefficients) << "\r\n";
    out << "Angular sine coefficients = " << format_double_list(analysis.angular_model.sine_coefficients) << "\r\n";
    out << "Angular lambda(theta=pi/2) = " << analysis.angular_lambda_pi_over_2 << "\r\n";
    out << "Angular phase_map(pi/2) = " << analysis.angular_phase_map_pi_over_2 << "\r\n";
    out << "Angular cos_f^(2)(pi/2) = " << analysis.angular_cos_mode_2_pi_over_2 << "\r\n\r\n";

    out << "Gaussian nonlinear fit epsilon = " << nonlinearFit.epsilon
        << " (95% CI: " << nonlinearFit.epsilon_ci95_low << ", " << nonlinearFit.epsilon_ci95_high << ")\r\n";
    out << "Gaussian nonlinear fit length_scale = " << nonlinearFit.length_scale
        << " (95% CI: " << nonlinearFit.length_scale_ci95_low << ", " << nonlinearFit.length_scale_ci95_high
        << ")\r\n";
    out << "Gaussian fit dataset source = " << analysis.measurement_source << "\r\n";
    out << "Gaussian fit sample count = " << analysis.measurement_samples.size() << "\r\n";
    out << "Gaussian nonlinear fit converged = " << (nonlinearFit.converged ? "true" : "false")
        << "\r\n\r\n";

    out << "Model comparison best = "
        << InverseRecovery::modelKindName(modelComparison.best_model)
        << " (weight=" << modelComparison.best_model_weight << ")\r\n";
    out << "Model score constant: rms=" << modelComparison.constant_score.rms_relative_error
        << " aic=" << modelComparison.constant_score.aic
        << " bic=" << modelComparison.constant_score.bic
        << " weight=" << modelComparison.constant_weight << "\r\n";
    out << "Model score power-law: rms=" << modelComparison.power_law_score.rms_relative_error
        << " aic=" << modelComparison.power_law_score.aic
        << " bic=" << modelComparison.power_law_score.bic
        << " weight=" << modelComparison.power_law_weight << "\r\n";
    out << "Model score gaussian: rms=" << modelComparison.gaussian_score.rms_relative_error
        << " aic=" << modelComparison.gaussian_score.aic
        << " bic=" << modelComparison.gaussian_score.bic
        << " weight=" << modelComparison.gaussian_weight << "\r\n";
    if (!modelComparison.warning.empty()) {
        out << "Model comparison warning = " << modelComparison.warning << "\r\n";
    }
    out << "\r\n";

    out << "Geometry scene = " << geometryState.scene_label << "\r\n";
    out << "Geometry scene source = " << active_scene_source_label() << "\r\n";
    out << "Geometry backend = " << geometryState.backend_name << "\r\n";
    out << "Healing policy = " << BRepKernel::healing_policy_name(diag.healing_policy) << "\r\n";
    out << "OpenCascade native topology active = "
        << (geometryState.has_open_cascade_topology ? "true" : "false") << "\r\n";
    out << "Topology diagnostic success = " << (diag.success ? "true" : "false") << "\r\n";
    out << "Topology diagnostic op = " << diag.operation << "\r\n";
    out << "Topology healing attempted = " << (diag.healing_attempted ? "true" : "false") << "\r\n";
    out << "Topology healing succeeded = " << (diag.healing_succeeded ? "true" : "false") << "\r\n";
    out << "Topology edges reordered = " << (diag.edges_reordered ? "true" : "false") << "\r\n";
    out << "Topology bridge edges created = " << (diag.bridge_edges_created ? "true" : "false") << "\r\n";
    out << "Face id = " << faceId << ", body id = " << bodyId << "\r\n";
    out << "BRep counts: V=" << geometryState.vertices.size() << " E=" << geometryState.edges.size()
        << " F=" << geometryState.faces.size() << " B=" << geometryState.bodies.size() << "\r\n";

    return out.str();
}

void append_json_string(std::ostringstream& out, const std::string& value) {
    out << '"';
    for (unsigned char ch : value) {
        switch (ch) {
        case '"':
            out << "\\\"";
            break;
        case '\\':
            out << "\\\\";
            break;
        case '\b':
            out << "\\b";
            break;
        case '\f':
            out << "\\f";
            break;
        case '\n':
            out << "\\n";
            break;
        case '\r':
            out << "\\r";
            break;
        case '\t':
            out << "\\t";
            break;
        default:
            if (ch < 0x20) {
                out << "\\u00" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(ch)
                    << std::dec << std::setw(0) << std::setfill(' ');
            } else {
                out << static_cast<char>(ch);
            }
            break;
        }
    }
    out << '"';
}

void append_json_number(std::ostringstream& out, double value) {
    if (!std::isfinite(value)) {
        out << "null";
        return;
    }

    std::ostringstream number;
    number << std::setprecision(15) << value;
    out << number.str();
}

void append_json_area_samples(std::ostringstream& out, const std::vector<adaptivecad::core::AreaSample>& samples) {
    out << "[\n";
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const auto& sample = samples[index];
        out << "      {\"radius\":";
        append_json_number(out, sample.radius);
        out << ",\"area\":";
        append_json_number(out, sample.area);
        out << "}";
        if (index + 1 < samples.size()) {
            out << ',';
        }
        out << "\n";
    }
    out << "    ]";
}

void append_json_plot_points(std::ostringstream& out, const std::vector<PlotPoint>& points) {
    out << "[\n";
    for (std::size_t index = 0; index < points.size(); ++index) {
        const auto& point = points[index];
        out << "      {\"x\":";
        append_json_number(out, point.x);
        out << ",\"y\":";
        append_json_number(out, point.y);
        out << "}";
        if (index + 1 < points.size()) {
            out << ',';
        }
        out << "\n";
    }
    out << "    ]";
}

void append_json_entity_id_array(std::ostringstream& out, const std::vector<adaptivecad::geometry::EntityId>& ids) {
    out << '[';
    for (std::size_t index = 0; index < ids.size(); ++index) {
        if (index > 0) {
            out << ',';
        }
        out << ids[index];
    }
    out << ']';
}

void append_json_point3(std::ostringstream& out, const adaptivecad::geometry::Point3D& point) {
    out << "{\"x\":";
    append_json_number(out, point.x);
    out << ",\"y\":";
    append_json_number(out, point.y);
    out << ",\"z\":";
    append_json_number(out, point.z);
    out << '}';
}

void append_json_scene_color(std::ostringstream& out, const adaptivecad::tool::SceneColor& color) {
    out << "{\"red\":";
    append_json_number(out, color.red);
    out << ",\"green\":";
    append_json_number(out, color.green);
    out << ",\"blue\":";
    append_json_number(out, color.blue);
    out << ",\"alpha\":";
    append_json_number(out, color.alpha);
    out << '}';
}

std::string build_analysis_report_json(
    const ActiveFitAnalysisState& analysis,
    const GeometryViewportState& geometryState,
    const FitPlotState& plotState,
    const std::string& summaryText) {
    using adaptivecad::core::InverseRecovery;
    using adaptivecad::geometry::BRepKernel;

    std::ostringstream out;
    out << "{\n";
    out << "  \"schema_version\": 2,\n";
    out << "  \"generated_at\": ";
    append_json_string(out, make_timestamp("%Y-%m-%d %H:%M:%S"));
    out << ",\n  \"workspace\": ";
    append_json_string(out, g_workspaceRoot.string());
    out << ",\n  \"dataset\": {\n";
    out << "    \"source\": ";
    append_json_string(out, g_datasetInfo.using_imported_csv ? "csv" : "synthetic");
    out << ",\n    \"source_label\": ";
    append_json_string(out, analysis.measurement_source);
    out << ",\n    \"active_csv_path\": ";
    append_json_string(out, g_datasetInfo.active_csv_path.string());
    out << ",\n    \"valid_rows\": " << g_datasetInfo.valid_rows;
    out << ",\n    \"rejected_rows\": " << g_datasetInfo.rejected_rows;
    out << ",\n    \"summary\": ";
    append_json_string(out, g_datasetInfo.summary);
    out << ",\n    \"samples\": ";
    append_json_area_samples(out, analysis.measurement_samples);
    out << "\n  },\n";

    out << "  \"field_demo\": {\n";
    out << "    \"pi_f_at_2\": ";
    append_json_number(out, analysis.pi_f_at_2);
    out << ",\n    \"lambda_f_at_2\": ";
    append_json_number(out, analysis.lambda_f_at_2);
    out << ",\n    \"eta_at_2\": ";
    append_json_number(out, analysis.eta_at_2);
    out << ",\n    \"shell_weight_at_2\": ";
    append_json_number(out, analysis.shell_weight_at_2);
    out << ",\n    \"effective_dimension\": ";
    append_json_number(out, analysis.effective_dimension);
    out << ",\n    \"power_law_demo_fit\": {\n";
    out << "      \"lambda0\": ";
    append_json_number(out, analysis.power_law_demo_fit.lambda0);
    out << ",\n      \"beta\": ";
    append_json_number(out, analysis.power_law_demo_fit.beta);
    out << ",\n      \"r0\": ";
    append_json_number(out, analysis.power_law_demo_fit.r0);
    out << ",\n      \"rms_relative_error\": ";
    append_json_number(out, analysis.power_law_demo_fit.rms_relative_error);
    out << "\n    },\n";
    out << "    \"angular_branch\": {\n";
    out << "      \"lambda0\": ";
    append_json_number(out, analysis.angular_model.lambda0);
    out << ",\n      \"cosine_coefficients\": [";
    for (std::size_t index = 0; index < analysis.angular_model.cosine_coefficients.size(); ++index) {
        if (index > 0) {
            out << ',';
        }
        append_json_number(out, analysis.angular_model.cosine_coefficients[index]);
    }
    out << "],\n      \"sine_coefficients\": [";
    for (std::size_t index = 0; index < analysis.angular_model.sine_coefficients.size(); ++index) {
        if (index > 0) {
            out << ',';
        }
        append_json_number(out, analysis.angular_model.sine_coefficients[index]);
    }
    out << "],\n      \"lambda_theta_pi_over_2\": ";
    append_json_number(out, analysis.angular_lambda_pi_over_2);
    out << ",\n      \"phase_map_pi_over_2\": ";
    append_json_number(out, analysis.angular_phase_map_pi_over_2);
    out << ",\n      \"cos_mode_2_pi_over_2\": ";
    append_json_number(out, analysis.angular_cos_mode_2_pi_over_2);
    out << "\n    }\n  },\n";

    out << "  \"gaussian_nonlinear_fit\": {\n";
    out << "    \"epsilon\": ";
    append_json_number(out, analysis.nonlinear_fit.epsilon);
    out << ",\n    \"length_scale\": ";
    append_json_number(out, analysis.nonlinear_fit.length_scale);
    out << ",\n    \"rms_relative_error\": ";
    append_json_number(out, analysis.nonlinear_fit.rms_relative_error);
    out << ",\n    \"sse\": ";
    append_json_number(out, analysis.nonlinear_fit.sse);
    out << ",\n    \"iterations\": " << analysis.nonlinear_fit.iterations;
    out << ",\n    \"converged\": " << (analysis.nonlinear_fit.converged ? "true" : "false");
    out << ",\n    \"epsilon_stddev\": ";
    append_json_number(out, analysis.nonlinear_fit.epsilon_stddev);
    out << ",\n    \"length_scale_stddev\": ";
    append_json_number(out, analysis.nonlinear_fit.length_scale_stddev);
    out << ",\n    \"epsilon_ci95\": {\"low\":";
    append_json_number(out, analysis.nonlinear_fit.epsilon_ci95_low);
    out << ",\"high\":";
    append_json_number(out, analysis.nonlinear_fit.epsilon_ci95_high);
    out << "},\n    \"length_scale_ci95\": {\"low\":";
    append_json_number(out, analysis.nonlinear_fit.length_scale_ci95_low);
    out << ",\"high\":";
    append_json_number(out, analysis.nonlinear_fit.length_scale_ci95_high);
    out << "},\n    \"covariance\": {\"c00\":";
    append_json_number(out, analysis.nonlinear_fit.covariance_00);
    out << ",\"c01\":";
    append_json_number(out, analysis.nonlinear_fit.covariance_01);
    out << ",\"c11\":";
    append_json_number(out, analysis.nonlinear_fit.covariance_11);
    out << "}\n  },\n";

    const auto append_score = [&](const adaptivecad::core::ModelScore& score) {
        out << "{\"model\":";
        append_json_string(out, InverseRecovery::modelKindName(score.model));
        out << ",\"parameter_count\":" << score.parameter_count;
        out << ",\"sse\":";
        append_json_number(out, score.sse);
        out << ",\"rms_relative_error\":";
        append_json_number(out, score.rms_relative_error);
        out << ",\"aic\":";
        append_json_number(out, score.aic);
        out << ",\"bic\":";
        append_json_number(out, score.bic);
        out << ",\"converged\":" << (score.converged ? "true" : "false") << '}';
    };

    out << "  \"model_comparison\": {\n";
    out << "    \"best_model\": ";
    append_json_string(out, InverseRecovery::modelKindName(analysis.model_comparison.best_model));
    out << ",\n    \"best_model_weight\": ";
    append_json_number(out, analysis.model_comparison.best_model_weight);
    out << ",\n    \"warning\": ";
    append_json_string(out, analysis.model_comparison.warning);
    out << ",\n    \"constant\": {\n";
    out << "      \"fit\": {\"lambda_constant\":";
    append_json_number(out, analysis.model_comparison.constant_fit.lambda_constant);
    out << ",\"epsilon\":";
    append_json_number(out, analysis.model_comparison.constant_fit.epsilon);
    out << ",\"rms_relative_error\":";
    append_json_number(out, analysis.model_comparison.constant_fit.rms_relative_error);
    out << ",\"sse\":";
    append_json_number(out, analysis.model_comparison.constant_fit.sse);
    out << "},\n      \"score\": ";
    append_score(analysis.model_comparison.constant_score);
    out << ",\n      \"weight\": ";
    append_json_number(out, analysis.model_comparison.constant_weight);
    out << "\n    },\n";
    out << "    \"power_law\": {\n";
    out << "      \"fit\": {\"lambda0\":";
    append_json_number(out, analysis.model_comparison.power_law_fit.lambda0);
    out << ",\"beta\":";
    append_json_number(out, analysis.model_comparison.power_law_fit.beta);
    out << ",\"r0\":";
    append_json_number(out, analysis.model_comparison.power_law_fit.r0);
    out << ",\"rms_relative_error\":";
    append_json_number(out, analysis.model_comparison.power_law_fit.rms_relative_error);
    out << "},\n      \"score\": ";
    append_score(analysis.model_comparison.power_law_score);
    out << ",\n      \"weight\": ";
    append_json_number(out, analysis.model_comparison.power_law_weight);
    out << "\n    },\n";
    out << "    \"gaussian\": {\n";
    out << "      \"fit\": {\"epsilon\":";
    append_json_number(out, analysis.model_comparison.gaussian_fit.epsilon);
    out << ",\"length_scale\":";
    append_json_number(out, analysis.model_comparison.gaussian_fit.length_scale);
    out << ",\"rms_relative_error\":";
    append_json_number(out, analysis.model_comparison.gaussian_fit.rms_relative_error);
    out << ",\"sse\":";
    append_json_number(out, analysis.model_comparison.gaussian_fit.sse);
    out << ",\"iterations\":" << analysis.model_comparison.gaussian_fit.iterations;
    out << ",\"converged\":" << (analysis.model_comparison.gaussian_fit.converged ? "true" : "false") << "},\n      \"score\": ";
    append_score(analysis.model_comparison.gaussian_score);
    out << ",\n      \"weight\": ";
    append_json_number(out, analysis.model_comparison.gaussian_weight);
    out << "\n    }\n  },\n";

    out << "  \"plot\": {\n";
    out << "    \"ready\": " << (plotState.ready ? "true" : "false");
    out << ",\n    \"title\": ";
    append_json_string(out, plotState.title);
    out << ",\n    \"subtitle\": ";
    append_json_string(out, plotState.subtitle);
    out << ",\n    \"residual_caption\": ";
    append_json_string(out, plotState.residual_caption);
    out << ",\n    \"best_model\": ";
    append_json_string(out, InverseRecovery::modelKindName(plotState.best_model));
    out << ",\n    \"measured_points\": ";
    append_json_plot_points(out, plotState.measured_points);
    out << ",\n    \"constant_curve\": ";
    append_json_plot_points(out, plotState.constant_curve);
    out << ",\n    \"power_law_curve\": ";
    append_json_plot_points(out, plotState.power_law_curve);
    out << ",\n    \"gaussian_curve\": ";
    append_json_plot_points(out, plotState.gaussian_curve);
    out << ",\n    \"residual_points\": ";
    append_json_plot_points(out, plotState.residual_points);
    out << "\n  },\n";

    out << "  \"geometry\": {\n";
    out << "    \"ready\": " << (geometryState.ready ? "true" : "false");
    out << ",\n    \"backend_name\": ";
    append_json_string(out, geometryState.backend_name);
    out << ",\n    \"scene_label\": ";
    append_json_string(out, geometryState.scene_label);
    out << ",\n    \"scene_source_kind\": ";
    append_json_string(out, geometryState.scene_source_kind);
    out << ",\n    \"scene_source_path\": ";
    append_json_string(out, geometryState.scene_source_path);
    out << ",\n    \"subtitle\": ";
    append_json_string(out, geometryState.subtitle);
    out << ",\n    \"diagnostic_summary\": ";
    append_json_string(out, geometryState.diagnostic_summary);
    out << ",\n    \"healing_policy\": ";
    append_json_string(out, BRepKernel::healing_policy_name(geometryState.diagnostic.healing_policy));
    out << ",\n    \"has_open_cascade_topology\": "
        << (geometryState.has_open_cascade_topology ? "true" : "false");
    out << ",\n    \"counts\": {\"vertices\":" << geometryState.vertices.size()
        << ",\"edges\":" << geometryState.edges.size()
        << ",\"faces\":" << geometryState.faces.size()
        << ",\"bodies\":" << geometryState.bodies.size() << "},\n";
    out << "    \"selection\": {\"kind\":";
    append_json_string(out, viewport_selection_kind_name(geometryState.selection_kind));
    out << ",\"selected_id\":" << geometryState.selected_id << ",\"label\":";
    append_json_string(out, viewport_selection_label());
    out << "},\n";
    out << "    \"viewport\": {\"projection\":";
    append_json_string(out, viewport_projection_name(g_geometryViewportViewState.projection));
    out << ",\"zoom\":";
    append_json_number(out, g_geometryViewportViewState.zoom);
    out << ",\"pan_x\":";
    append_json_number(out, g_geometryViewportViewState.pan_x);
    out << ",\"pan_y\":";
    append_json_number(out, g_geometryViewportViewState.pan_y);
    out << "},\n";
    out << "    \"topology_diagnostic\": {\n";
    out << "      \"success\": " << (geometryState.diagnostic.success ? "true" : "false");
    out << ",\n      \"operation\": ";
    append_json_string(out, geometryState.diagnostic.operation);
    out << ",\n      \"backend\": ";
    append_json_string(out, geometryState.diagnostic.backend);
    out << ",\n      \"message\": ";
    append_json_string(out, geometryState.diagnostic.message);
    out << ",\n      \"healing_attempted\": " << (geometryState.diagnostic.healing_attempted ? "true" : "false");
    out << ",\n      \"healing_succeeded\": " << (geometryState.diagnostic.healing_succeeded ? "true" : "false");
    out << ",\n      \"edges_reordered\": " << (geometryState.diagnostic.edges_reordered ? "true" : "false");
    out << ",\n      \"bridge_edges_created\": " << (geometryState.diagnostic.bridge_edges_created ? "true" : "false");
    out << ",\n      \"non_planar_input_detected\": "
        << (geometryState.diagnostic.non_planar_input_detected ? "true" : "false");
    out << ",\n      \"occ_wire_error_code\": " << geometryState.diagnostic.occ_wire_error_code;
    out << ",\n      \"occ_face_error_code\": " << geometryState.diagnostic.occ_face_error_code;
    out << ",\n      \"occ_wire_closed\": " << (geometryState.diagnostic.occ_wire_closed ? "true" : "false");
    out << ",\n      \"occ_wire_valid\": " << (geometryState.diagnostic.occ_wire_valid ? "true" : "false");
    out << ",\n      \"occ_face_valid\": " << (geometryState.diagnostic.occ_face_valid ? "true" : "false");
    out << ",\n      \"healing_edge_ids\": ";
    append_json_entity_id_array(out, geometryState.diagnostic.healing_edge_ids);
    out << ",\n      \"input_edge_ids\": ";
    append_json_entity_id_array(out, geometryState.diagnostic.input_edge_ids);
    out << ",\n      \"invalid_edge_ids\": ";
    append_json_entity_id_array(out, geometryState.diagnostic.invalid_edge_ids);
    out << "\n    },\n";
    out << "    \"vertices\": [\n";
    for (std::size_t index = 0; index < geometryState.vertices.size(); ++index) {
        const auto& vertex = geometryState.vertices[index];
        out << "      {\"id\":" << vertex.id << ",\"point\":";
        append_json_point3(out, vertex.point);
        out << '}';
        if (index + 1 < geometryState.vertices.size()) {
            out << ',';
        }
        out << "\n";
    }
    out << "    ],\n    \"edges\": [\n";
    for (std::size_t index = 0; index < geometryState.edges.size(); ++index) {
        const auto& edge = geometryState.edges[index];
        out << "      {\"id\":" << edge.id
            << ",\"start_vertex_id\":" << edge.start_vertex_id
            << ",\"end_vertex_id\":" << edge.end_vertex_id
            << ",\"length\":";
        append_json_number(out, edge.length);
        out << '}';
        if (index + 1 < geometryState.edges.size()) {
            out << ',';
        }
        out << "\n";
    }
    out << "    ],\n    \"faces\": [\n";
    for (std::size_t index = 0; index < geometryState.faces.size(); ++index) {
        const auto& face = geometryState.faces[index];
        out << "      {\"id\":" << face.id << ",\"edge_ids\":";
        append_json_entity_id_array(out, face.edge_ids);
        out << ",\"area\":";
        append_json_number(out, face.area);
        out << '}';
        if (index + 1 < geometryState.faces.size()) {
            out << ',';
        }
        out << "\n";
    }
    out << "    ],\n    \"bodies\": [\n";
    for (std::size_t index = 0; index < geometryState.bodies.size(); ++index) {
        const auto& body = geometryState.bodies[index];
        out << "      {\"id\":" << body.id << ",\"face_ids\":";
        append_json_entity_id_array(out, body.face_ids);
        out << ",\"volume\":";
        append_json_number(out, body.volume);
        out << '}';
        if (index + 1 < geometryState.bodies.size()) {
            out << ',';
        }
        out << "\n";
    }
    out << "    ],\n    \"body_infos\": [\n";
    for (std::size_t index = 0; index < geometryState.body_infos.size(); ++index) {
        const auto& bodyInfo = geometryState.body_infos[index];
        out << "      {\"body_id\":" << bodyInfo.body_id << ",\"name\":";
        append_json_string(out, bodyInfo.name);
        out << ",\"material_name\":";
        append_json_string(out, bodyInfo.material_name);
        out << ",\"has_color\":" << (bodyInfo.has_color ? "true" : "false");
        if (bodyInfo.has_color) {
            out << ",\"color\":";
            append_json_scene_color(out, bodyInfo.color);
        }
        out << '}';
        if (index + 1 < geometryState.body_infos.size()) {
            out << ',';
        }
        out << "\n";
    }
    out << "    ],\n    \"face_outlines\": [\n";
    for (std::size_t index = 0; index < geometryState.face_outlines.size(); ++index) {
        const auto& faceOutline = geometryState.face_outlines[index];
        out << "      {\"face_id\":" << faceOutline.face_id << ",\"material_name\":";
        append_json_string(out, faceOutline.material_name);
        out << ",\"has_color\":" << (faceOutline.has_color ? "true" : "false");
        if (faceOutline.has_color) {
            out << ",\"color\":";
            append_json_scene_color(out, faceOutline.color);
        }
        out << ",\"points\":[";
        for (std::size_t pointIndex = 0; pointIndex < faceOutline.points.size(); ++pointIndex) {
            if (pointIndex > 0) {
                out << ',';
            }
            append_json_point3(out, faceOutline.points[pointIndex]);
        }
        out << "]}";
        if (index + 1 < geometryState.face_outlines.size()) {
            out << ',';
        }
        out << "\n";
    }
    out << "    ]\n  },\n";

    out << "  \"summary_text\": ";
    append_json_string(out, summaryText);
    out << "\n}\n";
    return out.str();
}

void refresh_output(HWND ownerWindow) {
    if (!g_outputEdit) {
        return;
    }

    try {
        const ViewportSelectionKind previousSelectionKind = g_geometryViewportState.selection_kind;
        const adaptivecad::geometry::EntityId previousSelectedId = g_geometryViewportState.selected_id;
        g_activeFitAnalysisState = build_active_fit_analysis_state();
        g_plotState = build_fit_plot_state(g_activeFitAnalysisState);
        g_geometryViewportState = build_geometry_viewport_state();
        if (geometry_selection_exists(previousSelectionKind, previousSelectedId)) {
            g_geometryViewportState.selection_kind = previousSelectionKind;
            g_geometryViewportState.selected_id = previousSelectedId;
        }
        g_lastOutput = build_summary_text(g_activeFitAnalysisState, g_geometryViewportState);
        SetWindowTextA(g_outputEdit, g_lastOutput.c_str());
        populate_model_tree();
        if (g_plotPanel) {
            InvalidateRect(g_plotPanel, nullptr, TRUE);
        }
        if (g_viewportPanel) {
            InvalidateRect(g_viewportPanel, nullptr, TRUE);
        }
        update_dataset_panel_controls();
        update_angular_branch_controls();
        update_body_edit_controls();
        set_status_text("Status: refreshed");
    } catch (const std::exception& ex) {
        const std::string message = std::string("Refresh failed: ") + ex.what();
        g_activeFitAnalysisState = {};
        set_plot_error_state(std::string("Plot refresh failed: ") + ex.what());
        set_geometry_error_state(std::string("Viewport refresh failed: ") + ex.what());
        populate_model_tree();
        update_dataset_panel_controls();
        update_angular_branch_controls();
        update_body_edit_controls();
        set_status_text("Status: refresh failed");
        MessageBoxA(ownerWindow, message.c_str(), "AdaptiveCAD UI", MB_ICONERROR);
    }
}

void open_spec_document(HWND ownerWindow) {
    const std::filesystem::path specPath = g_workspaceRoot / kSpecFileName;
    if (!path_exists(specPath)) {
        set_status_text("Status: spec file not found");
        MessageBoxA(ownerWindow, "Spec file was not found in workspace root.", "AdaptiveCAD UI", MB_ICONWARNING);
        return;
    }

    const HINSTANCE openResult = ShellExecuteA(
        ownerWindow,
        "open",
        specPath.string().c_str(),
        nullptr,
        g_workspaceRoot.string().c_str(),
        SW_SHOWNORMAL);

    if (reinterpret_cast<intptr_t>(openResult) <= 32) {
        set_status_text("Status: failed to open spec");
        MessageBoxA(ownerWindow, "Failed to open spec file.", "AdaptiveCAD UI", MB_ICONERROR);
        return;
    }

    set_status_text("Status: opened spec document");
}

void import_csv_measurements(HWND ownerWindow) {
    char filePathBuffer[MAX_PATH] = {};

    OPENFILENAMEA ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = ownerWindow;
    ofn.lpstrFile = filePathBuffer;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrFilter =
        "CSV files (*.csv)\0*.csv\0All files (*.*)\0*.*\0";
    ofn.nFilterIndex = 1;
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;
    ofn.lpstrTitle = "Import Measurement CSV";

    if (!GetOpenFileNameA(&ofn)) {
        return;
    }

    try {
        const std::filesystem::path csvPath(filePathBuffer);
        const ParsedCsvDataset parsed = parse_area_samples_csv(csvPath);

        g_importedAreaSamples = parsed.samples;
        g_importedCsvPath = csvPath;
        set_dataset_info_import_success(csvPath, parsed.samples.size(), parsed.rejected_rows, parsed.summary);
        update_dataset_panel_controls();

        set_status_text(
            std::string("Status: imported ") + std::to_string(g_importedAreaSamples.size()) +
            " CSV rows for nonlinear fit");
        refresh_output(ownerWindow);
    } catch (const std::exception& ex) {
        const std::string message = std::string("CSV import failed: ") + ex.what();
        set_dataset_info_import_failure(std::string("Last CSV import failed: ") + ex.what());
        update_dataset_panel_controls();
        set_status_text("Status: CSV import failed");
        MessageBoxA(ownerWindow, message.c_str(), "AdaptiveCAD UI", MB_ICONERROR);
    }
}

void clear_imported_dataset(HWND ownerWindow) {
    g_importedAreaSamples.clear();
    g_importedCsvPath.clear();
    set_dataset_info_to_synthetic();
    update_dataset_panel_controls();
    refresh_output(ownerWindow);
    set_status_text("Status: cleared imported CSV dataset");
}

void refit_current_dataset(HWND ownerWindow) {
    set_status_text("Status: refitting current dataset");
    refresh_output(ownerWindow);
    set_status_text("Status: refit completed");
}

bool ensure_report_state_ready(HWND ownerWindow, const char* emptyStatus, const char* emptyMessage) {
    if (g_lastOutput.empty() || !g_activeFitAnalysisState.ready || !g_plotState.ready || !g_geometryViewportState.ready) {
        refresh_output(ownerWindow);
    }

    if (g_lastOutput.empty() || !g_activeFitAnalysisState.ready || !g_plotState.ready || !g_geometryViewportState.ready) {
        set_status_text(emptyStatus);
        MessageBoxA(ownerWindow, emptyMessage, "AdaptiveCAD UI", MB_ICONWARNING);
        return false;
    }

    return true;
}

void write_report_bundle_files(const std::filesystem::path& textPath, const std::filesystem::path& jsonPath) {
    std::ofstream outFile(textPath, std::ios::binary);
    if (!outFile) {
        throw std::runtime_error("could not open snapshot output file");
    }

    outFile << g_lastOutput;
    outFile << "\r\n";
    outFile.flush();
    if (!outFile) {
        throw std::runtime_error("failed while writing snapshot file");
    }

    const std::string jsonReport =
        build_analysis_report_json(g_activeFitAnalysisState, g_geometryViewportState, g_plotState, g_lastOutput);

    std::ofstream jsonFile(jsonPath, std::ios::binary);
    if (!jsonFile) {
        throw std::runtime_error("could not open JSON report output file");
    }

    jsonFile << jsonReport;
    jsonFile.flush();
    if (!jsonFile) {
        throw std::runtime_error("failed while writing JSON report file");
    }
}

void save_session_bundle_to_path(HWND ownerWindow, const std::filesystem::path& requestedSessionPath) {
    if (!ensure_report_state_ready(
            ownerWindow,
            "Status: no data to save session",
            "No output is available to save into a session bundle.")) {
        return;
    }

    try {
        std::filesystem::path sessionPath = requestedSessionPath;
        if (sessionPath.extension().empty()) {
            sessionPath += ".ini";
        }

        std::error_code ec;
        if (!sessionPath.parent_path().empty()) {
            std::filesystem::create_directories(sessionPath.parent_path(), ec);
            if (ec) {
                throw std::runtime_error("could not create session directory");
            }
        }

        const std::filesystem::path basePath = sessionPath.parent_path() / sessionPath.stem();
        const std::filesystem::path textPath = basePath.string() + "_report.txt";
        const std::filesystem::path jsonPath = basePath.string() + "_report.json";

        adaptivecad::tool::SessionBundleData bundle;
        bundle.saved_at = make_timestamp("%Y-%m-%d %H:%M:%S");
        bundle.workspace = g_workspaceRoot;
        bundle.report_text_path = textPath;
        bundle.report_json_path = jsonPath;
        bundle.scene_document = g_sceneDocument;
        bundle.dataset.using_embedded_samples = g_datasetInfo.using_imported_csv;
        bundle.dataset.original_csv_path = g_importedCsvPath;
        bundle.dataset.valid_rows = g_datasetInfo.valid_rows;
        bundle.dataset.rejected_rows = g_datasetInfo.rejected_rows;
        bundle.dataset.summary = g_datasetInfo.summary;
        bundle.dataset.samples = g_importedAreaSamples;
        bundle.view_state.healing_policy = adaptivecad::geometry::BRepKernel::healing_policy_name(g_healingPolicy);
        bundle.view_state.projection = viewport_projection_name(g_geometryViewportViewState.projection);
        bundle.view_state.zoom = g_geometryViewportViewState.zoom;
        bundle.view_state.pan_x = g_geometryViewportViewState.pan_x;
        bundle.view_state.pan_y = g_geometryViewportViewState.pan_y;
        bundle.view_state.selection_kind = viewport_selection_kind_name(g_geometryViewportState.selection_kind);
        bundle.view_state.selected_id = g_geometryViewportState.selected_id;
        adaptivecad::tool::save_session_bundle_file(sessionPath, bundle);

        write_report_bundle_files(textPath, jsonPath);
        set_status_text(std::string("Status: saved session bundle to ") + sessionPath.string());
    } catch (const std::exception& ex) {
        const std::string message = std::string("Save session failed: ") + ex.what();
        set_status_text("Status: save session failed");
        MessageBoxA(ownerWindow, message.c_str(), "AdaptiveCAD UI", MB_ICONERROR);
    }
}

void load_session_bundle_from_path(HWND ownerWindow, const std::filesystem::path& sessionPath) {
    try {
        const adaptivecad::tool::SessionBundleData bundle = adaptivecad::tool::load_session_bundle_file(sessionPath);
        g_sceneDocument = bundle.scene_document;

        g_healingPolicy = parse_healing_policy_string(bundle.view_state.healing_policy);
        initialize_healing_policy_combo();

        if (!bundle.dataset.using_embedded_samples) {
            g_importedAreaSamples.clear();
            g_importedCsvPath.clear();
            set_dataset_info_to_synthetic();
        } else {
            g_importedAreaSamples = bundle.dataset.samples;
            g_importedCsvPath = bundle.dataset.original_csv_path.empty()
                ? std::filesystem::path("(session embedded dataset)")
                : bundle.dataset.original_csv_path;

            std::string summary = bundle.dataset.summary;
            if (summary.empty()) {
                summary = "Loaded " + std::to_string(g_importedAreaSamples.size()) + " embedded session samples.";
            }
            set_dataset_info_import_success(g_importedCsvPath, g_importedAreaSamples.size(), bundle.dataset.rejected_rows, summary);
        }

        g_geometryViewportViewState.projection =
            parse_viewport_projection_string(bundle.view_state.projection);
        g_geometryViewportViewState.zoom =
            std::clamp(bundle.view_state.zoom, 0.25, 8.0);
        g_geometryViewportViewState.pan_x = bundle.view_state.pan_x;
        g_geometryViewportViewState.pan_y = bundle.view_state.pan_y;
        g_geometryViewportViewState.panning = false;
        g_geometryViewportViewState.last_pan_point = POINT{0, 0};
        initialize_viewport_projection_combo();

        g_geometryViewportState.selection_kind =
            parse_viewport_selection_kind_string(bundle.view_state.selection_kind);
        g_geometryViewportState.selected_id = bundle.view_state.selected_id;

        refresh_output(ownerWindow);
        if (g_viewportPanel) {
            InvalidateRect(g_viewportPanel, nullptr, TRUE);
        }
        set_status_text(std::string("Status: loaded session from ") + sessionPath.string());
    } catch (const std::exception& ex) {
        const std::string message = std::string("Load session failed: ") + ex.what();
        set_status_text("Status: load session failed");
        MessageBoxA(ownerWindow, message.c_str(), "AdaptiveCAD UI", MB_ICONERROR);
    }
}

void save_session_bundle(HWND ownerWindow) {
    char filePathBuffer[MAX_PATH] = {};
    const std::filesystem::path sessionsDirectory = g_workspaceRoot / "sessions";
    std::error_code ec;
    std::filesystem::create_directories(sessionsDirectory, ec);
    const std::string initialDir = sessionsDirectory.string();
    const std::string defaultFileName = "adaptivecad_session_" + make_timestamp("%Y%m%d_%H%M%S") + ".ini";
    std::snprintf(filePathBuffer, sizeof(filePathBuffer), "%s", defaultFileName.c_str());

    OPENFILENAMEA ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = ownerWindow;
    ofn.lpstrFile = filePathBuffer;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrFilter =
        "AdaptiveCAD session (*.ini)\0*.ini\0All files (*.*)\0*.*\0";
    ofn.nFilterIndex = 1;
    ofn.lpstrDefExt = "ini";
    ofn.lpstrInitialDir = initialDir.c_str();
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT | OFN_HIDEREADONLY;
    ofn.lpstrTitle = "Save AdaptiveCAD Session";

    if (!GetSaveFileNameA(&ofn)) {
        return;
    }

    save_session_bundle_to_path(ownerWindow, std::filesystem::path(filePathBuffer));
}

void load_session_bundle(HWND ownerWindow) {
    char filePathBuffer[MAX_PATH] = {};
    const std::filesystem::path sessionsDirectory = g_workspaceRoot / "sessions";
    const std::string initialDir = sessionsDirectory.string();

    OPENFILENAMEA ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = ownerWindow;
    ofn.lpstrFile = filePathBuffer;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrFilter =
        "AdaptiveCAD session (*.ini)\0*.ini\0All files (*.*)\0*.*\0";
    ofn.nFilterIndex = 1;
    ofn.lpstrInitialDir = initialDir.c_str();
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;
    ofn.lpstrTitle = "Load AdaptiveCAD Session";

    if (!GetOpenFileNameA(&ofn)) {
        return;
    }

    load_session_bundle_from_path(ownerWindow, std::filesystem::path(filePathBuffer));
}

void export_snapshot(HWND ownerWindow) {
    if (!ensure_report_state_ready(ownerWindow, "Status: no data to export", "No output is available to export.")) {
        return;
    }

    try {
        const std::filesystem::path snapshotDirectory = g_workspaceRoot / "snapshots";
        std::error_code ec;
        std::filesystem::create_directories(snapshotDirectory, ec);

        if (ec) {
            throw std::runtime_error("could not create snapshots directory");
        }

        const std::string fileStem = "adaptivecad_snapshot_" + make_timestamp("%Y%m%d_%H%M%S");
        const std::filesystem::path snapshotPath = snapshotDirectory / (fileStem + ".txt");
        const std::filesystem::path jsonPath = snapshotDirectory / (fileStem + ".json");

        write_report_bundle_files(snapshotPath, jsonPath);

        const std::string statusText =
            std::string("Status: exported text snapshot and JSON report to ") + snapshotDirectory.string();
        set_status_text(statusText);
    } catch (const std::exception& ex) {
        const std::string message = std::string("Export failed: ") + ex.what();
        set_status_text("Status: export failed");
        MessageBoxA(ownerWindow, message.c_str(), "AdaptiveCAD UI", MB_ICONERROR);
    }
}

HMENU create_main_menu() {
    HMENU menuBar = CreateMenu();
    HMENU fileMenu = CreatePopupMenu();
    HMENU createMenu = CreatePopupMenu();
    if (!menuBar || !fileMenu || !createMenu) {
        return menuBar;
    }

    AppendMenuA(createMenu, MF_STRING, kCreateBoxButtonId, "Box");
    AppendMenuA(createMenu, MF_STRING, kCreateWedgeButtonId, "Wedge");
    AppendMenuA(createMenu, MF_STRING, kCreatePlaneButtonId, "Plane");
    AppendMenuA(createMenu, MF_STRING, kCreateTorusButtonId, "Torus");
    AppendMenuA(createMenu, MF_SEPARATOR, 0, nullptr);
    AppendMenuA(createMenu, MF_STRING, kCreateScaffoldButtonId, "Metamaterial Scaffold");

    AppendMenuA(fileMenu, MF_STRING, kImportGeometryButtonId, "Import Geometry...");
    AppendMenuA(fileMenu, MF_STRING, kExportGeometryButtonId, "Export Geometry...");
    AppendMenuA(fileMenu, MF_SEPARATOR, 0, nullptr);
    AppendMenuA(fileMenu, MF_STRING, kSaveSessionButtonId, "Save Session...");
    AppendMenuA(fileMenu, MF_STRING, kLoadSessionButtonId, "Load Session...");
    AppendMenuA(fileMenu, MF_STRING, kExportButtonId, "Export Snapshot...");
    AppendMenuA(fileMenu, MF_SEPARATOR, 0, nullptr);
    AppendMenuA(fileMenu, MF_STRING, kOpenSpecButtonId, "Open Spec");
    AppendMenuA(fileMenu, MF_SEPARATOR, 0, nullptr);
    AppendMenuA(fileMenu, MF_STRING, kCloseButtonId, "Exit");
    AppendMenuA(menuBar, MF_POPUP, reinterpret_cast<UINT_PTR>(fileMenu), "File");
    AppendMenuA(menuBar, MF_POPUP, reinterpret_cast<UINT_PTR>(createMenu), "Create");

    return menuBar;
}

LRESULT CALLBACK window_proc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_CREATE: {
        CreateWindowExA(
            0,
            "BUTTON",
            "Refresh",
            WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
            12,
            12,
            80,
            30,
            hwnd,
            reinterpret_cast<HMENU>(kRefreshButtonId),
            GetModuleHandleA(nullptr),
            nullptr);

        CreateWindowExA(
            0,
            "BUTTON",
            "Open Spec",
            WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
            100,
            12,
            90,
            30,
            hwnd,
            reinterpret_cast<HMENU>(kOpenSpecButtonId),
            GetModuleHandleA(nullptr),
            nullptr);

        CreateWindowExA(
            0,
            "BUTTON",
            "Import CSV",
            WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
            198,
            12,
            96,
            30,
            hwnd,
            reinterpret_cast<HMENU>(kImportCsvButtonId),
            GetModuleHandleA(nullptr),
            nullptr);

        CreateWindowExA(
            0,
            "BUTTON",
            "Import Geometry",
            WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
            302,
            12,
            120,
            30,
            hwnd,
            reinterpret_cast<HMENU>(kImportGeometryButtonId),
            GetModuleHandleA(nullptr),
            nullptr);

        CreateWindowExA(
            0,
            "BUTTON",
            "Save Session",
            WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
            430,
            12,
            104,
            30,
            hwnd,
            reinterpret_cast<HMENU>(kSaveSessionButtonId),
            GetModuleHandleA(nullptr),
            nullptr);

        CreateWindowExA(
            0,
            "BUTTON",
            "Load Session",
            WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
            542,
            12,
            104,
            30,
            hwnd,
            reinterpret_cast<HMENU>(kLoadSessionButtonId),
            GetModuleHandleA(nullptr),
            nullptr);

        CreateWindowExA(
            0,
            "BUTTON",
            "Export Snapshot",
            WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
            654,
            12,
            126,
            30,
            hwnd,
            reinterpret_cast<HMENU>(kExportButtonId),
            GetModuleHandleA(nullptr),
            nullptr);

        CreateWindowExA(
            0,
            "BUTTON",
            "Close",
            WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
            788,
            12,
            82,
            30,
            hwnd,
            reinterpret_cast<HMENU>(kCloseButtonId),
            GetModuleHandleA(nullptr),
            nullptr);

        CreateWindowExA(
            0,
            "STATIC",
            "Healing Policy:",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            884,
            18,
            100,
            20,
            hwnd,
            reinterpret_cast<HMENU>(kHealingPolicyLabelId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_healingPolicyCombo = CreateWindowExA(
            0,
            "COMBOBOX",
            "",
            WS_CHILD | WS_VISIBLE | WS_TABSTOP | CBS_DROPDOWNLIST | WS_VSCROLL,
            880,
            12,
            180,
            200,
            hwnd,
            reinterpret_cast<HMENU>(kHealingPolicyComboId),
            GetModuleHandleA(nullptr),
            nullptr);
        initialize_healing_policy_combo();

        g_datasetGroup = CreateWindowExA(
            0,
            "BUTTON",
            "Measurement Dataset",
            WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
            12,
            52,
            760,
            92,
            hwnd,
            reinterpret_cast<HMENU>(kDatasetGroupId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_datasetPathLabel = CreateWindowExA(
            0,
            "STATIC",
            "Active CSV path: (none; synthetic dataset active)",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            24,
            72,
            560,
            18,
            hwnd,
            reinterpret_cast<HMENU>(kDatasetPathLabelId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_datasetValidLabel = CreateWindowExA(
            0,
            "STATIC",
            "Valid rows: 0",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            24,
            92,
            180,
            18,
            hwnd,
            reinterpret_cast<HMENU>(kDatasetValidLabelId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_datasetRejectedLabel = CreateWindowExA(
            0,
            "STATIC",
            "Rejected rows: 0",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            210,
            92,
            220,
            18,
            hwnd,
            reinterpret_cast<HMENU>(kDatasetRejectedLabelId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_datasetSummaryLabel = CreateWindowExA(
            0,
            "STATIC",
            "Summary: No CSV imported.",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            24,
            112,
            560,
            18,
            hwnd,
            reinterpret_cast<HMENU>(kDatasetSummaryLabelId),
            GetModuleHandleA(nullptr),
            nullptr);

        CreateWindowExA(
            0,
            "BUTTON",
            "Refit",
            WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
            620,
            78,
            130,
            26,
            hwnd,
            reinterpret_cast<HMENU>(kRefitButtonId),
            GetModuleHandleA(nullptr),
            nullptr);

        CreateWindowExA(
            0,
            "BUTTON",
            "Clear Dataset",
            WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
            620,
            108,
            130,
            26,
            hwnd,
            reinterpret_cast<HMENU>(kClearDatasetButtonId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_angularGroup = CreateWindowExA(
            0,
            "BUTTON",
            "Angular Branch",
            WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
            12,
            152,
            760,
            86,
            hwnd,
            reinterpret_cast<HMENU>(kAngularGroupId),
            GetModuleHandleA(nullptr),
            nullptr);

        CreateWindowExA(
            0,
            "STATIC",
            "lambda0:",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            24,
            174,
            56,
            18,
            hwnd,
            reinterpret_cast<HMENU>(kAngularLambdaLabelId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_angularLambdaEdit = CreateWindowExA(
            WS_EX_CLIENTEDGE,
            "EDIT",
            "",
            WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_AUTOHSCROLL,
            84,
            170,
            84,
            22,
            hwnd,
            reinterpret_cast<HMENU>(kAngularLambdaEditId),
            GetModuleHandleA(nullptr),
            nullptr);

        CreateWindowExA(
            0,
            "STATIC",
            "Cos coeffs:",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            184,
            174,
            80,
            18,
            hwnd,
            reinterpret_cast<HMENU>(kAngularCosLabelId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_angularCosEdit = CreateWindowExA(
            WS_EX_CLIENTEDGE,
            "EDIT",
            "",
            WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_AUTOHSCROLL,
            266,
            170,
            336,
            22,
            hwnd,
            reinterpret_cast<HMENU>(kAngularCosEditId),
            GetModuleHandleA(nullptr),
            nullptr);

        CreateWindowExA(
            0,
            "STATIC",
            "Sine coeffs:",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            24,
            202,
            80,
            18,
            hwnd,
            reinterpret_cast<HMENU>(kAngularSinLabelId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_angularSinEdit = CreateWindowExA(
            WS_EX_CLIENTEDGE,
            "EDIT",
            "",
            WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_AUTOHSCROLL,
            108,
            198,
            494,
            22,
            hwnd,
            reinterpret_cast<HMENU>(kAngularSinEditId),
            GetModuleHandleA(nullptr),
            nullptr);

        CreateWindowExA(
            0,
            "BUTTON",
            "Apply",
            WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
            620,
            168,
            130,
            26,
            hwnd,
            reinterpret_cast<HMENU>(kAngularApplyButtonId),
            GetModuleHandleA(nullptr),
            nullptr);

        CreateWindowExA(
            0,
            "BUTTON",
            "Revert",
            WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
            620,
            198,
            130,
            26,
            hwnd,
            reinterpret_cast<HMENU>(kAngularRevertButtonId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_primitiveGroup = CreateWindowExA(
            0,
            "BUTTON",
            "Primitive",
            WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
            12,
            246,
            760,
            92,
            hwnd,
            reinterpret_cast<HMENU>(kPrimitiveGroupId),
            GetModuleHandleA(nullptr),
            nullptr);

        CreateWindowExA(
            0,
            "STATIC",
            "Type:",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            24,
            270,
            42,
            18,
            hwnd,
            reinterpret_cast<HMENU>(kPrimitiveTypeLabelId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_primitiveTypeCombo = CreateWindowExA(
            0,
            "COMBOBOX",
            "",
            WS_CHILD | WS_VISIBLE | WS_TABSTOP | CBS_DROPDOWNLIST | WS_VSCROLL,
            68,
            264,
            104,
            180,
            hwnd,
            reinterpret_cast<HMENU>(kPrimitiveTypeComboId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_primitiveAppendCheck = CreateWindowExA(
            0,
            "BUTTON",
            "Add to scene",
            WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_AUTOCHECKBOX,
            24,
            302,
            120,
            22,
            hwnd,
            reinterpret_cast<HMENU>(kPrimitiveAppendCheckId),
            GetModuleHandleA(nullptr),
            nullptr);

        auto createPrimitiveLabel = [&](int id, int x, int y) {
            CreateWindowExA(
                0,
                "STATIC",
                "",
                WS_CHILD | WS_VISIBLE | SS_LEFT,
                x,
                y,
                74,
                18,
                hwnd,
                reinterpret_cast<HMENU>(id),
                GetModuleHandleA(nullptr),
                nullptr);
        };

        auto createPrimitiveEdit = [&](int id, int x, int y) -> HWND {
            return CreateWindowExA(
                WS_EX_CLIENTEDGE,
                "EDIT",
                "",
                WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_AUTOHSCROLL,
                x,
                y,
                76,
                22,
                hwnd,
                reinterpret_cast<HMENU>(id),
                GetModuleHandleA(nullptr),
                nullptr);
        };

        createPrimitiveLabel(kPrimitiveParam1LabelId, 184, 270);
        g_primitiveParam1Edit = createPrimitiveEdit(kPrimitiveParam1EditId, 250, 266);
        createPrimitiveLabel(kPrimitiveParam2LabelId, 340, 270);
        g_primitiveParam2Edit = createPrimitiveEdit(kPrimitiveParam2EditId, 406, 266);
        createPrimitiveLabel(kPrimitiveParam3LabelId, 496, 270);
        g_primitiveParam3Edit = createPrimitiveEdit(kPrimitiveParam3EditId, 566, 266);
        createPrimitiveLabel(kPrimitiveParam4LabelId, 184, 302);
        g_primitiveParam4Edit = createPrimitiveEdit(kPrimitiveParam4EditId, 250, 298);
        createPrimitiveLabel(kPrimitiveParam5LabelId, 340, 302);
        g_primitiveParam5Edit = createPrimitiveEdit(kPrimitiveParam5EditId, 406, 298);

        CreateWindowExA(
            0,
            "BUTTON",
            "Create",
            WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
            620,
            282,
            130,
            26,
            hwnd,
            reinterpret_cast<HMENU>(kCreatePrimitiveButtonId),
            GetModuleHandleA(nullptr),
            nullptr);
        initialize_primitive_type_combo();

        g_bodyEditGroup = CreateWindowExA(
            0,
            "BUTTON",
            "Selected Body",
            WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
            12,
            346,
            760,
            94,
            hwnd,
            reinterpret_cast<HMENU>(kBodyEditGroupId),
            GetModuleHandleA(nullptr),
            nullptr);

        auto createBodyLabel = [&](int id, const char* text, int x, int y, int width) {
            CreateWindowExA(
                0,
                "STATIC",
                text,
                WS_CHILD | WS_VISIBLE | SS_LEFT,
                x,
                y,
                width,
                18,
                hwnd,
                reinterpret_cast<HMENU>(id),
                GetModuleHandleA(nullptr),
                nullptr);
        };

        auto createBodyEdit = [&](int id, const char* text, int x, int y, int width) -> HWND {
            return CreateWindowExA(
                WS_EX_CLIENTEDGE,
                "EDIT",
                text,
                WS_CHILD | WS_VISIBLE | WS_TABSTOP | ES_AUTOHSCROLL,
                x,
                y,
                width,
                22,
                hwnd,
                reinterpret_cast<HMENU>(id),
                GetModuleHandleA(nullptr),
                nullptr);
        };

        createBodyLabel(kBodyNameLabelId, "Name:", 24, 370, 50);
        g_bodyNameEdit = createBodyEdit(kBodyNameEditId, "", 76, 366, 164);
        CreateWindowExA(
            0,
            "BUTTON",
            "Rename",
            WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
            250,
            365,
            76,
            26,
            hwnd,
            reinterpret_cast<HMENU>(kRenameBodyButtonId),
            GetModuleHandleA(nullptr),
            nullptr);

        createBodyLabel(kBodyDxLabelId, "X:", 340, 370, 18);
        g_bodyDxEdit = createBodyEdit(kBodyDxEditId, "0", 362, 366, 58);
        createBodyLabel(kBodyDyLabelId, "Y:", 426, 370, 18);
        g_bodyDyEdit = createBodyEdit(kBodyDyEditId, "0", 448, 366, 58);
        createBodyLabel(kBodyDzLabelId, "Z:", 512, 370, 18);
        g_bodyDzEdit = createBodyEdit(kBodyDzEditId, "0", 534, 366, 58);

        createBodyLabel(kBodyScaleLabelId, "Scale:", 24, 404, 48);
        g_bodyScaleEdit = createBodyEdit(kBodyScaleEditId, "1", 76, 400, 58);
        createBodyLabel(kBodyRotateLabelId, "Rot Z:", 146, 404, 54);
        g_bodyRotateEdit = createBodyEdit(kBodyRotateEditId, "0", 204, 400, 58);

        CreateWindowExA(
            0,
            "BUTTON",
            "Apply Transform",
            WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
            278,
            398,
            118,
            26,
            hwnd,
            reinterpret_cast<HMENU>(kApplyBodyTransformButtonId),
            GetModuleHandleA(nullptr),
            nullptr);
        CreateWindowExA(
            0,
            "BUTTON",
            "Duplicate",
            WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
            406,
            398,
            88,
            26,
            hwnd,
            reinterpret_cast<HMENU>(kDuplicateBodyButtonId),
            GetModuleHandleA(nullptr),
            nullptr);
        CreateWindowExA(
            0,
            "BUTTON",
            "Delete",
            WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
            504,
            398,
            76,
            26,
            hwnd,
            reinterpret_cast<HMENU>(kDeleteBodyButtonId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_plotGroup = CreateWindowExA(
            0,
            "BUTTON",
            "Fit Plot",
            WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
            12,
            246,
            760,
            300,
            hwnd,
            reinterpret_cast<HMENU>(kPlotGroupId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_plotPanel = CreateWindowExA(
            WS_EX_CLIENTEDGE,
            kPlotWindowClassName,
            "",
            WS_CHILD | WS_VISIBLE,
            24,
            268,
            736,
            266,
            hwnd,
            reinterpret_cast<HMENU>(kPlotPanelId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_modelTreeGroup = CreateWindowExA(
            0,
            "BUTTON",
            "Model Tree",
            WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
            560,
            460,
            212,
            120,
            hwnd,
            reinterpret_cast<HMENU>(kModelTreeGroupId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_modelTreeView = CreateWindowExA(
            WS_EX_CLIENTEDGE,
            WC_TREEVIEWA,
            "",
            WS_CHILD | WS_VISIBLE | WS_TABSTOP | TVS_HASBUTTONS | TVS_HASLINES | TVS_LINESATROOT | TVS_SHOWSELALWAYS,
            572,
            482,
            188,
            86,
            hwnd,
            reinterpret_cast<HMENU>(kModelTreeViewId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_viewportGroup = CreateWindowExA(
            0,
            "BUTTON",
            "Geometry Viewport",
            WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
            560,
            560,
            212,
            200,
            hwnd,
            reinterpret_cast<HMENU>(kViewportGroupId),
            GetModuleHandleA(nullptr),
            nullptr);

        CreateWindowExA(
            0,
            "STATIC",
            "Projection:",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            572,
            578,
            70,
            20,
            hwnd,
            reinterpret_cast<HMENU>(kViewportProjectionLabelId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_viewportProjectionCombo = CreateWindowExA(
            0,
            "COMBOBOX",
            "",
            WS_CHILD | WS_VISIBLE | WS_TABSTOP | CBS_DROPDOWNLIST | WS_VSCROLL,
            644,
            572,
            120,
            200,
            hwnd,
            reinterpret_cast<HMENU>(kViewportProjectionComboId),
            GetModuleHandleA(nullptr),
            nullptr);
        initialize_viewport_projection_combo();

        CreateWindowExA(
            0,
            "BUTTON",
            "Reset View",
            WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_PUSHBUTTON,
            664,
            572,
            96,
            26,
            hwnd,
            reinterpret_cast<HMENU>(kViewportResetButtonId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_viewportPanel = CreateWindowExA(
            WS_EX_CLIENTEDGE,
            kViewportWindowClassName,
            "",
            WS_CHILD | WS_VISIBLE,
            572,
            582,
            188,
            166,
            hwnd,
            reinterpret_cast<HMENU>(kViewportPanelId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_outputEdit = CreateWindowExA(
            WS_EX_CLIENTEDGE,
            "EDIT",
            "",
            WS_CHILD | WS_VISIBLE | WS_VSCROLL | ES_LEFT | ES_MULTILINE | ES_AUTOVSCROLL | ES_READONLY,
            12,
            460,
            540,
            220,
            hwnd,
            reinterpret_cast<HMENU>(kOutputEditId),
            GetModuleHandleA(nullptr),
            nullptr);

        g_statusLabel = CreateWindowExA(
            0,
            "STATIC",
            "Status: ready",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            12,
            556,
            760,
            20,
            hwnd,
            reinterpret_cast<HMENU>(kStatusLabelId),
            GetModuleHandleA(nullptr),
            nullptr);

        update_dataset_panel_controls();

        refresh_output(hwnd);
        return 0;
    }
    case WM_SIZE: {
        const int width = LOWORD(lParam);
        const int height = HIWORD(lParam);
        const int datasetTop = 52;
        const int datasetHeight = 92;
        const int angularTop = datasetTop + datasetHeight + 8;
        const int angularHeight = 86;
        const int primitiveTop = angularTop + angularHeight + 8;
        const int primitiveHeight = 92;
        const int bodyEditTop = primitiveTop + primitiveHeight + 8;
        const int bodyEditHeight = 94;
        const int plotTop = bodyEditTop + bodyEditHeight + 8;
        const int plotHeight = std::clamp(((height - 438) / 2) + 70, 180, 320);
        const int outputTop = plotTop + plotHeight + 8;
        int statusTop = std::max(outputTop + 20, height - 32);
        const int rightButtonX = std::max(120, width - 150);
        const int textWidth = std::max(120, width - 220);
        const bool splitBottom = width >= 980;
        const int bottomGap = 8;
        const int viewportHeaderHeight = 50;
        const int bottomAvailableHeight = std::max(180, statusTop - outputTop - 6);
        int outputHeight = bottomAvailableHeight;
        int treeTop = outputTop;
        int treeHeight = 0;
        int viewportTop = outputTop;
        int viewportHeight = bottomAvailableHeight;
        int stackedBottomTop = outputTop;
        int stackedBottomHeight = 0;

        if (!splitBottom) {
            stackedBottomHeight = std::clamp((bottomAvailableHeight - bottomGap) * 48 / 100, 170, 250);
            outputHeight = std::max(120, bottomAvailableHeight - stackedBottomHeight - bottomGap);
            stackedBottomTop = outputTop + outputHeight + bottomGap;
            statusTop = std::max(
                stackedBottomTop + stackedBottomHeight + 8,
                std::min(height - 32, stackedBottomTop + stackedBottomHeight + 8));
            if (statusTop > height - 32) {
                statusTop = height - 32;
                stackedBottomHeight = std::max(140, statusTop - stackedBottomTop - 8);
            }

            treeTop = stackedBottomTop;
            treeHeight = stackedBottomHeight;
            viewportTop = stackedBottomTop;
            viewportHeight = stackedBottomHeight;
        } else {
            treeTop = outputTop;
            treeHeight = std::clamp((bottomAvailableHeight - bottomGap) * 40 / 100, 96, 140);
            viewportTop = treeTop + treeHeight + bottomGap;
            viewportHeight = std::max(100, bottomAvailableHeight - treeHeight - bottomGap);
        }

        HWND policyLabel = GetDlgItem(hwnd, kHealingPolicyLabelId);
        if (policyLabel) {
            MoveWindow(policyLabel, std::max(12, width - 320), 18, 100, 20, TRUE);
        }
        if (g_healingPolicyCombo) {
            MoveWindow(g_healingPolicyCombo, std::max(120, width - 210), 12, 180, 240, TRUE);
        }

        if (g_datasetGroup) {
            MoveWindow(g_datasetGroup, 12, datasetTop, std::max(120, width - 24), datasetHeight, TRUE);
        }
        if (g_datasetPathLabel) {
            MoveWindow(g_datasetPathLabel, 24, datasetTop + 20, textWidth, 18, TRUE);
        }
        if (g_datasetValidLabel) {
            MoveWindow(g_datasetValidLabel, 24, datasetTop + 40, 170, 18, TRUE);
        }
        if (g_datasetRejectedLabel) {
            MoveWindow(g_datasetRejectedLabel, 200, datasetTop + 40, 220, 18, TRUE);
        }
        if (g_datasetSummaryLabel) {
            MoveWindow(g_datasetSummaryLabel, 24, datasetTop + 60, textWidth, 18, TRUE);
        }

        if (g_plotGroup) {
            MoveWindow(g_plotGroup, 12, plotTop, std::max(120, width - 24), plotHeight, TRUE);
        }
        if (g_plotPanel) {
            MoveWindow(g_plotPanel, 24, plotTop + 22, std::max(120, width - 48), std::max(120, plotHeight - 34), TRUE);
            InvalidateRect(g_plotPanel, nullptr, TRUE);
        }

        HWND refitButton = GetDlgItem(hwnd, kRefitButtonId);
        if (refitButton) {
            MoveWindow(refitButton, rightButtonX, datasetTop + 26, 130, 26, TRUE);
        }
        HWND clearButton = GetDlgItem(hwnd, kClearDatasetButtonId);
        if (clearButton) {
            MoveWindow(clearButton, rightButtonX, datasetTop + 56, 130, 26, TRUE);
        }

        if (g_angularGroup) {
            MoveWindow(g_angularGroup, 12, angularTop, std::max(120, width - 24), angularHeight, TRUE);
        }
        HWND angularLambdaLabel = GetDlgItem(hwnd, kAngularLambdaLabelId);
        if (angularLambdaLabel) {
            MoveWindow(angularLambdaLabel, 24, angularTop + 22, 56, 18, TRUE);
        }
        if (g_angularLambdaEdit) {
            MoveWindow(g_angularLambdaEdit, 84, angularTop + 18, 84, 22, TRUE);
        }
        HWND angularCosLabel = GetDlgItem(hwnd, kAngularCosLabelId);
        if (angularCosLabel) {
            MoveWindow(angularCosLabel, 184, angularTop + 22, 80, 18, TRUE);
        }
        if (g_angularCosEdit) {
            MoveWindow(g_angularCosEdit, 266, angularTop + 18, std::max(120, rightButtonX - 278), 22, TRUE);
        }
        HWND angularSinLabel = GetDlgItem(hwnd, kAngularSinLabelId);
        if (angularSinLabel) {
            MoveWindow(angularSinLabel, 24, angularTop + 50, 80, 18, TRUE);
        }
        if (g_angularSinEdit) {
            MoveWindow(g_angularSinEdit, 108, angularTop + 46, std::max(120, rightButtonX - 120), 22, TRUE);
        }
        HWND angularApplyButton = GetDlgItem(hwnd, kAngularApplyButtonId);
        if (angularApplyButton) {
            MoveWindow(angularApplyButton, rightButtonX, angularTop + 16, 130, 26, TRUE);
        }
        HWND angularRevertButton = GetDlgItem(hwnd, kAngularRevertButtonId);
        if (angularRevertButton) {
            MoveWindow(angularRevertButton, rightButtonX, angularTop + 46, 130, 26, TRUE);
        }

        if (g_primitiveGroup) {
            MoveWindow(g_primitiveGroup, 12, primitiveTop, std::max(120, width - 24), primitiveHeight, TRUE);
        }
        HWND primitiveTypeLabel = GetDlgItem(hwnd, kPrimitiveTypeLabelId);
        if (primitiveTypeLabel) {
            MoveWindow(primitiveTypeLabel, 24, primitiveTop + 24, 42, 18, TRUE);
        }
        if (g_primitiveTypeCombo) {
            MoveWindow(g_primitiveTypeCombo, 68, primitiveTop + 18, 104, 180, TRUE);
        }
        if (g_primitiveAppendCheck) {
            MoveWindow(g_primitiveAppendCheck, 24, primitiveTop + 52, 124, 22, TRUE);
        }

        const int primitiveEditWidth = 76;
        const int primitiveLabelWidth = 74;
        const int param1LabelX = 184;
        const int param1EditX = 250;
        const int param2LabelX = 340;
        const int param2EditX = 406;
        const int param3LabelX = 496;
        const int param3EditX = 566;
        const int labelRow1Y = primitiveTop + 24;
        const int editRow1Y = primitiveTop + 20;
        const int labelRow2Y = primitiveTop + 56;
        const int editRow2Y = primitiveTop + 52;

        auto movePrimitiveLabel = [&](int id, int x, int y) {
            HWND label = GetDlgItem(hwnd, id);
            if (label) {
                MoveWindow(label, x, y, primitiveLabelWidth, 18, TRUE);
            }
        };

        movePrimitiveLabel(kPrimitiveParam1LabelId, param1LabelX, labelRow1Y);
        if (g_primitiveParam1Edit) {
            MoveWindow(g_primitiveParam1Edit, param1EditX, editRow1Y, primitiveEditWidth, 22, TRUE);
        }
        movePrimitiveLabel(kPrimitiveParam2LabelId, param2LabelX, labelRow1Y);
        if (g_primitiveParam2Edit) {
            MoveWindow(g_primitiveParam2Edit, param2EditX, editRow1Y, primitiveEditWidth, 22, TRUE);
        }
        movePrimitiveLabel(kPrimitiveParam3LabelId, param3LabelX, labelRow1Y);
        if (g_primitiveParam3Edit) {
            MoveWindow(g_primitiveParam3Edit, param3EditX, editRow1Y, primitiveEditWidth, 22, TRUE);
        }
        movePrimitiveLabel(kPrimitiveParam4LabelId, param1LabelX, labelRow2Y);
        if (g_primitiveParam4Edit) {
            MoveWindow(g_primitiveParam4Edit, param1EditX, editRow2Y, primitiveEditWidth, 22, TRUE);
        }
        movePrimitiveLabel(kPrimitiveParam5LabelId, param2LabelX, labelRow2Y);
        if (g_primitiveParam5Edit) {
            MoveWindow(g_primitiveParam5Edit, param2EditX, editRow2Y, primitiveEditWidth, 22, TRUE);
        }
        HWND primitiveCreateButton = GetDlgItem(hwnd, kCreatePrimitiveButtonId);
        if (primitiveCreateButton) {
            MoveWindow(primitiveCreateButton, rightButtonX, primitiveTop + 32, 130, 26, TRUE);
        }

        if (g_bodyEditGroup) {
            MoveWindow(g_bodyEditGroup, 12, bodyEditTop, std::max(120, width - 24), bodyEditHeight, TRUE);
        }
        HWND bodyNameLabel = GetDlgItem(hwnd, kBodyNameLabelId);
        if (bodyNameLabel) {
            MoveWindow(bodyNameLabel, 24, bodyEditTop + 24, 50, 18, TRUE);
        }
        if (g_bodyNameEdit) {
            MoveWindow(g_bodyNameEdit, 76, bodyEditTop + 20, 164, 22, TRUE);
        }
        HWND renameButton = GetDlgItem(hwnd, kRenameBodyButtonId);
        if (renameButton) {
            MoveWindow(renameButton, 250, bodyEditTop + 19, 76, 26, TRUE);
        }

        auto moveBodyLabel = [&](int id, int x, int y, int w) {
            HWND label = GetDlgItem(hwnd, id);
            if (label) {
                MoveWindow(label, x, y, w, 18, TRUE);
            }
        };

        moveBodyLabel(kBodyDxLabelId, 340, bodyEditTop + 24, 18);
        if (g_bodyDxEdit) {
            MoveWindow(g_bodyDxEdit, 362, bodyEditTop + 20, 58, 22, TRUE);
        }
        moveBodyLabel(kBodyDyLabelId, 426, bodyEditTop + 24, 18);
        if (g_bodyDyEdit) {
            MoveWindow(g_bodyDyEdit, 448, bodyEditTop + 20, 58, 22, TRUE);
        }
        moveBodyLabel(kBodyDzLabelId, 512, bodyEditTop + 24, 18);
        if (g_bodyDzEdit) {
            MoveWindow(g_bodyDzEdit, 534, bodyEditTop + 20, 58, 22, TRUE);
        }
        moveBodyLabel(kBodyScaleLabelId, 24, bodyEditTop + 58, 48);
        if (g_bodyScaleEdit) {
            MoveWindow(g_bodyScaleEdit, 76, bodyEditTop + 54, 58, 22, TRUE);
        }
        moveBodyLabel(kBodyRotateLabelId, 146, bodyEditTop + 58, 54);
        if (g_bodyRotateEdit) {
            MoveWindow(g_bodyRotateEdit, 204, bodyEditTop + 54, 58, 22, TRUE);
        }
        HWND transformButton = GetDlgItem(hwnd, kApplyBodyTransformButtonId);
        if (transformButton) {
            MoveWindow(transformButton, 278, bodyEditTop + 52, 118, 26, TRUE);
        }
        HWND duplicateButton = GetDlgItem(hwnd, kDuplicateBodyButtonId);
        if (duplicateButton) {
            MoveWindow(duplicateButton, 406, bodyEditTop + 52, 88, 26, TRUE);
        }
        HWND deleteButton = GetDlgItem(hwnd, kDeleteBodyButtonId);
        if (deleteButton) {
            MoveWindow(deleteButton, 504, bodyEditTop + 52, 76, 26, TRUE);
        }

        if (g_outputEdit) {
            if (splitBottom) {
                const int sidePaneWidth = std::max(240, (width - 36) * 38 / 100);
                const int outputWidth = std::max(260, width - 24 - sidePaneWidth - 8);
                MoveWindow(g_outputEdit, 12, outputTop, outputWidth, outputHeight, TRUE);
            } else {
                MoveWindow(g_outputEdit, 12, outputTop, std::max(100, width - 24), outputHeight, TRUE);
            }
        }
        if (g_modelTreeGroup && g_modelTreeView) {
            if (splitBottom) {
                const int sidePaneWidth = std::max(240, (width - 36) * 38 / 100);
                const int sidePaneX = width - 12 - sidePaneWidth;
                MoveWindow(g_modelTreeGroup, sidePaneX, treeTop, sidePaneWidth, treeHeight, TRUE);
                MoveWindow(
                    g_modelTreeView,
                    sidePaneX + 12,
                    treeTop + 22,
                    sidePaneWidth - 24,
                    std::max(72, treeHeight - 34),
                    TRUE);
            } else {
                const int treeWidth = std::max(220, (width - 36) * 36 / 100);
                MoveWindow(g_modelTreeGroup, 12, treeTop, treeWidth, treeHeight, TRUE);
                MoveWindow(g_modelTreeView, 24, treeTop + 22, treeWidth - 24, std::max(72, treeHeight - 34), TRUE);
            }
        }
        if (g_viewportGroup && g_viewportPanel) {
            HWND projectionLabel = GetDlgItem(hwnd, kViewportProjectionLabelId);
            HWND resetButton = GetDlgItem(hwnd, kViewportResetButtonId);
            if (splitBottom) {
                const int viewportWidth = std::max(240, (width - 36) * 38 / 100);
                const int viewportX = width - 12 - viewportWidth;
                MoveWindow(g_viewportGroup, viewportX, viewportTop, viewportWidth, viewportHeight, TRUE);
                if (projectionLabel) {
                    MoveWindow(projectionLabel, viewportX + 14, viewportTop + 20, 72, 18, TRUE);
                }
                if (g_viewportProjectionCombo) {
                    MoveWindow(g_viewportProjectionCombo, viewportX + 88, viewportTop + 16, 120, 220, TRUE);
                }
                if (resetButton) {
                    MoveWindow(resetButton, viewportX + viewportWidth - 108, viewportTop + 16, 92, 26, TRUE);
                }
                MoveWindow(
                    g_viewportPanel,
                    viewportX + 12,
                    viewportTop + viewportHeaderHeight,
                    viewportWidth - 24,
                    std::max(100, viewportHeight - viewportHeaderHeight - 12),
                    TRUE);
            } else {
                const int treeWidth = std::max(220, (width - 36) * 36 / 100);
                const int viewportX = 12 + treeWidth + 8;
                const int viewportWidth = std::max(220, width - 24 - treeWidth - 8);
                MoveWindow(g_viewportGroup, viewportX, viewportTop, viewportWidth, viewportHeight, TRUE);
                if (projectionLabel) {
                    MoveWindow(projectionLabel, viewportX + 14, viewportTop + 20, 72, 18, TRUE);
                }
                if (g_viewportProjectionCombo) {
                    MoveWindow(g_viewportProjectionCombo, viewportX + 88, viewportTop + 16, 120, 220, TRUE);
                }
                if (resetButton) {
                    MoveWindow(resetButton, viewportX + viewportWidth - 108, viewportTop + 16, 92, 26, TRUE);
                }
                MoveWindow(
                    g_viewportPanel,
                    viewportX + 12,
                    viewportTop + viewportHeaderHeight,
                    viewportWidth - 24,
                    std::max(100, viewportHeight - viewportHeaderHeight - 12),
                    TRUE);
            }
            InvalidateRect(g_viewportPanel, nullptr, TRUE);
        }
        if (g_statusLabel) {
            MoveWindow(g_statusLabel, 12, statusTop, std::max(120, width - 24), 20, TRUE);
        }
        return 0;
    }
    case WM_NOTIFY: {
        const auto* header = reinterpret_cast<const NMHDR*>(lParam);
        if (!header) {
            break;
        }

        if (header->idFrom == kModelTreeViewId && header->code == TVN_SELCHANGEDA) {
            if (g_treeSelectionSyncInProgress) {
                return 0;
            }

            const auto* treeView = reinterpret_cast<const NMTREEVIEWA*>(lParam);
            const ViewportSelectionKind kind = decode_model_tree_payload_kind(treeView->itemNew.lParam);
            const adaptivecad::geometry::EntityId id = decode_model_tree_payload_id(treeView->itemNew.lParam);
            if (!geometry_selection_exists(kind, id)) {
                apply_geometry_selection(ViewportSelectionKind::None, 0, "Status: model tree selection cleared");
                return 0;
            }

            apply_geometry_selection(kind, id, "Status: model tree selection cleared");
            if (g_viewportPanel) {
                SetFocus(g_viewportPanel);
            }
            return 0;
        }

        break;
    }
    case WM_COMMAND: {
        const int commandId = LOWORD(wParam);
        const int notifyCode = HIWORD(wParam);

        if (commandId == kHealingPolicyComboId && notifyCode == CBN_SELCHANGE && g_healingPolicyCombo) {
            const LRESULT selection = SendMessageA(g_healingPolicyCombo, CB_GETCURSEL, 0, 0);
            g_healingPolicy = combo_index_to_healing_policy(static_cast<int>(selection));
            set_status_text(std::string("Status: healing policy set to ") +
                adaptivecad::geometry::BRepKernel::healing_policy_name(g_healingPolicy));
            refresh_output(hwnd);
            return 0;
        }

        if (commandId == kViewportProjectionComboId && notifyCode == CBN_SELCHANGE && g_viewportProjectionCombo) {
            const LRESULT selection = SendMessageA(g_viewportProjectionCombo, CB_GETCURSEL, 0, 0);
            set_viewport_projection_mode(combo_index_to_viewport_projection(static_cast<int>(selection)));
            return 0;
        }

        if (commandId == kPrimitiveTypeComboId && notifyCode == CBN_SELCHANGE && g_primitiveTypeCombo) {
            update_primitive_parameter_controls();
            set_status_text("Status: primitive type changed");
            return 0;
        }

        if (commandId == kViewportResetButtonId) {
            reset_viewport_interaction();
            return 0;
        }

        if (commandId == kSaveSessionButtonId) {
            save_session_bundle(hwnd);
            return 0;
        }

        if (commandId == kLoadSessionButtonId) {
            load_session_bundle(hwnd);
            return 0;
        }

        if (commandId == kImportGeometryButtonId) {
            import_geometry_document(hwnd);
            return 0;
        }

        if (commandId == kExportGeometryButtonId) {
            export_geometry_document(hwnd);
            return 0;
        }

        if (commandId == kCreateBoxButtonId) {
            create_box_scene(hwnd);
            return 0;
        }

        if (commandId == kCreateWedgeButtonId) {
            create_wedge_scene(hwnd);
            return 0;
        }

        if (commandId == kCreatePlaneButtonId) {
            create_plane_scene(hwnd);
            return 0;
        }

        if (commandId == kCreateTorusButtonId) {
            create_torus_scene(hwnd);
            return 0;
        }

        if (commandId == kCreateScaffoldButtonId) {
            create_scaffold_scene(hwnd);
            return 0;
        }

        if (commandId == kCreatePrimitiveButtonId) {
            create_primitive_from_controls(hwnd);
            return 0;
        }

        if (commandId == kApplyBodyTransformButtonId) {
            apply_selected_body_transform(hwnd);
            return 0;
        }

        if (commandId == kDuplicateBodyButtonId) {
            duplicate_selected_body(hwnd);
            return 0;
        }

        if (commandId == kDeleteBodyButtonId) {
            delete_selected_body(hwnd);
            return 0;
        }

        if (commandId == kRenameBodyButtonId) {
            rename_selected_body(hwnd);
            return 0;
        }

        if (commandId == kAngularApplyButtonId) {
            apply_angular_branch_controls(hwnd);
            return 0;
        }

        if (commandId == kAngularRevertButtonId) {
            revert_angular_branch_controls();
            return 0;
        }

        if (commandId == kRefreshButtonId) {
            refresh_output(hwnd);
            return 0;
        }
        if (commandId == kOpenSpecButtonId) {
            open_spec_document(hwnd);
            return 0;
        }
        if (commandId == kImportCsvButtonId) {
            import_csv_measurements(hwnd);
            return 0;
        }
        if (commandId == kRefitButtonId) {
            refit_current_dataset(hwnd);
            return 0;
        }
        if (commandId == kClearDatasetButtonId) {
            clear_imported_dataset(hwnd);
            return 0;
        }
        if (commandId == kExportButtonId) {
            export_snapshot(hwnd);
            return 0;
        }
        if (commandId == kCloseButtonId) {
            DestroyWindow(hwnd);
            return 0;
        }
        break;
    }
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    default:
        break;
    }

    return DefWindowProcA(hwnd, msg, wParam, lParam);
}

} // namespace

namespace {

struct StartupActions {
    std::filesystem::path import_geometry_path;
    std::filesystem::path load_session_path;
    std::filesystem::path save_session_path;
    bool has_import_geometry = false;
    bool has_load_session = false;
    bool has_save_session = false;
    bool exit_after_actions = false;
};

std::string narrow_from_wide(const wchar_t* text) {
    if (!text) {
        return {};
    }

    const int required = WideCharToMultiByte(CP_UTF8, 0, text, -1, nullptr, 0, nullptr, nullptr);
    if (required <= 0) {
        return {};
    }

    std::string converted(static_cast<std::size_t>(required), '\0');
    WideCharToMultiByte(
        CP_UTF8,
        0,
        text,
        -1,
        converted.data(),
        required,
        nullptr,
        nullptr);
    if (!converted.empty() && converted.back() == '\0') {
        converted.pop_back();
    }
    return converted;
}

std::filesystem::path resolve_startup_path(std::filesystem::path path) {
    if (path.empty()) {
        return path;
    }

    if (path.is_relative()) {
        if (!g_workspaceRoot.empty()) {
            path = g_workspaceRoot / path;
        } else {
            std::error_code ec;
            const std::filesystem::path absolutePath = std::filesystem::absolute(path, ec);
            if (!ec) {
                path = absolutePath;
            }
        }
    }

    return path.lexically_normal();
}

StartupActions parse_startup_actions() {
    StartupActions actions{};
    int argc = 0;
    LPWSTR* argv = CommandLineToArgvW(GetCommandLineW(), &argc);
    if (!argv) {
        return actions;
    }

    for (int index = 1; index < argc; ++index) {
        const std::string argument = narrow_from_wide(argv[index]);
        if (argument == "--exit-after-actions") {
            actions.exit_after_actions = true;
            continue;
        }

        const auto parse_path_option = [&](const char* optionName, std::filesystem::path* path, bool* present) {
            const std::string optionPrefix = std::string(optionName) + "=";
            if (argument.rfind(optionPrefix, 0) == 0) {
                *path = resolve_startup_path(argument.substr(optionPrefix.size()));
                *present = true;
                return true;
            }

            if (argument == optionName && index + 1 < argc) {
                *path = resolve_startup_path(narrow_from_wide(argv[++index]));
                *present = true;
                return true;
            }

            return false;
        };

        if (parse_path_option("--load-session", &actions.load_session_path, &actions.has_load_session)) {
            continue;
        }

        if (parse_path_option("--import-geometry", &actions.import_geometry_path, &actions.has_import_geometry)) {
            continue;
        }

        if (parse_path_option("--save-session", &actions.save_session_path, &actions.has_save_session)) {
            continue;
        }
    }

    LocalFree(argv);
    return actions;
}

} // namespace

int WINAPI WinMain(HINSTANCE instance, HINSTANCE, LPSTR, int cmdShow) {
    const char* className = "AdaptiveCAD_UI_WindowClass";
    g_workspaceRoot = detect_workspace_root();
    set_dataset_info_to_synthetic();

    INITCOMMONCONTROLSEX commonControls{};
    commonControls.dwSize = sizeof(commonControls);
    commonControls.dwICC = ICC_TREEVIEW_CLASSES;
    InitCommonControlsEx(&commonControls);

    WNDCLASSA plotClass{};
    plotClass.lpfnWndProc = plot_panel_proc;
    plotClass.hInstance = instance;
    plotClass.lpszClassName = kPlotWindowClassName;
    plotClass.hCursor = LoadCursor(nullptr, IDC_CROSS);
    plotClass.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);

    if (!RegisterClassA(&plotClass)) {
        MessageBoxA(nullptr, "Failed to register AdaptiveCAD plot window class.", "AdaptiveCAD UI", MB_ICONERROR);
        return 1;
    }

    WNDCLASSA viewportClass{};
    viewportClass.lpfnWndProc = viewport_panel_proc;
    viewportClass.hInstance = instance;
    viewportClass.lpszClassName = kViewportWindowClassName;
    viewportClass.hCursor = LoadCursor(nullptr, IDC_ARROW);
    viewportClass.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);

    if (!RegisterClassA(&viewportClass)) {
        MessageBoxA(nullptr, "Failed to register AdaptiveCAD viewport window class.", "AdaptiveCAD UI", MB_ICONERROR);
        return 1;
    }

    WNDCLASSA wc{};
    wc.lpfnWndProc = window_proc;
    wc.hInstance = instance;
    wc.lpszClassName = className;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);

    if (!RegisterClassA(&wc)) {
        MessageBoxA(nullptr, "Failed to register AdaptiveCAD window class.", "AdaptiveCAD UI", MB_ICONERROR);
        return 1;
    }

    HWND window = CreateWindowExA(
        0,
        className,
        "AdaptiveCAD UI",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        1100,
        860,
        nullptr,
        nullptr,
        instance,
        nullptr);

    if (!window) {
        MessageBoxA(nullptr, "Failed to create AdaptiveCAD UI window.", "AdaptiveCAD UI", MB_ICONERROR);
        return 1;
    }

    SetMenu(window, create_main_menu());

    ShowWindow(window, cmdShow);
    UpdateWindow(window);

    const StartupActions startupActions = parse_startup_actions();
    if (startupActions.has_import_geometry) {
        import_geometry_document(window, startupActions.import_geometry_path);
    }
    if (startupActions.has_load_session) {
        load_session_bundle_from_path(window, startupActions.load_session_path);
    }
    if (startupActions.has_save_session) {
        save_session_bundle_to_path(window, startupActions.save_session_path);
    }
    if (startupActions.exit_after_actions) {
        DestroyWindow(window);
    }

    MSG msg{};
    while (GetMessageA(&msg, nullptr, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessageA(&msg);
    }

    return static_cast<int>(msg.wParam);
}
