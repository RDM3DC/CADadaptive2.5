#pragma once

#include "adaptivecad/core/InverseRecovery.hpp"
#include "adaptivecad/tool/SceneDocument.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace adaptivecad::tool {

struct SessionMeasurementData {
    bool using_embedded_samples = false;
    std::filesystem::path original_csv_path;
    std::size_t valid_rows = 0;
    std::size_t rejected_rows = 0;
    std::string summary;
    std::vector<core::AreaSample> samples;
};

struct SessionViewState {
    std::string healing_policy = "RepairFirst";
    std::string projection = "Isometric";
    double zoom = 1.0;
    double pan_x = 0.0;
    double pan_y = 0.0;
    std::string selection_kind = "none";
    std::uint64_t selected_id = 0;
};

struct SessionBundleData {
    std::uint64_t schema_version = 2;
    std::string saved_at;
    std::filesystem::path workspace;
    std::filesystem::path report_text_path;
    std::filesystem::path report_json_path;
    SceneDocument scene_document;
    SessionMeasurementData dataset;
    SessionViewState view_state;
};

SessionBundleData load_session_bundle_file(const std::filesystem::path& sessionPath);
void save_session_bundle_file(const std::filesystem::path& sessionPath, const SessionBundleData& bundle);

} // namespace adaptivecad::tool