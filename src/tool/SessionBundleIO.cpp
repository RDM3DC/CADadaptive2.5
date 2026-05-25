#include "adaptivecad/tool/SessionBundleIO.hpp"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace adaptivecad::tool {
namespace {

std::string trim_copy(const std::string& input) {
    std::size_t start = 0;
    while (start < input.size() && std::isspace(static_cast<unsigned char>(input[start])) != 0) {
        ++start;
    }

    std::size_t end = input.size();
    while (end > start && std::isspace(static_cast<unsigned char>(input[end - 1])) != 0) {
        --end;
    }

    return input.substr(start, end - start);
}

std::string lowercase_copy(const std::string& value) {
    std::string lowered = value;
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return lowered;
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
    return true;
}

bool try_parse_uint64(const std::string& text, std::uint64_t* value) {
    if (!value) {
        return false;
    }

    const std::string trimmed = trim_copy(text);
    if (trimmed.empty()) {
        return false;
    }

    char* endPointer = nullptr;
    const unsigned long long parsed = std::strtoull(trimmed.c_str(), &endPointer, 10);
    if (endPointer == trimmed.c_str()) {
        return false;
    }

    while (*endPointer != '\0') {
        if (std::isspace(static_cast<unsigned char>(*endPointer)) == 0) {
            return false;
        }
        ++endPointer;
    }

    *value = static_cast<std::uint64_t>(parsed);
    return true;
}

std::string read_ini_string(
    const std::filesystem::path& iniPath,
    const char* section,
    const char* key,
    const char* defaultValue = "") {
    char buffer[4096] = {};
    const std::string pathText = iniPath.string();
    GetPrivateProfileStringA(section, key, defaultValue, buffer, sizeof(buffer), pathText.c_str());
    return buffer;
}

double read_ini_double(
    const std::filesystem::path& iniPath,
    const char* section,
    const char* key,
    double defaultValue = 0.0) {
    std::ostringstream defaultText;
    defaultText << std::setprecision(15) << defaultValue;
    const std::string valueText = read_ini_string(iniPath, section, key, defaultText.str().c_str());
    double value = defaultValue;
    if (!try_parse_double(valueText, &value)) {
        throw std::runtime_error(std::string("Invalid numeric session value for ") + section + "." + key);
    }
    return value;
}

std::uint64_t read_ini_uint64(
    const std::filesystem::path& iniPath,
    const char* section,
    const char* key,
    std::uint64_t defaultValue = 0) {
    const std::string defaultText = std::to_string(defaultValue);
    const std::string valueText = read_ini_string(iniPath, section, key, defaultText.c_str());
    std::uint64_t value = defaultValue;
    if (!try_parse_uint64(valueText, &value)) {
        throw std::runtime_error(std::string("Invalid integer session value for ") + section + "." + key);
    }
    return value;
}

void write_ini_string(
    const std::filesystem::path& iniPath,
    const char* section,
    const char* key,
    const std::string& value) {
    const std::string pathText = iniPath.string();
    if (WritePrivateProfileStringA(section, key, value.c_str(), pathText.c_str()) == FALSE) {
        throw std::runtime_error(std::string("Failed writing session value ") + section + "." + key);
    }
}

void write_ini_double(
    const std::filesystem::path& iniPath,
    const char* section,
    const char* key,
    double value) {
    std::ostringstream out;
    out << std::setprecision(15) << value;
    write_ini_string(iniPath, section, key, out.str());
}

void write_ini_uint64(
    const std::filesystem::path& iniPath,
    const char* section,
    const char* key,
    std::uint64_t value) {
    write_ini_string(iniPath, section, key, std::to_string(value));
}

std::string join_double_list_for_ini(const std::vector<double>& values) {
    std::ostringstream out;
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            out << ',';
        }
        out << std::setprecision(15) << values[index];
    }
    return out.str();
}

std::string join_index_list_for_ini(const std::vector<std::size_t>& values) {
    std::ostringstream out;
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            out << ',';
        }
        out << values[index];
    }
    return out.str();
}

std::vector<double> parse_double_list_from_ini(const std::string& text) {
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
            throw std::runtime_error("Invalid angular coefficient list value");
        }
        values.push_back(value);
    }
    return values;
}

std::vector<std::size_t> parse_index_list_from_ini(const std::string& text) {
    std::vector<std::size_t> values;
    std::istringstream input(text);
    std::string token;
    while (std::getline(input, token, ',')) {
        const std::string trimmed = trim_copy(token);
        if (trimmed.empty()) {
            continue;
        }

        std::uint64_t parsed = 0;
        if (!try_parse_uint64(trimmed, &parsed)) {
            throw std::runtime_error("Invalid scene face vertex index list value");
        }
        values.push_back(static_cast<std::size_t>(parsed));
    }
    return values;
}

void write_ini_bool(
    const std::filesystem::path& iniPath,
    const char* section,
    const char* key,
    bool value) {
    write_ini_string(iniPath, section, key, value ? "1" : "0");
}

bool read_ini_bool(
    const std::filesystem::path& iniPath,
    const char* section,
    const char* key,
    bool defaultValue = false) {
    const std::string value = lowercase_copy(trim_copy(read_ini_string(
        iniPath,
        section,
        key,
        defaultValue ? "1" : "0")));
    return value == "1" || value == "true" || value == "yes";
}

void write_scene_color_to_session(
    const std::filesystem::path& sessionPath,
    const char* section,
    const SceneColor& color) {
    write_ini_double(sessionPath, section, "color_red", color.red);
    write_ini_double(sessionPath, section, "color_green", color.green);
    write_ini_double(sessionPath, section, "color_blue", color.blue);
    write_ini_double(sessionPath, section, "color_alpha", color.alpha);
}

SceneColor read_scene_color_from_session(
    const std::filesystem::path& sessionPath,
    const char* section) {
    return SceneColor{
        read_ini_double(sessionPath, section, "color_red", 0.0),
        read_ini_double(sessionPath, section, "color_green", 0.0),
        read_ini_double(sessionPath, section, "color_blue", 0.0),
        read_ini_double(sessionPath, section, "color_alpha", 1.0)};
}

void write_scene_document_to_session(const std::filesystem::path& sessionPath, const SceneDocument& document) {
    write_ini_string(sessionPath, "scene", "label", document.scene_label);
    write_ini_string(sessionPath, "scene", "source_kind", scene_source_kind_name(document.source_kind));
    write_ini_string(sessionPath, "scene", "source_path", document.source_path.string());
    write_ini_uint64(sessionPath, "scene", "material_count", document.materials.size());
    write_ini_uint64(sessionPath, "scene", "body_count", document.bodies.size());

    write_ini_double(sessionPath, "angular_branch", "lambda0", document.angular_model.lambda0);
    write_ini_string(
        sessionPath,
        "angular_branch",
        "cosine_coefficients",
        join_double_list_for_ini(document.angular_model.cosine_coefficients));
    write_ini_string(
        sessionPath,
        "angular_branch",
        "sine_coefficients",
        join_double_list_for_ini(document.angular_model.sine_coefficients));

    for (std::size_t materialIndex = 0; materialIndex < document.materials.size(); ++materialIndex) {
        const SceneMaterial& material = document.materials[materialIndex];
        const std::string materialSection = "scene_material_" + std::to_string(materialIndex);
        write_ini_string(sessionPath, materialSection.c_str(), "name", material.name);
        write_ini_bool(sessionPath, materialSection.c_str(), "has_diffuse_color", material.has_diffuse_color);
        write_scene_color_to_session(sessionPath, materialSection.c_str(), material.diffuse_color);
    }

    for (std::size_t bodyIndex = 0; bodyIndex < document.bodies.size(); ++bodyIndex) {
        const auto& body = document.bodies[bodyIndex];
        const std::string bodySection = "scene_body_" + std::to_string(bodyIndex);
        write_ini_string(sessionPath, bodySection.c_str(), "name", body.name);
        write_ini_string(sessionPath, bodySection.c_str(), "material_name", body.material_name);
        write_ini_bool(sessionPath, bodySection.c_str(), "has_color", body.has_color);
        write_scene_color_to_session(sessionPath, bodySection.c_str(), body.color);
        write_ini_double(sessionPath, bodySection.c_str(), "volume_hint", body.volume_hint);
        write_ini_uint64(sessionPath, bodySection.c_str(), "vertex_count", body.vertices.size());
        write_ini_uint64(sessionPath, bodySection.c_str(), "face_count", body.faces.size());

        for (std::size_t vertexIndex = 0; vertexIndex < body.vertices.size(); ++vertexIndex) {
            const auto& vertex = body.vertices[vertexIndex];
            const std::string vertexSection = bodySection + "_vertex_" + std::to_string(vertexIndex);
            write_ini_double(sessionPath, vertexSection.c_str(), "x", vertex.x);
            write_ini_double(sessionPath, vertexSection.c_str(), "y", vertex.y);
            write_ini_double(sessionPath, vertexSection.c_str(), "z", vertex.z);
        }

        for (std::size_t faceIndex = 0; faceIndex < body.faces.size(); ++faceIndex) {
            const auto& face = body.faces[faceIndex];
            const std::string faceSection = bodySection + "_face_" + std::to_string(faceIndex);
            write_ini_string(sessionPath, faceSection.c_str(), "vertex_indices", join_index_list_for_ini(face.vertex_indices));
            write_ini_double(sessionPath, faceSection.c_str(), "area_hint", face.area_hint);
            write_ini_string(sessionPath, faceSection.c_str(), "material_name", face.material_name);
            write_ini_bool(sessionPath, faceSection.c_str(), "has_color", face.has_color);
            write_scene_color_to_session(sessionPath, faceSection.c_str(), face.color);
        }
    }
}

SceneDocument read_scene_document_from_session(const std::filesystem::path& sessionPath, std::uint64_t schemaVersion) {
    SceneDocument document = make_demo_scene_document();
    if (schemaVersion < 2) {
        return document;
    }

    document.scene_label = read_ini_string(sessionPath, "scene", "label", document.scene_label.c_str());
    document.source_kind = parse_scene_source_kind(
        read_ini_string(sessionPath, "scene", "source_kind", scene_source_kind_name(document.source_kind)));
    document.source_path = std::filesystem::path(
        read_ini_string(sessionPath, "scene", "source_path", document.source_path.string().c_str()));
    document.angular_model.lambda0 = read_ini_double(sessionPath, "angular_branch", "lambda0", document.angular_model.lambda0);
    document.angular_model.cosine_coefficients =
        parse_double_list_from_ini(read_ini_string(sessionPath, "angular_branch", "cosine_coefficients", ""));
    document.angular_model.sine_coefficients =
        parse_double_list_from_ini(read_ini_string(sessionPath, "angular_branch", "sine_coefficients", ""));

    const std::uint64_t materialCount = read_ini_uint64(sessionPath, "scene", "material_count", 0);
    document.materials.clear();
    document.materials.reserve(static_cast<std::size_t>(materialCount));
    for (std::uint64_t materialIndex = 0; materialIndex < materialCount; ++materialIndex) {
        const std::string materialSection = "scene_material_" + std::to_string(materialIndex);
        SceneMaterial material;
        material.name = read_ini_string(sessionPath, materialSection.c_str(), "name", "");
        material.has_diffuse_color = read_ini_bool(sessionPath, materialSection.c_str(), "has_diffuse_color", false);
        material.diffuse_color = read_scene_color_from_session(sessionPath, materialSection.c_str());
        if (!material.name.empty()) {
            document.materials.push_back(std::move(material));
        }
    }

    const std::uint64_t bodyCount = read_ini_uint64(sessionPath, "scene", "body_count", 0);
    if (bodyCount == 0) {
        return document;
    }

    document.bodies.clear();
    document.bodies.reserve(static_cast<std::size_t>(bodyCount));
    for (std::uint64_t bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex) {
        SceneBody body;
        const std::string bodySection = "scene_body_" + std::to_string(bodyIndex);
        body.name = read_ini_string(sessionPath, bodySection.c_str(), "name", ("Body " + std::to_string(bodyIndex + 1)).c_str());
        body.material_name = read_ini_string(sessionPath, bodySection.c_str(), "material_name", "");
        body.has_color = read_ini_bool(sessionPath, bodySection.c_str(), "has_color", false);
        body.color = read_scene_color_from_session(sessionPath, bodySection.c_str());
        body.volume_hint = read_ini_double(sessionPath, bodySection.c_str(), "volume_hint", 0.0);
        const std::uint64_t vertexCount = read_ini_uint64(sessionPath, bodySection.c_str(), "vertex_count", 0);
        const std::uint64_t faceCount = read_ini_uint64(sessionPath, bodySection.c_str(), "face_count", 0);
        if (vertexCount == 0 || faceCount == 0) {
            throw std::runtime_error("session scene body is missing vertices or faces");
        }

        body.vertices.reserve(static_cast<std::size_t>(vertexCount));
        for (std::uint64_t vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex) {
            const std::string vertexSection = bodySection + "_vertex_" + std::to_string(vertexIndex);
            body.vertices.push_back(geometry::Point3D{
                read_ini_double(sessionPath, vertexSection.c_str(), "x", 0.0),
                read_ini_double(sessionPath, vertexSection.c_str(), "y", 0.0),
                read_ini_double(sessionPath, vertexSection.c_str(), "z", 0.0)});
        }

        body.faces.reserve(static_cast<std::size_t>(faceCount));
        for (std::uint64_t faceIndex = 0; faceIndex < faceCount; ++faceIndex) {
            const std::string faceSection = bodySection + "_face_" + std::to_string(faceIndex);
            SceneFace face;
            face.vertex_indices = parse_index_list_from_ini(read_ini_string(sessionPath, faceSection.c_str(), "vertex_indices", ""));
            face.area_hint = read_ini_double(sessionPath, faceSection.c_str(), "area_hint", 0.0);
            face.material_name = read_ini_string(sessionPath, faceSection.c_str(), "material_name", "");
            face.has_color = read_ini_bool(sessionPath, faceSection.c_str(), "has_color", false);
            face.color = read_scene_color_from_session(sessionPath, faceSection.c_str());
            if (face.vertex_indices.size() < 3) {
                throw std::runtime_error("session scene face must reference at least three vertices");
            }
            body.faces.push_back(std::move(face));
        }

        document.bodies.push_back(std::move(body));
    }

    return document;
}

} // namespace

SessionBundleData load_session_bundle_file(const std::filesystem::path& sessionPath) {
    if (!std::filesystem::exists(sessionPath)) {
        throw std::runtime_error("session file was not found");
    }

    SessionBundleData bundle;
    bundle.schema_version = read_ini_uint64(sessionPath, "session", "schema_version", 0);
    if (bundle.schema_version != 1 && bundle.schema_version != 2) {
        throw std::runtime_error("unsupported session schema version");
    }

    bundle.saved_at = read_ini_string(sessionPath, "session", "saved_at", "");
    bundle.workspace = std::filesystem::path(read_ini_string(sessionPath, "session", "workspace", ""));
    bundle.report_text_path = std::filesystem::path(read_ini_string(sessionPath, "session", "report_text_path", ""));
    bundle.report_json_path = std::filesystem::path(read_ini_string(sessionPath, "session", "report_json_path", ""));
    bundle.scene_document = read_scene_document_from_session(sessionPath, bundle.schema_version);

    const std::string datasetSource = lowercase_copy(read_ini_string(sessionPath, "dataset", "source", "synthetic"));
    bundle.dataset.using_embedded_samples = datasetSource != "synthetic";
    bundle.dataset.original_csv_path = std::filesystem::path(read_ini_string(sessionPath, "dataset", "original_csv_path", ""));
    bundle.dataset.valid_rows = static_cast<std::size_t>(read_ini_uint64(sessionPath, "dataset", "valid_rows", 0));
    bundle.dataset.rejected_rows = static_cast<std::size_t>(read_ini_uint64(sessionPath, "dataset", "rejected_rows", 0));
    bundle.dataset.summary = read_ini_string(sessionPath, "dataset", "summary", "");

    const std::uint64_t sampleCount = read_ini_uint64(sessionPath, "dataset", "sample_count", 0);
    if (bundle.dataset.using_embedded_samples) {
        if (sampleCount < 3) {
            throw std::runtime_error("session dataset must contain at least 3 samples");
        }

        bundle.dataset.samples.reserve(static_cast<std::size_t>(sampleCount));
        for (std::uint64_t index = 0; index < sampleCount; ++index) {
            const std::string section = "sample_" + std::to_string(index);
            const double radius = read_ini_double(sessionPath, section.c_str(), "radius", 0.0);
            const double area = read_ini_double(sessionPath, section.c_str(), "area", 0.0);
            if (radius <= 0.0 || area <= 0.0) {
                throw std::runtime_error("session dataset contains non-positive radius or area");
            }
            bundle.dataset.samples.push_back(core::AreaSample{radius, area});
        }
    }

    bundle.view_state.healing_policy = read_ini_string(sessionPath, "state", "healing_policy", "RepairFirst");
    bundle.view_state.projection = read_ini_string(sessionPath, "state", "projection", "Isometric");
    bundle.view_state.zoom = read_ini_double(sessionPath, "state", "zoom", 1.0);
    bundle.view_state.pan_x = read_ini_double(sessionPath, "state", "pan_x", 0.0);
    bundle.view_state.pan_y = read_ini_double(sessionPath, "state", "pan_y", 0.0);
    bundle.view_state.selection_kind = read_ini_string(sessionPath, "state", "selection_kind", "none");
    bundle.view_state.selected_id = read_ini_uint64(sessionPath, "state", "selected_id", 0);
    return bundle;
}

void save_session_bundle_file(const std::filesystem::path& requestedSessionPath, const SessionBundleData& bundle) {
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

    std::ofstream resetFile(sessionPath, std::ios::trunc);
    if (!resetFile) {
        throw std::runtime_error("could not create session file");
    }
    resetFile.close();

    write_ini_string(sessionPath, "session", "schema_version", std::to_string(bundle.schema_version));
    write_ini_string(sessionPath, "session", "saved_at", bundle.saved_at);
    write_ini_string(sessionPath, "session", "workspace", bundle.workspace.string());
    write_ini_string(sessionPath, "session", "report_text_path", bundle.report_text_path.string());
    write_ini_string(sessionPath, "session", "report_json_path", bundle.report_json_path.string());
    write_scene_document_to_session(sessionPath, bundle.scene_document);

    write_ini_string(
        sessionPath,
        "dataset",
        "source",
        bundle.dataset.using_embedded_samples ? "embedded_samples" : "synthetic");
    write_ini_string(sessionPath, "dataset", "original_csv_path", bundle.dataset.original_csv_path.string());
    write_ini_uint64(sessionPath, "dataset", "valid_rows", bundle.dataset.valid_rows);
    write_ini_uint64(sessionPath, "dataset", "rejected_rows", bundle.dataset.rejected_rows);
    write_ini_string(sessionPath, "dataset", "summary", bundle.dataset.summary);
    write_ini_uint64(sessionPath, "dataset", "sample_count", bundle.dataset.samples.size());

    for (std::size_t index = 0; index < bundle.dataset.samples.size(); ++index) {
        const std::string section = "sample_" + std::to_string(index);
        write_ini_double(sessionPath, section.c_str(), "radius", bundle.dataset.samples[index].radius);
        write_ini_double(sessionPath, section.c_str(), "area", bundle.dataset.samples[index].area);
    }

    write_ini_string(sessionPath, "state", "healing_policy", bundle.view_state.healing_policy);
    write_ini_string(sessionPath, "state", "projection", bundle.view_state.projection);
    write_ini_double(sessionPath, "state", "zoom", bundle.view_state.zoom);
    write_ini_double(sessionPath, "state", "pan_x", bundle.view_state.pan_x);
    write_ini_double(sessionPath, "state", "pan_y", bundle.view_state.pan_y);
    write_ini_string(sessionPath, "state", "selection_kind", bundle.view_state.selection_kind);
    write_ini_uint64(sessionPath, "state", "selected_id", bundle.view_state.selected_id);
}

} // namespace adaptivecad::tool
