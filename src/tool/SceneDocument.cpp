#include "adaptivecad/tool/SceneDocument.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef ADAPTIVECAD_HAS_OPENCASCADE
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Reader.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#endif

namespace adaptivecad::tool {
namespace {

std::string trim_copy(const std::string& text) {
    std::size_t start = 0;
    while (start < text.size() && std::isspace(static_cast<unsigned char>(text[start])) != 0) {
        ++start;
    }

    std::size_t end = text.size();
    while (end > start && std::isspace(static_cast<unsigned char>(text[end - 1])) != 0) {
        --end;
    }

    return text.substr(start, end - start);
}

std::string lowercase_copy(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return text;
}

double clamp01(double value) noexcept {
    return std::max(0.0, std::min(1.0, value));
}

SceneColor make_scene_color(double red, double green, double blue, double alpha = 1.0) noexcept {
    return SceneColor{clamp01(red), clamp01(green), clamp01(blue), clamp01(alpha)};
}

bool colors_match(const SceneColor& left, const SceneColor& right) noexcept {
    constexpr double kTolerance = 1.0e-9;
    return std::fabs(left.red - right.red) <= kTolerance &&
        std::fabs(left.green - right.green) <= kTolerance &&
        std::fabs(left.blue - right.blue) <= kTolerance &&
        std::fabs(left.alpha - right.alpha) <= kTolerance;
}

int color_component_to_byte(double value) noexcept {
    return static_cast<int>(std::lround(clamp01(value) * 255.0));
}

std::string color_key(const SceneColor& color) {
    std::ostringstream out;
    out << color_component_to_byte(color.red) << '_'
        << color_component_to_byte(color.green) << '_'
        << color_component_to_byte(color.blue) << '_'
        << color_component_to_byte(color.alpha);
    return out.str();
}

bool material_names_match(const std::string& left, const std::string& right) {
    return trim_copy(left) == trim_copy(right);
}

const SceneMaterial* find_material_by_name(
    const std::vector<SceneMaterial>& materials,
    const std::string& name) {
    const std::string trimmedName = trim_copy(name);
    if (trimmedName.empty()) {
        return nullptr;
    }

    for (const SceneMaterial& material : materials) {
        if (material_names_match(material.name, trimmedName)) {
            return &material;
        }
    }
    return nullptr;
}

void upsert_scene_material(std::vector<SceneMaterial>* materials, const SceneMaterial& material) {
    if (!materials || trim_copy(material.name).empty()) {
        return;
    }

    for (SceneMaterial& existing : *materials) {
        if (material_names_match(existing.name, material.name)) {
            existing = material;
            return;
        }
    }
    materials->push_back(material);
}

void apply_material_to_face(
    SceneFace* face,
    const std::vector<SceneMaterial>& materials,
    const std::string& materialName) {
    if (!face) {
        return;
    }

    face->material_name = trim_copy(materialName);
    if (const SceneMaterial* material = find_material_by_name(materials, face->material_name)) {
        if (material->has_diffuse_color) {
            face->color = material->diffuse_color;
            face->has_color = true;
        }
    }
}

void derive_body_metadata_from_faces(SceneBody* body) {
    if (!body || body->faces.empty()) {
        return;
    }

    if (body->material_name.empty()) {
        const std::string firstMaterial = trim_copy(body->faces.front().material_name);
        if (!firstMaterial.empty()) {
            bool allMatch = true;
            for (const SceneFace& face : body->faces) {
                if (!material_names_match(face.material_name, firstMaterial)) {
                    allMatch = false;
                    break;
                }
            }
            if (allMatch) {
                body->material_name = firstMaterial;
            }
        }
    }

    if (!body->has_color && body->faces.front().has_color) {
        const SceneColor firstColor = body->faces.front().color;
        bool allMatch = true;
        for (const SceneFace& face : body->faces) {
            if (!face.has_color || !colors_match(face.color, firstColor)) {
                allMatch = false;
                break;
            }
        }
        if (allMatch) {
            body->color = firstColor;
            body->has_color = true;
        }
    }
}

enum class PlyFormat {
    Unknown,
    Ascii,
    BinaryLittleEndian,
    BinaryBigEndian
};

enum class PlyScalarType {
    Invalid,
    Int8,
    UInt8,
    Int16,
    UInt16,
    Int32,
    UInt32,
    Float32,
    Float64
};

struct PlyPropertySpec {
    std::string name;
    bool is_list = false;
    PlyScalarType scalar_type = PlyScalarType::Invalid;
    PlyScalarType list_count_type = PlyScalarType::Invalid;
    PlyScalarType list_item_type = PlyScalarType::Invalid;
};

struct PlyElementSpec {
    std::string name;
    std::size_t count = 0;
    std::vector<PlyPropertySpec> properties;
};

struct PlyHeaderSpec {
    PlyFormat format = PlyFormat::Unknown;
    std::vector<PlyElementSpec> elements;
};

PlyScalarType parse_ply_scalar_type(const std::string& text) {
    const std::string normalized = lowercase_copy(text);
    if (normalized == "char" || normalized == "int8") {
        return PlyScalarType::Int8;
    }
    if (normalized == "uchar" || normalized == "uint8" || normalized == "unsigned_char") {
        return PlyScalarType::UInt8;
    }
    if (normalized == "short" || normalized == "int16") {
        return PlyScalarType::Int16;
    }
    if (normalized == "ushort" || normalized == "uint16") {
        return PlyScalarType::UInt16;
    }
    if (normalized == "int" || normalized == "int32") {
        return PlyScalarType::Int32;
    }
    if (normalized == "uint" || normalized == "uint32") {
        return PlyScalarType::UInt32;
    }
    if (normalized == "float" || normalized == "float32") {
        return PlyScalarType::Float32;
    }
    if (normalized == "double" || normalized == "float64") {
        return PlyScalarType::Float64;
    }
    return PlyScalarType::Invalid;
}

bool ply_type_is_integer(PlyScalarType type) noexcept {
    switch (type) {
    case PlyScalarType::Int8:
    case PlyScalarType::UInt8:
    case PlyScalarType::Int16:
    case PlyScalarType::UInt16:
    case PlyScalarType::Int32:
    case PlyScalarType::UInt32:
        return true;
    case PlyScalarType::Invalid:
    case PlyScalarType::Float32:
    case PlyScalarType::Float64:
        return false;
    }

    return false;
}

bool ply_type_is_signed(PlyScalarType type) noexcept {
    switch (type) {
    case PlyScalarType::Int8:
    case PlyScalarType::Int16:
    case PlyScalarType::Int32:
        return true;
    case PlyScalarType::Invalid:
    case PlyScalarType::UInt8:
    case PlyScalarType::UInt16:
    case PlyScalarType::UInt32:
    case PlyScalarType::Float32:
    case PlyScalarType::Float64:
        return false;
    }

    return false;
}

std::size_t ply_scalar_size(PlyScalarType type) {
    switch (type) {
    case PlyScalarType::Int8:
    case PlyScalarType::UInt8:
        return 1;
    case PlyScalarType::Int16:
    case PlyScalarType::UInt16:
        return 2;
    case PlyScalarType::Int32:
    case PlyScalarType::UInt32:
    case PlyScalarType::Float32:
        return 4;
    case PlyScalarType::Float64:
        return 8;
    case PlyScalarType::Invalid:
        break;
    }

    throw std::runtime_error("PLY header contains an unsupported scalar type");
}

std::uint64_t read_unsigned_integer_bytes(
    std::istream& input,
    std::size_t byteCount,
    bool littleEndian,
    const std::string& context) {
    if (byteCount == 0 || byteCount > 8) {
        throw std::runtime_error(context);
    }

    std::array<unsigned char, 8> bytes{};
    input.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(byteCount));
    if (!input) {
        throw std::runtime_error(context);
    }

    std::uint64_t value = 0;
    if (littleEndian) {
        for (std::size_t index = 0; index < byteCount; ++index) {
            value |= static_cast<std::uint64_t>(bytes[index]) << (8U * index);
        }
    } else {
        for (std::size_t index = 0; index < byteCount; ++index) {
            value = (value << 8U) | static_cast<std::uint64_t>(bytes[index]);
        }
    }
    return value;
}

std::int64_t sign_extend_integer(std::uint64_t value, std::size_t byteCount) {
    const unsigned int bitCount = static_cast<unsigned int>(byteCount * 8U);
    if (bitCount >= 64U) {
        return static_cast<std::int64_t>(value);
    }

    const std::uint64_t signBit = std::uint64_t{1} << (bitCount - 1U);
    if ((value & signBit) == 0U) {
        return static_cast<std::int64_t>(value);
    }

    const std::uint64_t extensionMask = ~((std::uint64_t{1} << bitCount) - 1U);
    return static_cast<std::int64_t>(value | extensionMask);
}

std::uint16_t read_little_uint16(std::istream& input, const std::string& context) {
    return static_cast<std::uint16_t>(read_unsigned_integer_bytes(input, 2, true, context));
}

std::uint32_t read_little_uint32(std::istream& input, const std::string& context) {
    return static_cast<std::uint32_t>(read_unsigned_integer_bytes(input, 4, true, context));
}

double read_binary_numeric_scalar(
    std::istream& input,
    PlyScalarType type,
    bool littleEndian,
    const std::string& context) {
    if (type == PlyScalarType::Float32) {
        const std::uint32_t raw =
            static_cast<std::uint32_t>(read_unsigned_integer_bytes(input, 4, littleEndian, context));
        float value = 0.0F;
        std::memcpy(&value, &raw, sizeof(value));
        return static_cast<double>(value);
    }
    if (type == PlyScalarType::Float64) {
        const std::uint64_t raw = read_unsigned_integer_bytes(input, 8, littleEndian, context);
        double value = 0.0;
        std::memcpy(&value, &raw, sizeof(value));
        return value;
    }

    if (!ply_type_is_integer(type)) {
        throw std::runtime_error(context);
    }

    const std::size_t byteCount = ply_scalar_size(type);
    const std::uint64_t raw = read_unsigned_integer_bytes(input, byteCount, littleEndian, context);
    if (ply_type_is_signed(type)) {
        return static_cast<double>(sign_extend_integer(raw, byteCount));
    }
    return static_cast<double>(raw);
}

std::int64_t read_binary_integer_scalar(
    std::istream& input,
    PlyScalarType type,
    bool littleEndian,
    const std::string& context) {
    if (!ply_type_is_integer(type)) {
        throw std::runtime_error(context);
    }

    const std::size_t byteCount = ply_scalar_size(type);
    const std::uint64_t raw = read_unsigned_integer_bytes(input, byteCount, littleEndian, context);
    if (ply_type_is_signed(type)) {
        return sign_extend_integer(raw, byteCount);
    }
    if (raw > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
        throw std::runtime_error(context);
    }
    return static_cast<std::int64_t>(raw);
}

float read_little_float32(std::istream& input, const std::string& context) {
    return static_cast<float>(read_binary_numeric_scalar(input, PlyScalarType::Float32, true, context));
}

double parse_ascii_numeric_token(const std::string& token, PlyScalarType type, const std::string& context) {
    try {
        std::size_t consumed = 0;
        if (type == PlyScalarType::Float32 || type == PlyScalarType::Float64) {
            const double value = std::stod(token, &consumed);
            if (consumed != token.size()) {
                throw std::runtime_error(context);
            }
            return value;
        }
        if (ply_type_is_signed(type)) {
            const long long value = std::stoll(token, &consumed);
            if (consumed != token.size()) {
                throw std::runtime_error(context);
            }
            return static_cast<double>(value);
        }
        if (ply_type_is_integer(type)) {
            if (!token.empty() && token.front() == '-') {
                throw std::runtime_error(context);
            }
            const unsigned long long value = std::stoull(token, &consumed);
            if (consumed != token.size()) {
                throw std::runtime_error(context);
            }
            return static_cast<double>(value);
        }
    } catch (const std::exception&) {
        throw std::runtime_error(context);
    }

    throw std::runtime_error(context);
}

std::int64_t parse_ascii_integer_token(const std::string& token, PlyScalarType type, const std::string& context) {
    if (!ply_type_is_integer(type)) {
        throw std::runtime_error(context);
    }

    try {
        std::size_t consumed = 0;
        if (ply_type_is_signed(type)) {
            const long long value = std::stoll(token, &consumed);
            if (consumed != token.size()) {
                throw std::runtime_error(context);
            }
            return static_cast<std::int64_t>(value);
        }

        if (!token.empty() && token.front() == '-') {
            throw std::runtime_error(context);
        }
        const unsigned long long value = std::stoull(token, &consumed);
        if (consumed != token.size() ||
            value > static_cast<unsigned long long>(std::numeric_limits<std::int64_t>::max())) {
            throw std::runtime_error(context);
        }
        return static_cast<std::int64_t>(value);
    } catch (const std::exception&) {
        throw std::runtime_error(context);
    }
}

double read_ascii_numeric_scalar(std::istringstream& input, PlyScalarType type, const std::string& context) {
    std::string token;
    if (!(input >> token)) {
        throw std::runtime_error(context);
    }
    return parse_ascii_numeric_token(token, type, context);
}

std::int64_t read_ascii_integer_scalar(std::istringstream& input, PlyScalarType type, const std::string& context) {
    std::string token;
    if (!(input >> token)) {
        throw std::runtime_error(context);
    }
    return parse_ascii_integer_token(token, type, context);
}

std::size_t checked_count(std::int64_t count, const std::string& context) {
    if (count < 0) {
        throw std::runtime_error(context);
    }
    return static_cast<std::size_t>(count);
}

bool is_vertex_index_property(const std::string& name) {
    const std::string normalized = lowercase_copy(name);
    return normalized == "vertex_indices" || normalized == "vertex_index";
}

PlyHeaderSpec parse_ply_header(std::istream& input) {
    PlyHeaderSpec header;
    bool sawEndHeader = false;

    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trim_copy(line);
        if (trimmed.empty()) {
            continue;
        }

        std::istringstream lineStream(trimmed);
        std::string keyword;
        lineStream >> keyword;
        keyword = lowercase_copy(keyword);

        if (keyword == "comment" || keyword == "obj_info") {
            continue;
        }
        if (keyword == "end_header") {
            sawEndHeader = true;
            break;
        }
        if (keyword == "format") {
            std::string formatKind;
            lineStream >> formatKind;
            formatKind = lowercase_copy(formatKind);
            if (formatKind == "ascii") {
                header.format = PlyFormat::Ascii;
            } else if (formatKind == "binary_little_endian") {
                header.format = PlyFormat::BinaryLittleEndian;
            } else if (formatKind == "binary_big_endian") {
                header.format = PlyFormat::BinaryBigEndian;
            } else {
                throw std::runtime_error("PLY import encountered an unsupported format");
            }
            continue;
        }
        if (keyword == "element") {
            std::string elementName;
            std::size_t count = 0;
            if (!(lineStream >> elementName >> count)) {
                throw std::runtime_error("PLY element header is malformed");
            }

            PlyElementSpec element;
            element.name = lowercase_copy(elementName);
            element.count = count;
            header.elements.push_back(std::move(element));
            continue;
        }
        if (keyword == "property") {
            if (header.elements.empty()) {
                throw std::runtime_error("PLY property appeared before an element declaration");
            }

            std::string typeToken;
            if (!(lineStream >> typeToken)) {
                throw std::runtime_error("PLY property header is malformed");
            }
            typeToken = lowercase_copy(typeToken);

            PlyPropertySpec property;
            if (typeToken == "list") {
                std::string countTypeToken;
                std::string itemTypeToken;
                std::string propertyName;
                if (!(lineStream >> countTypeToken >> itemTypeToken >> propertyName)) {
                    throw std::runtime_error("PLY list property header is malformed");
                }

                property.is_list = true;
                property.name = lowercase_copy(propertyName);
                property.list_count_type = parse_ply_scalar_type(countTypeToken);
                property.list_item_type = parse_ply_scalar_type(itemTypeToken);
                if (!ply_type_is_integer(property.list_count_type) ||
                    property.list_item_type == PlyScalarType::Invalid) {
                    throw std::runtime_error("PLY list property uses an unsupported scalar type");
                }
            } else {
                std::string propertyName;
                if (!(lineStream >> propertyName)) {
                    throw std::runtime_error("PLY scalar property header is malformed");
                }

                property.name = lowercase_copy(propertyName);
                property.scalar_type = parse_ply_scalar_type(typeToken);
                if (property.scalar_type == PlyScalarType::Invalid) {
                    throw std::runtime_error("PLY scalar property uses an unsupported type");
                }
            }

            header.elements.back().properties.push_back(std::move(property));
        }
    }

    if (!sawEndHeader) {
        throw std::runtime_error("PLY import did not find an end_header marker");
    }
    if (header.format == PlyFormat::Unknown) {
        throw std::runtime_error("PLY import requires a format declaration");
    }

    return header;
}

double polygon_area_hint(const std::vector<geometry::Point3D>& points) {
    if (points.size() < 3) {
        return 0.0;
    }

    double nx = 0.0;
    double ny = 0.0;
    double nz = 0.0;
    for (std::size_t index = 0; index < points.size(); ++index) {
        const auto& current = points[index];
        const auto& next = points[(index + 1) % points.size()];
        nx += (current.y - next.y) * (current.z + next.z);
        ny += (current.z - next.z) * (current.x + next.x);
        nz += (current.x - next.x) * (current.y + next.y);
    }

    return 0.5 * std::sqrt(nx * nx + ny * ny + nz * nz);
}

std::string default_body_name(const std::filesystem::path& sourcePath, std::size_t index) {
    const std::string stem = sourcePath.stem().string();
    if (index == 0 && !stem.empty()) {
        return stem;
    }
    return "Body " + std::to_string(index + 1);
}

std::string obj_safe_identifier(
    const std::string& name,
    const std::string& fallbackPrefix,
    std::size_t fallbackIndex) {
    const std::string trimmed = trim_copy(name);
    if (trimmed.empty()) {
        return fallbackPrefix + "_" + std::to_string(fallbackIndex + 1);
    }

    std::string safe;
    safe.reserve(trimmed.size());
    for (char ch : trimmed) {
        const unsigned char uch = static_cast<unsigned char>(ch);
        if (std::isalnum(uch) != 0 || ch == '_' || ch == '-' || ch == '.') {
            safe.push_back(ch);
        } else if (std::isspace(uch) != 0) {
            safe.push_back('_');
        }
    }

    if (safe.empty()) {
        return fallbackPrefix + "_" + std::to_string(fallbackIndex + 1);
    }
    return safe;
}

std::string obj_safe_name(const std::string& name, std::size_t fallbackIndex) {
    return obj_safe_identifier(name, "Body", fallbackIndex);
}

std::string obj_safe_material_name(const std::string& name, std::size_t fallbackIndex) {
    return obj_safe_identifier(name, "Material", fallbackIndex);
}

std::string point_key(const geometry::Point3D& point) {
    std::ostringstream out;
    out << std::setprecision(12) << point.x << ',' << point.y << ',' << point.z;
    return out.str();
}

struct EdgeKey {
    geometry::EntityId first = 0;
    geometry::EntityId second = 0;

    bool operator==(const EdgeKey& other) const noexcept {
        return first == other.first && second == other.second;
    }
};

struct EdgeKeyHash {
    std::size_t operator()(const EdgeKey& key) const noexcept {
        const std::size_t firstHash = std::hash<geometry::EntityId>{}(key.first);
        const std::size_t secondHash = std::hash<geometry::EntityId>{}(key.second);
        return firstHash ^ (secondHash << 1);
    }
};

EdgeKey make_edge_key(geometry::EntityId startId, geometry::EntityId endId) {
    if (startId <= endId) {
        return EdgeKey{startId, endId};
    }
    return EdgeKey{endId, startId};
}

void normalize_scene_document_topology(SceneDocument* document);

std::size_t append_unique_vertex(
    SceneBody* body,
    std::unordered_map<std::string, std::size_t>* vertexLookup,
    const geometry::Point3D& point) {
    if (!body || !vertexLookup) {
        throw std::invalid_argument("append_unique_vertex requires valid body storage");
    }

    const std::string key = point_key(point);
    const auto existing = vertexLookup->find(key);
    if (existing != vertexLookup->end()) {
        return existing->second;
    }

    const std::size_t index = body->vertices.size();
    body->vertices.push_back(point);
    vertexLookup->emplace(key, index);
    return index;
}

bool stl_file_has_binary_layout(const std::filesystem::path& sourcePath) {
    std::ifstream input(sourcePath, std::ios::binary | std::ios::ate);
    if (!input) {
        throw std::runtime_error("could not open STL file");
    }

    const std::streamoff fileSizeOff = input.tellg();
    if (fileSizeOff < 84) {
        return false;
    }

    const std::uint64_t fileSize = static_cast<std::uint64_t>(fileSizeOff);
    input.seekg(80, std::ios::beg);
    const std::uint32_t triangleCount = read_little_uint32(input, "binary STL triangle count is missing");
    const std::uint64_t expectedSize = 84ULL + static_cast<std::uint64_t>(triangleCount) * 50ULL;
    return expectedSize == fileSize;
}

bool try_parse_binary_stl_header_color(const std::array<char, 80>& header, SceneColor* color) {
    if (!color) {
        return false;
    }

    constexpr char kMarker[] = "COLOR=";
    const auto markerStart = std::search(
        header.begin(),
        header.end(),
        kMarker,
        kMarker + 6);
    if (markerStart == header.end()) {
        return false;
    }

    const std::size_t offset = static_cast<std::size_t>(std::distance(header.begin(), markerStart)) + 6U;
    if (offset + 4U > header.size()) {
        return false;
    }

    *color = make_scene_color(
        static_cast<unsigned char>(header[offset]) / 255.0,
        static_cast<unsigned char>(header[offset + 1U]) / 255.0,
        static_cast<unsigned char>(header[offset + 2U]) / 255.0,
        static_cast<unsigned char>(header[offset + 3U]) / 255.0);
    return true;
}

bool try_decode_binary_stl_attribute_color(std::uint16_t attribute, SceneColor* color) {
    if (!color || (attribute & 0x8000U) == 0U) {
        return false;
    }

    *color = make_scene_color(
        static_cast<double>((attribute >> 10U) & 0x1FU) / 31.0,
        static_cast<double>((attribute >> 5U) & 0x1FU) / 31.0,
        static_cast<double>(attribute & 0x1FU) / 31.0,
        1.0);
    return true;
}

SceneDocument import_binary_stl_document(const std::filesystem::path& sourcePath) {
    std::ifstream input(sourcePath, std::ios::binary);
    if (!input) {
        throw std::runtime_error("could not open STL file");
    }

    std::array<char, 80> header{};
    input.read(header.data(), static_cast<std::streamsize>(header.size()));
    if (!input) {
        throw std::runtime_error("binary STL header is incomplete");
    }

    const std::uint32_t triangleCount = read_little_uint32(input, "binary STL triangle count is missing");

    SceneDocument document;
    document.scene_label = sourcePath.filename().string();
    document.source_kind = SceneSourceKind::StlMesh;
    document.source_path = sourcePath;

    SceneBody body;
    body.name = default_body_name(sourcePath, 0);
    std::unordered_map<std::string, std::size_t> vertexLookup;
    SceneColor headerColor;
    const bool hasHeaderColor = try_parse_binary_stl_header_color(header, &headerColor);
    if (hasHeaderColor) {
        body.color = headerColor;
        body.has_color = true;
    }

    for (std::uint32_t triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex) {
        (void)read_little_float32(input, "binary STL facet normal is incomplete");
        (void)read_little_float32(input, "binary STL facet normal is incomplete");
        (void)read_little_float32(input, "binary STL facet normal is incomplete");

        std::vector<geometry::Point3D> points;
        points.reserve(3);
        SceneFace face;
        face.vertex_indices.reserve(3);
        for (std::size_t vertexIndex = 0; vertexIndex < 3; ++vertexIndex) {
            const geometry::Point3D point{
                static_cast<double>(read_little_float32(input, "binary STL vertex coordinate is incomplete")),
                static_cast<double>(read_little_float32(input, "binary STL vertex coordinate is incomplete")),
                static_cast<double>(read_little_float32(input, "binary STL vertex coordinate is incomplete"))};
            points.push_back(point);
            face.vertex_indices.push_back(append_unique_vertex(&body, &vertexLookup, point));
        }

        const std::uint16_t attribute = read_little_uint16(input, "binary STL attribute byte count is incomplete");
        SceneColor attributeColor;
        if (try_decode_binary_stl_attribute_color(attribute, &attributeColor)) {
            face.color = attributeColor;
            face.has_color = true;
        } else if (hasHeaderColor) {
            face.color = headerColor;
            face.has_color = true;
        }
        face.area_hint = polygon_area_hint(points);
        body.faces.push_back(std::move(face));
    }

    if (body.faces.empty()) {
        throw std::runtime_error("binary STL import found no triangle facets");
    }

    document.bodies.push_back(std::move(body));
    normalize_scene_document_topology(&document);
    return document;
}

void skip_binary_ply_property(
    std::istream& input,
    const PlyPropertySpec& property,
    bool littleEndian,
    const std::string& context) {
    if (!property.is_list) {
        (void)read_binary_numeric_scalar(input, property.scalar_type, littleEndian, context);
        return;
    }

    const std::size_t count = checked_count(
        read_binary_integer_scalar(input, property.list_count_type, littleEndian, context),
        context);
    for (std::size_t index = 0; index < count; ++index) {
        (void)read_binary_numeric_scalar(input, property.list_item_type, littleEndian, context);
    }
}

void skip_ascii_ply_property(
    std::istringstream& input,
    const PlyPropertySpec& property,
    const std::string& context) {
    if (!property.is_list) {
        (void)read_ascii_numeric_scalar(input, property.scalar_type, context);
        return;
    }

    const std::size_t count = checked_count(
        read_ascii_integer_scalar(input, property.list_count_type, context),
        context);
    for (std::size_t index = 0; index < count; ++index) {
        (void)read_ascii_numeric_scalar(input, property.list_item_type, context);
    }
}

struct PlyColorState {
    SceneColor color;
    bool has_red = false;
    bool has_green = false;
    bool has_blue = false;
    bool has_alpha = false;

    bool has_rgb() const noexcept {
        return has_red && has_green && has_blue;
    }
};

double normalize_ply_color_component(double value, PlyScalarType type) noexcept {
    if (ply_type_is_integer(type)) {
        return clamp01(value / 255.0);
    }
    if (value > 1.0) {
        return clamp01(value / 255.0);
    }
    return clamp01(value);
}

bool apply_ply_color_property(
    PlyColorState* state,
    const PlyPropertySpec& property,
    double value) {
    if (!state || property.is_list) {
        return false;
    }

    const std::string name = lowercase_copy(property.name);
    const double normalized = normalize_ply_color_component(value, property.scalar_type);
    if (name == "red" || name == "diffuse_red" || name == "r") {
        state->color.red = normalized;
        state->has_red = true;
        return true;
    }
    if (name == "green" || name == "diffuse_green" || name == "g") {
        state->color.green = normalized;
        state->has_green = true;
        return true;
    }
    if (name == "blue" || name == "diffuse_blue" || name == "b") {
        state->color.blue = normalized;
        state->has_blue = true;
        return true;
    }
    if (name == "alpha" || name == "diffuse_alpha" || name == "a") {
        state->color.alpha = normalized;
        state->has_alpha = true;
        return true;
    }

    return false;
}

void append_ply_vertex_color(
    std::vector<SceneColor>* vertexColors,
    std::vector<bool>* vertexHasColors,
    const PlyColorState& colorState) {
    if (!vertexColors || !vertexHasColors) {
        return;
    }

    if (colorState.has_rgb()) {
        SceneColor color = colorState.color;
        if (!colorState.has_alpha) {
            color.alpha = 1.0;
        }
        vertexColors->push_back(color);
        vertexHasColors->push_back(true);
    } else {
        vertexColors->push_back(SceneColor{});
        vertexHasColors->push_back(false);
    }
}

bool try_average_face_vertex_colors(
    const SceneFace& face,
    const std::vector<SceneColor>& vertexColors,
    const std::vector<bool>& vertexHasColors,
    SceneColor* color) {
    if (!color || face.vertex_indices.empty() || vertexColors.empty()) {
        return false;
    }

    SceneColor sum{};
    for (std::size_t vertexIndex : face.vertex_indices) {
        if (vertexIndex >= vertexColors.size() ||
            vertexIndex >= vertexHasColors.size() ||
            !vertexHasColors[vertexIndex]) {
            return false;
        }

        const SceneColor& vertexColor = vertexColors[vertexIndex];
        sum.red += vertexColor.red;
        sum.green += vertexColor.green;
        sum.blue += vertexColor.blue;
        sum.alpha += vertexColor.alpha;
    }

    const double invCount = 1.0 / static_cast<double>(face.vertex_indices.size());
    *color = make_scene_color(
        sum.red * invCount,
        sum.green * invCount,
        sum.blue * invCount,
        sum.alpha * invCount);
    return true;
}

void read_ascii_ply_vertex(
    std::istream& input,
    const PlyElementSpec& element,
    SceneBody* body,
    std::vector<SceneColor>* vertexColors,
    std::vector<bool>* vertexHasColors) {
    std::string line;
    if (!std::getline(input, line)) {
        throw std::runtime_error("PLY file ended before all vertices were read");
    }

    std::istringstream row(trim_copy(line));
    geometry::Point3D point{};
    bool hasX = false;
    bool hasY = false;
    bool hasZ = false;
    PlyColorState colorState;
    const std::string context = "PLY vertex row is malformed";

    for (const PlyPropertySpec& property : element.properties) {
        if (property.is_list) {
            skip_ascii_ply_property(row, property, context);
            continue;
        }

        const double value = read_ascii_numeric_scalar(row, property.scalar_type, context);
        if (property.name == "x") {
            point.x = value;
            hasX = true;
        } else if (property.name == "y") {
            point.y = value;
            hasY = true;
        } else if (property.name == "z") {
            point.z = value;
            hasZ = true;
        } else {
            (void)apply_ply_color_property(&colorState, property, value);
        }
    }

    if (!hasX || !hasY || !hasZ) {
        throw std::runtime_error("PLY vertex element must include x, y, and z properties");
    }
    body->vertices.push_back(point);
    append_ply_vertex_color(vertexColors, vertexHasColors, colorState);
}

void read_binary_ply_vertex(
    std::istream& input,
    const PlyElementSpec& element,
    bool littleEndian,
    SceneBody* body,
    std::vector<SceneColor>* vertexColors,
    std::vector<bool>* vertexHasColors) {
    geometry::Point3D point{};
    bool hasX = false;
    bool hasY = false;
    bool hasZ = false;
    PlyColorState colorState;
    const std::string context = "binary PLY vertex row is incomplete";

    for (const PlyPropertySpec& property : element.properties) {
        if (property.is_list) {
            skip_binary_ply_property(input, property, littleEndian, context);
            continue;
        }

        const double value = read_binary_numeric_scalar(input, property.scalar_type, littleEndian, context);
        if (property.name == "x") {
            point.x = value;
            hasX = true;
        } else if (property.name == "y") {
            point.y = value;
            hasY = true;
        } else if (property.name == "z") {
            point.z = value;
            hasZ = true;
        } else {
            (void)apply_ply_color_property(&colorState, property, value);
        }
    }

    if (!hasX || !hasY || !hasZ) {
        throw std::runtime_error("PLY vertex element must include x, y, and z properties");
    }
    body->vertices.push_back(point);
    append_ply_vertex_color(vertexColors, vertexHasColors, colorState);
}

void append_checked_face_index(SceneFace* face, std::int64_t rawIndex, std::size_t vertexCount) {
    if (rawIndex < 0 || static_cast<std::uint64_t>(rawIndex) >= static_cast<std::uint64_t>(vertexCount)) {
        throw std::runtime_error("PLY face references an invalid vertex index");
    }
    face->vertex_indices.push_back(static_cast<std::size_t>(rawIndex));
}

void read_ascii_ply_face(
    std::istream& input,
    const PlyElementSpec& element,
    SceneBody* body,
    const std::vector<SceneColor>& vertexColors,
    const std::vector<bool>& vertexHasColors) {
    std::string line;
    if (!std::getline(input, line)) {
        throw std::runtime_error("PLY file ended before all faces were read");
    }

    std::istringstream row(trim_copy(line));
    SceneFace face;
    bool sawVertexIndices = false;
    PlyColorState colorState;
    const std::string context = "PLY face row is malformed";

    for (const PlyPropertySpec& property : element.properties) {
        if (!property.is_list) {
            const double value = read_ascii_numeric_scalar(row, property.scalar_type, context);
            (void)apply_ply_color_property(&colorState, property, value);
            continue;
        }

        const std::size_t count = checked_count(
            read_ascii_integer_scalar(row, property.list_count_type, context),
            context);
        if (is_vertex_index_property(property.name)) {
            face.vertex_indices.reserve(count);
            for (std::size_t index = 0; index < count; ++index) {
                append_checked_face_index(
                    &face,
                    read_ascii_integer_scalar(row, property.list_item_type, context),
                    body->vertices.size());
            }
            sawVertexIndices = true;
            continue;
        }

        for (std::size_t index = 0; index < count; ++index) {
            (void)read_ascii_numeric_scalar(row, property.list_item_type, context);
        }
    }

    if (!sawVertexIndices) {
        throw std::runtime_error("PLY face element is missing a vertex_indices list property");
    }
    if (face.vertex_indices.size() < 3) {
        throw std::runtime_error("PLY face line must contain at least three vertex indices");
    }
    if (colorState.has_rgb()) {
        face.color = colorState.color;
        if (!colorState.has_alpha) {
            face.color.alpha = 1.0;
        }
        face.has_color = true;
    } else {
        SceneColor averagedColor;
        if (try_average_face_vertex_colors(face, vertexColors, vertexHasColors, &averagedColor)) {
            face.color = averagedColor;
            face.has_color = true;
        }
    }
    body->faces.push_back(std::move(face));
}

void read_binary_ply_face(
    std::istream& input,
    const PlyElementSpec& element,
    bool littleEndian,
    SceneBody* body,
    const std::vector<SceneColor>& vertexColors,
    const std::vector<bool>& vertexHasColors) {
    SceneFace face;
    bool sawVertexIndices = false;
    PlyColorState colorState;
    const std::string context = "binary PLY face row is incomplete";

    for (const PlyPropertySpec& property : element.properties) {
        if (!property.is_list) {
            const double value = read_binary_numeric_scalar(input, property.scalar_type, littleEndian, context);
            (void)apply_ply_color_property(&colorState, property, value);
            continue;
        }

        const std::size_t count = checked_count(
            read_binary_integer_scalar(input, property.list_count_type, littleEndian, context),
            context);
        if (is_vertex_index_property(property.name)) {
            face.vertex_indices.reserve(count);
            for (std::size_t index = 0; index < count; ++index) {
                append_checked_face_index(
                    &face,
                    read_binary_integer_scalar(input, property.list_item_type, littleEndian, context),
                    body->vertices.size());
            }
            sawVertexIndices = true;
            continue;
        }

        for (std::size_t index = 0; index < count; ++index) {
            (void)read_binary_numeric_scalar(input, property.list_item_type, littleEndian, context);
        }
    }

    if (!sawVertexIndices) {
        throw std::runtime_error("PLY face element is missing a vertex_indices list property");
    }
    if (face.vertex_indices.size() < 3) {
        throw std::runtime_error("PLY face line must contain at least three vertex indices");
    }
    if (colorState.has_rgb()) {
        face.color = colorState.color;
        if (!colorState.has_alpha) {
            face.color.alpha = 1.0;
        }
        face.has_color = true;
    } else {
        SceneColor averagedColor;
        if (try_average_face_vertex_colors(face, vertexColors, vertexHasColors, &averagedColor)) {
            face.color = averagedColor;
            face.has_color = true;
        }
    }
    body->faces.push_back(std::move(face));
}

void skip_ascii_ply_element_row(std::istream& input) {
    std::string ignored;
    if (!std::getline(input, ignored)) {
        throw std::runtime_error("PLY file ended before all element rows were read");
    }
}

void skip_binary_ply_element_row(
    std::istream& input,
    const PlyElementSpec& element,
    bool littleEndian) {
    const std::string context = "binary PLY element row is incomplete";
    for (const PlyPropertySpec& property : element.properties) {
        skip_binary_ply_property(input, property, littleEndian, context);
    }
}

std::size_t parse_obj_vertex_index(const std::string& token, std::size_t vertexCount) {
    const std::size_t slashPos = token.find('/');
    const std::string vertexToken = slashPos == std::string::npos ? token : token.substr(0, slashPos);
    if (vertexToken.empty()) {
        throw std::runtime_error("OBJ face token is missing a vertex index");
    }

    std::size_t consumed = 0;
    const long long rawIndex = std::stoll(vertexToken, &consumed);
    if (consumed != vertexToken.size()) {
        throw std::runtime_error("OBJ face token contains an invalid vertex index");
    }

    long long resolvedIndex = rawIndex;
    if (resolvedIndex < 0) {
        resolvedIndex = static_cast<long long>(vertexCount) + resolvedIndex + 1;
    }
    if (resolvedIndex <= 0 || resolvedIndex > static_cast<long long>(vertexCount)) {
        throw std::runtime_error("OBJ face references a vertex index outside the available range");
    }

    return static_cast<std::size_t>(resolvedIndex - 1);
}

std::string rest_after_keyword(const std::string& trimmedLine, const std::string& keyword) {
    if (trimmedLine.size() <= keyword.size()) {
        return {};
    }
    return trim_copy(trimmedLine.substr(keyword.size()));
}

void import_obj_material_library(
    const std::filesystem::path& sourcePath,
    const std::string& libraryName,
    std::vector<SceneMaterial>* materials) {
    if (!materials) {
        return;
    }

    const std::filesystem::path materialPath = sourcePath.parent_path() / trim_copy(libraryName);
    std::ifstream input(materialPath, std::ios::binary);
    if (!input) {
        return;
    }

    SceneMaterial currentMaterial;
    bool hasCurrentMaterial = false;
    auto flush_current_material = [&]() {
        if (hasCurrentMaterial && !trim_copy(currentMaterial.name).empty()) {
            upsert_scene_material(materials, currentMaterial);
        }
    };

    std::string line;
    while (std::getline(input, line)) {
        const std::size_t commentPos = line.find('#');
        if (commentPos != std::string::npos) {
            line.erase(commentPos);
        }

        const std::string trimmed = trim_copy(line);
        if (trimmed.empty()) {
            continue;
        }

        std::istringstream lineStream(trimmed);
        std::string keyword;
        lineStream >> keyword;
        keyword = lowercase_copy(keyword);

        if (keyword == "newmtl") {
            flush_current_material();
            currentMaterial = SceneMaterial{};
            currentMaterial.name = rest_after_keyword(trimmed, "newmtl");
            hasCurrentMaterial = true;
            continue;
        }

        if (!hasCurrentMaterial) {
            continue;
        }

        if (keyword == "kd") {
            double red = 0.0;
            double green = 0.0;
            double blue = 0.0;
            if (lineStream >> red >> green >> blue) {
                const double alpha = currentMaterial.has_diffuse_color
                    ? currentMaterial.diffuse_color.alpha
                    : 1.0;
                currentMaterial.diffuse_color = make_scene_color(red, green, blue, alpha);
                currentMaterial.has_diffuse_color = true;
            }
            continue;
        }

        if (keyword == "d" || keyword == "tr") {
            double value = 1.0;
            if (lineStream >> value) {
                if (!currentMaterial.has_diffuse_color) {
                    currentMaterial.diffuse_color = make_scene_color(0.8, 0.8, 0.8, 1.0);
                    currentMaterial.has_diffuse_color = true;
                }
                currentMaterial.diffuse_color.alpha = keyword == "tr" ? clamp01(1.0 - value) : clamp01(value);
            }
        }
    }

    flush_current_material();
}

struct WorkingBody {
    SceneBody body;
    std::unordered_map<std::size_t, std::size_t> global_to_local_vertex;
};

WorkingBody& ensure_working_body(
    std::vector<WorkingBody>* bodies,
    const std::filesystem::path& sourcePath,
    std::size_t index,
    const std::string& requestedName = "") {
    if (!bodies) {
        throw std::invalid_argument("ensure_working_body requires a target body container");
    }

    if (index >= bodies->size()) {
        bodies->resize(index + 1);
    }

    WorkingBody& body = (*bodies)[index];
    if (body.body.name.empty()) {
        body.body.name = requestedName.empty() ? default_body_name(sourcePath, index) : requestedName;
    } else if (!requestedName.empty() && body.body.faces.empty() && body.body.vertices.empty()) {
        body.body.name = requestedName;
    }

    return body;
}

struct FaceEdgeUse {
    std::size_t face_index = 0;
    bool forward = true;
};

void normalize_scene_body_topology(SceneBody* body) {
    if (!body) {
        throw std::invalid_argument("normalize_scene_body_topology requires a valid body");
    }

    std::unordered_map<EdgeKey, std::vector<FaceEdgeUse>, EdgeKeyHash> edgeUses;
    edgeUses.reserve(body->faces.size() * 3);

    for (std::size_t faceIndex = 0; faceIndex < body->faces.size(); ++faceIndex) {
        const SceneFace& face = body->faces[faceIndex];
        if (face.vertex_indices.size() < 3) {
            throw std::runtime_error("scene face must reference at least three vertices");
        }

        for (std::size_t edgeIndex = 0; edgeIndex < face.vertex_indices.size(); ++edgeIndex) {
            const std::size_t startIndex = face.vertex_indices[edgeIndex];
            const std::size_t endIndex = face.vertex_indices[(edgeIndex + 1) % face.vertex_indices.size()];
            if (startIndex >= body->vertices.size() || endIndex >= body->vertices.size()) {
                throw std::runtime_error("scene face references a vertex index outside the body vertex list");
            }
            if (startIndex == endIndex) {
                throw std::runtime_error("scene face contains a zero-length edge");
            }

            const EdgeKey key = make_edge_key(
                static_cast<geometry::EntityId>(startIndex),
                static_cast<geometry::EntityId>(endIndex));
            std::vector<FaceEdgeUse>& uses = edgeUses[key];
            uses.push_back(FaceEdgeUse{faceIndex, key.first == startIndex && key.second == endIndex});
            if (uses.size() > 2) {
                throw std::runtime_error("non-manifold scene body edge is referenced by more than two faces");
            }
        }
    }

    std::vector<std::vector<std::pair<std::size_t, bool>>> adjacency(body->faces.size());
    for (const auto& entry : edgeUses) {
        const std::vector<FaceEdgeUse>& uses = entry.second;
        if (uses.size() != 2 || uses[0].face_index == uses[1].face_index) {
            continue;
        }

        const bool neighborMustFlipDifferently = uses[0].forward == uses[1].forward;
        adjacency[uses[0].face_index].push_back(std::make_pair(uses[1].face_index, neighborMustFlipDifferently));
        adjacency[uses[1].face_index].push_back(std::make_pair(uses[0].face_index, neighborMustFlipDifferently));
    }

    std::vector<bool> assigned(body->faces.size(), false);
    std::vector<bool> shouldFlip(body->faces.size(), false);
    for (std::size_t seedFace = 0; seedFace < body->faces.size(); ++seedFace) {
        if (assigned[seedFace]) {
            continue;
        }

        std::deque<std::size_t> queue;
        assigned[seedFace] = true;
        queue.push_back(seedFace);

        while (!queue.empty()) {
            const std::size_t faceIndex = queue.front();
            queue.pop_front();

            for (const auto& neighbor : adjacency[faceIndex]) {
                const std::size_t neighborFace = neighbor.first;
                const bool requiredFlip = shouldFlip[faceIndex] != neighbor.second;
                if (!assigned[neighborFace]) {
                    assigned[neighborFace] = true;
                    shouldFlip[neighborFace] = requiredFlip;
                    queue.push_back(neighborFace);
                    continue;
                }
                if (shouldFlip[neighborFace] != requiredFlip) {
                    throw std::runtime_error("inconsistent scene face winding around a shared edge");
                }
            }
        }
    }

    for (std::size_t faceIndex = 0; faceIndex < body->faces.size(); ++faceIndex) {
        if (shouldFlip[faceIndex]) {
            std::reverse(body->faces[faceIndex].vertex_indices.begin(), body->faces[faceIndex].vertex_indices.end());
        }
    }
}

void normalize_scene_document_topology(SceneDocument* document) {
    if (!document) {
        throw std::invalid_argument("normalize_scene_document_topology requires a valid document");
    }

    for (SceneBody& body : document->bodies) {
        normalize_scene_body_topology(&body);
        derive_body_metadata_from_faces(&body);
    }
}

geometry::Edge orient_edge_for_scene_loop(
    const geometry::Edge& source,
    geometry::EntityId requestedStartId,
    geometry::EntityId requestedEndId) {
    if (source.start_vertex_id == requestedStartId && source.end_vertex_id == requestedEndId) {
        return source;
    }
    if (source.start_vertex_id != requestedEndId || source.end_vertex_id != requestedStartId) {
        throw std::runtime_error("cached scene edge does not match the requested face-loop vertices");
    }

    geometry::Edge oriented = source;
    std::swap(oriented.start_vertex_id, oriented.end_vertex_id);

#ifdef ADAPTIVECAD_HAS_OPENCASCADE
    if (source.native_handle.backend == geometry::NativeTopologyBackend::OpenCascade &&
        source.native_handle.is_valid() && source.native_handle.type_name == "TopoDS_Edge") {
        const std::shared_ptr<TopoDS_Edge> occEdge = std::static_pointer_cast<TopoDS_Edge>(source.native_handle.handle);
        if (occEdge) {
            const TopoDS_Edge reversedOccEdge = TopoDS::Edge(occEdge->Reversed());
            oriented.native_handle.backend = geometry::NativeTopologyBackend::OpenCascade;
            oriented.native_handle.type_name = "TopoDS_Edge";
            oriented.native_handle.handle = std::static_pointer_cast<void>(std::make_shared<TopoDS_Edge>(reversedOccEdge));
        }
    }
#endif

    return oriented;
}

} // namespace

const char* scene_source_kind_name(SceneSourceKind kind) noexcept {
    switch (kind) {
    case SceneSourceKind::Demo:
        return "demo";
    case SceneSourceKind::WavefrontObj:
        return "wavefront_obj";
    case SceneSourceKind::StlMesh:
        return "stl_mesh";
    case SceneSourceKind::PlyMesh:
        return "ply_mesh";
    case SceneSourceKind::StepModel:
        return "step_model";
    case SceneSourceKind::GeneratedScaffold:
        return "generated_scaffold";
    case SceneSourceKind::GeneratedPrimitive:
        return "generated_primitive";
    case SceneSourceKind::SessionEmbedded:
        return "session_embedded";
    }

    return "demo";
}

SceneSourceKind parse_scene_source_kind(const std::string& value) {
    const std::string normalized = lowercase_copy(trim_copy(value));
    if (normalized == "wavefront_obj" || normalized == "obj" || normalized == "wavefront") {
        return SceneSourceKind::WavefrontObj;
    }
    if (normalized == "stl_mesh" || normalized == "stl") {
        return SceneSourceKind::StlMesh;
    }
    if (normalized == "ply_mesh" || normalized == "ply") {
        return SceneSourceKind::PlyMesh;
    }
    if (normalized == "step_model" || normalized == "step" || normalized == "stp") {
        return SceneSourceKind::StepModel;
    }
    if (normalized == "generated_scaffold" || normalized == "scaffold" || normalized == "metamaterial") {
        return SceneSourceKind::GeneratedScaffold;
    }
    if (normalized == "generated_primitive" || normalized == "primitive") {
        return SceneSourceKind::GeneratedPrimitive;
    }
    if (normalized == "session_embedded" || normalized == "session") {
        return SceneSourceKind::SessionEmbedded;
    }
    return SceneSourceKind::Demo;
}

SceneDocument make_demo_scene_document() {
    SceneDocument document;
    document.scene_label = "Demo triangle scene";
    document.source_kind = SceneSourceKind::Demo;
    document.angular_model.lambda0 = 1.10;
    document.angular_model.cosine_coefficients = {0.08};
    document.angular_model.sine_coefficients = {0.02};

    SceneBody body;
    body.name = "Demo Body";
    body.vertices = {
        geometry::Point3D{0.0, 0.0, 0.0},
        geometry::Point3D{1.0, 0.0, 0.0},
        geometry::Point3D{0.0, 1.0, 0.0}};
    body.faces.push_back(SceneFace{{0, 1, 2}, 0.5});
    document.bodies.push_back(std::move(body));
    return document;
}

SceneBody make_box_body(
    const std::string& name,
    double minX,
    double maxX,
    double minY,
    double maxY,
    double minZ,
    double maxZ) {
    SceneBody body;
    body.name = name;
    body.vertices = {
        geometry::Point3D{minX, minY, minZ},
        geometry::Point3D{maxX, minY, minZ},
        geometry::Point3D{maxX, maxY, minZ},
        geometry::Point3D{minX, maxY, minZ},
        geometry::Point3D{minX, minY, maxZ},
        geometry::Point3D{maxX, minY, maxZ},
        geometry::Point3D{maxX, maxY, maxZ},
        geometry::Point3D{minX, maxY, maxZ}};
    body.faces = {
        SceneFace{{0, 3, 2, 1}},
        SceneFace{{4, 5, 6, 7}},
        SceneFace{{0, 4, 7, 3}},
        SceneFace{{1, 2, 6, 5}},
        SceneFace{{0, 1, 5, 4}},
        SceneFace{{3, 7, 6, 2}}};
    body.volume_hint = std::max(0.0, maxX - minX) *
        std::max(0.0, maxY - minY) *
        std::max(0.0, maxZ - minZ);
    return body;
}

SceneDocument make_box_scene_document(double width, double depth, double height) {
    if (width <= 0.0 || depth <= 0.0 || height <= 0.0) {
        throw std::invalid_argument("box primitive dimensions must be positive");
    }

    SceneDocument document;
    document.scene_label = "Box Primitive";
    document.source_kind = SceneSourceKind::GeneratedPrimitive;
    document.bodies.push_back(make_box_body(
        "Box",
        -0.5 * width,
        0.5 * width,
        -0.5 * depth,
        0.5 * depth,
        -0.5 * height,
        0.5 * height));
    return document;
}

SceneDocument make_plane_scene_document(double width, double depth) {
    if (width <= 0.0 || depth <= 0.0) {
        throw std::invalid_argument("plane primitive dimensions must be positive");
    }

    const double halfWidth = 0.5 * width;
    const double halfDepth = 0.5 * depth;

    SceneDocument document;
    document.scene_label = "Plane Primitive";
    document.source_kind = SceneSourceKind::GeneratedPrimitive;

    SceneBody body;
    body.name = "Plane";
    body.vertices = {
        geometry::Point3D{-halfWidth, -halfDepth, 0.0},
        geometry::Point3D{halfWidth, -halfDepth, 0.0},
        geometry::Point3D{halfWidth, halfDepth, 0.0},
        geometry::Point3D{-halfWidth, halfDepth, 0.0}};
    body.faces = {SceneFace{{0, 1, 2, 3}, width * depth}};
    document.bodies.push_back(std::move(body));
    return document;
}

SceneDocument make_wedge_scene_document(double width, double depth, double height) {
    if (width <= 0.0 || depth <= 0.0 || height <= 0.0) {
        throw std::invalid_argument("wedge primitive dimensions must be positive");
    }

    const double halfWidth = 0.5 * width;
    const double halfDepth = 0.5 * depth;

    SceneDocument document;
    document.scene_label = "Wedge Primitive";
    document.source_kind = SceneSourceKind::GeneratedPrimitive;

    SceneBody body;
    body.name = "Wedge";
    body.vertices = {
        geometry::Point3D{-halfWidth, -halfDepth, 0.0},
        geometry::Point3D{halfWidth, -halfDepth, 0.0},
        geometry::Point3D{-halfWidth, -halfDepth, height},
        geometry::Point3D{-halfWidth, halfDepth, 0.0},
        geometry::Point3D{halfWidth, halfDepth, 0.0},
        geometry::Point3D{-halfWidth, halfDepth, height}};
    body.faces = {
        SceneFace{{0, 3, 4, 1}},
        SceneFace{{0, 2, 5, 3}},
        SceneFace{{2, 1, 4, 5}},
        SceneFace{{0, 1, 2}},
        SceneFace{{3, 5, 4}}};
    body.volume_hint = 0.5 * width * depth * height;
    document.bodies.push_back(std::move(body));
    return document;
}

SceneDocument make_torus_scene_document(
    double major_radius,
    double minor_radius,
    std::size_t major_segments,
    std::size_t minor_segments) {
    if (major_radius <= 0.0 || minor_radius <= 0.0) {
        throw std::invalid_argument("torus primitive radii must be positive");
    }
    if (minor_radius >= major_radius) {
        throw std::invalid_argument("torus primitive minor radius must be smaller than major radius");
    }
    if (major_segments < 3 || minor_segments < 3) {
        throw std::invalid_argument("torus primitive requires at least 3 major and minor segments");
    }

    SceneDocument document;
    document.scene_label = "Torus Primitive";
    document.source_kind = SceneSourceKind::GeneratedPrimitive;

    SceneBody body;
    body.name = "Torus";
    body.vertices.reserve(major_segments * minor_segments);
    for (std::size_t majorIndex = 0; majorIndex < major_segments; ++majorIndex) {
        const double u = 2.0 * std::numbers::pi_v<double> *
            static_cast<double>(majorIndex) / static_cast<double>(major_segments);
        for (std::size_t minorIndex = 0; minorIndex < minor_segments; ++minorIndex) {
            const double v = 2.0 * std::numbers::pi_v<double> *
                static_cast<double>(minorIndex) / static_cast<double>(minor_segments);
            const double radial = major_radius + minor_radius * std::cos(v);
            body.vertices.push_back(geometry::Point3D{
                radial * std::cos(u),
                radial * std::sin(u),
                minor_radius * std::sin(v)});
        }
    }

    auto vertex_index = [minor_segments](std::size_t majorIndex, std::size_t minorIndex) {
        return majorIndex * minor_segments + minorIndex;
    };

    body.faces.reserve(major_segments * minor_segments);
    for (std::size_t majorIndex = 0; majorIndex < major_segments; ++majorIndex) {
        const std::size_t nextMajor = (majorIndex + 1) % major_segments;
        for (std::size_t minorIndex = 0; minorIndex < minor_segments; ++minorIndex) {
            const std::size_t nextMinor = (minorIndex + 1) % minor_segments;
            body.faces.push_back(SceneFace{{
                vertex_index(majorIndex, minorIndex),
                vertex_index(nextMajor, minorIndex),
                vertex_index(nextMajor, nextMinor),
                vertex_index(majorIndex, nextMinor)}});
        }
    }
    body.volume_hint = 2.0 * std::numbers::pi_v<double> * std::numbers::pi_v<double> *
        major_radius * minor_radius * minor_radius;

    document.bodies.push_back(std::move(body));
    normalize_scene_document_topology(&document);
    return document;
}

BuiltScene build_scene_geometry(
    const SceneDocument& document,
    geometry::TopologyHealingPolicy policy,
    bool preferOpenCascade) {
    if (document.bodies.empty()) {
        throw std::runtime_error("scene document contains no bodies to build");
    }

    geometry::BRepKernel kernel = geometry::BRepKernel::create_preferred(preferOpenCascade);
    kernel.set_healing_policy(policy);

    BuiltScene scene;
    scene.backend_name = kernel.backend_name();
    scene.has_open_cascade_topology = kernel.has_open_cascade_topology();

    for (const auto& sourceBody : document.bodies) {
        SceneBody normalizedBody = sourceBody;
        normalize_scene_body_topology(&normalizedBody);

        if (normalizedBody.vertices.empty()) {
            throw std::runtime_error("scene body contains no vertices");
        }
        if (normalizedBody.faces.empty()) {
            throw std::runtime_error("scene body contains no faces");
        }

        std::vector<geometry::Vertex> builtVertices;
        builtVertices.reserve(normalizedBody.vertices.size());
        for (const auto& point : normalizedBody.vertices) {
            geometry::Vertex vertex = kernel.create_vertex(point.x, point.y, point.z);
            builtVertices.push_back(vertex);
            scene.vertices.push_back(vertex);
        }

        std::unordered_map<EdgeKey, geometry::Edge, EdgeKeyHash> builtEdgeLookup;
        builtEdgeLookup.reserve(normalizedBody.faces.size() * 3);

        std::vector<geometry::Face> builtFaces;
        builtFaces.reserve(normalizedBody.faces.size());
        for (const auto& sourceFace : normalizedBody.faces) {
            if (sourceFace.vertex_indices.size() < 3) {
                throw std::runtime_error("scene face must reference at least three vertices");
            }

            std::vector<geometry::Edge> builtEdges;
            std::vector<geometry::Point3D> outline;
            builtEdges.reserve(sourceFace.vertex_indices.size());
            outline.reserve(sourceFace.vertex_indices.size());

            for (std::size_t index = 0; index < sourceFace.vertex_indices.size(); ++index) {
                const std::size_t startIndex = sourceFace.vertex_indices[index];
                const std::size_t endIndex = sourceFace.vertex_indices[(index + 1) % sourceFace.vertex_indices.size()];
                if (startIndex >= builtVertices.size() || endIndex >= builtVertices.size()) {
                    throw std::runtime_error("scene face references a vertex index outside the body vertex list");
                }

                const geometry::Vertex& start = builtVertices[startIndex];
                const geometry::Vertex& end = builtVertices[endIndex];
                outline.push_back(start.point);

                const EdgeKey edgeKey = make_edge_key(start.id, end.id);
                const auto existingEdge = builtEdgeLookup.find(edgeKey);
                geometry::Edge edge;
                if (existingEdge == builtEdgeLookup.end()) {
                    edge = kernel.create_edge(start, end);
                    builtEdgeLookup.emplace(edgeKey, edge);
                    scene.edges.push_back(edge);
                } else {
                    edge = orient_edge_for_scene_loop(existingEdge->second, start.id, end.id);
                }

                builtEdges.push_back(edge);
            }

            const double areaHint = sourceFace.area_hint > 0.0 ? sourceFace.area_hint : polygon_area_hint(outline);
            geometry::Face face = kernel.create_face_from_edges(builtEdges, areaHint);
            builtFaces.push_back(face);
            scene.faces.push_back(face);
            SceneFaceOutline faceOutline;
            faceOutline.face_id = face.id;
            faceOutline.points = std::move(outline);
            faceOutline.material_name = sourceFace.material_name.empty()
                ? normalizedBody.material_name
                : sourceFace.material_name;
            if (sourceFace.has_color) {
                faceOutline.color = sourceFace.color;
                faceOutline.has_color = true;
            } else if (normalizedBody.has_color) {
                faceOutline.color = normalizedBody.color;
                faceOutline.has_color = true;
            }
            scene.face_outlines.push_back(std::move(faceOutline));
        }

        geometry::Body body = kernel.create_body_from_faces(builtFaces, normalizedBody.volume_hint);
        scene.bodies.push_back(body);
        SceneBodyInfo bodyInfo;
        bodyInfo.body_id = body.id;
        bodyInfo.name = normalizedBody.name;
        bodyInfo.material_name = normalizedBody.material_name;
        bodyInfo.color = normalizedBody.color;
        bodyInfo.has_color = normalizedBody.has_color;
        scene.body_infos.push_back(std::move(bodyInfo));
    }

    scene.diagnostic = kernel.last_topology_diagnostic();
    return scene;
}

SceneDocument import_wavefront_obj(const std::filesystem::path& sourcePath) {
    std::ifstream input(sourcePath, std::ios::binary);
    if (!input) {
        throw std::runtime_error("could not open OBJ file");
    }

    SceneDocument document;
    document.scene_label = sourcePath.filename().string();
    document.source_kind = SceneSourceKind::WavefrontObj;
    document.source_path = sourcePath;

    std::vector<geometry::Point3D> globalVertices;
    std::vector<WorkingBody> workingBodies;
    std::size_t currentBodyIndex = 0;
    ensure_working_body(&workingBodies, sourcePath, currentBodyIndex);
    std::string currentMaterialName;

    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        const std::size_t commentPos = line.find('#');
        if (commentPos != std::string::npos) {
            line.erase(commentPos);
        }
        const std::string trimmed = trim_copy(line);
        if (trimmed.empty()) {
            continue;
        }

        std::istringstream lineStream(trimmed);
        std::string keyword;
        lineStream >> keyword;
        const std::string normalizedKeyword = lowercase_copy(keyword);
        if (normalizedKeyword == "mtllib") {
            std::string libraryName;
            while (lineStream >> libraryName) {
                import_obj_material_library(sourcePath, libraryName, &document.materials);
            }
            continue;
        }

        if (normalizedKeyword == "usemtl") {
            currentMaterialName = rest_after_keyword(trimmed, keyword);
            continue;
        }

        if (normalizedKeyword == "v") {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            if (!(lineStream >> x >> y >> z)) {
                throw std::runtime_error("OBJ vertex line is missing x/y/z coordinates at line " + std::to_string(lineNumber));
            }
            globalVertices.push_back(geometry::Point3D{x, y, z});
            continue;
        }

        if (normalizedKeyword == "o" || normalizedKeyword == "g") {
            const std::string requestedName = trim_copy(trimmed.substr(keyword.size()));
            if (!workingBodies[currentBodyIndex].body.faces.empty() ||
                !workingBodies[currentBodyIndex].body.vertices.empty()) {
                currentBodyIndex = workingBodies.size();
            }
            ensure_working_body(&workingBodies, sourcePath, currentBodyIndex, requestedName);
            continue;
        }

        if (normalizedKeyword == "f") {
            WorkingBody& body = ensure_working_body(&workingBodies, sourcePath, currentBodyIndex);

            SceneFace face;
            std::string token;
            while (lineStream >> token) {
                const std::size_t globalIndex = parse_obj_vertex_index(token, globalVertices.size());
                auto mapping = body.global_to_local_vertex.find(globalIndex);
                if (mapping == body.global_to_local_vertex.end()) {
                    const std::size_t localIndex = body.body.vertices.size();
                    body.global_to_local_vertex.emplace(globalIndex, localIndex);
                    body.body.vertices.push_back(globalVertices[globalIndex]);
                    face.vertex_indices.push_back(localIndex);
                } else {
                    face.vertex_indices.push_back(mapping->second);
                }
            }

            if (face.vertex_indices.size() < 3) {
                throw std::runtime_error("OBJ face must reference at least three vertices at line " + std::to_string(lineNumber));
            }

            apply_material_to_face(&face, document.materials, currentMaterialName);
            body.body.faces.push_back(std::move(face));
            continue;
        }
    }

    for (auto& workingBody : workingBodies) {
        if (!workingBody.body.faces.empty()) {
            if (workingBody.body.name.empty()) {
                workingBody.body.name = default_body_name(sourcePath, document.bodies.size());
            }
            for (SceneFace& face : workingBody.body.faces) {
                if (!face.material_name.empty() && !face.has_color) {
                    apply_material_to_face(&face, document.materials, face.material_name);
                }
            }
            derive_body_metadata_from_faces(&workingBody.body);
            document.bodies.push_back(std::move(workingBody.body));
        }
    }

    if (document.bodies.empty()) {
        throw std::runtime_error("OBJ import found no polygon faces to build");
    }

    normalize_scene_document_topology(&document);
    return document;
}

SceneDocument import_ascii_stl(const std::filesystem::path& sourcePath) {
    if (stl_file_has_binary_layout(sourcePath)) {
        return import_binary_stl_document(sourcePath);
    }

    std::ifstream input(sourcePath, std::ios::binary);
    if (!input) {
        throw std::runtime_error("could not open STL file");
    }

    SceneDocument document;
    document.scene_label = sourcePath.filename().string();
    document.source_kind = SceneSourceKind::StlMesh;
    document.source_path = sourcePath;

    SceneBody body;
    body.name = default_body_name(sourcePath, 0);
    std::unordered_map<std::string, std::size_t> vertexLookup;

    std::string line;
    std::size_t lineNumber = 0;
    std::vector<geometry::Point3D> pendingFacetPoints;
    while (std::getline(input, line)) {
        ++lineNumber;
        const std::string trimmed = trim_copy(line);
        if (trimmed.empty()) {
            continue;
        }

        std::istringstream lineStream(trimmed);
        std::string keyword;
        lineStream >> keyword;
        keyword = lowercase_copy(keyword);
        if (keyword == "vertex") {
            geometry::Point3D point{};
            if (!(lineStream >> point.x >> point.y >> point.z)) {
                throw std::runtime_error("ASCII STL vertex line is malformed at line " + std::to_string(lineNumber));
            }
            pendingFacetPoints.push_back(point);
            continue;
        }

        if (keyword == "endfacet") {
            if (pendingFacetPoints.size() != 3) {
                throw std::runtime_error("ASCII STL facet did not contain exactly three vertices by line " + std::to_string(lineNumber));
            }

            SceneFace face;
            for (const auto& point : pendingFacetPoints) {
                face.vertex_indices.push_back(append_unique_vertex(&body, &vertexLookup, point));
            }
            body.faces.push_back(std::move(face));
            pendingFacetPoints.clear();
            continue;
        }
    }

    if (!pendingFacetPoints.empty()) {
        throw std::runtime_error("ASCII STL ended with an incomplete facet");
    }
    if (body.faces.empty()) {
        throw std::runtime_error("ASCII STL import found no triangle facets");
    }

    document.bodies.push_back(std::move(body));
    normalize_scene_document_topology(&document);
    return document;
}

SceneDocument import_ascii_ply(const std::filesystem::path& sourcePath) {
    std::ifstream input(sourcePath, std::ios::binary);
    if (!input) {
        throw std::runtime_error("could not open PLY file");
    }

    std::string line;
    if (!std::getline(input, line) || lowercase_copy(trim_copy(line)) != "ply") {
        throw std::runtime_error("PLY import requires a file beginning with 'ply'");
    }

    const PlyHeaderSpec header = parse_ply_header(input);

    SceneDocument document;
    document.scene_label = sourcePath.filename().string();
    document.source_kind = SceneSourceKind::PlyMesh;
    document.source_path = sourcePath;

    SceneBody body;
    body.name = default_body_name(sourcePath, 0);
    std::vector<SceneColor> vertexColors;
    std::vector<bool> vertexHasColors;

    const bool isAscii = header.format == PlyFormat::Ascii;
    const bool littleEndian = header.format == PlyFormat::BinaryLittleEndian;
    for (const PlyElementSpec& element : header.elements) {
        if (element.name == "vertex") {
            body.vertices.reserve(body.vertices.size() + element.count);
            vertexColors.reserve(vertexColors.size() + element.count);
            vertexHasColors.reserve(vertexHasColors.size() + element.count);
            for (std::size_t index = 0; index < element.count; ++index) {
                if (isAscii) {
                    read_ascii_ply_vertex(input, element, &body, &vertexColors, &vertexHasColors);
                } else {
                    read_binary_ply_vertex(input, element, littleEndian, &body, &vertexColors, &vertexHasColors);
                }
            }
            continue;
        }

        if (element.name == "face") {
            body.faces.reserve(body.faces.size() + element.count);
            for (std::size_t index = 0; index < element.count; ++index) {
                if (isAscii) {
                    read_ascii_ply_face(input, element, &body, vertexColors, vertexHasColors);
                } else {
                    read_binary_ply_face(input, element, littleEndian, &body, vertexColors, vertexHasColors);
                }
            }
            continue;
        }

        for (std::size_t index = 0; index < element.count; ++index) {
            if (isAscii) {
                skip_ascii_ply_element_row(input);
            } else {
                skip_binary_ply_element_row(input, element, littleEndian);
            }
        }
    }

    if (body.vertices.empty() || body.faces.empty()) {
        throw std::runtime_error("PLY import requires both vertex and face elements");
    }

    document.bodies.push_back(std::move(body));
    normalize_scene_document_topology(&document);
    return document;
}

SceneDocument import_step_model(const std::filesystem::path& sourcePath) {
#ifdef ADAPTIVECAD_HAS_OPENCASCADE
    STEPControl_Reader reader;
    const std::string sourceText = sourcePath.string();
    const IFSelect_ReturnStatus readStatus = reader.ReadFile(sourceText.c_str());
    if (readStatus != IFSelect_RetDone) {
        throw std::runtime_error("OpenCascade STEP reader could not parse the requested file");
    }

    reader.TransferRoots();
    const TopoDS_Shape shape = reader.OneShape();
    if (shape.IsNull()) {
        throw std::runtime_error("STEP import produced an empty shape");
    }

    BRepMesh_IncrementalMesh mesh(shape, 0.5, false, 0.5, true);

    SceneDocument document;
    document.scene_label = sourcePath.filename().string();
    document.source_kind = SceneSourceKind::StepModel;
    document.source_path = sourcePath;

    std::size_t bodyIndex = 0;
    for (TopExp_Explorer solidExplorer(shape, TopAbs_SOLID); solidExplorer.More(); solidExplorer.Next(), ++bodyIndex) {
        SceneBody body;
        body.name = default_body_name(sourcePath, bodyIndex);
        std::unordered_map<std::string, std::size_t> vertexLookup;

        const TopoDS_Shape solid = solidExplorer.Current();
        for (TopExp_Explorer faceExplorer(solid, TopAbs_FACE); faceExplorer.More(); faceExplorer.Next()) {
            const TopoDS_Face face = TopoDS::Face(faceExplorer.Current());
            TopLoc_Location location;
            const Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);
            if (triangulation.IsNull()) {
                continue;
            }

            const gp_Trsf transform = location.Transformation();
            const Poly_Array1OfTriangle& triangles = triangulation->Triangles();
            for (int triangleIndex = triangles.Lower(); triangleIndex <= triangles.Upper(); ++triangleIndex) {
                int n1 = 0;
                int n2 = 0;
                int n3 = 0;
                triangles(triangleIndex).Get(n1, n2, n3);
                const gp_Pnt p1 = triangulation->Node(n1).Transformed(transform);
                const gp_Pnt p2 = triangulation->Node(n2).Transformed(transform);
                const gp_Pnt p3 = triangulation->Node(n3).Transformed(transform);

                SceneFace meshFace;
                meshFace.vertex_indices.push_back(
                    append_unique_vertex(&body, &vertexLookup, geometry::Point3D{p1.X(), p1.Y(), p1.Z()}));
                meshFace.vertex_indices.push_back(
                    append_unique_vertex(&body, &vertexLookup, geometry::Point3D{p2.X(), p2.Y(), p2.Z()}));
                meshFace.vertex_indices.push_back(
                    append_unique_vertex(&body, &vertexLookup, geometry::Point3D{p3.X(), p3.Y(), p3.Z()}));
                body.faces.push_back(std::move(meshFace));
            }
        }

        if (!body.faces.empty()) {
            document.bodies.push_back(std::move(body));
        }
    }

    if (document.bodies.empty()) {
        SceneBody body;
        body.name = default_body_name(sourcePath, 0);
        std::unordered_map<std::string, std::size_t> vertexLookup;
        for (TopExp_Explorer faceExplorer(shape, TopAbs_FACE); faceExplorer.More(); faceExplorer.Next()) {
            const TopoDS_Face face = TopoDS::Face(faceExplorer.Current());
            TopLoc_Location location;
            const Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);
            if (triangulation.IsNull()) {
                continue;
            }

            const gp_Trsf transform = location.Transformation();
            const Poly_Array1OfTriangle& triangles = triangulation->Triangles();
            for (int triangleIndex = triangles.Lower(); triangleIndex <= triangles.Upper(); ++triangleIndex) {
                int n1 = 0;
                int n2 = 0;
                int n3 = 0;
                triangles(triangleIndex).Get(n1, n2, n3);
                const gp_Pnt p1 = triangulation->Node(n1).Transformed(transform);
                const gp_Pnt p2 = triangulation->Node(n2).Transformed(transform);
                const gp_Pnt p3 = triangulation->Node(n3).Transformed(transform);

                SceneFace meshFace;
                meshFace.vertex_indices.push_back(
                    append_unique_vertex(&body, &vertexLookup, geometry::Point3D{p1.X(), p1.Y(), p1.Z()}));
                meshFace.vertex_indices.push_back(
                    append_unique_vertex(&body, &vertexLookup, geometry::Point3D{p2.X(), p2.Y(), p2.Z()}));
                meshFace.vertex_indices.push_back(
                    append_unique_vertex(&body, &vertexLookup, geometry::Point3D{p3.X(), p3.Y(), p3.Z()}));
                body.faces.push_back(std::move(meshFace));
            }
        }

        if (!body.faces.empty()) {
            document.bodies.push_back(std::move(body));
        }
    }

    if (document.bodies.empty()) {
        throw std::runtime_error("STEP import could not extract triangulated faces from the loaded shape");
    }

    normalize_scene_document_topology(&document);
    return document;
#else
    (void)sourcePath;
    throw std::runtime_error("STEP import requires an OpenCascade-enabled build");
#endif
}

SceneDocument import_scene_document(const std::filesystem::path& sourcePath) {
    const std::string extension = lowercase_copy(sourcePath.extension().string());
    if (extension == ".obj") {
        return import_wavefront_obj(sourcePath);
    }
    if (extension == ".stl") {
        return import_ascii_stl(sourcePath);
    }
    if (extension == ".ply") {
        return import_ascii_ply(sourcePath);
    }
    if (extension == ".step" || extension == ".stp") {
        return import_step_model(sourcePath);
    }

    throw std::runtime_error("unsupported geometry import format; expected .obj, .stl, .ply, .step, or .stp");
}

struct ObjExportMaterial {
    std::string obj_name;
    SceneColor color;
};

struct ObjExportMaterialRegistry {
    std::vector<ObjExportMaterial> materials;
    std::unordered_map<std::string, std::string> material_name_by_key;
    std::unordered_map<std::string, std::size_t> used_names;
};

std::string unique_obj_material_name(
    ObjExportMaterialRegistry* registry,
    const std::string& requestedName,
    std::size_t fallbackIndex) {
    std::string baseName = obj_safe_material_name(requestedName, fallbackIndex);
    if (baseName.empty()) {
        baseName = "Material_" + std::to_string(fallbackIndex + 1);
    }

    std::string name = baseName;
    auto existing = registry->used_names.find(name);
    if (existing == registry->used_names.end()) {
        registry->used_names.emplace(name, 1U);
        return name;
    }

    std::size_t suffix = existing->second + 1U;
    existing->second = suffix;
    do {
        name = baseName + "_" + std::to_string(suffix);
        ++suffix;
    } while (registry->used_names.find(name) != registry->used_names.end());

    registry->used_names.emplace(name, 1U);
    return name;
}

bool resolve_scene_material(
    const SceneDocument& document,
    const SceneBody& body,
    const SceneFace& face,
    std::string* materialName,
    SceneColor* color) {
    if (!materialName || !color) {
        return false;
    }

    *materialName = !face.material_name.empty() ? face.material_name : body.material_name;
    if (face.has_color) {
        *color = face.color;
        return true;
    }
    if (body.has_color) {
        *color = body.color;
        return true;
    }

    if (const SceneMaterial* material = find_material_by_name(document.materials, *materialName)) {
        if (material->has_diffuse_color) {
            *color = material->diffuse_color;
            return true;
        }
    }

    if (!trim_copy(*materialName).empty()) {
        *color = make_scene_color(0.8, 0.8, 0.8, 1.0);
        return true;
    }

    return false;
}

std::string register_obj_export_material(
    ObjExportMaterialRegistry* registry,
    const std::string& requestedName,
    const SceneColor& color) {
    if (!registry) {
        return {};
    }

    const std::string trimmedName = trim_copy(requestedName);
    const std::string baseName = trimmedName.empty() ? "AdaptiveCAD_Color_" + color_key(color) : trimmedName;
    const std::string key = baseName + "|" + color_key(color);
    const auto existing = registry->material_name_by_key.find(key);
    if (existing != registry->material_name_by_key.end()) {
        return existing->second;
    }

    const std::string objName = unique_obj_material_name(registry, baseName, registry->materials.size());
    registry->materials.push_back(ObjExportMaterial{objName, color});
    registry->material_name_by_key.emplace(key, objName);
    return objName;
}

ObjExportMaterialRegistry collect_obj_export_materials(const SceneDocument& document) {
    ObjExportMaterialRegistry registry;
    for (const SceneBody& body : document.bodies) {
        for (const SceneFace& face : body.faces) {
            std::string materialName;
            SceneColor color;
            if (resolve_scene_material(document, body, face, &materialName, &color)) {
                (void)register_obj_export_material(&registry, materialName, color);
            }
        }
    }
    return registry;
}

bool scene_has_unmaterialed_faces(const SceneDocument& document) {
    for (const SceneBody& body : document.bodies) {
        for (const SceneFace& face : body.faces) {
            std::string materialName;
            SceneColor color;
            if (!resolve_scene_material(document, body, face, &materialName, &color)) {
                return true;
            }
        }
    }
    return false;
}

std::filesystem::path obj_material_library_path(const std::filesystem::path& objPath) {
    std::filesystem::path materialPath = objPath;
    materialPath.replace_extension(".mtl");
    return materialPath;
}

void write_obj_material_library(
    const std::filesystem::path& materialPath,
    const ObjExportMaterialRegistry& registry) {
    if (registry.materials.empty()) {
        return;
    }

    std::ofstream output(materialPath, std::ios::binary | std::ios::trunc);
    if (!output) {
        throw std::runtime_error("could not create OBJ material library");
    }

    output << "# AdaptiveCAD MTL export\n";
    for (const ObjExportMaterial& material : registry.materials) {
        output << "\nnewmtl " << material.obj_name << '\n';
        output << std::setprecision(9)
               << "Kd " << clamp01(material.color.red)
               << ' ' << clamp01(material.color.green)
               << ' ' << clamp01(material.color.blue) << '\n';
        output << std::setprecision(9) << "d " << clamp01(material.color.alpha) << '\n';
    }

    if (!output) {
        throw std::runtime_error("failed while writing OBJ material library");
    }
}

void export_wavefront_obj(const SceneDocument& document, const std::filesystem::path& targetPath) {
    if (document.bodies.empty()) {
        throw std::runtime_error("scene document contains no bodies to export");
    }

    if (targetPath.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(targetPath.parent_path(), ec);
        if (ec) {
            throw std::runtime_error("could not create OBJ export directory");
        }
    }

    std::ofstream output(targetPath, std::ios::binary | std::ios::trunc);
    if (!output) {
        throw std::runtime_error("could not create OBJ export file");
    }

    ObjExportMaterialRegistry materialRegistry = collect_obj_export_materials(document);
    std::string defaultMaterialName;
    if (!materialRegistry.materials.empty() && scene_has_unmaterialed_faces(document)) {
        defaultMaterialName = register_obj_export_material(
            &materialRegistry,
            "AdaptiveCAD_Default",
            make_scene_color(0.86, 0.90, 0.94, 1.0));
    }
    const std::filesystem::path materialPath = obj_material_library_path(targetPath);
    if (!materialRegistry.materials.empty()) {
        write_obj_material_library(materialPath, materialRegistry);
    }

    output << "# AdaptiveCAD OBJ export\n";
    if (!document.scene_label.empty()) {
        output << "# scene_label " << document.scene_label << '\n';
    }
    output << "# source_kind " << scene_source_kind_name(document.source_kind) << '\n';
    if (!materialRegistry.materials.empty()) {
        output << "mtllib " << materialPath.filename().string() << '\n';
    }

    std::size_t vertexOffset = 0;
    std::string activeMaterialName;
    for (std::size_t bodyIndex = 0; bodyIndex < document.bodies.size(); ++bodyIndex) {
        const SceneBody& body = document.bodies[bodyIndex];
        if (body.vertices.empty()) {
            throw std::runtime_error("cannot export a scene body with no vertices");
        }
        if (body.faces.empty()) {
            throw std::runtime_error("cannot export a scene body with no faces");
        }

        output << "\no " << obj_safe_name(body.name, bodyIndex) << '\n';
        for (const geometry::Point3D& point : body.vertices) {
            output << std::setprecision(17)
                   << "v " << point.x << ' ' << point.y << ' ' << point.z << '\n';
        }

        for (const SceneFace& face : body.faces) {
            if (face.vertex_indices.size() < 3) {
                throw std::runtime_error("cannot export a scene face with fewer than three vertices");
            }

            std::string requestedMaterialName;
            SceneColor materialColor;
            if (resolve_scene_material(document, body, face, &requestedMaterialName, &materialColor)) {
                const std::string objMaterialName =
                    register_obj_export_material(&materialRegistry, requestedMaterialName, materialColor);
                if (objMaterialName != activeMaterialName) {
                    output << "usemtl " << objMaterialName << '\n';
                    activeMaterialName = objMaterialName;
                }
            } else if (!activeMaterialName.empty()) {
                if (!defaultMaterialName.empty()) {
                    output << "usemtl " << defaultMaterialName << '\n';
                    activeMaterialName = defaultMaterialName;
                } else {
                    activeMaterialName.clear();
                }
            }

            output << "f";
            for (std::size_t localIndex : face.vertex_indices) {
                if (localIndex >= body.vertices.size()) {
                    throw std::runtime_error("cannot export a scene face with an out-of-range vertex index");
                }
                output << ' ' << (vertexOffset + localIndex + 1);
            }
            output << '\n';
        }

        vertexOffset += body.vertices.size();
    }

    if (!output) {
        throw std::runtime_error("failed while writing OBJ export file");
    }
}

void export_scene_document(const SceneDocument& document, const std::filesystem::path& targetPath) {
    const std::string extension = lowercase_copy(targetPath.extension().string());
    if (extension == ".obj") {
        export_wavefront_obj(document, targetPath);
        return;
    }

    throw std::runtime_error("unsupported geometry export format; expected .obj");
}

} // namespace adaptivecad::tool
