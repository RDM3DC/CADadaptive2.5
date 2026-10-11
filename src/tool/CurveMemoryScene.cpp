#include "adaptivecad/tool/CurveMemoryScene.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <unordered_map>

namespace adaptivecad::tool {
namespace {

bool same_double(double a, double b) noexcept {
    return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

bool same_point(const geometry::Point3D& a, const geometry::Point3D& b) noexcept {
    return same_double(a.x, b.x) && same_double(a.y, b.y) && same_double(a.z, b.z);
}

bool same_color(const SceneColor& a, const SceneColor& b) noexcept {
    return same_double(a.red, b.red) && same_double(a.green, b.green) &&
        same_double(a.blue, b.blue) && same_double(a.alpha, b.alpha);
}

bool same_faces(const std::vector<SceneFace>& a, const std::vector<SceneFace>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i].vertex_indices != b[i].vertex_indices ||
            !same_double(a[i].area_hint, b[i].area_hint) ||
            a[i].material_name != b[i].material_name || a[i].has_color != b[i].has_color ||
            !same_color(a[i].color, b[i].color)) return false;
    }
    return true;
}

void hash_word(std::uint64_t& hash, std::uint64_t word) noexcept {
    for (unsigned int shift = 0; shift != 64; shift += 8) {
        hash ^= (word >> shift) & 0xffU;
        hash *= 1099511628211ULL;
    }
}

void hash_string(std::uint64_t& hash, const std::string& text) noexcept {
    hash_word(hash, text.size());
    for (unsigned char c : text) {
        hash ^= c;
        hash *= 1099511628211ULL;
    }
}

void hash_color(std::uint64_t& hash, const SceneColor& color) noexcept {
    hash_word(hash, std::bit_cast<std::uint64_t>(color.red));
    hash_word(hash, std::bit_cast<std::uint64_t>(color.green));
    hash_word(hash, std::bit_cast<std::uint64_t>(color.blue));
    hash_word(hash, std::bit_cast<std::uint64_t>(color.alpha));
}

std::uint64_t shape_hash(const SceneBody& body) {
    std::uint64_t hash = 14695981039346656037ULL;
    hash_word(hash, body.vertices.size());
    for (const auto& p : body.vertices) {
        hash_word(hash, std::bit_cast<std::uint64_t>(p.x));
        hash_word(hash, std::bit_cast<std::uint64_t>(p.y));
        hash_word(hash, std::bit_cast<std::uint64_t>(p.z));
    }
    hash_word(hash, std::bit_cast<std::uint64_t>(body.volume_hint));
    hash_word(hash, body.faces.size());
    for (const auto& face : body.faces) {
        hash_word(hash, face.vertex_indices.size());
        for (const auto index : face.vertex_indices) hash_word(hash, index);
        hash_word(hash, std::bit_cast<std::uint64_t>(face.area_hint));
        hash_string(hash, face.material_name);
        hash_word(hash, face.has_color ? 1 : 0);
        hash_color(hash, face.color);
    }
    return hash;
}

void validate_placement(const CurveMemoryAffine& matrix) {
    for (const auto value : matrix.values) {
        if (!std::isfinite(value)) throw std::invalid_argument("curve-memory placement must be finite");
    }
    if (matrix.values[12] != 0.0 || matrix.values[13] != 0.0 ||
        matrix.values[14] != 0.0 || matrix.values[15] != 1.0) {
        throw std::invalid_argument("curve-memory placement must be affine");
    }
}

bool is_identity(const CurveMemoryAffine& matrix) noexcept {
    return matrix.values == CurveMemoryAffine{}.values;
}

geometry::Point3D transform(const CurveMemoryAffine& matrix, const geometry::Point3D& point) {
    // Avoid arithmetic for identity reads: +0 and -0 retain their original bits.
    if (is_identity(matrix)) return point;
    const auto& m = matrix.values;
    geometry::Point3D result{
        m[0] * point.x + m[1] * point.y + m[2] * point.z + m[3],
        m[4] * point.x + m[5] * point.y + m[6] * point.z + m[7],
        m[8] * point.x + m[9] * point.y + m[10] * point.z + m[11]};
    if (!std::isfinite(result.x) || !std::isfinite(result.y) || !std::isfinite(result.z)) {
        throw std::overflow_error("curve-memory transformed point is not finite");
    }
    return result;
}

// Include string capacities conservatively; implementations may store short strings inline.
std::size_t string_bytes(const std::string& value) noexcept { return value.capacity() + 1; }

std::size_t header_bytes(const SceneDocument& header) {
    std::size_t bytes = string_bytes(header.scene_label) +
        (header.source_path.native().capacity() + 1) * sizeof(std::filesystem::path::value_type) +
        header.angular_model.cosine_coefficients.capacity() * sizeof(double) +
        header.angular_model.sine_coefficients.capacity() * sizeof(double) +
        header.metadata.capacity() * sizeof(SceneMetadataEntry) +
        header.materials.capacity() * sizeof(SceneMaterial);
    for (const auto& item : header.metadata) bytes += string_bytes(item.key) + string_bytes(item.value);
    for (const auto& item : header.materials) bytes += string_bytes(item.name);
    return bytes;
}

} // namespace

CurveMemoryAffine CurveMemoryAffine::identity() noexcept { return {}; }

CurveMemoryAffine CurveMemoryAffine::translation(double x, double y, double z) {
    CurveMemoryAffine matrix;
    matrix.values[3] = x;
    matrix.values[7] = y;
    matrix.values[11] = z;
    validate_placement(matrix);
    return matrix;
}

CurveMemoryScene::CurveMemoryScene(std::size_t cacheBlocks, CurveStoragePolicy policy) : bank_(cacheBlocks, policy) {}

CurveMemoryScene CurveMemoryScene::from_document(const SceneDocument& document, std::size_t cacheBlocks,
    CurveStoragePolicy policy) {
    CurveMemoryScene scene(cacheBlocks, policy);
    // Copy the header fields explicitly to avoid temporarily copying dense geometry.
    scene.header_.scene_label = document.scene_label;
    scene.header_.source_kind = document.source_kind;
    scene.header_.source_path = document.source_path;
    scene.header_.angular_model = document.angular_model;
    scene.header_.metadata = document.metadata;
    scene.header_.materials = document.materials;
    scene.instances_.reserve(document.bodies.size());
    std::unordered_map<std::uint64_t, std::vector<std::size_t>> candidates;
    // Source references exist only during import, avoiding repeated decode work
    // when sharing many bodies with the cache deliberately disabled.
    std::vector<const SceneBody*> representatives;
    for (const auto& body : document.bodies) {
        for (const auto& face : body.faces) {
            for (const auto index : face.vertex_indices) {
                if (index >= body.vertices.size()) throw std::invalid_argument("curve-memory face index outside body");
            }
        }
        const auto hash = shape_hash(body);
        auto& sameHash = candidates[hash];
        std::size_t shapeIndex = scene.shapes_.size();
        for (const auto candidate : sameHash) {
            const auto& shape = scene.shapes_[candidate];
            if (!same_double(shape.volume_hint, body.volume_hint) || !same_faces(shape.faces, body.faces) ||
                scene.bank_.point_count(shape.curve) != body.vertices.size()) continue;
            bool equal = true;
            for (std::size_t i = 0; equal && i < body.vertices.size(); ++i) {
                equal = same_point(representatives[candidate]->vertices[i], body.vertices[i]);
            }
            if (equal) {
                shapeIndex = candidate;
                break;
            }
        }
        if (shapeIndex == scene.shapes_.size()) {
            Shape shape;
            shape.curve = scene.bank_.add_points(std::span<const geometry::Point3D>(body.vertices));
            shape.faces = body.faces;
            shape.volume_hint = body.volume_hint;
            scene.shapes_.push_back(std::move(shape));
            representatives.push_back(&body);
            sameHash.push_back(shapeIndex);
        }
        Instance instance;
        instance.shape = shapeIndex;
        instance.name = body.name;
        instance.material_name = body.material_name;
        instance.color = body.color;
        instance.has_color = body.has_color;
        scene.instances_.push_back(std::move(instance));
    }
    if (policy == CurveStoragePolicy::Automatic && scene.bank_.stats().compressed_curves != 0) {
        const auto cost = scene.stats();
        const auto minimumSaving = cost.raw_policy_bytes / 20 + (cost.raw_policy_bytes % 20 != 0);
        // A useful point-array saving can be negligible once face topology and
        // scene metadata are included. In that case retain direct raw reads.
        if (cost.budgeted_resident_bytes > cost.raw_policy_bytes ||
            cost.raw_policy_bytes - cost.budgeted_resident_bytes < minimumSaving) {
            CurveMemoryBank raw(cacheBlocks, CurveStoragePolicy::Raw);
            for (std::size_t i = 0; i < scene.shapes_.size(); ++i) {
                scene.shapes_[i].curve = raw.add_points(representatives[i]->vertices);
            }
            scene.bank_ = std::move(raw);
        }
    }
    scene.clear_cache();
    return scene;
}

std::size_t CurveMemoryScene::instance_count() const noexcept { return instances_.size(); }

std::size_t CurveMemoryScene::shape_id(std::size_t instance) const {
    return instances_.at(instance).shape;
}

std::size_t CurveMemoryScene::point_count(std::size_t instance) const {
    return bank_.point_count(shapes_.at(instances_.at(instance).shape).curve);
}

geometry::Point3D CurveMemoryScene::local_point(const Instance& instance, std::size_t vertex) const {
    const auto curve = shapes_.at(instance.shape).curve;
    if (vertex >= bank_.point_count(curve)) throw std::out_of_range("curve-memory vertex index outside instance");
    const auto found = std::lower_bound(instance.overrides.begin(), instance.overrides.end(), vertex,
        [](const auto& entry, std::size_t index) { return entry.first < index; });
    if (found != instance.overrides.end() && found->first == vertex) return found->second;
    return bank_.point(curve, vertex);
}

geometry::Point3D CurveMemoryScene::point(std::size_t instance, std::size_t vertex) const {
    const auto& selected = instances_.at(instance);
    return transform(selected.placement, local_point(selected, vertex));
}

void CurveMemoryScene::points(std::size_t instance, std::span<const std::size_t> vertices,
    std::span<geometry::Point3D> output) const {
    const auto& selected = instances_.at(instance);
    const auto curve = shapes_.at(selected.shape).curve;
    // The bank validates the entire request before writing output, and groups
    // block reads even when callers submit unordered or repeated indices.
    bank_.read_points(curve, vertices, output);
    if (!selected.overrides.empty()) {
        for (std::size_t i = 0; i < vertices.size(); ++i) {
            const auto found = std::lower_bound(selected.overrides.begin(), selected.overrides.end(), vertices[i],
                [](const auto& entry, std::size_t index) { return entry.first < index; });
            if (found != selected.overrides.end() && found->first == vertices[i]) output[i] = found->second;
        }
    }
    if (!is_identity(selected.placement)) {
        for (auto& vertex : output) vertex = transform(selected.placement, vertex);
    }
}

std::vector<geometry::Point3D> CurveMemoryScene::points(std::size_t instance) const {
    const auto& selected = instances_.at(instance);
    auto result = bank_.points(shapes_.at(selected.shape).curve);
    for (const auto& edit : selected.overrides) result.at(edit.first) = edit.second;
    if (!is_identity(selected.placement)) {
        for (auto& vertex : result) vertex = transform(selected.placement, vertex);
    }
    return result;
}

std::size_t CurveMemoryScene::add_instance(std::size_t existing, const CurveMemoryAffine& placement, std::string name) {
    validate_placement(placement);
    Instance instance = instances_.at(existing);
    instance.placement = placement;
    instance.name = std::move(name);
    instances_.push_back(std::move(instance));
    return instances_.size() - 1;
}

void CurveMemoryScene::set_point(std::size_t instance, std::size_t vertex, geometry::Point3D localPoint) {
    auto& selected = instances_.at(instance);
    const auto curve = shapes_.at(selected.shape).curve;
    if (vertex >= bank_.point_count(curve)) throw std::out_of_range("curve-memory vertex index outside instance");
    if (!std::isfinite(localPoint.x) || !std::isfinite(localPoint.y) || !std::isfinite(localPoint.z)) {
        throw std::invalid_argument("curve-memory point edit must be finite");
    }
    auto found = std::lower_bound(selected.overrides.begin(), selected.overrides.end(), vertex,
        [](const auto& entry, std::size_t index) { return entry.first < index; });
    if (same_point(localPoint, bank_.point(curve, vertex))) {
        if (found != selected.overrides.end() && found->first == vertex) selected.overrides.erase(found);
    } else if (found != selected.overrides.end() && found->first == vertex) {
        found->second = localPoint;
    } else {
        selected.overrides.insert(found, {vertex, localPoint});
    }
}

void CurveMemoryScene::add_relation(std::size_t from, std::size_t to) {
    if (from >= instances_.size() || to >= instances_.size()) throw std::out_of_range("curve-memory relation index outside scene");
    if (from == to) throw std::invalid_argument("curve-memory relation must connect distinct instances");
    const std::pair<std::size_t, std::size_t> relation{from, to};
    if (std::find(relations_.begin(), relations_.end(), relation) == relations_.end()) relations_.push_back(relation);
}

const std::vector<std::pair<std::size_t, std::size_t>>& CurveMemoryScene::relations() const noexcept { return relations_; }
const CurveMemoryBank& CurveMemoryScene::bank() const noexcept { return bank_; }
void CurveMemoryScene::clear_cache() const { bank_.clear_cache(); }

SceneDocument CurveMemoryScene::materialize() const {
    SceneDocument document = header_;
    document.bodies.reserve(instances_.size());
    for (std::size_t i = 0; i < instances_.size(); ++i) {
        const auto& instance = instances_[i];
        const auto& shape = shapes_.at(instance.shape);
        SceneBody body;
        body.name = instance.name;
        body.material_name = instance.material_name;
        body.color = instance.color;
        body.has_color = instance.has_color;
        body.faces = shape.faces;
        body.volume_hint = shape.volume_hint;
        // Decode blocks once even when caching is disabled. This returned array
        // is the explicitly requested expanded geometry, not retained bank state.
        body.vertices = points(i);
        // Existing native build computes area when its hint is absent. Never retain
        // stale hints after an edit or placement; volume zero means unspecified.
        if (!instance.overrides.empty() || !is_identity(instance.placement)) {
            body.volume_hint = 0.0;
            for (auto& face : body.faces) face.area_hint = 0.0;
        }
        document.bodies.push_back(std::move(body));
    }
    return document;
}

CurveMemorySceneStats CurveMemoryScene::stats() const {
    CurveMemorySceneStats result;
    result.unique_shapes = shapes_.size();
    result.instances = instances_.size();
    result.relations = relations_.size();
    const auto bankStats = bank_.stats();
    result.cache_bytes = bankStats.cache_bytes;
    result.cache_limit_bytes = bankStats.cache_limit_bytes;
    std::size_t common = sizeof(*this) - sizeof(bank_) + header_bytes(header_) +
        shapes_.capacity() * sizeof(Shape) + instances_.capacity() * sizeof(Instance) +
        relations_.capacity() * sizeof(decltype(relations_)::value_type);
    std::size_t denseBytes = 0;
    for (const auto& shape : shapes_) {
        common += shape.faces.capacity() * sizeof(SceneFace);
        denseBytes += bank_.point_count(shape.curve) * sizeof(geometry::Point3D);
        for (const auto& face : shape.faces) {
            common += face.vertex_indices.capacity() * sizeof(std::size_t) + string_bytes(face.material_name);
        }
    }
    for (const auto& instance : instances_) {
        common += string_bytes(instance.name) + string_bytes(instance.material_name) +
            instance.overrides.capacity() * sizeof(decltype(instance.overrides)::value_type);
        result.overrides += instance.overrides.size();
        result.logical_point_bytes += bank_.point_count(shapes_[instance.shape].curve) * sizeof(geometry::Point3D);
    }
    result.resident_bytes = common + bankStats.resident_bytes;
    result.raw_policy_bytes = common + bankStats.dense_equivalent_bytes;
    result.budgeted_resident_bytes = common + bankStats.budgeted_resident_bytes;
    result.ordinary_shared_bytes = common + sizeof(std::vector<std::vector<geometry::Point3D>>) +
        shapes_.size() * sizeof(std::vector<geometry::Point3D>) + denseBytes;
    return result;
}

} // namespace adaptivecad::tool
