#include "adaptivecad/tool/CurveMemoryScene.hpp"

#include <array>
#include <bit>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using adaptivecad::geometry::Point3D;
using adaptivecad::tool::CurveMemoryAffine;
using adaptivecad::tool::CurveMemoryScene;
using adaptivecad::tool::CurveStoragePolicy;
using adaptivecad::tool::ParametricCurveDefinition;
using adaptivecad::tool::ParametricCurveKind;
using adaptivecad::tool::SceneBody;
using adaptivecad::tool::SceneColor;
using adaptivecad::tool::SceneDocument;
using adaptivecad::tool::SceneFace;

void exact(double a, double b) {
    assert(std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b));
}

void exact(const Point3D& a, const Point3D& b) {
    exact(a.x, b.x);
    exact(a.y, b.y);
    exact(a.z, b.z);
}

void exact(const SceneColor& a, const SceneColor& b) {
    exact(a.red, b.red);
    exact(a.green, b.green);
    exact(a.blue, b.blue);
    exact(a.alpha, b.alpha);
}

void exact_document(const SceneDocument& a, const SceneDocument& b) {
    assert(a.scene_label == b.scene_label);
    assert(a.source_kind == b.source_kind);
    assert(a.source_path == b.source_path);
    exact(a.angular_model.lambda0, b.angular_model.lambda0);
    assert(a.angular_model.cosine_coefficients.size() == b.angular_model.cosine_coefficients.size());
    assert(a.angular_model.sine_coefficients.size() == b.angular_model.sine_coefficients.size());
    for (std::size_t i = 0; i < a.angular_model.cosine_coefficients.size(); ++i)
        exact(a.angular_model.cosine_coefficients[i], b.angular_model.cosine_coefficients[i]);
    for (std::size_t i = 0; i < a.angular_model.sine_coefficients.size(); ++i)
        exact(a.angular_model.sine_coefficients[i], b.angular_model.sine_coefficients[i]);
    assert(a.metadata.size() == b.metadata.size());
    for (std::size_t i = 0; i < a.metadata.size(); ++i) {
        assert(a.metadata[i].key == b.metadata[i].key);
        assert(a.metadata[i].value == b.metadata[i].value);
    }
    assert(a.materials.size() == b.materials.size());
    for (std::size_t i = 0; i < a.materials.size(); ++i) {
        assert(a.materials[i].name == b.materials[i].name);
        assert(a.materials[i].has_diffuse_color == b.materials[i].has_diffuse_color);
        exact(a.materials[i].diffuse_color, b.materials[i].diffuse_color);
    }
    assert(a.bodies.size() == b.bodies.size());
    for (std::size_t i = 0; i < a.bodies.size(); ++i) {
        const auto& x = a.bodies[i];
        const auto& y = b.bodies[i];
        assert(x.name == y.name);
        assert(x.material_name == y.material_name);
        assert(x.has_color == y.has_color);
        exact(x.color, y.color);
        exact(x.volume_hint, y.volume_hint);
        assert(x.vertices.size() == y.vertices.size());
        for (std::size_t j = 0; j < x.vertices.size(); ++j) exact(x.vertices[j], y.vertices[j]);
        assert(x.faces.size() == y.faces.size());
        for (std::size_t j = 0; j < x.faces.size(); ++j) {
            assert(x.faces[j].vertex_indices == y.faces[j].vertex_indices);
            assert(x.faces[j].material_name == y.faces[j].material_name);
            assert(x.faces[j].has_color == y.faces[j].has_color);
            exact(x.faces[j].area_hint, y.faces[j].area_hint);
            exact(x.faces[j].color, y.faces[j].color);
        }
    }
}

template <class Exception, class Action>
void throws(Action action) {
    bool caught = false;
    try { action(); }
    catch (const Exception&) { caught = true; }
    assert(caught);
}

void roundtrip_and_dedup() {
    auto document = adaptivecad::tool::make_box_scene_document(2.0, 3.0, 4.0);
    document.scene_label = "Native curve bank with metadata";
    document.source_kind = adaptivecad::tool::SceneSourceKind::SessionEmbedded;
    document.source_path = std::filesystem::path("examples") / "preserved source.obj";
    document.angular_model.lambda0 = -0.0;
    document.angular_model.cosine_coefficients = {-0.0, 0.25};
    document.angular_model.sine_coefficients = {0.0, -0.125};
    document.metadata = {{"kind", "native"}, {"kind", "duplicate keys preserved"}};
    document.materials.push_back({"copper", {0.9, 0.4, 0.1, 0.75}, true});
    auto& body = document.bodies.front();
    body.name = "Original";
    body.material_name = "copper";
    body.color = {0.1, 0.2, 0.3, 0.9};
    body.has_color = true;
    body.vertices[0] = {-0.0, 0.0, std::numeric_limits<double>::denorm_min()};
    body.faces[0].area_hint = 4.5;
    body.faces[0].material_name = "copper";
    body.faces[0].color = {0.0, -0.0, 1.0, 0.5};
    body.faces[0].has_color = true;
    document.bodies.push_back(body);
    document.bodies.back().name = "Appearance instance";
    document.bodies.back().material_name = "paint";
    document.bodies.back().has_color = false;
    document.bodies.back().color = {0.7, 0.6, 0.5, -0.0};

    auto scene = CurveMemoryScene::from_document(document, 2);
    assert(scene.instance_count() == 2);
    assert(scene.stats().unique_shapes == 1);
    assert(scene.shape_id(0) == scene.shape_id(1));
    assert(scene.stats().cache_bytes == 0);
    assert(scene.point_count(0) == 8);
    exact(scene.point(0, 0), document.bodies[0].vertices[0]);
    exact_document(document, scene.materialize());
    assert(scene.stats().logical_point_bytes == 16 * sizeof(Point3D));
    assert(scene.stats().cache_bytes <= scene.stats().cache_limit_bytes);
    scene.clear_cache();
    assert(scene.stats().cache_bytes == 0);

    // Face topology, face appearance, hints, and signed-zero point bits all
    // participate in exact sharing; body appearance remains per instance.
    document.bodies[1].faces[0].area_hint = 5.5;
    assert(CurveMemoryScene::from_document(document).stats().unique_shapes == 2);
    document.bodies[1] = document.bodies[0];
    document.bodies[1].faces[0].color.red = 0.5;
    assert(CurveMemoryScene::from_document(document).stats().unique_shapes == 2);
    document.bodies[1] = document.bodies[0];
    document.bodies[1].faces[0].vertex_indices[0] = 1;
    assert(CurveMemoryScene::from_document(document).stats().unique_shapes == 2);
    document.bodies[1] = document.bodies[0];
    document.bodies[1].vertices[0].x = 0.0;
    assert(CurveMemoryScene::from_document(document).stats().unique_shapes == 2);
    document.bodies[1] = document.bodies[0];
    document.bodies[1].volume_hint += 1.0;
    assert(CurveMemoryScene::from_document(document).stats().unique_shapes == 2);
}

void placements_edits_and_relations() {
    auto document = adaptivecad::tool::make_box_scene_document(2.0, 2.0, 2.0);
    document.bodies[0].faces[0].area_hint = 4.0;
    auto scene = CurveMemoryScene::from_document(document);
    CurveMemoryAffine affine;
    affine.values = {0.0, -3.0, 0.0, 10.0,
                     2.0, 0.0, 0.0, 20.0,
                     0.0, 0.0, 4.0, 30.0,
                     0.0, 0.0, 0.0, 1.0};
    const auto placed = scene.add_instance(0, affine, "Placed box");
    assert(placed == 1);
    exact(scene.point(placed, 0), Point3D{13.0, 18.0, 26.0});
    exact(scene.point(0, 0), Point3D{-1.0, -1.0, -1.0});
    scene.set_point(placed, 0, Point3D{1.0, 2.0, 3.0});
    exact(scene.point(placed, 0), Point3D{4.0, 22.0, 42.0});
    exact(scene.point(0, 0), Point3D{-1.0, -1.0, -1.0});
    assert(scene.stats().overrides == 1);

    const auto cloned = scene.add_instance(placed, CurveMemoryAffine::translation(5.0, 0.0, 0.0), "Clone edits");
    exact(scene.point(cloned, 0), Point3D{6.0, 2.0, 3.0});
    scene.set_point(placed, 0, Point3D{-1.0, -1.0, -1.0});
    assert(scene.stats().overrides == 1);
    exact(scene.point(cloned, 0), Point3D{6.0, 2.0, 3.0});

    scene.add_relation(0, placed);
    scene.add_relation(0, placed);
    assert(scene.relations().size() == 1);
    assert(scene.stats().relations == 1);
    const auto restored = scene.materialize();
    exact(restored.bodies[0].volume_hint, document.bodies[0].volume_hint);
    exact(restored.bodies[0].faces[0].area_hint, 4.0);
    assert(restored.bodies[placed].name == "Placed box");
    exact(restored.bodies[placed].volume_hint, 0.0);
    exact(restored.bodies[placed].faces[0].area_hint, 0.0);
    scene.set_point(0, 0, Point3D{-2.0, -1.0, -1.0});
    exact(scene.materialize().bodies[0].volume_hint, 0.0);
    exact(scene.materialize().bodies[0].faces[0].area_hint, 0.0);
}

void batched_reads() {
    SceneDocument document;
    document.scene_label = "Batched curve instances";
    document.metadata = {{"purpose", "unordered repeated indices across blocks"}};
    SceneBody body;
    body.name = "Shared sampled shape";
    constexpr std::size_t block = adaptivecad::tool::CurveMemoryBank::block_point_capacity;
    for (std::size_t i = 0; i < 4 * block + 17; ++i) {
        body.vertices.push_back({static_cast<double>(i), static_cast<double>(i % 13), 0.0});
    }
    body.vertices[0] = {-0.0, 0.0, std::numeric_limits<double>::denorm_min()};
    body.faces.emplace_back(std::vector<std::size_t>{0, 1, 2}, 0.0);
    document.bodies = {body, body};
    const std::array<std::size_t, 10> indices{4 * block + 16, 0, block + 7, 2 * block,
        block + 7, 1, 3 * block + 2, 0, block - 1, 4 * block + 16};
    for (const auto policy : {CurveStoragePolicy::Automatic, CurveStoragePolicy::Raw}) {
        for (const std::size_t cacheBlocks : {0U, 2U}) {
            auto scene = CurveMemoryScene::from_document(document, cacheBlocks, policy);
            assert(scene.shape_id(0) == scene.shape_id(1));
            assert(scene.stats().unique_shapes == 1);
            assert(scene.stats().budgeted_resident_bytes <= scene.stats().raw_policy_bytes);
            if (policy == CurveStoragePolicy::Raw) {
                assert(scene.stats().resident_bytes == scene.stats().raw_policy_bytes);
                assert(scene.stats().cache_limit_bytes == 0);
            }
            std::array<Point3D, indices.size()> output;
            scene.points(0, indices, output);
            for (std::size_t i = 0; i < indices.size(); ++i) {
                exact(output[i], body.vertices[indices[i]]);
                exact(output[i], scene.point(0, indices[i]));
            }
            const auto full = scene.points(0);
            assert(full.size() == body.vertices.size());
            for (std::size_t i = 0; i < full.size(); ++i) exact(full[i], body.vertices[i]);
            exact_document(document, scene.materialize());

            CurveMemoryAffine affine;
            affine.values = {0.0, -3.0, 0.0, 10.0,
                             2.0, 0.0, 0.0, 20.0,
                             0.0, 0.0, 4.0, 30.0,
                             0.0, 0.0, 0.0, 1.0};
            const auto placed = scene.add_instance(0, affine, "Placed instance");
            scene.set_point(placed, block + 7, {1.0, 2.0, 3.0});
            scene.set_point(placed, 0, {-0.0, 0.0, 5.0});
            scene.add_relation(0, placed);
            scene.clear_cache();
            scene.points(placed, indices, output);
            for (std::size_t i = 0; i < indices.size(); ++i) {
                exact(output[i], scene.point(placed, indices[i]));
            }
            exact(output[2], Point3D{4.0, 22.0, 42.0});
            exact(output[4], output[2]);
            const auto placedFull = scene.points(placed);
            for (std::size_t i = 0; i < placedFull.size(); ++i) exact(placedFull[i], scene.point(placed, i));
            // Cache eviction cannot alter sharing, edits, relations, or metadata.
            const auto restored = scene.materialize();
            scene.clear_cache();
            assert(scene.stats().cache_bytes == 0);
            assert(scene.shape_id(placed) == scene.shape_id(0));
            assert(scene.stats().overrides == 2);
            assert(scene.relations().size() == 1);
            exact_document(restored, scene.materialize());
            scene.points(0, indices, output);
            for (std::size_t i = 0; i < indices.size(); ++i) exact(output[i], body.vertices[indices[i]]);

            std::array<std::size_t, 3> invalid{0, block + 7, body.vertices.size()};
            std::array<Point3D, 3> unchanged{Point3D{-7.0, 8.0, -0.0}, Point3D{1.0, -2.0, 3.0}, Point3D{0.0, 4.0, 5.0}};
            const auto expected = unchanged;
            scene.clear_cache();
            throws<std::out_of_range>([&] { scene.points(placed, invalid, unchanged); });
            assert(scene.stats().cache_bytes == 0);
            for (std::size_t i = 0; i < unchanged.size(); ++i) exact(unchanged[i], expected[i]);
            throws<std::invalid_argument>([&] { scene.points(0, indices, unchanged); });
            throws<std::invalid_argument>([&] {
                scene.points(0, std::span<const std::size_t>(indices).first(1), unchanged);
            });
            for (std::size_t i = 0; i < unchanged.size(); ++i) exact(unchanged[i], expected[i]);
            throws<std::out_of_range>([&] { scene.points(scene.instance_count(), indices, output); });
            throws<std::out_of_range>([&] { (void)scene.points(scene.instance_count()); });
            scene.points(placed, {}, {});
            assert(scene.stats().cache_bytes == 0);
        }
    }
}

void total_scene_cost_selection() {
    SceneDocument document;
    document.scene_label = "Topology-heavy compressible points";
    SceneBody body;
    body.name = "Dense samples";
    body.vertices.assign(1024, Point3D{1.0, -0.0, 2.0});
    document.bodies.push_back(body);
    adaptivecad::tool::CurveMemoryBank bank(2);
    bank.add_points(body.vertices);
    assert(bank.stats().compressed_curves == 1);

    auto dense = CurveMemoryScene::from_document(document, 2);
    assert(dense.bank().stats().compressed_curves == 1);
    const auto denseCost = dense.stats();
    const auto minimumSaving = denseCost.raw_policy_bytes / 20 + (denseCost.raw_policy_bytes % 20 != 0);
    assert(denseCost.raw_policy_bytes - denseCost.budgeted_resident_bytes >= minimumSaving);
    exact_document(document, dense.materialize());

    // The identical point payload still compresses, but its saving cannot pay
    // for the decode penalty when total scene topology dominates the memory.
    document.bodies[0].faces.assign(4096, SceneFace(std::vector<std::size_t>{0, 1, 2}));
    document.bodies.push_back(document.bodies[0]);
    document.bodies[1].name = "Shared topology instance";
    document.metadata = {{"selection", "topology and metadata included"}};
    auto heavy = CurveMemoryScene::from_document(document, 2);
    assert(heavy.bank().stats().raw_curves == 1);
    assert(heavy.bank().stats().compressed_curves == 0);
    assert(heavy.shape_id(0) == heavy.shape_id(1));
    assert(heavy.stats().resident_bytes == heavy.stats().raw_policy_bytes);
    assert(heavy.stats().cache_limit_bytes == 0);
    exact_document(document, heavy.materialize());
    const auto placed = heavy.add_instance(0, CurveMemoryAffine::translation(4.0, 5.0, 6.0), "Raw placed");
    heavy.set_point(placed, 2, {2.0, 3.0, 4.0});
    heavy.add_relation(0, placed);
    const std::array<std::size_t, 3> indices{2, 0, 2};
    std::array<Point3D, indices.size()> output;
    heavy.points(placed, indices, output);
    exact(output[0], Point3D{6.0, 8.0, 10.0});
    exact(output[2], output[0]);
    exact(heavy.point(0, 2), body.vertices[2]);
    assert(heavy.shape_id(placed) == heavy.shape_id(0));
    assert(heavy.relations().size() == 1);
}

void invalid_inputs() {
    const auto document = adaptivecad::tool::make_box_scene_document();
    auto scene = CurveMemoryScene::from_document(document);
    throws<std::out_of_range>([&] { (void)scene.point(1, 0); });
    throws<std::out_of_range>([&] { (void)scene.point(0, 8); });
    throws<std::out_of_range>([&] { (void)scene.point_count(1); });
    throws<std::out_of_range>([&] { (void)scene.shape_id(1); });
    throws<std::out_of_range>([&] { scene.set_point(0, 8, {}); });
    throws<std::out_of_range>([&] { scene.set_point(1, 0, {}); });
    throws<std::out_of_range>([&] { (void)scene.add_instance(1, {}, "invalid"); });
    throws<std::out_of_range>([&] { scene.add_relation(0, 1); });
    throws<std::invalid_argument>([&] { scene.add_relation(0, 0); });
    throws<std::invalid_argument>([&] {
        scene.set_point(0, 0, {std::numeric_limits<double>::infinity(), 0.0, 0.0});
    });
    CurveMemoryAffine projective;
    projective.values[12] = 1.0;
    throws<std::invalid_argument>([&] { (void)scene.add_instance(0, projective, "projective"); });
    CurveMemoryAffine nan;
    nan.values[0] = std::numeric_limits<double>::quiet_NaN();
    throws<std::invalid_argument>([&] { (void)scene.add_instance(0, nan, "nan"); });
    throws<std::invalid_argument>([&] {
        (void)CurveMemoryAffine::translation(std::numeric_limits<double>::infinity(), 0.0, 0.0);
    });
    auto broken = document;
    broken.bodies[0].faces[0].vertex_indices[0] = 500;
    throws<std::invalid_argument>([&] { (void)CurveMemoryScene::from_document(broken); });
    assert(scene.instance_count() == 1);
    exact_document(document, scene.materialize());
}

void native_geometry_and_export() {
    const auto original = adaptivecad::tool::make_torus_scene_document(2.0, 0.5, 16, 8);
    auto scene = CurveMemoryScene::from_document(original);
    const auto second = scene.add_instance(0, CurveMemoryAffine::translation(6.0, 0.0, 0.0), "Second torus");
    assert(second == 1);
    const auto materialized = scene.materialize();
    const auto built = adaptivecad::tool::build_scene_geometry(
        materialized, adaptivecad::geometry::TopologyHealingPolicy::Strict, false);
    assert(built.diagnostic.success);
    assert(!built.has_open_cascade_topology);
    assert(built.bodies.size() == 2);
    assert(built.faces.size() == original.bodies[0].faces.size() * 2);

    const auto token = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    const auto destination = std::filesystem::temp_directory_path() / ("adaptivecad_curve_memory_" + token + ".obj");
    adaptivecad::tool::export_wavefront_obj(materialized, destination);
    const auto imported = adaptivecad::tool::import_wavefront_obj(destination);
    const auto rebuilt = adaptivecad::tool::build_scene_geometry(
        imported, adaptivecad::geometry::TopologyHealingPolicy::Strict, false);
    assert(rebuilt.diagnostic.success);
    assert(rebuilt.bodies.size() == 2);
    assert(rebuilt.faces.size() == built.faces.size());
    std::filesystem::remove(destination);

    SceneDocument empty;
    empty.scene_label = "Empty scene";
    auto emptyScene = CurveMemoryScene::from_document(empty);
    assert(emptyScene.stats().unique_shapes == 0);
    exact_document(empty, emptyScene.materialize());
}

void parametric_evaluation_and_retention() {
    const auto line = ParametricCurveDefinition::line({-0.0, 2.0, 4.0}, {8.0, 6.0, -0.0}, 33);
    exact(line.evaluate(0), line.control_points[0]);
    exact(line.evaluate(32), line.control_points[1]);
    exact(line.evaluate(16), Point3D{4.0, 4.0, 2.0});
    const auto bezier = ParametricCurveDefinition::cubic_bezier(
        {0.0, 0.0, -0.0}, {0.0, 4.0, 0.0}, {4.0, 4.0, 0.0}, {4.0, 0.0, -0.0}, 65);
    exact(bezier.evaluate(0), bezier.control_points[0]);
    exact(bezier.evaluate(64), bezier.control_points[3]);
    exact(bezier.evaluate(32), Point3D{2.0, 3.0, 0.0});
    const auto circle = ParametricCurveDefinition::circle(
        {1.0, 2.0, 3.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 2.0, 32);
    exact(circle.evaluate(0), Point3D{3.0, 2.0, 3.0});
    const auto quarter = circle.evaluate(8);
    assert(std::abs(quarter.x - 1.0) < 1e-15 && quarter.y == 4.0 && quarter.z == 3.0);
    const auto last = circle.evaluate(31);
    assert(last.y != circle.evaluate(0).y); // No repeated endpoint.

    const std::array definitions{line, bezier, circle, line};
    auto scene = CurveMemoryScene::from_parametric(definitions);
    assert(scene.instance_count() == definitions.size());
    assert(scene.stats().unique_shapes == 3);
    assert(scene.stats().parametric_shapes == 3);
    assert(scene.shape_id(0) == scene.shape_id(3));
    assert(scene.bank().curve_count() == 0);
    assert(scene.stats().cache_bytes == 0 && scene.stats().cache_limit_bytes == 0);
    const auto materialized = scene.materialize();
    for (std::size_t instance = 0; instance < definitions.size(); ++instance) {
        const auto& definition = definitions[instance];
        assert(scene.parametric_definition(instance) != nullptr);
        assert(scene.point_count(instance) == definition.sample_count);
        assert(materialized.bodies[instance].faces.empty());
        const auto points = scene.points(instance);
        for (std::size_t i = 0; i < definition.sample_count; ++i) {
            exact(points[i], definition.evaluate(i));
            exact(scene.point(instance, i), points[i]);
            exact(materialized.bodies[instance].vertices[i], points[i]);
        }
        const std::array<std::size_t, 6> indices{definition.sample_count - 1, 0, 1, 3, 1, 0};
        std::array<Point3D, indices.size()> batch;
        std::array<Point3D, indices.size()> direct;
        scene.points(instance, indices, batch);
        definition.evaluate(indices, direct);
        for (std::size_t i = 0; i < indices.size(); ++i) {
            exact(batch[i], direct[i]);
            exact(batch[i], definition.evaluate(indices[i]));
        }
    }

    // Creating a 16-million-sample curve retains the same metadata allocation
    // as a two-sample curve. The sampled bank stays empty even after reads.
    const std::array small{ParametricCurveDefinition::line({0, 0, 0}, {1, 2, 3}, 2)};
    const std::array large{ParametricCurveDefinition::line({0, 0, 0}, {1, 2, 3},
        ParametricCurveDefinition::maximum_samples)};
    const auto sparse = CurveMemoryScene::from_parametric(small);
    auto dense = CurveMemoryScene::from_parametric(large);
    assert(sparse.stats().resident_bytes == dense.stats().resident_bytes);
    assert(dense.stats().parametric_definition_bytes == sizeof(ParametricCurveDefinition));
    assert(dense.stats().logical_point_bytes == ParametricCurveDefinition::maximum_samples * sizeof(Point3D));
    assert(dense.stats().ordinary_shared_bytes > dense.stats().resident_bytes * 1000);
    assert(dense.stats().raw_policy_bytes > dense.stats().resident_bytes * 1000);
    exact(dense.point(0, large[0].sample_count - 1), Point3D{1, 2, 3});
    assert(dense.bank().curve_count() == 0 && dense.stats().cache_bytes == 0);
    const std::array duplicateLarge{large[0], large[0]};
    assert(CurveMemoryScene::from_parametric(duplicateLarge).stats().parametric_shapes == 1);
    auto overLimit = large[0];
    overLimit.control_points[1].z = 4.0;
    const std::array tooManySamples{large[0], overLimit};
    throws<std::length_error>([&] { (void)CurveMemoryScene::from_parametric(tooManySamples); });
    const auto shared = dense.add_instance(0, {}, "Shared large definition");
    throws<std::length_error>([&] { dense.revise_parametric(shared, small[0]); });
    assert(dense.stats().parametric_shapes == 1);
    assert(dense.shape_id(shared) == dense.shape_id(0));
    const auto empty = CurveMemoryScene::from_parametric({});
    assert(empty.stats().parametric_shapes == 0 && empty.instance_count() == 0);
}

void parametric_placement_edits_and_revision() {
    const std::array definitions{ParametricCurveDefinition::cubic_bezier(
        {-0.0, 0, 0}, {1, 2, 0}, {3, 4, 0}, {5, 6, -0.0}, 17)};
    auto scene = CurveMemoryScene::from_parametric(definitions);
    const auto placed = scene.add_instance(0, CurveMemoryAffine::translation(10, 20, 30), "Placed curve");
    scene.set_point(0, 3, {1, 1, 1});
    scene.set_point(placed, 3, {2, 3, 4});
    scene.add_relation(0, placed);
    exact(scene.point(placed, 3), Point3D{12, 23, 34});
    exact(scene.point(0, 3), Point3D{1, 1, 1});
    const std::array<std::size_t, 4> indices{16, 3, 0, 3};
    std::array<Point3D, indices.size()> output;
    scene.points(placed, indices, output);
    for (std::size_t i = 0; i < indices.size(); ++i) exact(output[i], scene.point(placed, indices[i]));
    scene.set_point(placed, 3, definitions[0].evaluate(3));
    assert(scene.stats().overrides == 1);

    const auto originalShape = scene.shape_id(0);
    const auto revised = ParametricCurveDefinition::line({-2, -3, -4}, {3, 4, 5}, 5);
    scene.revise_parametric(0, revised);
    assert(scene.shape_id(0) != originalShape && scene.shape_id(placed) == originalShape);
    assert(scene.stats().parametric_shapes == 2);
    assert(scene.stats().overrides == 0);
    assert(scene.point_count(0) == 5 && scene.point_count(placed) == 17);
    exact(scene.point(0, 0), revised.evaluate(0));
    const auto oldPoint = definitions[0].evaluate(3);
    exact(scene.point(placed, 3), Point3D{oldPoint.x + 10, oldPoint.y + 20, oldPoint.z + 30});
    assert(scene.relations().size() == 1);

    const auto revisionShape = scene.shape_id(0);
    const auto allocated = scene.stats().resident_bytes;
    for (int i = 0; i < 100; ++i) {
        auto next = revised;
        next.control_points[1].z += i;
        scene.revise_parametric(0, next);
        assert(scene.shape_id(0) == revisionShape);
        assert(scene.stats().resident_bytes == allocated);
        assert(scene.stats().parametric_shapes == 2);
    }
    // Passing the retained definition back is safe and clears sparse edits;
    // revising an identical shared definition does not consume another shape.
    scene.set_point(0, 0, {99, 99, 99});
    scene.revise_parametric(0, *scene.parametric_definition(0));
    assert(scene.stats().overrides == 0);
    const auto clone = scene.add_instance(0, {}, "Unchanged revision");
    scene.revise_parametric(clone, *scene.parametric_definition(0));
    assert(scene.shape_id(clone) == scene.shape_id(0));
    assert(scene.stats().parametric_shapes == 2);
    const auto expanded = scene.materialize();
    for (std::size_t body = 0; body < expanded.bodies.size(); ++body) {
        for (std::size_t i = 0; i < expanded.bodies[body].vertices.size(); ++i) {
            exact(expanded.bodies[body].vertices[i], scene.point(body, i));
        }
    }
    const auto imported = CurveMemoryScene::from_document(expanded);
    assert(imported.parametric_definition(0) == nullptr);
    exact_document(expanded, imported.materialize());
}

void invalid_parametric_inputs() {
    const auto line = ParametricCurveDefinition::line({0, 0, 0}, {1, 2, 3}, 5);
    throws<std::invalid_argument>([&] { (void)ParametricCurveDefinition::line({}, {}, 1); });
    throws<std::length_error>([&] {
        (void)ParametricCurveDefinition::line({}, {}, ParametricCurveDefinition::maximum_samples + 1);
    });
    throws<std::invalid_argument>([&] {
        (void)ParametricCurveDefinition::circle({}, {1, 0, 0}, {0, 1, 0}, 1, 2);
    });
    throws<std::invalid_argument>([&] {
        (void)ParametricCurveDefinition::circle({}, {2, 0, 0}, {0, 1, 0}, 1, 8);
    });
    throws<std::invalid_argument>([&] {
        (void)ParametricCurveDefinition::circle({}, {1, 0, 0}, {1, 0, 0}, 1, 8);
    });
    for (const auto radius : {0.0, -1.0, std::numeric_limits<double>::infinity()}) {
        throws<std::invalid_argument>([&] {
            (void)ParametricCurveDefinition::circle({}, {1, 0, 0}, {0, 1, 0}, radius, 8);
        });
    }
    throws<std::invalid_argument>([&] {
        (void)ParametricCurveDefinition::circle({}, {1, 0, 0}, {0, 1, 0},
            std::numeric_limits<double>::max(), 8);
    });
    auto invalid = line;
    invalid.kind = static_cast<ParametricCurveKind>(100);
    throws<std::invalid_argument>([&] { invalid.validate(); });
    invalid = line;
    invalid.control_points[0].x = std::numeric_limits<double>::quiet_NaN();
    throws<std::invalid_argument>([&] { invalid.validate(); });
    const std::array invalidDefinitions{invalid};
    throws<std::invalid_argument>([&] { (void)CurveMemoryScene::from_parametric(invalidDefinitions); });
    throws<std::out_of_range>([&] { (void)line.evaluate(5); });
    const std::array<std::size_t, 3> badIndices{0, 1, 5};
    std::array<Point3D, 3> output{Point3D{1, 2, 3}, Point3D{4, 5, 6}, Point3D{-0.0, 8, 9}};
    const auto unchanged = output;
    throws<std::out_of_range>([&] { line.evaluate(badIndices, output); });
    throws<std::invalid_argument>([&] { line.evaluate({}, output); });
    const std::array definitions{line};
    auto scene = CurveMemoryScene::from_parametric(definitions);
    throws<std::out_of_range>([&] { scene.points(0, badIndices, output); });
    throws<std::invalid_argument>([&] { scene.points(0, {}, output); });
    for (std::size_t i = 0; i < output.size(); ++i) exact(output[i], unchanged[i]);
    throws<std::invalid_argument>([&] { scene.revise_parametric(0, invalid); });
    throws<std::out_of_range>([&] { scene.revise_parametric(1, line); });
    throws<std::out_of_range>([&] { scene.set_point(0, 5, {}); });
    throws<std::out_of_range>([&] { (void)scene.parametric_definition(1); });
    exact(scene.point(0, 4), line.evaluate(4));
    auto imported = CurveMemoryScene::from_document(adaptivecad::tool::make_box_scene_document());
    throws<std::invalid_argument>([&] { imported.revise_parametric(0, line); });
    assert(imported.parametric_definition(0) == nullptr);

    const double maximum = std::numeric_limits<double>::max();
    const auto extreme = ParametricCurveDefinition::line({-maximum, -0.0, 0}, {maximum, 0, 0}, 3);
    assert(extreme.evaluate(1).x == 0 && std::isfinite(extreme.evaluate(1).x));
}

} // namespace

int main() {
    roundtrip_and_dedup();
    placements_edits_and_relations();
    batched_reads();
    total_scene_cost_selection();
    invalid_inputs();
    native_geometry_and_export();
    parametric_evaluation_and_retention();
    parametric_placement_edits_and_revision();
    invalid_parametric_inputs();
}
