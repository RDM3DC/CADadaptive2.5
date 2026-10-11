#include "CurveMemoryCommand.hpp"

#include "adaptivecad/tool/CurveMemoryScene.hpp"
#include "adaptivecad/tool/SceneDocument.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cwctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using adaptivecad::geometry::Point3D;
using adaptivecad::tool::CurveMemoryAffine;
using adaptivecad::tool::CurveMemoryScene;
using adaptivecad::tool::SceneDocument;
using Clock = std::chrono::steady_clock;

constexpr std::size_t benchmark_repeats = 3;
constexpr std::uint64_t checksum_seed = 1469598103934665603ULL;

struct Options {
    bool demo = false;
    std::filesystem::path input;
    std::optional<std::filesystem::path> output;
    std::optional<std::filesystem::path> report;
    std::size_t instances = 128;
    std::size_t cache_blocks = 4;
    adaptivecad::tool::CurveStoragePolicy policy = adaptivecad::tool::CurveStoragePolicy::Automatic;
};

bool same_destination(const std::filesystem::path& left, const std::filesystem::path& right) {
    std::error_code error;
    if (std::filesystem::equivalent(left, right, error) && !error) {
        return true;
    }
    auto a = std::filesystem::weakly_canonical(left).native();
    auto b = std::filesystem::weakly_canonical(right).native();
#ifdef _WIN32
    // Windows paths are normally case insensitive, including absent outputs.
    std::transform(a.begin(), a.end(), a.begin(), [](wchar_t c) {
        return static_cast<wchar_t>(std::towlower(c));
    });
    std::transform(b.begin(), b.end(), b.begin(), [](wchar_t c) {
        return static_cast<wchar_t>(std::towlower(c));
    });
#endif
    return a == b;
}

std::size_t unsigned_option(const std::string& name, const std::string& value,
                            std::size_t minimum, std::size_t maximum) {
    if (value.empty() || !std::all_of(value.begin(), value.end(), [](unsigned char c) {
            return c >= '0' && c <= '9';
        })) {
        throw std::invalid_argument(name + " requires an unsigned integer");
    }
    std::size_t consumed = 0;
    unsigned long long number = 0;
    try {
        number = std::stoull(value, &consumed);
    } catch (const std::exception&) {
        throw std::invalid_argument(name + " is out of range");
    }
    if (consumed != value.size() || number < minimum || number > maximum) {
        throw std::invalid_argument(name + " must be between " + std::to_string(minimum)
                                    + " and " + std::to_string(maximum));
    }
    return static_cast<std::size_t>(number);
}

Options parse_options(int argc, char** argv) {
    Options options;
    options.demo = argc > 1 && std::string(argv[1]) == "--curve-memory-demo";
    int next = 2;
    if (!options.demo) {
        if (argc <= 2 || std::string(argv[2]).starts_with("--")) {
            throw std::invalid_argument("--curve-memory requires an OBJ/STL/PLY/STEP input path");
        }
        options.input = argv[next++];
    }
    while (next < argc) {
        const std::string option = argv[next++];
        if (option != "--output" && option != "--report-json"
            && option != "--instances" && option != "--cache-blocks" && option != "--storage") {
            throw std::invalid_argument("unknown curve-memory option: " + option);
        }
        if (next >= argc || std::string(argv[next]).starts_with("--")) {
            throw std::invalid_argument(option + " requires a value");
        }
        const std::string value = argv[next++];
        if (option == "--output") {
            options.output = value;
        } else if (option == "--report-json") {
            options.report = value;
        } else if (option == "--storage") {
            if (value == "raw") options.policy = adaptivecad::tool::CurveStoragePolicy::Raw;
            else if (value != "auto") throw std::invalid_argument("--storage requires raw or auto");
        } else if (option == "--cache-blocks") {
            options.cache_blocks = unsigned_option(option, value, 0,
                adaptivecad::tool::CurveMemoryBank::maximum_cache_blocks);
        } else {
            if (!options.demo) {
                throw std::invalid_argument("--instances is available only for --curve-memory-demo");
            }
            options.instances = unsigned_option(option, value, 1, 4096);
        }
    }
    if (options.output) {
        std::string extension = options.output->extension().string();
        std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        if (extension != ".obj") {
            throw std::invalid_argument("--output requires an .obj destination");
        }
    }
    if (options.output && !options.demo && same_destination(options.input, *options.output)) {
        throw std::invalid_argument("curve-memory output must differ from the input path");
    }
    if (options.report && ((!options.demo && same_destination(options.input, *options.report))
        || (options.output && same_destination(*options.output, *options.report)))) {
        throw std::invalid_argument("curve-memory report must differ from the input and output paths");
    }
    return options;
}

bool exact_double(double left, double right) {
    return std::bit_cast<std::uint64_t>(left) == std::bit_cast<std::uint64_t>(right);
}

bool exact_point(const Point3D& left, const Point3D& right) {
    return exact_double(left.x, right.x) && exact_double(left.y, right.y)
           && exact_double(left.z, right.z);
}

void verify_source_round_trip(const SceneDocument& source, const SceneDocument& restored) {
    if (source.bodies.size() != restored.bodies.size()) {
        throw std::runtime_error("curve-memory verification failed: source body count changed");
    }
    for (std::size_t body = 0; body < source.bodies.size(); ++body) {
        const auto& a = source.bodies[body];
        const auto& b = restored.bodies[body];
        if (a.vertices.size() != b.vertices.size() || a.faces.size() != b.faces.size()) {
            throw std::runtime_error("curve-memory verification failed: source topology count changed");
        }
        for (std::size_t point = 0; point < a.vertices.size(); ++point) {
            if (!exact_point(a.vertices[point], b.vertices[point])) {
                throw std::runtime_error("curve-memory verification failed: float64 point bits changed");
            }
        }
        for (std::size_t face = 0; face < a.faces.size(); ++face) {
            if (a.faces[face].vertex_indices != b.faces[face].vertex_indices) {
                throw std::runtime_error("curve-memory verification failed: face indices changed");
            }
        }
    }
}

// The reference reader has the same local-point overrides and absolute affine
// placement semantics as CurveMemoryScene. Its point arrays are ordinary doubles.
struct ReferenceInstance {
    std::size_t source_body = 0;
    CurveMemoryAffine placement = CurveMemoryAffine::identity();
    std::vector<std::pair<std::size_t, Point3D>> overrides;
};

Point3D reference_point(const SceneDocument& source,
                        const std::vector<ReferenceInstance>& instances,
                        std::size_t instance, std::size_t vertex) {
    const auto& item = instances.at(instance);
    Point3D local = source.bodies.at(item.source_body).vertices.at(vertex);
    const auto override = std::lower_bound(item.overrides.begin(), item.overrides.end(), vertex,
        [](const auto& entry, std::size_t index) { return entry.first < index; });
    if (override != item.overrides.end() && override->first == vertex) {
        local = override->second;
    }
    const auto& m = item.placement.values;
    if (m == CurveMemoryAffine::identity().values) {
        return local;
    }
    const Point3D result{m[0] * local.x + m[1] * local.y + m[2] * local.z + m[3],
                         m[4] * local.x + m[5] * local.y + m[6] * local.z + m[7],
                         m[8] * local.x + m[9] * local.y + m[10] * local.z + m[11]};
    if (!std::isfinite(result.x) || !std::isfinite(result.y) || !std::isfinite(result.z)) {
        throw std::overflow_error("ordinary shared transformed point is not finite");
    }
    return result;
}

struct Query {
    std::size_t instance;
    std::size_t vertex;
};

std::uint64_t next_random(std::uint64_t& state) {
    state ^= state >> 12;
    state ^= state << 25;
    state ^= state >> 27;
    return state * 2685821657736338717ULL;
}

std::vector<Query> random_queries(const CurveMemoryScene& scene) {
    std::vector<std::size_t> nonempty;
    for (std::size_t i = 0; i < scene.instance_count(); ++i) {
        if (scene.point_count(i) != 0) {
            nonempty.push_back(i);
        }
    }
    std::vector<Query> queries;
    if (nonempty.empty()) {
        return queries;
    }
    queries.reserve(8192);
    std::uint64_t state = 0x43555256454d454dULL;
    for (std::size_t q = 0; q < 8192; ++q) {
        const auto instance = nonempty[next_random(state) % nonempty.size()];
        queries.push_back({instance, static_cast<std::size_t>(next_random(state) % scene.point_count(instance))});
    }
    return queries;
}

std::vector<Query> sequential_queries(const CurveMemoryScene& scene) {
    std::vector<Query> queries;
    queries.reserve(32768);
    for (std::size_t i = 0; i < scene.instance_count() && queries.size() < 32768; ++i) {
        for (std::size_t vertex = 0; vertex < scene.point_count(i) && queries.size() < 32768; ++vertex) {
            queries.push_back({i, vertex});
        }
    }
    return queries;
}

std::uint64_t mix_point(std::uint64_t checksum, const Point3D& point) {
    for (const auto value : {point.x, point.y, point.z}) {
        checksum ^= std::bit_cast<std::uint64_t>(value);
        checksum *= 1099511628211ULL;
    }
    return checksum;
}

struct Timing {
    double milliseconds = 0.0;
    std::uint64_t checksum = checksum_seed;
};

template <typename ReadPoint, typename BeforeRepeat>
Timing time_reads(const std::vector<Query>& queries, ReadPoint read_point, BeforeRepeat before_repeat) {
    Timing result;
    for (std::size_t repeat = 0; repeat < benchmark_repeats; ++repeat) {
        before_repeat();
        const auto start = Clock::now();
        for (const auto& query : queries) {
            result.checksum = mix_point(result.checksum, read_point(query.instance, query.vertex));
        }
        result.milliseconds += std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    }
    return result;
}

struct QueryBatch {
    std::size_t instance = 0;
    std::vector<std::size_t> indices;
    std::vector<std::size_t> destinations;
    std::vector<Point3D> output;
};

std::vector<QueryBatch> batch_queries(const std::vector<Query>& queries, std::size_t instances) {
    std::vector<QueryBatch> batches(instances);
    for (std::size_t i = 0; i < instances; ++i) batches[i].instance = i;
    for (std::size_t i = 0; i < queries.size(); ++i) {
        auto& batch = batches.at(queries[i].instance);
        batch.indices.push_back(queries[i].vertex);
        batch.destinations.push_back(i);
    }
    std::erase_if(batches, [](const QueryBatch& batch) { return batch.indices.empty(); });
    for (auto& batch : batches) batch.output.resize(batch.indices.size());
    return batches;
}

template <typename ReadBatch, typename BeforeRepeat>
Timing time_batch_reads(std::vector<QueryBatch>& batches, std::size_t count,
                        ReadBatch read_batch, BeforeRepeat before_repeat) {
    std::vector<Point3D> ordered(count);
    Timing result;
    for (std::size_t repeat = 0; repeat < benchmark_repeats; ++repeat) {
        before_repeat();
        const auto start = Clock::now();
        for (auto& batch : batches) {
            read_batch(batch.instance, std::span<const std::size_t>(batch.indices),
                       std::span<Point3D>(batch.output));
            for (std::size_t i = 0; i < batch.output.size(); ++i)
                ordered[batch.destinations[i]] = batch.output[i];
        }
        for (const auto& point : ordered) result.checksum = mix_point(result.checksum, point);
        result.milliseconds += std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    }
    return result;
}

std::string escaped_json(const std::string& value) {
    std::ostringstream output;
    output << '"';
    for (const unsigned char c : value) {
        switch (c) {
        case '"': output << "\\\""; break;
        case '\\': output << "\\\\"; break;
        case '\n': output << "\\n"; break;
        case '\r': output << "\\r"; break;
        case '\t': output << "\\t"; break;
        default:
            if (c < 0x20) {
                output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                       << static_cast<unsigned int>(c) << std::dec;
            } else {
                output << static_cast<char>(c);
            }
        }
    }
    output << '"';
    return output.str();
}

void write_timing(std::ostream& output, const std::string& key, const Timing& timing,
                  std::size_t query_count, bool comma = true) {
    output << "    " << escaped_json(key) << ": {\"total_ms\": " << timing.milliseconds
           << ", \"queries_per_repeat\": " << query_count
           << ", \"repeats\": " << benchmark_repeats
           << ", \"checksum_hex\": \"" << std::hex << timing.checksum << std::dec << "\"}"
           << (comma ? ",\n" : "\n");
}

} // namespace

int run_curve_memory_command(int argc, char** argv) {
    const Options options = parse_options(argc, argv);
    const SceneDocument source = options.demo
        ? adaptivecad::tool::make_torus_scene_document(1.0, 0.25, 128, 32)
        : adaptivecad::tool::import_scene_document(options.input);
    auto scene = CurveMemoryScene::from_document(source, options.cache_blocks, options.policy);
    {
        // Release the verification materialization before any resident estimates.
        const auto restored = scene.materialize();
        verify_source_round_trip(source, restored);
    }

    std::vector<ReferenceInstance> reference;
    reference.reserve(options.demo ? options.instances : source.bodies.size());
    std::vector<std::size_t> canonical_bodies(scene.stats().unique_shapes,
                                             std::numeric_limits<std::size_t>::max());
    for (std::size_t body = 0; body < source.bodies.size(); ++body) {
        auto& canonical = canonical_bodies.at(scene.shape_id(body));
        if (canonical == std::numeric_limits<std::size_t>::max()) canonical = body;
        reference.push_back({canonical, CurveMemoryAffine::identity(), {}});
    }
    if (options.demo) {
        for (std::size_t instance = 1; instance < options.instances; ++instance) {
            const auto transform = CurveMemoryAffine::translation(
                3.0 * static_cast<double>(instance % 16),
                3.0 * static_cast<double>(instance / 16), 0.0);
            const auto index = scene.add_instance(0, transform, "Torus " + std::to_string(instance));
            if (index != reference.size()) {
                throw std::runtime_error("curve-memory instance numbering is inconsistent");
            }
            reference.push_back({0, transform, {}});
        }
        if (options.instances > 1) {
            const auto last = scene.instance_count() - 1;
            auto adjusted = source.bodies.front().vertices.front();
            adjusted.z += 0.1;
            scene.set_point(last, 0, adjusted);
            reference.back().overrides.emplace_back(0, adjusted);
            scene.add_relation(0, last);
        }
    }

    scene.clear_cache();
    const auto cold_stats = scene.stats();
    const auto random = random_queries(scene);
    const auto sequential = sequential_queries(scene);
    const auto read_compressed = [&scene](std::size_t instance, std::size_t vertex) {
        return scene.point(instance, vertex);
    };
    const auto read_reference = [&source, &reference](std::size_t instance, std::size_t vertex) {
        return reference_point(source, reference, instance, vertex);
    };
    const auto clear_cache = [&scene]() { scene.clear_cache(); };
    const auto no_setup = []() {};
    const auto random_reference = time_reads(random, read_reference, no_setup);
    const auto random_cold = time_reads(random, read_compressed, clear_cache);
    // A prepass warms the configured cache. A working set larger than the cache
    // still incurs misses; this is explicitly a warm-start measurement.
    const auto prepass = time_reads(random, read_compressed, no_setup);
    const auto random_warm = time_reads(random, read_compressed, no_setup);
    const auto sequential_reference = time_reads(sequential, read_reference, no_setup);
    const auto sequential_compressed = time_reads(sequential, read_compressed, clear_cache);
    auto random_batches = batch_queries(random, scene.instance_count());
    auto sequential_batches = batch_queries(sequential, scene.instance_count());
    const auto batch_reference = [&read_reference](std::size_t instance,
            std::span<const std::size_t> indices, std::span<Point3D> output) {
        for (std::size_t i = 0; i < indices.size(); ++i) output[i] = read_reference(instance, indices[i]);
    };
    const auto batch_selected = [&scene](std::size_t instance,
            std::span<const std::size_t> indices, std::span<Point3D> output) {
        scene.points(instance, indices, output);
    };
    const auto random_batch_reference = time_batch_reads(random_batches, random.size(), batch_reference, no_setup);
    const auto random_batch_cold = time_batch_reads(random_batches, random.size(), batch_selected, clear_cache);
    const auto batch_prepass = time_batch_reads(random_batches, random.size(), batch_selected, no_setup);
    const auto random_batch_warm = time_batch_reads(random_batches, random.size(), batch_selected, no_setup);
    const auto sequential_batch_reference = time_batch_reads(sequential_batches, sequential.size(), batch_reference, no_setup);
    const auto sequential_batch_selected = time_batch_reads(sequential_batches, sequential.size(), batch_selected, clear_cache);
    if (random_reference.checksum != random_cold.checksum
        || random_reference.checksum != random_warm.checksum
        || random_reference.checksum != prepass.checksum
        || sequential_reference.checksum != sequential_compressed.checksum
        || random_reference.checksum != random_batch_reference.checksum
        || random_reference.checksum != random_batch_cold.checksum
        || random_reference.checksum != random_batch_warm.checksum
        || random_reference.checksum != batch_prepass.checksum
        || sequential_reference.checksum != sequential_batch_reference.checksum
        || sequential_reference.checksum != sequential_batch_selected.checksum) {
        throw std::runtime_error("curve-memory verification failed: read checksums differ from ordinary sharing");
    }
    const auto warm_stats = scene.stats();
    const auto bank_stats = scene.bank().stats();

    std::optional<double> materialization_ms;
    std::optional<double> export_ms;
    if (options.output) {
        const auto start = Clock::now();
        const auto restored = scene.materialize();
        materialization_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        if (restored.bodies.size() != reference.size()) {
            throw std::runtime_error("curve-memory export verification failed: instance count changed");
        }
        for (std::size_t instance = 0; instance < reference.size(); ++instance) {
            const auto& body = restored.bodies[instance];
            if (body.vertices.size() != scene.point_count(instance)) {
                throw std::runtime_error("curve-memory export verification failed: point count changed");
            }
            for (std::size_t vertex = 0; vertex < body.vertices.size(); ++vertex) {
                if (!exact_point(body.vertices[vertex], read_reference(instance, vertex))) {
                    throw std::runtime_error("curve-memory export verification failed: point bits differ");
                }
            }
        }
        const auto export_start = Clock::now();
        adaptivecad::tool::export_wavefront_obj(restored, *options.output);
        export_ms = std::chrono::duration<double, std::milli>(Clock::now() - export_start).count();
    }

    std::ostringstream report;
    report << std::setprecision(17)
           << "{\n  \"schema\": \"adaptivecad.curve-memory-benchmark.v2\",\n"
           << "  \"storage_policy\": " << escaped_json(options.policy == adaptivecad::tool::CurveStoragePolicy::Raw ? "raw" : "auto") << ",\n"
           << "  \"mode\": " << escaped_json(options.demo ? "native_torus_demo" : "imported_scene") << ",\n"
           << "  \"source\": " << escaped_json(options.demo ? "native torus, 128 x 32 segments" : options.input.string()) << ",\n"
           << "  \"measurement_scope\": \"Estimated capacity bytes for scene representation; excludes benchmark inputs, verification copies, allocator bookkeeping and process overhead. Cache is included.\",\n"
           << "  \"baseline\": \"Ordinary shared unique float64 point arrays with identical topology, instance matrices, sparse overrides and relations.\",\n"
           << "  \"source_float64_and_face_indices_round_trip\": true,\n"
           << "  \"benchmark_checksums_match\": true,\n"
           << "  \"unique_shapes\": " << cold_stats.unique_shapes << ",\n"
           << "  \"instances\": " << cold_stats.instances << ",\n"
           << "  \"relations\": " << cold_stats.relations << ",\n"
           << "  \"sparse_overrides\": " << cold_stats.overrides << ",\n"
           << "  \"logical_expanded_point_bytes\": " << cold_stats.logical_point_bytes << ",\n"
           << "  \"estimated_capacity_bytes\": {\n"
           << "    \"selected_scene_cache_empty\": " << cold_stats.resident_bytes << ",\n"
           << "    \"selected_scene_after_reads\": " << warm_stats.resident_bytes << ",\n"
           << "    \"selected_scene_full_cache_bound\": " << cold_stats.resident_bytes + cold_stats.cache_limit_bytes << ",\n"
           << "    \"ordinary_shared_scene\": " << cold_stats.ordinary_shared_bytes << ",\n"
           << "    \"raw_policy_scene\": " << cold_stats.raw_policy_bytes << ",\n"
           << "    \"selected_policy_full_budget\": " << cold_stats.budgeted_resident_bytes << ",\n"
           << "    \"cache_after_reads\": " << warm_stats.cache_bytes << ",\n"
           << "    \"cache_limit\": " << cold_stats.cache_limit_bytes << "\n  },\n"
           << "  \"encoded_point_payload_bytes\": " << bank_stats.encoded_payload_bytes << ",\n"
           << "  \"raw_blocks\": " << bank_stats.raw_blocks << ",\n"
           << "  \"compressed_blocks\": " << bank_stats.compressed_blocks << ",\n"
           << "  \"raw_curves\": " << bank_stats.raw_curves << ",\n"
           << "  \"compressed_curves\": " << bank_stats.compressed_curves << ",\n"
           << "  \"batch_timing_note\": \"Identical queries and checksums. Grouping by instance and allocating output buffers happen once outside timing for both readers. Batch calls include index validation, internal block grouping, scatter to original query order, and checksum work.\",\n"
           << "  \"timing_note\": \"Fixed deterministic query lists; three repeats; total wall-clock milliseconds including checksum work. Cold clears cache before each repeat. Warm-start uses a prepass but may still miss when the working set exceeds cache capacity.\",\n"
           << "  \"read_timings\": {\n";
    write_timing(report, "ordinary_shared_random", random_reference, random.size());
    write_timing(report, "selected_random_cold", random_cold, random.size());
    write_timing(report, "selected_random_warm_start", random_warm, random.size());
    write_timing(report, "ordinary_shared_sequential", sequential_reference, sequential.size());
    write_timing(report, "selected_sequential_cold", sequential_compressed, sequential.size());
    write_timing(report, "ordinary_shared_random_batch", random_batch_reference, random.size());
    write_timing(report, "selected_random_batch_cold", random_batch_cold, random.size());
    write_timing(report, "selected_random_batch_warm_start", random_batch_warm, random.size());
    write_timing(report, "ordinary_shared_sequential_batch", sequential_batch_reference, sequential.size());
    write_timing(report, "selected_sequential_batch_cold", sequential_batch_selected, sequential.size(), false);
    report << "  },\n  \"export\": ";
    if (options.output) {
        report << "{\"path\": " << escaped_json(options.output->string())
               << ", \"all_instance_points_verified\": true, \"materialization_ms\": " << *materialization_ms
               << ", \"obj_write_ms\": " << *export_ms << "}";
    } else {
        report << "null";
    }
    report << "\n}\n";

    if (options.report) {
        std::ofstream output(*options.report, std::ios::binary);
        output << report.str();
        output.flush();
        if (!output) {
            throw std::runtime_error("cannot write curve-memory report: " + options.report->string());
        }
    }
    std::cout << "Curve memory: " << cold_stats.instances << " instances, " << cold_stats.unique_shapes
              << " unique shapes, " << cold_stats.overrides << " sparse point overrides\n"
              << "Verified exact source float64 coordinates/face indices and matching benchmark checksums.\n"
              << "Estimated capacity bytes (representation only, including cache):\n"
              << "  Selected scene, cache empty: " << cold_stats.resident_bytes << '\n'
              << "  Selected scene, after reads: " << warm_stats.resident_bytes << '\n'
              << "  Ordinary shared scene: " << cold_stats.ordinary_shared_bytes << '\n'
              << "  Logical expanded point data: " << cold_stats.logical_point_bytes << '\n'
              << "  Encoded point payload: " << bank_stats.encoded_payload_bytes << '\n'
              << std::fixed << std::setprecision(3)
              << "Read timings, " << benchmark_repeats << " repeats (ms): ordinary random "
              << random_reference.milliseconds << ", selected cold random " << random_cold.milliseconds
              << ", selected warm-start random " << random_warm.milliseconds
              << "; ordinary sequential " << sequential_reference.milliseconds
              << ", selected sequential " << sequential_compressed.milliseconds << '\n';
    std::cout << "Selected " << bank_stats.raw_curves << " raw and " << bank_stats.compressed_curves
              << " compressed curves. Batch random (ms): ordinary " << random_batch_reference.milliseconds
              << ", selected cold " << random_batch_cold.milliseconds << ", selected warm " << random_batch_warm.milliseconds
              << "; batch sequential ordinary " << sequential_batch_reference.milliseconds
              << ", selected " << sequential_batch_selected.milliseconds << '\n';
    if (options.output) {
        std::cout << "Exported " << options.output->string() << " (all instance points verified; materialization "
                  << *materialization_ms << " ms).\n";
    }
    if (options.report) {
        std::cout << "Report: " << options.report->string() << '\n';
    }
    return 0;
}
