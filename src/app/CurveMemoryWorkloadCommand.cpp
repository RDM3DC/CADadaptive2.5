#include "CurveMemoryWorkloadCommand.hpp"

#include "adaptivecad/tool/CurveMemoryScene.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <psapi.h>
#endif

namespace {
using namespace adaptivecad::tool;
using adaptivecad::geometry::Point3D;
using Clock = std::chrono::steady_clock;
constexpr std::size_t chunk_size = 512;
constexpr std::uint64_t checksum_seed = 1469598103934665603ULL;

struct Options {
    std::string model = "sampled-curves";
    std::string trace = "mixed";
    std::string access = "individual";
    std::string selection = "lru";
    CurveStoragePolicy storage = CurveStoragePolicy::Automatic;
    CurveCacheOptions cache{CurveCachePolicy::Lru, 262144};
    std::size_t size = 1048576;
    std::size_t queries = 65536;
    std::filesystem::path input;
    std::filesystem::path report;
};

std::size_t number(const std::string& key, const std::string& value,
                   std::size_t minimum, std::size_t maximum) {
    if (value.empty() || !std::all_of(value.begin(), value.end(), [](unsigned char c) {
            return c >= '0' && c <= '9';
        })) throw std::invalid_argument(key + " requires an unsigned integer");
    const auto parsed = std::stoull(value);
    if (parsed < minimum || parsed > maximum)
        throw std::invalid_argument(key + " is outside the supported range "
            + std::to_string(minimum) + ".." + std::to_string(maximum));
    return static_cast<std::size_t>(parsed);
}

bool same_path(const std::filesystem::path& a, const std::filesystem::path& b) {
    if (a.empty() || b.empty()) return false;
    std::error_code error;
    if (std::filesystem::equivalent(a, b, error) && !error) return true;
    auto left = std::filesystem::weakly_canonical(a).wstring();
    auto right = std::filesystem::weakly_canonical(b).wstring();
#ifdef _WIN32
    return CompareStringOrdinal(left.c_str(), -1, right.c_str(), -1, TRUE) == CSTR_EQUAL;
#else
    return left == right;
#endif
}

Options parse_options(int argc, char** argv) {
    Options result;
    bool explicit_size = false;
    for (int i = 2; i < argc; ++i) {
        const std::string key = argv[i];
        if (++i == argc) throw std::invalid_argument(key + " requires a value");
        const std::string value = argv[i];
        if (key == "--model") result.model = value;
        else if (key == "--trace") result.trace = value;
        else if (key == "--access") result.access = value;
        else if (key == "--selection") result.selection = value;
        else if (key == "--input") result.input = value;
        else if (key == "--report-json") result.report = value;
        else if (key == "--size") {
            result.size = number(key, value, 32, 8 * 1024 * 1024);
            explicit_size = true;
        } else if (key == "--queries") result.queries = number(key, value, 1, 2 * 1024 * 1024);
        else if (key == "--cache-bytes") result.cache.budget_bytes = number(key, value, 0, 16 * 1024 * 1024);
        else if (key == "--cache-policy") {
            if (value != "lru" && value != "arp") throw std::invalid_argument("--cache-policy must be lru or arp");
            result.cache.policy = value == "lru" ? CurveCachePolicy::Lru : CurveCachePolicy::Arp;
        } else if (key == "--storage") {
            if (value != "raw" && value != "auto") throw std::invalid_argument("--storage must be raw or auto");
            result.storage = value == "raw" ? CurveStoragePolicy::Raw : CurveStoragePolicy::Automatic;
        } else throw std::invalid_argument("unknown curve-memory-workload option: " + key);
    }
    if (result.model != "sampled-curves" && result.model != "parametric-curves" && result.model != "input")
        throw std::invalid_argument("--model must be sampled-curves, parametric-curves or input");
    if ((result.model == "input") != !result.input.empty())
        throw std::invalid_argument("--model input requires --input; generated models do not accept --input");
    if (result.model == "input" && explicit_size) throw std::invalid_argument("--size does not apply to input");
    if (result.trace != "mixed" && result.trace != "shift" && result.trace != "random"
        && result.trace != "scan" && result.trace != "hot")
        throw std::invalid_argument("--trace must be mixed, shift, random, scan or hot");
    if (result.access != "individual" && result.access != "batch")
        throw std::invalid_argument("--access must be individual or batch");
    if (result.selection != "lru" && result.selection != "policy")
        throw std::invalid_argument("--selection must be lru or policy");
    if (result.model == "parametric-curves" && result.storage == CurveStoragePolicy::Raw)
        throw std::invalid_argument("--storage raw requires sampled-curves or input; parametric-curves retains definitions");
    if (same_path(result.report, result.input) || same_path(result.report, argv[0]))
        throw std::invalid_argument("report must differ from the input and executable");
    return result;
}

std::string quoted(const std::string& value) {
    std::ostringstream output;
    output << '"';
    for (const unsigned char character : value) {
        if (character == '"' || character == '\\') output << '\\' << static_cast<char>(character);
        else if (character < 32) output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                                     << static_cast<unsigned int>(character) << std::dec;
        else output << static_cast<char>(character);
    }
    output << '"';
    return output.str();
}

struct Memory {
    std::string phase;
    std::string provider;
    std::uint64_t resident = 0;
    std::uint64_t peak = 0;
    std::optional<std::uint64_t> committed;
    std::optional<std::uint64_t> peak_commit;
};

Memory memory(std::string phase) {
    Memory result;
    result.phase = std::move(phase);
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS_EX counters{};
    counters.cb = sizeof(counters);
    if (!GetProcessMemoryInfo(GetCurrentProcess(),
            reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&counters), sizeof(counters)))
        throw std::runtime_error("GetProcessMemoryInfo failed: " + std::to_string(GetLastError()));
    result.provider = "Windows GetProcessMemoryInfo";
    result.resident = counters.WorkingSetSize;
    result.peak = counters.PeakWorkingSetSize;
    result.committed = counters.PrivateUsage;
    result.peak_commit = counters.PeakPagefileUsage;
#elif defined(__linux__)
    result.provider = "Linux /proc/self/status";
    std::ifstream status("/proc/self/status");
    std::string line;
    while (std::getline(status, line)) {
        std::istringstream field(line);
        std::string key;
        std::uint64_t kilobytes = 0;
        if (field >> key >> kilobytes) {
            if (key == "VmRSS:") result.resident = kilobytes * 1024;
            if (key == "VmHWM:") result.peak = kilobytes * 1024;
        }
    }
    if (!result.resident) throw std::runtime_error("cannot measure /proc/self/status RSS");
#else
    throw std::runtime_error("process RAM measurement supports Windows and Linux only");
#endif
    return result;
}

std::vector<ParametricCurveDefinition> definitions(std::size_t total) {
    std::vector<ParametricCurveDefinition> result;
    const auto count = std::min<std::size_t>(4, std::max<std::size_t>(1, total / 65536));
    for (std::size_t i = 0, remaining = total; i < count; ++i) {
        const auto samples = remaining / (count - i);
        remaining -= samples;
        const auto z = static_cast<double>(i) * 16.0;
        if (i % 2 == 0) result.push_back(ParametricCurveDefinition::line(
            {0.0, 0.0, z}, {1024.0, 512.0, z}, samples));
        else result.push_back(ParametricCurveDefinition::cubic_bezier(
            {0.0, 0.0, z}, {256.0, 0.0, z}, {768.0, 512.0, z}, {1024.0, 512.0, z}, samples));
    }
    return result;
}

struct Query { std::size_t instance; std::size_t vertex; };
struct Edit { Query location; Point3D value; };
struct Batch {
    std::size_t instance = 0;
    std::vector<std::size_t> indices;
    std::vector<std::size_t> ordinals;
};
struct Chunk {
    std::size_t begin = 0;
    std::size_t count = 0;
    std::optional<Edit> edit;
    std::vector<Batch> batches;
};
struct Trace {
    std::vector<Query> queries;
    std::vector<Point3D> outputs;
    std::vector<Chunk> chunks;
    std::vector<double> chunk_ms;
    std::size_t edits = 0;
    std::size_t plan_bytes = 0;
    double plan_ms = 0;
};

std::uint64_t random_step(std::uint64_t& value) {
    value ^= value >> 12;
    value ^= value << 25;
    value ^= value >> 27;
    return value * 2685821657736338717ULL;
}

Trace make_trace(const CurveMemoryScene& scene, const Options& options) {
    const auto started = Clock::now();
    Trace trace;
    std::vector<std::size_t> blocks{0}, points{0};
    for (std::size_t i = 0; i < scene.instance_count(); ++i) {
        const auto count = scene.point_count(i);
        blocks.push_back(blocks.back() + (count + 255) / 256);
        points.push_back(points.back() + count);
    }
    if (!points.back()) throw std::invalid_argument("benchmark model contains no points");
    const auto at_block = [&](std::size_t block, std::size_t offset) {
        block %= blocks.back();
        const auto instance = static_cast<std::size_t>(std::upper_bound(blocks.begin(), blocks.end(), block) - blocks.begin() - 1);
        const auto first = (block - blocks[instance]) * 256;
        return Query{instance, first + offset % std::min<std::size_t>(256, scene.point_count(instance) - first)};
    };
    const auto at_point = [&](std::size_t point) {
        point %= points.back();
        const auto instance = static_cast<std::size_t>(std::upper_bound(points.begin(), points.end(), point) - points.begin() - 1);
        return Query{instance, point - points[instance]};
    };
    std::uint64_t random = 0x4152504341442026ULL;
    std::size_t scan = 0;
    const auto hot_count = std::min<std::size_t>(8, blocks.back());
    trace.queries.reserve(options.queries);
    trace.outputs.resize(options.queries);
    trace.chunks.reserve((options.queries + chunk_size - 1) / chunk_size);
    trace.chunk_ms.reserve(trace.chunks.capacity());
    for (std::size_t q = 0; q < options.queries; ++q) {
        Query query;
        const auto phase = options.trace == "shift" ? (q * 4 / options.queries) : 0;
        const auto hot_base = phase * (blocks.back() / 4);
        if (options.trace == "random") query = at_point(random_step(random));
        else if (options.trace == "scan") query = at_point(q);
        else if (options.trace == "hot" || options.trace == "shift" || q % 640 < 128)
            query = at_block(hot_base + random_step(random) % hot_count, random_step(random) % 256);
        else query = at_block(hot_count + scan++, 13);
        if (q % chunk_size == 0) {
            Chunk chunk;
            chunk.begin = q;
            chunk.count = std::min(chunk_size, options.queries - q);
            if ((options.trace == "mixed" || options.trace == "shift") && trace.chunks.size() % 8 == 0) {
                // Edits happen before this chunk's reads, and are immediately read.
                // Each group is bounded by edits; batching never crosses them.
                query = at_block(hot_base + trace.edits % hot_count, 0);
                chunk.edit = Edit{query, {static_cast<double>(trace.edits) + 0.25, -3.5, 7.75}};
                ++trace.edits;
            }
            trace.chunks.push_back(std::move(chunk));
        }
        trace.queries.push_back(query);
    }
    for (auto& chunk : trace.chunks) {
        std::map<std::size_t, std::size_t> groups;
        for (std::size_t i = chunk.begin; i < chunk.begin + chunk.count; ++i) {
            const auto& query = trace.queries[i];
            if (!groups.contains(query.instance)) {
                groups[query.instance] = chunk.batches.size();
                chunk.batches.push_back(Batch{query.instance, {}, {}});
            }
            auto& batch = chunk.batches[groups[query.instance]];
            batch.indices.push_back(query.vertex);
            batch.ordinals.push_back(i);
        }
        trace.plan_bytes += chunk.batches.capacity() * sizeof(Batch);
        for (const auto& batch : chunk.batches)
            trace.plan_bytes += (batch.indices.capacity() + batch.ordinals.capacity()) * sizeof(std::size_t);
    }
    trace.plan_bytes += trace.queries.capacity() * sizeof(Query) + trace.chunks.capacity() * sizeof(Chunk)
        + trace.chunk_ms.capacity() * sizeof(double);
    trace.plan_ms = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
    return trace;
}

double milliseconds(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

std::uint64_t trace_checksum(const Trace& trace) {
    std::uint64_t checksum = checksum_seed;
    const auto add = [&checksum](std::uint64_t value) {
        checksum ^= value;
        checksum *= 1099511628211ULL;
    };
    for (const auto& chunk : trace.chunks) {
        add(chunk.begin);
        add(chunk.count);
        add(chunk.edit.has_value());
        if (chunk.edit) {
            add(chunk.edit->location.instance);
            add(chunk.edit->location.vertex);
            for (const double coordinate : {chunk.edit->value.x, chunk.edit->value.y, chunk.edit->value.z})
                add(std::bit_cast<std::uint64_t>(coordinate));
        }
        for (std::size_t i = chunk.begin; i < chunk.begin + chunk.count; ++i) {
            add(trace.queries[i].instance);
            add(trace.queries[i].vertex);
        }
    }
    return checksum;
}

struct Timing { double read_ms = 0; double edit_ms = 0; double elapsed_ms = 0; };
Timing run_trace(CurveMemoryScene& scene, Trace& trace, bool batch_access) {
    Timing result;
    std::array<Point3D, chunk_size> scratch{};
    scene.clear_cache();
    scene.bank().reset_cache_metrics();
    const auto started = Clock::now();
    for (const auto& chunk : trace.chunks) {
        if (chunk.edit) {
            const auto edit_start = Clock::now();
            scene.set_point(chunk.edit->location.instance, chunk.edit->location.vertex, chunk.edit->value);
            result.edit_ms += milliseconds(edit_start);
        }
        const auto read_start = Clock::now();
        if (batch_access) {
            for (const auto& batch : chunk.batches) {
                scene.points(batch.instance, batch.indices, std::span<Point3D>(scratch.data(), batch.indices.size()));
                for (std::size_t i = 0; i < batch.indices.size(); ++i)
                    trace.outputs[batch.ordinals[i]] = scratch[i];
            }
        } else {
            for (std::size_t i = chunk.begin; i < chunk.begin + chunk.count; ++i)
                trace.outputs[i] = scene.point(trace.queries[i].instance, trace.queries[i].vertex);
        }
        const auto elapsed = milliseconds(read_start);
        trace.chunk_ms.push_back(elapsed);
        result.read_ms += elapsed;
    }
    result.elapsed_ms = milliseconds(started);
    return result;
}

bool exact(const Point3D& a, const Point3D& b) {
    return std::bit_cast<std::uint64_t>(a.x) == std::bit_cast<std::uint64_t>(b.x)
        && std::bit_cast<std::uint64_t>(a.y) == std::bit_cast<std::uint64_t>(b.y)
        && std::bit_cast<std::uint64_t>(a.z) == std::bit_cast<std::uint64_t>(b.z);
}

std::uint64_t verify(const Options& options, const Trace& trace,
                     const std::vector<ParametricCurveDefinition>& definitions) {
    // This reference is constructed only AFTER all timed reads and RAM samples.
    // Replaying sparse edits prevents final-state comparisons hiding chronology errors.
    SceneDocument source;
    if (options.model == "input") source = import_scene_document(options.input);
    std::map<std::pair<std::size_t, std::size_t>, Point3D> edits;
    std::uint64_t checksum = checksum_seed;
    for (const auto& chunk : trace.chunks) {
        if (chunk.edit) edits[{chunk.edit->location.instance, chunk.edit->location.vertex}] = chunk.edit->value;
        for (std::size_t i = chunk.begin; i < chunk.begin + chunk.count; ++i) {
            const auto& query = trace.queries[i];
            const auto edit = edits.find({query.instance, query.vertex});
            const auto expected = edit != edits.end() ? edit->second : options.model == "input"
                ? source.bodies.at(query.instance).vertices.at(query.vertex)
                : definitions.at(query.instance).evaluate(query.vertex);
            if (!exact(expected, trace.outputs[i]))
                throw std::runtime_error("bit-exact workload verification failed at read " + std::to_string(i));
            for (const double coordinate : {trace.outputs[i].x, trace.outputs[i].y, trace.outputs[i].z}) {
                checksum ^= std::bit_cast<std::uint64_t>(coordinate);
                checksum *= 1099511628211ULL;
            }
        }
    }
    return checksum;
}

double percentile(std::vector<double> samples, double fraction) {
    std::sort(samples.begin(), samples.end());
    const double position = fraction * static_cast<double>(samples.size() - 1);
    const auto index = static_cast<std::size_t>(position);
    return samples[index] + (samples[std::min(index + 1, samples.size() - 1)] - samples[index])
        * (position - static_cast<double>(index));
}

void optional(std::ostream& output, const std::optional<std::uint64_t>& value) {
    if (value) output << *value; else output << "null";
}
} // namespace

int run_curve_memory_workload_command(int argc, char** argv) {
    if (argc == 3 && std::string(argv[2]) == "--help") {
        std::cout << "Usage: adaptivecad --curve-memory-workload [options]\n"
            "  --model sampled-curves|parametric-curves|input\n"
            "  --input PATH                 Required for model=input\n"
            "  --size N                     Generated sample count (default 1048576)\n"
            "  --storage auto|raw           Raw is for sampled/imported geometry\n"
            "  --cache-policy lru|arp        Default lru\n"
            "  --cache-bytes N              Total retained cache cap (default 262144)\n"
            "  --selection lru|policy       Freeze LRU storage selection or use requested policy\n"
            "  --trace mixed|shift|random|scan|hot\n"
            "  --access individual|batch    Default individual\n"
            "  --queries N                  Ordered reads (default 65536)\n"
            "  --report-json PATH           Also write the JSON report to a file\n";
        return 0;
    }
    const auto options = parse_options(argc, argv);
    std::vector<Memory> snapshots{memory("before_construction")};
    auto generated = options.model == "input" ? std::vector<ParametricCurveDefinition>{} : definitions(options.size);
    auto selection_cache = options.cache;
    if (options.selection == "lru") selection_cache.policy = CurveCachePolicy::Lru;
    std::size_t source_points = 0, source_faces = 0;
    const auto construction_started = Clock::now();
    auto scene = [&]() {
        if (options.model == "parametric-curves") {
            source_points = options.size;
            auto result = CurveMemoryScene::from_parametric(generated, selection_cache);
            snapshots.push_back(memory("constructed_with_source_alive"));
            return result;
        }
        SceneDocument document;
        if (options.model == "input") document = import_scene_document(options.input);
        else {
            document.scene_label = "Synthetic line and cubic curves";
            document.source_kind = SceneSourceKind::GeneratedPrimitive;
            std::size_t remaining = options.size;
            for (std::size_t i = 0; i < generated.size(); ++i) {
                SceneBody body;
                body.name = "curve_" + std::to_string(i);
                const auto count = remaining / (generated.size() - i);
                remaining -= count;
                body.vertices.reserve(count);
                for (std::size_t point = 0; point < count; ++point) body.vertices.push_back(generated[i].evaluate(point));
                document.bodies.push_back(std::move(body));
            }
        }
        for (const auto& body : document.bodies) {
            source_points += body.vertices.size();
            source_faces += body.faces.size();
        }
        snapshots.push_back(memory("source_loaded"));
        auto result = CurveMemoryScene::from_document(document, selection_cache, options.storage);
        snapshots.push_back(memory("constructed_with_source_alive"));
        return result;
    }();
    scene.configure_cache(options.cache);
    const auto construction_ms = milliseconds(construction_started);
    snapshots.push_back(memory("source_released"));
    const auto cold_stats = scene.stats();
    const auto storage_stats = scene.bank().stats();
    auto trace = make_trace(scene, options);
    snapshots.push_back(memory("trace_prepared"));
    const auto timing = run_trace(scene, trace, options.access == "batch");
    snapshots.push_back(memory("after_reads"));
    const auto final_stats = scene.stats();
    const auto bank = scene.bank().stats();
    const auto metrics = scene.bank().cache_metrics();
    const auto verification_started = Clock::now();
    const auto checksum = verify(options, trace, generated);
    const auto verification_ms = milliseconds(verification_started);
    std::ostringstream report;
    report << std::setprecision(12)
        << "{\n  \"schema\": \"adaptivecad.curve-memory-workload.v1\",\n"
        << "  \"model\": " << quoted(options.model) << ",\n"
        << "  \"input\": " << quoted(options.input.string()) << ",\n"
        << "  \"storage_policy\": " << quoted(options.storage == CurveStoragePolicy::Raw ? "raw" : "auto") << ",\n"
        << "  \"cache_policy\": " << quoted(options.cache.policy == CurveCachePolicy::Lru ? "lru" : "arp") << ",\n"
        << "  \"selection\": " << quoted(options.selection) << ",\n"
        << "  \"cache_budget_bytes\": " << options.cache.budget_bytes << ",\n"
        << "  \"trace\": " << quoted(options.trace) << ",\n"
        << "  \"access\": " << quoted(options.access) << ",\n"
        << "  \"queries\": " << options.queries << ",\n"
        << "  \"edits\": " << trace.edits << ",\n"
        << "  \"source_points\": " << source_points << ",\n"
        << "  \"source_faces\": " << source_faces << ",\n"
        << "  \"scene_instances\": " << cold_stats.instances << ",\n"
        << "  \"unique_shapes\": " << cold_stats.unique_shapes << ",\n"
        << "  \"retained_parametric_definitions\": " << (options.model == "parametric-curves" ? generated.size() : 0) << ",\n"
        << "  \"construction_ms\": " << construction_ms << ",\n"
        << "  \"trace_plan_ms\": " << trace.plan_ms << ",\n"
        << "  \"read_ms\": " << timing.read_ms << ",\n"
        << "  \"edit_ms\": " << timing.edit_ms << ",\n"
        << "  \"trace_elapsed_ms\": " << timing.elapsed_ms << ",\n"
        << "  \"verification_ms\": " << verification_ms << ",\n"
        << "  \"bit_exact_source_and_edit_replay\": true,\n"
        << "  \"checksum_hex\": \"" << std::hex << checksum << std::dec << "\",\n"
        << "  \"trace_checksum_hex\": \"" << std::hex << trace_checksum(trace) << std::dec << "\",\n"
        << "  \"chunk_latency_ms\": {\"unit\": \"up to 512 ordered reads, excluding preceding edit\", \"count\": " << trace.chunks.size()
        << ", \"median\": " << percentile(trace.chunk_ms, 0.5)
        << ", \"p95\": " << percentile(trace.chunk_ms, 0.95)
        << ", \"p99\": " << percentile(trace.chunk_ms, 0.99) << "},\n"
        << "  \"benchmark_buffers_bytes\": {\"trace_plan\": " << trace.plan_bytes
        << ", \"retained_outputs\": " << trace.outputs.capacity() * sizeof(Point3D)
        << ", \"batch_stack_scratch_peak\": " << chunk_size * sizeof(Point3D)
        << ", \"reference_definition_capacity\": " << generated.capacity() * sizeof(ParametricCurveDefinition) << "},\n"
        << "  \"estimated_capacity_bytes\": {\"scene_source_released\": " << cold_stats.resident_bytes
        << ", \"scene_after_reads\": " << final_stats.resident_bytes
        << ", \"selected_full_cache_budget\": " << cold_stats.budgeted_resident_bytes
        << ", \"raw_policy_scene\": " << cold_stats.raw_policy_bytes
        << ", \"cache_after_reads\": " << bank.cache_bytes
        << ", \"cache_limit\": " << bank.cache_limit_bytes
        << ", \"cache_controller\": " << bank.cache_controller_bytes
        << ", \"cache_requested_budget\": " << bank.cache_requested_budget_bytes << "},\n"
        << "  \"bank_storage\": {\"raw_curves\": " << storage_stats.raw_curves
        << ", \"compressed_curves\": " << storage_stats.compressed_curves
        << ", \"raw_blocks\": " << storage_stats.raw_blocks
        << ", \"compressed_blocks\": " << storage_stats.compressed_blocks
        << ", \"encoded_payload_bytes\": " << storage_stats.encoded_payload_bytes
        << ", \"cache_entries_after_reads\": " << bank.cache_entries << "},\n"
        << "  \"cache_metrics\": {\"hits\": " << metrics.hits << ", \"misses\": " << metrics.misses
        << ", \"decoded_blocks\": " << metrics.decoded_blocks << ", \"decoded_points\": " << metrics.decoded_points
        << ", \"evictions\": " << metrics.evictions << ", \"admission_rejections\": " << metrics.admission_rejections
        << ", \"transient_decode_peak_bytes\": " << metrics.transient_decode_peak_bytes << "},\n"
        << "  \"process_memory_note\": \"Actual process counters. Source documents are destroyed at source_released; no heap trimming or working-set eviction is forced. Allocator-retained pages still count. Peaks include construction/source overlap and benchmark buffers. Verification and input reimport occur after the final RAM sample. Parametric construction never generates a dense source array.\",\n"
        << "  \"selection_note\": \"selection=lru ingests with LRU at the same total byte budget, then configures the requested policy without reencoding, isolating cache replacement. selection=policy measures requested-policy ingestion too. Controller metadata is charged to the cache budget. Raw and retained-parametric geometry bypass the decoded-block cache.\",\n"
        << "  \"timing_note\": \"Identical deterministic requests and edit chronology across policies and access modes. Timed read chunks contain at most 512 reads; batches group by instance only inside each chunk and scatter to original order. Edits run before their chunk and are timed separately. Plans, checksums and exact verification are outside read timing. Internal decode/sort allocations and output scatter are inside. Both access modes retain identical plan/output buffers. Caches start cold; this is one ordered workload, not a warm-cache-only claim.\",\n"
        << "  \"trace_note\": \"mixed repeats 128 hot reads across eight blocks then scans 512 other blocks, with sparse edits every eight chunks. shift moves an eight-block hot region at each quarter and edits every eight chunks. random samples all points uniformly. scan visits consecutive points. hot reuses eight blocks. Synthetic sampled and parametric fixtures evaluate the same line/cubic definitions and are not evidence about imported CAD.\",\n"
        << "  \"process_memory\": [\n";
    for (std::size_t i = 0; i < snapshots.size(); ++i) {
        const auto& snapshot = snapshots[i];
        report << "    {\"phase\": " << quoted(snapshot.phase) << ", \"provider\": " << quoted(snapshot.provider)
            << ", \"resident_bytes\": " << snapshot.resident << ", \"peak_resident_bytes\": " << snapshot.peak
            << ", \"private_committed_bytes\": ";
        optional(report, snapshot.committed);
        report << ", \"peak_pagefile_bytes\": ";
        optional(report, snapshot.peak_commit);
        report << '}' << (i + 1 < snapshots.size() ? ",\n" : "\n");
    }
    report << "  ]\n}\n";
    if (!options.report.empty()) {
        std::ofstream output(options.report);
        output << report.str();
        output.flush();
        if (!output) throw std::runtime_error("cannot write report: " + options.report.string());
    }
    std::cout << report.str();
    return 0;
}
