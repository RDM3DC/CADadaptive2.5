#include "CurveMemoryProcessCommand.hpp"

#include "adaptivecad/tool/CurveMemoryScene.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
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
#elif defined(__linux__)
#include <unistd.h>
#endif

namespace {
using adaptivecad::geometry::Point3D;
using adaptivecad::tool::CurveMemoryAffine;
using adaptivecad::tool::CurveMemoryScene;
using adaptivecad::tool::CurveStoragePolicy;
using adaptivecad::tool::SceneDocument;
using Clock = std::chrono::steady_clock;
constexpr std::uint64_t checksum_seed = 1469598103934665603ULL;

struct Options {
    std::string model = "torus";
    CurveStoragePolicy policy = CurveStoragePolicy::Automatic;
    std::optional<std::size_t> size;
    std::size_t instances = 1;
    std::size_t cache_blocks = 4;
    std::filesystem::path input;
    std::filesystem::path report;
};

std::size_t parse_size(const std::string& name, const std::string& value,
                       std::size_t minimum, std::size_t maximum) {
    if (value.empty() || !std::all_of(value.begin(), value.end(), [](unsigned char c) {
            return c >= '0' && c <= '9';
        })) {
        throw std::invalid_argument(name + " requires an unsigned integer");
    }
    const auto number = std::stoull(value);
    if (number < minimum || number > maximum) {
        throw std::invalid_argument(name + " must be between " + std::to_string(minimum)
                                    + " and " + std::to_string(maximum));
    }
    return static_cast<std::size_t>(number);
}

Options parse_options(int argc, char** argv) {
    Options options;
    for (int i = 2; i < argc; ++i) {
        const std::string key = argv[i];
        if (++i >= argc) {
            throw std::invalid_argument(key + " requires a value");
        }
        const std::string value = argv[i];
        if (key == "--model") {
            options.model = value;
        } else if (key == "--storage") {
            if (value != "raw" && value != "auto") {
                throw std::invalid_argument("--storage must be raw or auto");
            }
            options.policy = value == "raw" ? CurveStoragePolicy::Raw : CurveStoragePolicy::Automatic;
        } else if (key == "--size") {
            options.size = parse_size(key, value, 32, 8 * 1024 * 1024);
        } else if (key == "--instances") {
            options.instances = parse_size(key, value, 1, 256);
        } else if (key == "--cache-blocks") {
            options.cache_blocks = parse_size(key, value, 0,
                adaptivecad::tool::CurveMemoryBank::maximum_cache_blocks);
        } else if (key == "--input") {
            options.input = value;
        } else if (key == "--report-json") {
            options.report = value;
        } else {
            throw std::invalid_argument("unknown curve-memory-process option: " + key);
        }
    }
    if (options.model != "torus" && options.model != "dense-curves" && options.model != "input") {
        throw std::invalid_argument("--model must be torus, dense-curves or input");
    }
    if (options.model == "input" && options.input.empty()) {
        throw std::invalid_argument("--model input requires --input PATH");
    }
    if (options.model != "input" && !options.input.empty()) {
        throw std::invalid_argument("--input requires --model input");
    }
    if (options.model == "input" && options.size) {
        throw std::invalid_argument("--size does not apply to imported input");
    }
    if (options.model == "torus" && options.size.value_or(1024) > 2048) {
        throw std::invalid_argument("torus --size is capped at 2048 (1,048,576 unique vertices)");
    }
    if (!options.report.empty() && !options.input.empty()) {
        std::error_code error;
        if (std::filesystem::equivalent(options.report, options.input, error) && !error) {
            throw std::invalid_argument("report must differ from input");
        }
    }
    return options;
}

std::string json_string(const std::string& value) {
    std::ostringstream out;
    out << '"';
    for (const unsigned char c : value) {
        if (c == '"' || c == '\\') {
            out << '\\' << static_cast<char>(c);
        } else if (c < 0x20) {
            out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                << static_cast<unsigned int>(c) << std::dec;
        } else {
            out << static_cast<char>(c);
        }
    }
    out << '"';
    return out.str();
}

struct MemorySnapshot {
    std::string phase;
    std::string provider;
    std::uint64_t resident_bytes = 0;
    std::uint64_t peak_resident_bytes = 0;
    std::optional<std::uint64_t> private_committed_bytes;
    std::optional<std::uint64_t> peak_pagefile_bytes;
};

MemorySnapshot memory_snapshot(std::string phase) {
    MemorySnapshot result;
    result.phase = std::move(phase);
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS_EX counters{};
    counters.cb = sizeof(counters);
    if (!GetProcessMemoryInfo(GetCurrentProcess(),
            reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&counters), sizeof(counters))) {
        throw std::runtime_error("GetProcessMemoryInfo failed: " + std::to_string(GetLastError()));
    }
    result.provider = "Windows GetProcessMemoryInfo";
    result.resident_bytes = counters.WorkingSetSize;
    result.peak_resident_bytes = counters.PeakWorkingSetSize;
    result.private_committed_bytes = counters.PrivateUsage;
    result.peak_pagefile_bytes = counters.PeakPagefileUsage;
#elif defined(__linux__)
    result.provider = "Linux /proc/self/status (private commit unavailable)";
    std::ifstream status("/proc/self/status");
    std::string line;
    while (std::getline(status, line)) {
        std::istringstream field(line);
        std::string key;
        std::uint64_t kilobytes = 0;
        if (field >> key >> kilobytes) {
            if (key == "VmRSS:") result.resident_bytes = kilobytes * 1024;
            if (key == "VmHWM:") result.peak_resident_bytes = kilobytes * 1024;
        }
    }
    if (!result.resident_bytes) throw std::runtime_error("cannot read process RSS from /proc/self/status");
#else
    throw std::runtime_error("process RAM measurement supports Windows and Linux only");
#endif
    return result;
}

SceneDocument build_document(const Options& options) {
    if (options.model == "input") {
        return adaptivecad::tool::import_scene_document(options.input);
    }
    if (options.model == "torus") {
        const auto major = options.size.value_or(1024);
        return adaptivecad::tool::make_torus_scene_document(100.0, 25.0, major,
                                                          std::max<std::size_t>(8, major / 4));
    }
    // Synthetic analytic cubic curves, deliberately reported separately from a
    // topological CAD mesh. Dyadic sampling can compress unusually well.
    const auto total = options.size.value_or(1024 * 1024);
    const auto curve_count = std::min<std::size_t>(256, std::max<std::size_t>(1, total / 4096));
    SceneDocument document;
    document.scene_label = "Dense generated cubic curve fixture";
    document.source_kind = adaptivecad::tool::SceneSourceKind::GeneratedPrimitive;
    document.bodies.reserve(curve_count);
    std::size_t remaining = total;
    for (std::size_t curve = 0; curve < curve_count; ++curve) {
        adaptivecad::tool::SceneBody body;
        body.name = "sampled_cubic_" + std::to_string(curve);
        const auto count = remaining / (curve_count - curve);
        remaining -= count;
        body.vertices.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            const double t = static_cast<double>(i) / 65536.0;
            body.vertices.push_back({t, t * t * (3.0 - 2.0 * t), static_cast<double>(curve) * 4.0});
        }
        document.bodies.push_back(std::move(body));
    }
    return document;
}

struct Query { std::size_t instance; std::size_t vertex; };
struct Batch {
    std::size_t instance = 0;
    std::vector<std::size_t> indices;
    std::vector<std::size_t> output_ordinals;
    std::vector<Point3D> output;
};
struct Workload {
    std::vector<Query> queries;
    std::vector<Batch> batches;
    std::vector<Point3D> output;
    double plan_ms = 0.0;
};

std::uint64_t next_random(std::uint64_t& value) {
    value ^= value >> 12;
    value ^= value << 25;
    value ^= value >> 27;
    return value * 2685821657736338717ULL;
}

Workload make_workload(const CurveMemoryScene& scene, bool random) {
    Workload work;
    if (random) {
        std::vector<std::size_t> nonempty;
        for (std::size_t i = 0; i < scene.instance_count(); ++i) {
            if (scene.point_count(i)) nonempty.push_back(i);
        }
        if (nonempty.empty()) throw std::invalid_argument("benchmark input has no points");
        std::uint64_t state = 0x43555256454d454dULL;
        work.queries.reserve(8192);
        for (std::size_t q = 0; q < 8192; ++q) {
            const auto instance = nonempty[next_random(state) % nonempty.size()];
            work.queries.push_back({instance,
                static_cast<std::size_t>(next_random(state) % scene.point_count(instance))});
        }
    } else {
        work.queries.reserve(32768);
        for (std::size_t i = 0; i < scene.instance_count() && work.queries.size() < 32768; ++i) {
            for (std::size_t vertex = 0; vertex < scene.point_count(i)
                    && work.queries.size() < 32768; ++vertex) {
                work.queries.push_back({i, vertex});
            }
        }
    }
    const auto start = Clock::now();
    // A reusable query plan groups by instance, in batches of at most 512. Reads
    // scatter back to original order, so both paths hash identical query order.
    std::map<std::size_t, std::size_t> active;
    for (std::size_t q = 0; q < work.queries.size(); ++q) {
        const auto& query = work.queries[q];
        auto found = active.find(query.instance);
        if (found == active.end() || work.batches[found->second].indices.size() == 512) {
            active[query.instance] = work.batches.size();
            work.batches.push_back(Batch{query.instance, {}, {}, {}});
        }
        auto& batch = work.batches[active[query.instance]];
        batch.indices.push_back(query.vertex);
        batch.output_ordinals.push_back(q);
    }
    for (auto& batch : work.batches) batch.output.resize(batch.indices.size());
    work.output.resize(work.queries.size());
    work.plan_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    return work;
}

struct Timing { std::string name; double ms; std::uint64_t checksum; std::size_t queries; };

Timing read_workload(const CurveMemoryScene& scene, Workload& work,
                     bool batch, std::string name) {
    const auto start = Clock::now();
    if (batch) {
        for (auto& group : work.batches) {
            scene.points(group.instance, group.indices, group.output);
            for (std::size_t i = 0; i < group.output.size(); ++i) {
                work.output[group.output_ordinals[i]] = group.output[i];
            }
        }
    } else {
        for (std::size_t q = 0; q < work.queries.size(); ++q) {
            work.output[q] = scene.point(work.queries[q].instance, work.queries[q].vertex);
        }
    }
    std::uint64_t checksum = checksum_seed;
    for (const auto& point : work.output) {
        for (const auto coordinate : {point.x, point.y, point.z}) {
            checksum ^= std::bit_cast<std::uint64_t>(coordinate);
            checksum *= 1099511628211ULL;
        }
    }
    return {std::move(name), std::chrono::duration<double, std::milli>(Clock::now() - start).count(),
            checksum, work.queries.size()};
}

void run_reads(const CurveMemoryScene& scene, Workload& work, const std::string& prefix,
               bool batch, std::vector<Timing>& results) {
    scene.clear_cache();
    const std::string kind = batch ? "_batch" : "_individual";
    results.push_back(read_workload(scene, work, batch, prefix + kind + "_cold"));
    results.push_back(read_workload(scene, work, batch, prefix + kind + "_warm"));
    if (results.back().checksum != results[results.size() - 2].checksum) {
        throw std::runtime_error("cold/warm read checksums differ");
    }
}

void write_optional(std::ostream& out, const std::optional<std::uint64_t>& value) {
    if (value) out << *value; else out << "null";
}
} // namespace

int run_curve_memory_process_command(int argc, char** argv) {
    const auto options = parse_options(argc, argv);
    std::vector<MemorySnapshot> snapshots;
    snapshots.push_back(memory_snapshot("before_load"));
    std::size_t unique_source_points = 0;
    std::size_t source_faces = 0;
    const auto build_start = Clock::now();
    auto scene = [&]() {
        auto source = build_document(options);
        for (const auto& body : source.bodies) {
            unique_source_points += body.vertices.size();
            source_faces += body.faces.size();
        }
        snapshots.push_back(memory_snapshot("source_loaded"));
        auto result = CurveMemoryScene::from_document(source, options.cache_blocks, options.policy);
        const auto original_instances = result.instance_count();
        for (std::size_t copy = 1; copy < options.instances; ++copy) {
            for (std::size_t instance = 0; instance < original_instances; ++instance) {
                result.add_instance(instance, CurveMemoryAffine::translation(
                    250.0 * static_cast<double>(copy), 0.0, 0.0), "placed_" + std::to_string(copy));
            }
        }
        snapshots.push_back(memory_snapshot("converted_with_source_alive"));
        return result;
    }(); // source and conversion temporaries are destroyed before steady measurement.
    snapshots.push_back(memory_snapshot("source_released"));
    const double build_ms = std::chrono::duration<double, std::milli>(Clock::now() - build_start).count();
    const auto cold_stats = scene.stats();
    auto random = make_workload(scene, true);
    auto sequential = make_workload(scene, false);
    snapshots.push_back(memory_snapshot("query_plans_ready"));
    std::vector<Timing> timings;
    run_reads(scene, random, "random", false, timings);
    run_reads(scene, sequential, "sequential", false, timings);
    snapshots.push_back(memory_snapshot("after_individual_reads"));
    run_reads(scene, random, "random", true, timings);
    run_reads(scene, sequential, "sequential", true, timings);
    snapshots.push_back(memory_snapshot("after_batch_reads"));
    if (timings[0].checksum != timings[4].checksum || timings[2].checksum != timings[6].checksum) {
        throw std::runtime_error("individual/batch read checksums differ");
    }
    const auto final_stats = scene.stats();
    const auto bank_stats = scene.bank().stats();
    std::ostringstream report;
    report << std::setprecision(10)
        << "{\n  \"schema\": \"adaptivecad.curve-memory-process.v1\",\n"
        << "  \"storage_policy\": " << json_string(options.policy == CurveStoragePolicy::Raw ? "raw" : "auto") << ",\n"
        << "  \"model\": " << json_string(options.model) << ",\n"
        << "  \"input\": " << json_string(options.input.string()) << ",\n"
        << "  \"size\": " << options.size.value_or(options.model == "torus" ? 1024 : options.model == "dense-curves" ? 1048576 : 0) << ",\n"
        << "  \"placement_copies\": " << options.instances << ",\n"
        << "  \"unique_source_points\": " << unique_source_points << ",\n"
        << "  \"source_faces\": " << source_faces << ",\n"
        << "  \"scene_instances\": " << cold_stats.instances << ",\n"
        << "  \"unique_shapes\": " << cold_stats.unique_shapes << ",\n"
        << "  \"cache_blocks\": " << options.cache_blocks << ",\n"
        << "  \"construction_ms\": " << build_ms << ",\n"
        << "  \"process_memory_note\": \"Actual process counters. source_released is the retained scene before query buffers; allocator-retained free memory remains counted. after_batch_reads includes reusable query plans and outputs. Peak includes loading, conversion, source overlap, allocator high water and benchmark buffers; it is not steady scene RAM. No heap trimming or working-set eviction is forced.\",\n"
        << "  \"fixture_note\": \"Torus is a native topological CAD mesh. Dense-curves is a synthetic dyadically sampled cubic fixture with no faces, not evidence of savings on imported CAD or Benchy. Additional placement copies share existing shapes and do not enlarge unique geometry.\",\n"
        << "  \"timing_note\": \"One cold pass after cache clear and one immediate warm-start pass per workload. A working set larger than cache may still miss. Batch plan groups by instance in chunks <=512 and is prepared outside read timing; plan cost reported separately. Timed reads include outputs, scatter, checksum and internal decode/sort allocations. Parent runner repeats each configuration in separate processes.\",\n"
        << "  \"batch_plan_ms\": {\"random\": " << random.plan_ms << ", \"sequential\": " << sequential.plan_ms << "},\n"
        << "  \"estimated_capacity_bytes\": {\"scene_source_released\": " << cold_stats.resident_bytes
        << ", \"scene_after_reads\": " << final_stats.resident_bytes
        << ", \"dense_shared_equivalent\": " << cold_stats.ordinary_shared_bytes
        << ", \"raw_policy_scene\": " << cold_stats.raw_policy_bytes
        << ", \"selected_full_cache_budget\": " << cold_stats.budgeted_resident_bytes
        << ", \"cache_limit\": " << cold_stats.cache_limit_bytes
        << ", \"cache_after_reads\": " << final_stats.cache_bytes << "},\n"
        << "  \"bank_storage\": {\"raw_curves\": " << bank_stats.raw_curves
        << ", \"compressed_curves\": " << bank_stats.compressed_curves
        << ", \"raw_blocks\": " << bank_stats.raw_blocks
        << ", \"compressed_blocks\": " << bank_stats.compressed_blocks
        << ", \"budgeted_resident_bytes\": " << bank_stats.budgeted_resident_bytes << "},\n"
        << "  \"process_memory\": [\n";
    for (std::size_t i = 0; i < snapshots.size(); ++i) {
        const auto& memory = snapshots[i];
        report << "    {\"phase\": " << json_string(memory.phase)
            << ", \"provider\": " << json_string(memory.provider)
            << ", \"resident_bytes\": " << memory.resident_bytes
            << ", \"peak_resident_bytes\": " << memory.peak_resident_bytes
            << ", \"private_committed_bytes\": ";
        write_optional(report, memory.private_committed_bytes);
        report << ", \"peak_pagefile_bytes\": ";
        write_optional(report, memory.peak_pagefile_bytes);
        report << '}' << (i + 1 < snapshots.size() ? ",\n" : "\n");
    }
    report << "  ],\n  \"timings\": {\n";
    for (std::size_t i = 0; i < timings.size(); ++i) {
        const auto& timing = timings[i];
        report << "    " << json_string(timing.name) << ": {\"ms\": " << timing.ms
            << ", \"queries\": " << timing.queries << ", \"checksum_hex\": \""
            << std::hex << timing.checksum << std::dec << "\"}"
            << (i + 1 < timings.size() ? ",\n" : "\n");
    }
    report << "  },\n  \"individual_batch_exact\": true\n}\n";
    if (!options.report.empty()) {
        std::ofstream output(options.report);
        output << report.str();
        output.flush();
        if (!output) throw std::runtime_error("cannot write report: " + options.report.string());
    }
    std::cout << report.str();
    return 0;
}
