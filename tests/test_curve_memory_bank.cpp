#include "adaptivecad/tool/CurveMemoryBank.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using adaptivecad::geometry::Point3D;
using adaptivecad::tool::CurveMemoryBank;
using adaptivecad::tool::CurveStoragePolicy;

void check(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

template<class Exception, class Action>
void expect_throw(Action action, const char* message) {
    try {
        action();
    } catch (const Exception&) {
        return;
    }
    throw std::runtime_error(message);
}

bool same_bits(const Point3D& a, const Point3D& b) {
    return std::bit_cast<std::uint64_t>(a.x) == std::bit_cast<std::uint64_t>(b.x)
        && std::bit_cast<std::uint64_t>(a.y) == std::bit_cast<std::uint64_t>(b.y)
        && std::bit_cast<std::uint64_t>(a.z) == std::bit_cast<std::uint64_t>(b.z);
}

void check_curve(const CurveMemoryBank& bank, std::size_t id, const std::vector<Point3D>& expected) {
    check(bank.point_count(id) == expected.size(), "point count changed");
    const auto decoded = bank.points(id);
    check(decoded.size() == expected.size(), "decoded size changed");
    for (std::size_t i = 0; i < expected.size(); ++i) {
        check(same_bits(decoded[i], expected[i]), "bulk decode changed coordinate bits");
        check(same_bits(bank.point(id, i), expected[i]), "random access changed coordinate bits");
    }
}

std::string save(const CurveMemoryBank& bank) {
    std::ostringstream output(std::ios::binary);
    bank.write(output);
    return output.str();
}

CurveMemoryBank load(const std::string& bytes, std::size_t cache = 4) {
    std::istringstream input(bytes, std::ios::binary);
    return CurveMemoryBank::read(input, cache);
}

void put_u32(std::string& bytes, std::size_t offset, std::uint32_t value) {
    for (unsigned i = 0; i < 4; ++i) {
        bytes.at(offset + i) = static_cast<char>(value >> (8 * i));
    }
}

void refresh_first_checksum(std::string& bytes) {
    // v1 header = 24 bytes, first curve header = 13, block checksum follows
    // point count (4), encoding (1) and payload length (4).
    constexpr std::size_t header = 37;
    constexpr std::size_t payload = 50;
    std::uint32_t size = 0;
    for (unsigned i = 0; i < 4; ++i) {
        size |= static_cast<std::uint32_t>(static_cast<unsigned char>(bytes.at(header + 5 + i))) << (8 * i);
    }
    std::uint32_t crc = 0xFFFFFFFFU;
    const auto add = [&crc](unsigned char byte) {
        crc ^= byte;
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
        }
    };
    for (std::size_t i = header; i < header + 9; ++i) {
        add(static_cast<unsigned char>(bytes.at(i)));
    }
    for (std::size_t i = payload; i < payload + size; ++i) {
        add(static_cast<unsigned char>(bytes.at(i)));
    }
    put_u32(bytes, header + 9, crc ^ 0xFFFFFFFFU);
}

void test_boundaries_and_ranges() {
    CurveMemoryBank bank(0);
    check(bank.curve_count() == 0, "new bank is nonempty");
    check(bank.stats().resident_bytes == sizeof(CurveMemoryBank), "empty resident accounting");
    const std::array<std::size_t, 9> lengths{0, 1, 2, 255, 256, 257, 511, 512, 513};
    std::vector<std::vector<Point3D>> curves;
    for (const auto length : lengths) {
        std::vector<Point3D> points;
        for (std::size_t i = 0; i < length; ++i) {
            points.push_back({static_cast<double>(i), static_cast<double>(i / 8), -0.0});
        }
        const auto id = bank.add_points(points);
        check(id == curves.size(), "IDs not stable and consecutive");
        check_curve(bank, id, points);
        expect_throw<std::out_of_range>([&] { (void)bank.point(id, length); }, "end point accepted");
        curves.push_back(std::move(points));
    }
    check(bank.stats().cache_bytes == 0 && bank.stats().cache_limit_bytes == 0,
          "disabled cache owns storage");
    auto restored = load(save(bank), 0);
    for (std::size_t i = 0; i < curves.size(); ++i) {
        check_curve(restored, i, curves[i]);
    }
    expect_throw<std::out_of_range>([&] { (void)bank.point_count(bank.curve_count()); }, "bad curve accepted");
    expect_throw<std::out_of_range>([&] { (void)bank.points(bank.curve_count()); }, "bad bulk curve accepted");
    expect_throw<std::out_of_range>([&] { (void)bank.point(bank.curve_count(), 0); }, "bad point curve accepted");
    expect_throw<std::length_error>([] { CurveMemoryBank invalid(CurveMemoryBank::maximum_cache_blocks + 1); },
                                   "oversized cache accepted");
    check(load(save(CurveMemoryBank())).curve_count() == 0, "empty bank round trip");
}

void test_exact_ieee_payloads_and_fallback() {
    const std::array<std::uint64_t, 12> special{
        0x0000000000000000ULL, 0x8000000000000000ULL, // positive/negative zero
        0x7FF0000000000000ULL, 0xFFF0000000000000ULL, // infinities
        0x7FF8000000001234ULL, 0xFFF800000000FEDCULL, // quiet NaN payloads
        0x7FF0000000000001ULL, 0xFFF0000000004321ULL, // signaling NaN payloads
        0x0000000000000001ULL, 0x8000000000000001ULL, // subnormals
        0x7FEFFFFFFFFFFFFFULL, 0xFFEFFFFFFFFFFFFFULL};
    CurveMemoryBank bank(2);
    std::vector<Point3D> repeated;
    for (std::size_t i = 0; i < 1025; ++i) {
        repeated.push_back({std::bit_cast<double>(special[(i / 40) % special.size()]),
                            std::bit_cast<double>(special[(i / 50 + 3) % special.size()]),
                            std::bit_cast<double>(special[(i / 70 + 6) % special.size()])});
    }
    const auto exactId = bank.add_points(repeated);
    check(bank.stats().compressed_blocks > 0, "special bit patterns did not exercise compression");
    check_curve(bank, exactId, repeated);

    std::mt19937_64 random(0x513587D2ULL);
    std::vector<Point3D> noise;
    for (std::size_t i = 0; i < 2051; ++i) {
        noise.push_back({std::bit_cast<double>(random()), std::bit_cast<double>(random()),
                         std::bit_cast<double>(random())});
    }
    CurveMemoryBank raw(1);
    const auto noiseId = raw.add_points(noise);
    check(raw.stats().raw_blocks == 1 && raw.stats().compressed_blocks == 0,
          "incompressible curve was not collapsed to raw storage");
    check(raw.stats().encoded_payload_bytes == noise.size() * 24, "raw payload accounting");
    check_curve(raw, noiseId, noise);
    check(raw.stats().cache_bytes == 0, "raw random access filled decode cache");
    check_curve(load(save(raw)), noiseId, noise);
    const auto id = bank.add_points(noise);
    auto restored = load(save(bank), 1);
    check_curve(restored, exactId, repeated);
    check_curve(restored, id, noise);
    check(save(restored) == save(bank), "archive is not deterministic after round trip");
    // Copies must not depend on lifetime or mutation of the input span.
    noise.front() = {1, 2, 3};
    check(!same_bits(raw.point(noiseId, 0), noise.front()), "bank retained caller memory");
}

void test_bank_wide_cache_and_memory() {
    CurveMemoryBank bank(2);
    std::vector<Point3D> points(1024, Point3D{12.5, -0.0, 8192});
    for (unsigned i = 0; i < 7; ++i) {
        points.front().x = static_cast<double>(i);
        bank.add_points(points);
    }
    const auto cold = bank.stats();
    check(cold.cache_bytes == 0, "cache is not initially lazy");
    check(cold.encoded_payload_bytes < cold.logical_point_bytes / 4, "repeated data failed to compress");
    check(cold.resident_bytes > cold.encoded_payload_bytes, "metadata omitted from resident bytes");
    check(cold.resident_bytes < cold.logical_point_bytes, "resident accounting erases useful saving");
    for (unsigned pass = 0; pass < 3; ++pass) {
        for (std::size_t curve = 0; curve < bank.curve_count(); ++curve) {
            for (const auto index : {0U, 256U, 768U, 512U, 0U}) {
                const auto point = bank.point(curve, index);
                check(point.x == (index == 0 ? static_cast<double>(curve) : 12.5), "eviction changed decoded data");
                const auto current = bank.stats();
                check(current.cache_bytes <= current.cache_limit_bytes, "bank-wide cache exceeded bound");
                check(current.resident_bytes == cold.resident_bytes + current.cache_bytes,
                      "resident cache storage not accounted exactly");
            }
        }
    }
    check(bank.stats().cache_bytes == bank.stats().cache_limit_bytes, "cache did not reach expected bound");
    bank.clear_cache();
    check(bank.stats().cache_bytes == 0 && bank.stats().resident_bytes == cold.resident_bytes,
          "clear_cache retained allocations");
    check(bank.point(0, 0).x == 0, "read after clear failed");
    auto restored = load(save(bank), 0);
    check(restored.stats().cache_bytes == 0, "archive restored unwanted cache state");
}

void test_total_cost_storage_selection() {
    CurveMemoryBank small(64);
    small.add_points(std::vector<Point3D>(256, Point3D{2, 4, 8}));
    check(small.stats().raw_curves == 1, "full decoded cache cost did not select raw storage");
    check(small.stats().cache_limit_bytes == 0, "raw curve advertises decoded cache");
    (void)small.point(0, 120);
    check(small.stats().cache_bytes == 0, "raw access retained a decoded block");

    CurveMemoryBank noCache(0);
    noCache.add_points(std::vector<Point3D>(256, Point3D{2, 4, 8}));
    check(noCache.stats().compressed_curves == 1, "cache-free dense curve lost useful compression");

    const std::vector<Point3D> points(8192, Point3D{2, 4, 8});
    CurveMemoryBank larger(2);
    larger.add_points(points);
    const auto cold = larger.stats();
    check(cold.compressed_curves == 1 && cold.cache_limit_bytes > 0, "useful curve was not compressed");
    check(cold.budgeted_resident_bytes * 100 <= cold.dense_equivalent_bytes * 95,
          "full budget does not preserve meaningful savings");
    for (const auto index : {0U, 256U, 512U, 768U}) {
        (void)larger.point(0, index);
        check(larger.stats().resident_bytes <= cold.budgeted_resident_bytes, "cache exceeded charged budget");
    }
    check(larger.stats().cache_bytes == cold.cache_limit_bytes, "configured two-block cache was throttled");
    auto loaded = load(save(larger), 64);
    check(loaded.stats().raw_curves == 1, "archive loading did not reapply full requested cache cost");
    check_curve(loaded, 0, points);

    CurveMemoryBank forcedRaw(0, CurveStoragePolicy::Raw);
    forcedRaw.add_points(points);
    check(forcedRaw.stats().raw_curves == 1 && forcedRaw.stats().compressed_blocks == 0,
          "forced raw policy compressed a curve");
    check(forcedRaw.stats().resident_bytes == forcedRaw.stats().dense_equivalent_bytes,
          "raw equivalent excludes retained metadata");
    std::istringstream input(save(larger), std::ios::binary);
    auto loadedRaw = CurveMemoryBank::read(input, 0, CurveStoragePolicy::Raw);
    check(loadedRaw.stats().raw_curves == 1, "forced raw archive load compressed data");
    check_curve(loadedRaw, 0, points);

    // Roughly 1-3% payload savings: rare exact repetitions among random binary64
    // values cannot pay block metadata and the required 5% retained-cost margin.
    std::mt19937_64 random(0x143751U);
    std::vector<Point3D> marginal;
    Point3D previous{};
    for (std::size_t i = 0; i < 16384; ++i) {
        if (i % 7 != 0) previous = {std::bit_cast<double>(random()), std::bit_cast<double>(random()),
                                    std::bit_cast<double>(random())};
        marginal.push_back(previous);
    }
    CurveMemoryBank marginalBank(0);
    marginalBank.add_points(marginal);
    check(marginalBank.stats().raw_curves == 1, "marginal compression passed the total-cost threshold");
    check_curve(marginalBank, 0, marginal);

    // Smooth geometric curves have a different distribution from repeated dense
    // coordinates. A fully reachable requested cache must still count in cost.
    std::vector<Point3D> torus;
    for (std::size_t i = 0; i < 4096; ++i) {
        const auto u = static_cast<double>(i) * 0.0187;
        const auto v = static_cast<double>(i) * 0.0351;
        torus.push_back({(10 + 2 * std::cos(v)) * std::cos(u),
                         (10 + 2 * std::cos(v)) * std::sin(u), 2 * std::sin(v)});
    }
    CurveMemoryBank torusBank(64);
    torusBank.add_points(torus);
    check(torusBank.stats().raw_curves == 1, "torus cache cost was ignored");
    larger.add_points({});
    check(larger.stats().cache_bytes == 0, "insertion did not reset the cache budget");
}

void test_batched_random_reads() {
    for (const auto cache : {std::size_t{0}, std::size_t{2}}) {
        for (const auto policy : {CurveStoragePolicy::Automatic, CurveStoragePolicy::Raw}) {
            CurveMemoryBank bank(cache, policy);
            std::vector<Point3D> points(8193);
            for (std::size_t i = 0; i < points.size(); ++i) {
                points[i] = {static_cast<double>(i / 16), -0.0,
                             std::bit_cast<double>(0x7FF8000000001234ULL)};
            }
            // One incompressible block in an otherwise compact curve exercises
            // mixed representations within a batch and across block boundaries.
            std::mt19937_64 noise(0x164732ULL);
            for (std::size_t i = 512; i < 768; ++i) {
                points[i] = {std::bit_cast<double>(noise()), std::bit_cast<double>(noise()),
                             std::bit_cast<double>(noise())};
            }
            const auto id = bank.add_points(points);
            check(bank.stats().compressed_curves == (policy == CurveStoragePolicy::Automatic ? 1U : 0U),
                  "batch fixture did not exercise selected representation");
            check(bank.stats().raw_blocks >= 1, "batch fixture did not exercise raw blocks");
            std::vector<std::size_t> indices{8192, 256, 255, 0, 8191, 257, 256, 0, 512, 511};
            std::mt19937_64 random(0x587223ULL);
            for (std::size_t i = 0; i < 16384; ++i) indices.push_back(random() % points.size());
            std::vector<Point3D> output(indices.size());
            bank.read_points(id, indices, output);
            for (std::size_t i = 0; i < indices.size(); ++i) {
                check(same_bits(output[i], points[indices[i]]), "batch changed bits, order or repeated index");
            }
            check(bank.stats().cache_bytes <= bank.stats().cache_limit_bytes, "batch exceeded cache bound");
            if (cache == 0) check(bank.stats().cache_bytes == 0, "cache-free batch retained decoded storage");
            const std::array<std::size_t, 4> invalid{0, 256, points.size(), 1};
            std::array<Point3D, 4> untouched;
            untouched.fill(Point3D{123, 456, 789});
            expect_throw<std::out_of_range>([&] { bank.read_points(id, invalid, untouched); },
                                             "batch accepted invalid index");
            for (const auto& point : untouched) check(same_bits(point, Point3D{123, 456, 789}),
                                                      "invalid batch partially wrote output");
            expect_throw<std::invalid_argument>([&] { bank.read_points(id, indices, untouched); },
                                                 "batch accepted mismatched output length");
            expect_throw<std::out_of_range>([&] { bank.read_points(id + 1, {}, {}); },
                                             "empty batch accepted invalid curve");
            bank.read_points(id, {}, {});
            const auto empty = bank.add_points({});
            bank.read_points(empty, {}, {});
        }
    }
}

void test_corruption_and_limits() {
    CurveMemoryBank bank(0);
    bank.add_points(std::vector<Point3D>(10, Point3D{2, 4, 8}));
    const auto original = save(bank);
    check(original.size() == 101, "unexpected fixture format size");
    for (std::size_t size = 0; size < original.size(); ++size) {
        expect_throw<std::runtime_error>([&] { (void)load(original.substr(0, size)); }, "truncated archive accepted");
    }
    for (std::size_t offset = 0; offset < original.size(); ++offset) {
        auto damaged = original;
        damaged[offset] = static_cast<char>(static_cast<unsigned char>(damaged[offset]) ^ 0x80U);
        expect_throw<std::runtime_error>([&] { (void)load(damaged); }, "single-byte corruption accepted");
    }
    expect_throw<std::runtime_error>([&] { (void)load(original + 'x'); }, "trailing archive garbage accepted");
    auto oversized = original;
    // Total curve count and total point count resource guards.
    for (std::size_t offset = 16; offset < 24; ++offset) oversized[offset] = static_cast<char>(0xFF);
    expect_throw<std::runtime_error>([&] { (void)load(oversized); }, "oversized curve count accepted");
    oversized = original;
    for (std::size_t offset = 24; offset < 32; ++offset) oversized[offset] = static_cast<char>(0xFF);
    expect_throw<std::runtime_error>([&] { (void)load(oversized); }, "oversized point count accepted");
    oversized = original;
    put_u32(oversized, 42, 0xFFFFFFFFU);
    expect_throw<std::runtime_error>([&] { (void)load(oversized); }, "oversized payload accepted");

    // Valid CRC cannot hide an invalid descriptor: validate encoded structure at
    // load time, including zero-length and out-of-range byte windows.
    for (const auto descriptor : {0x90, 0x18, 0x09}) {
        auto malformed = original;
        malformed[74] = static_cast<char>(descriptor);
        refresh_first_checksum(malformed);
        expect_throw<std::runtime_error>([&] { (void)load(malformed); }, "invalid XOR descriptor accepted");
    }
    auto unusedPayload = original + '\0';
    put_u32(unusedPayload, 42, 52);
    refresh_first_checksum(unusedPayload);
    expect_throw<std::runtime_error>([&] { (void)load(unusedPayload); }, "unused compressed bytes accepted");

    CurveMemoryBank raw;
    raw.add_points(std::vector<Point3D>{{1, 2, 3}});
    auto badRaw = save(raw);
    badRaw.back() ^= 1;
    expect_throw<std::runtime_error>([&] { (void)load(badRaw); }, "raw checksum failure accepted");
    std::ostream failed(nullptr);
    expect_throw<std::runtime_error>([&] { bank.write(failed); }, "failed write was not reported");
}

} // namespace

int main() {
    try {
        test_boundaries_and_ranges();
        test_exact_ieee_payloads_and_fallback();
        test_bank_wide_cache_and_memory();
        test_total_cost_storage_selection();
        test_batched_random_reads();
        test_corruption_and_limits();
        std::cout << "CurveMemoryBank: exact recovery, cache, fallback, and persistence checks passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
