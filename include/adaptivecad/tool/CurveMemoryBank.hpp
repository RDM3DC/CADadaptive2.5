#pragma once

#include "adaptivecad/geometry/GeometryAdapter.hpp"

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <span>
#include <vector>

namespace adaptivecad::tool {

using CurveId = std::size_t;

enum class CurveStoragePolicy { Automatic, Raw };
enum class CurveCachePolicy { Lru, Arp };

struct CurveCacheOptions {
    CurveCachePolicy policy = CurveCachePolicy::Lru;
    std::size_t budget_bytes = 256 * 1024;
};

struct CurveCacheMetrics {
    std::uint64_t hits = 0;
    std::uint64_t misses = 0;
    std::uint64_t decoded_blocks = 0;
    std::uint64_t decoded_points = 0;
    std::uint64_t evictions = 0;
    std::uint64_t admission_rejections = 0;
    // One temporary decoded block can coexist with a full cache on a miss.
    // Batch ordinal scratch and caller output buffers are not included.
    std::size_t transient_decode_peak_bytes = 0;
};

struct CurveMemoryStats {
    std::size_t logical_point_bytes = 0;
    std::size_t encoded_payload_bytes = 0;
    // Includes sizeof(bank) and all owned vector capacities, including cache.
    // Allocator bookkeeping, temporaries and vectors returned to callers are excluded.
    std::size_t resident_bytes = 0;
    std::size_t cache_bytes = 0;
    std::size_t cache_limit_bytes = 0;
    // cache_bytes includes this controller subset; no controller is allocated for LRU.
    std::size_t cache_controller_bytes = 0;
    std::size_t cache_requested_budget_bytes = 0;
    std::size_t cache_entries = 0;
    // Same bank/curve metadata with every curve stored as a dense point vector.
    std::size_t dense_equivalent_bytes = 0;
    // Retained storage plus the full configured, reachable decoded-cache budget.
    std::size_t budgeted_resident_bytes = 0;
    std::size_t raw_curves = 0;
    std::size_t compressed_curves = 0;
    std::size_t raw_blocks = 0;
    std::size_t compressed_blocks = 0;
};

// Lossless sampled-point storage. Coordinates retain every binary64 bit, including
// signed zero and NaN payloads. This is an XOR byte codec, not a geometric fitter
// or an implementation of the proposed Curve Memory Alphabet algorithm.
//
// Curves are immutable after insertion. Const reads mutate a bank-wide cache;
// a bank is therefore not thread safe. External synchronization is required even
// for concurrent const access. Setting cacheBlocks to zero disables caching.
// Automatic storage requires at least 5% less total per-curve memory than raw,
// including curve/block metadata, full reachable cache entries and ARP history.
// This conservative per-curve charge permits the configured
// bank-wide cache without relying on cache throttling to claim a saving.
class CurveMemoryBank {
public:
    using CurveId = adaptivecad::tool::CurveId;
    static constexpr std::size_t block_point_capacity = 256;
    static constexpr std::size_t maximum_cache_blocks = 4096;
    static constexpr std::size_t maximum_points = 16 * 1024 * 1024;
    static constexpr std::size_t maximum_curves = 1024 * 1024;

    explicit CurveMemoryBank(std::size_t cacheBlocks = 4,
                             CurveStoragePolicy policy = CurveStoragePolicy::Automatic);
    explicit CurveMemoryBank(CurveCacheOptions options,
                             CurveStoragePolicy policy = CurveStoragePolicy::Automatic);

    CurveId add_points(std::span<const geometry::Point3D> points);
    std::size_t curve_count() const noexcept;
    std::size_t point_count(CurveId curve) const;
    geometry::Point3D point(CurveId curve, std::size_t index) const;
    // Preserves request order and repeats; each touched block is decoded at most
    // once even with caching disabled. Raw reads allocate no scratch. Compressed
    // reads use O(indices.size()) ordinal scratch and one decoded block at a time.
    // Output size must match (invalid_argument); all ranges are validated before
    // output writes, so an invalid index leaves caller output unchanged.
    void read_points(CurveId curve, std::span<const std::size_t> indices,
                     std::span<geometry::Point3D> output) const;
    std::vector<geometry::Point3D> points(CurveId curve) const;
    // Releases decoded entries and policy history; metrics are kept separately.
    void clear_cache() const;
    // Changes only the cache, preserving the encoded representation and metrics.
    void set_cache_options(CurveCacheOptions options);
    CurveCacheOptions cache_options() const noexcept;
    CurveCacheMetrics cache_metrics() const noexcept;
    void reset_cache_metrics() const noexcept;
    CurveMemoryStats stats() const;
    // Per-curve descriptor cost for external hypothetical dense-bank estimates.
    static std::size_t raw_curve_metadata_bytes() noexcept;

    // Portable little-endian v1 format with checksummed blocks. read() validates
    // every compressed block eagerly, rejects trailing data, and enforces the
    // public point/curve limits before allocation. Cache contents are not saved.
    // Invalid archives and failed I/O throw std::runtime_error; invalid IDs or
    // point indices throw std::out_of_range; resource limits throw length_error.
    void write(std::ostream& output) const;
    // Loading reapplies the selected storage/cache policy to validated v1 data.
    static CurveMemoryBank read(std::istream& input, std::size_t cacheBlocks = 4,
                                CurveStoragePolicy policy = CurveStoragePolicy::Automatic);

private:
    struct Block {
        std::size_t point_count = 0;
        bool compressed = false;
        std::vector<std::uint8_t> payload;
    };
    struct Curve {
        std::size_t point_count = 0;
        bool flat_raw = true;
        std::vector<geometry::Point3D> raw_points;
        std::vector<Block> blocks;
    };
    struct CacheEntry {
        CurveId curve = 0;
        std::size_t block = 0;
        std::vector<geometry::Point3D> points;
    };
    struct FrequencyCell {
        std::uint32_t count = 0;
        std::uint64_t epoch = 0;
    };
    struct CacheLayout {
        std::size_t entries = 0;
        std::size_t history_width = 0;
        std::size_t bytes = 0;
    };

    static std::vector<geometry::Point3D> decode_block(const Block& block);
    void select_storage(Curve& curve, std::span<const geometry::Point3D> points);
    const std::vector<geometry::Point3D>& cached_block(
        CurveId curve, std::size_t block, std::vector<geometry::Point3D>& scratch) const;
    CacheLayout cache_layout(std::size_t reachableBlocks) const noexcept;
    void allocate_cache() const;
    std::uint32_t frequency(CurveId curve, std::size_t block, bool reinforce) const;
    void update_cache_budget();

    CurveCacheOptions cache_options_{};
    CurveStoragePolicy storage_policy_ = CurveStoragePolicy::Automatic;
    std::size_t total_points_ = 0;
    std::size_t compressed_blocks_ = 0;
    CacheLayout cache_layout_{};
    std::vector<Curve> curves_;
    mutable std::vector<CacheEntry> cache_;
    mutable std::vector<FrequencyCell> frequency_;
    mutable std::uint64_t cache_requests_ = 0;
    mutable CurveCacheMetrics cache_metrics_{};
};

} // namespace adaptivecad::tool
