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

struct CurveMemoryStats {
    std::size_t logical_point_bytes = 0;
    std::size_t encoded_payload_bytes = 0;
    // Includes sizeof(bank) and all owned vector capacities, including cache.
    // Allocator bookkeeping, temporaries and vectors returned to callers are excluded.
    std::size_t resident_bytes = 0;
    std::size_t cache_bytes = 0;
    std::size_t cache_limit_bytes = 0;
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
// Curves are immutable after insertion. Const reads mutate a bank-wide LRU cache;
// a bank is therefore not thread safe. External synchronization is required even
// for concurrent const access. Setting cacheBlocks to zero disables caching.
// Automatic storage requires at least 5% less total per-curve memory than raw,
// including curve/block metadata and min(cacheBlocks, compressed block count)
// full cache entries. This conservative per-curve charge permits the configured
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
    void clear_cache() const;
    CurveMemoryStats stats() const;

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

    static std::vector<geometry::Point3D> decode_block(const Block& block);
    void select_storage(Curve& curve, std::span<const geometry::Point3D> points);
    const std::vector<geometry::Point3D>& cached_block(CurveId curve, std::size_t block) const;
    void update_cache_budget();

    std::size_t cache_blocks_ = 0;
    CurveStoragePolicy storage_policy_ = CurveStoragePolicy::Automatic;
    std::size_t total_points_ = 0;
    std::size_t compressed_blocks_ = 0;
    std::size_t effective_cache_blocks_ = 0;
    std::vector<Curve> curves_;
    mutable std::vector<CacheEntry> cache_;
};

} // namespace adaptivecad::tool
