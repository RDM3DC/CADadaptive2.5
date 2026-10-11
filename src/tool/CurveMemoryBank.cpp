#include "adaptivecad/tool/CurveMemoryBank.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <istream>
#include <limits>
#include <numeric>
#include <ostream>
#include <stdexcept>
#include <utility>

namespace adaptivecad::tool {
namespace {

static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);
constexpr std::size_t point_bytes = 3 * sizeof(double);
constexpr std::array<char, 8> magic{'A', 'C', 'V', 'C', 'M', 'B', '0', '1'};

[[noreturn]] void invalid_archive() {
    throw std::runtime_error("Invalid or truncated CurveMemoryBank archive");
}

void append_word(std::vector<std::uint8_t>& output, std::uint64_t word) {
    for (unsigned shift = 0; shift < 64; shift += 8) {
        output.push_back(static_cast<std::uint8_t>(word >> shift));
    }
}

std::uint64_t word_at(std::span<const std::uint8_t> bytes, std::size_t offset) {
    if (offset > bytes.size() || bytes.size() - offset < 8) {
        invalid_archive();
    }
    std::uint64_t word = 0;
    for (unsigned i = 0; i < 8; ++i) {
        word |= static_cast<std::uint64_t>(bytes[offset + i]) << (i * 8);
    }
    return word;
}

std::array<std::uint64_t, 3> point_bits(const geometry::Point3D& point) {
    return {std::bit_cast<std::uint64_t>(point.x),
            std::bit_cast<std::uint64_t>(point.y),
            std::bit_cast<std::uint64_t>(point.z)};
}

geometry::Point3D bits_point(const std::array<std::uint64_t, 3>& bits) {
    return {std::bit_cast<double>(bits[0]), std::bit_cast<double>(bits[1]),
            std::bit_cast<double>(bits[2])};
}

geometry::Point3D raw_point(std::span<const std::uint8_t> bytes, std::size_t index) {
    const auto offset = index * point_bytes;
    return bits_point({word_at(bytes, offset), word_at(bytes, offset + 8),
                       word_at(bytes, offset + 16)});
}

std::vector<std::uint8_t> raw_payload(std::span<const geometry::Point3D> points) {
    std::vector<std::uint8_t> bytes;
    bytes.reserve(points.size() * point_bytes);
    for (const auto& point : points) {
        for (const auto word : point_bits(point)) {
            append_word(bytes, word);
        }
    }
    return bytes;
}

std::vector<std::uint8_t> xor_payload(std::span<const geometry::Point3D> points) {
    std::vector<std::uint8_t> bytes;
    bytes.reserve(points.size() * point_bytes);
    auto previous = point_bits(points.front());
    for (const auto word : previous) {
        append_word(bytes, word);
    }
    for (std::size_t i = 1; i < points.size(); ++i) {
        const auto current = point_bits(points[i]);
        for (std::size_t coordinate = 0; coordinate < 3; ++coordinate) {
            const auto difference = current[coordinate] ^ previous[coordinate];
            if (difference == 0) {
                bytes.push_back(0);
                continue;
            }
            const auto trailing = static_cast<unsigned>(std::countr_zero(difference) / 8);
            const auto leading = static_cast<unsigned>(std::countl_zero(difference) / 8);
            const auto count = 8U - trailing - leading;
            bytes.push_back(static_cast<std::uint8_t>((trailing << 4) | count));
            for (unsigned j = 0; j < count; ++j) {
                bytes.push_back(static_cast<std::uint8_t>(difference >> ((trailing + j) * 8)));
            }
        }
        previous = current;
    }
    return bytes;
}

// CRC-32/ISO-HDLC; detects accidental corruption, not a cryptographic signature.
void crc_byte(std::uint32_t& crc, std::uint8_t byte) {
    crc ^= byte;
    for (unsigned i = 0; i < 8; ++i) {
        crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
    }
}

std::uint32_t checksum_prefix(std::size_t count, bool compressed, std::size_t payloadBytes) {
    std::uint32_t crc = 0xFFFFFFFFU;
    for (unsigned i = 0; i < 4; ++i) {
        crc_byte(crc, static_cast<std::uint8_t>(count >> (i * 8)));
    }
    crc_byte(crc, compressed ? 1 : 0);
    for (unsigned i = 0; i < 4; ++i) {
        crc_byte(crc, static_cast<std::uint8_t>(payloadBytes >> (i * 8)));
    }
    return crc;
}

std::uint32_t checksum(std::size_t count, bool compressed,
                       std::span<const std::uint8_t> payload) {
    auto crc = checksum_prefix(count, compressed, payload.size());
    for (const auto byte : payload) {
        crc_byte(crc, byte);
    }
    return crc ^ 0xFFFFFFFFU;
}

void write_number(std::ostream& output, std::uint64_t value, unsigned bytes) {
    for (unsigned i = 0; i < bytes; ++i) {
        output.put(static_cast<char>(value >> (i * 8)));
    }
}

std::uint64_t read_number(std::istream& input, unsigned bytes) {
    std::uint64_t value = 0;
    for (unsigned i = 0; i < bytes; ++i) {
        const auto byte = input.get();
        if (byte == std::char_traits<char>::eof()) {
            invalid_archive();
        }
        value |= static_cast<std::uint64_t>(static_cast<unsigned char>(byte)) << (i * 8);
    }
    return value;
}

} // namespace

CurveMemoryBank::CurveMemoryBank(std::size_t cacheBlocks, CurveStoragePolicy policy)
    : cache_blocks_(cacheBlocks), storage_policy_(policy) {
    if (cacheBlocks > maximum_cache_blocks) {
        throw std::length_error("CurveMemoryBank cache exceeds block limit");
    }
}

void CurveMemoryBank::select_storage(Curve& curve, std::span<const geometry::Point3D> points) {
    curve.point_count = points.size();
    if (storage_policy_ == CurveStoragePolicy::Raw || points.empty()) {
        return;
    }
    curve.flat_raw = false;
    const auto blockCount = (points.size() + block_point_capacity - 1) / block_point_capacity;
    curve.blocks.reserve(blockCount);
    std::size_t payloadBytes = 0;
    std::size_t compressedBlocks = 0;
    for (std::size_t start = 0; start < points.size(); start += block_point_capacity) {
        const auto blockPoints = points.subspan(start, std::min(block_point_capacity, points.size() - start));
        Block block;
        block.point_count = blockPoints.size();
        auto encoded = xor_payload(blockPoints);
        block.compressed = encoded.size() < blockPoints.size() * point_bytes;
        block.payload = block.compressed ? std::vector<std::uint8_t>(encoded.begin(), encoded.end())
                                         : raw_payload(blockPoints);
        payloadBytes += block.payload.capacity();
        compressedBlocks += block.compressed ? 1 : 0;
        curve.blocks.push_back(std::move(block));
    }
    // Charge the full reachable configured cache for this curve. Charging each
    // curve separately is deliberately conservative when several curves share
    // the bank-wide LRU. No cache is reduced merely to make compression qualify.
    const auto cacheCost = std::min(cache_blocks_, compressedBlocks) *
        (sizeof(CacheEntry) + block_point_capacity * sizeof(geometry::Point3D));
    const auto compressedCost = sizeof(Curve) + payloadBytes +
        curve.blocks.capacity() * sizeof(Block) + cacheCost;
    const auto rawCost = sizeof(Curve) + points.size() * sizeof(geometry::Point3D);
    const auto minimumSaving = (rawCost + 19) / 20; // At least 5%, rounded up.
    if (compressedBlocks == 0 || compressedCost > rawCost - minimumSaving) {
        Curve raw;
        raw.point_count = points.size();
        curve = std::move(raw);
    }
}

CurveId CurveMemoryBank::add_points(std::span<const geometry::Point3D> points) {
    if (curves_.size() >= maximum_curves || points.size() > maximum_points - total_points_) {
        throw std::length_error("CurveMemoryBank exceeds curve or point limit");
    }
    Curve curve;
    select_storage(curve, points);
    if (curve.flat_raw) curve.raw_points.assign(points.begin(), points.end());
    const auto id = curves_.size();
    curves_.push_back(std::move(curve));
    total_points_ += points.size();
    for (const auto& block : curves_.back().blocks) compressed_blocks_ += block.compressed ? 1 : 0;
    update_cache_budget();
    return id;
}

void CurveMemoryBank::update_cache_budget() {
    effective_cache_blocks_ = std::min(cache_blocks_, compressed_blocks_);
    // Insertion can change the number of reachable compressed cache entries.
    clear_cache();
}

std::size_t CurveMemoryBank::curve_count() const noexcept { return curves_.size(); }

std::size_t CurveMemoryBank::point_count(CurveId curve) const {
    return curves_.at(curve).point_count;
}

std::vector<geometry::Point3D> CurveMemoryBank::decode_block(const Block& block) {
    std::vector<geometry::Point3D> result;
    result.reserve(block.point_count);
    if (!block.compressed) {
        if (block.payload.size() != block.point_count * point_bytes) {
            invalid_archive();
        }
        for (std::size_t i = 0; i < block.point_count; ++i) {
            result.push_back(raw_point(block.payload, i));
        }
        return result;
    }
    if (block.point_count == 0 || block.point_count > block_point_capacity) {
        invalid_archive();
    }
    std::array<std::uint64_t, 3> previous{
        word_at(block.payload, 0), word_at(block.payload, 8), word_at(block.payload, 16)};
    result.push_back(bits_point(previous));
    std::size_t offset = point_bytes;
    for (std::size_t i = 1; i < block.point_count; ++i) {
        for (auto& word : previous) {
            if (offset >= block.payload.size()) {
                invalid_archive();
            }
            const auto descriptor = block.payload[offset++];
            if (descriptor == 0) {
                continue;
            }
            const unsigned trailing = descriptor >> 4;
            const unsigned count = descriptor & 0x0FU;
            if (count == 0 || count > 8 || trailing + count > 8
                || block.payload.size() - offset < count) {
                invalid_archive();
            }
            std::uint64_t difference = 0;
            for (unsigned j = 0; j < count; ++j) {
                difference |= static_cast<std::uint64_t>(block.payload[offset++]) << ((trailing + j) * 8);
            }
            word ^= difference;
        }
        result.push_back(bits_point(previous));
    }
    if (offset != block.payload.size()) {
        invalid_archive();
    }
    return result;
}

const std::vector<geometry::Point3D>& CurveMemoryBank::cached_block(CurveId curve, std::size_t block) const {
    const auto found = std::find_if(cache_.begin(), cache_.end(), [curve, block](const CacheEntry& entry) {
        return entry.curve == curve && entry.block == block;
    });
    if (found != cache_.end()) {
        std::rotate(found, found + 1, cache_.end());
        return cache_.back().points;
    }
    auto decoded = decode_block(curves_[curve].blocks[block]);
    if (cache_.capacity() < effective_cache_blocks_) {
        cache_.reserve(effective_cache_blocks_);
    }
    if (cache_.size() == effective_cache_blocks_) {
        cache_.erase(cache_.begin());
    }
    cache_.push_back(CacheEntry{curve, block, std::move(decoded)});
    return cache_.back().points;
}

geometry::Point3D CurveMemoryBank::point(CurveId curveId, std::size_t index) const {
    const auto& curve = curves_.at(curveId);
    if (index >= curve.point_count) {
        throw std::out_of_range("CurveMemoryBank point index out of range");
    }
    if (curve.flat_raw) {
        return curve.raw_points[index];
    }
    const auto blockIndex = index / block_point_capacity;
    const auto localIndex = index % block_point_capacity;
    const auto& block = curve.blocks[blockIndex];
    if (!block.compressed) {
        return raw_point(block.payload, localIndex);
    }
    if (effective_cache_blocks_ == 0) {
        return decode_block(block)[localIndex];
    }
    return cached_block(curveId, blockIndex)[localIndex];
}

void CurveMemoryBank::read_points(CurveId curveId, std::span<const std::size_t> indices,
                                  std::span<geometry::Point3D> output) const {
    const auto& curve = curves_.at(curveId);
    if (indices.size() != output.size()) {
        throw std::invalid_argument("CurveMemoryBank batch output size must match indices");
    }
    for (const auto index : indices) {
        if (index >= curve.point_count) {
            throw std::out_of_range("CurveMemoryBank batch point index out of range");
        }
    }
    if (curve.flat_raw) {
        for (std::size_t i = 0; i < indices.size(); ++i) output[i] = curve.raw_points[indices[i]];
        return;
    }
    std::vector<std::size_t> order(indices.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::sort(order.begin(), order.end(), [&indices](std::size_t a, std::size_t b) {
        return indices[a] / block_point_capacity < indices[b] / block_point_capacity;
    });
    for (std::size_t begin = 0; begin < order.size();) {
        const auto blockIndex = indices[order[begin]] / block_point_capacity;
        auto end = begin + 1;
        while (end < order.size() && indices[order[end]] / block_point_capacity == blockIndex) ++end;
        const auto& block = curve.blocks[blockIndex];
        if (!block.compressed) {
            for (auto i = begin; i < end; ++i) {
                const auto ordinal = order[i];
                output[ordinal] = raw_point(block.payload, indices[ordinal] % block_point_capacity);
            }
        } else {
            // The local decode dies at this group's end; cache=0 still decodes
            // just once for arbitrarily ordered/repeated indices in the group.
            std::vector<geometry::Point3D> scratch;
            const auto& decoded = effective_cache_blocks_ == 0
                ? (scratch = decode_block(block)) : cached_block(curveId, blockIndex);
            for (auto i = begin; i < end; ++i) {
                const auto ordinal = order[i];
                output[ordinal] = decoded[indices[ordinal] % block_point_capacity];
            }
        }
        begin = end;
    }
}

std::vector<geometry::Point3D> CurveMemoryBank::points(CurveId curveId) const {
    const auto& curve = curves_.at(curveId);
    if (curve.flat_raw) return curve.raw_points;
    std::vector<geometry::Point3D> result;
    result.reserve(curve.point_count);
    for (std::size_t i = 0; i < curve.blocks.size(); ++i) {
        const auto& block = curve.blocks[i];
        if (!block.compressed) {
            for (std::size_t j = 0; j < block.point_count; ++j) {
                result.push_back(raw_point(block.payload, j));
            }
        } else if (effective_cache_blocks_ == 0) {
            const auto decoded = decode_block(block);
            result.insert(result.end(), decoded.begin(), decoded.end());
        } else {
            const auto& decoded = cached_block(curveId, i);
            result.insert(result.end(), decoded.begin(), decoded.end());
        }
    }
    return result;
}

void CurveMemoryBank::clear_cache() const {
    std::vector<CacheEntry>().swap(cache_);
}

CurveMemoryStats CurveMemoryBank::stats() const {
    CurveMemoryStats result;
    result.logical_point_bytes = total_points_ * point_bytes;
    result.cache_limit_bytes = effective_cache_blocks_ * (sizeof(CacheEntry) + block_point_capacity * sizeof(geometry::Point3D));
    result.cache_bytes = cache_.capacity() * sizeof(CacheEntry);
    for (const auto& entry : cache_) {
        result.cache_bytes += entry.points.capacity() * sizeof(geometry::Point3D);
    }
    result.resident_bytes = sizeof(*this) + curves_.capacity() * sizeof(Curve) + result.cache_bytes;
    result.dense_equivalent_bytes = sizeof(*this) + curves_.capacity() * sizeof(Curve) +
        total_points_ * sizeof(geometry::Point3D);
    for (const auto& curve : curves_) {
        if (curve.flat_raw) {
            ++result.raw_curves;
            result.raw_blocks += curve.raw_points.empty() ? 0 : 1;
            result.encoded_payload_bytes += curve.raw_points.size() * point_bytes;
            result.resident_bytes += curve.raw_points.capacity() * sizeof(geometry::Point3D);
        } else {
            ++result.compressed_curves;
        }
        result.resident_bytes += curve.blocks.capacity() * sizeof(Block);
        for (const auto& block : curve.blocks) {
            result.encoded_payload_bytes += block.payload.size();
            result.resident_bytes += block.payload.capacity();
            if (block.compressed) {
                ++result.compressed_blocks;
            } else {
                ++result.raw_blocks;
            }
        }
    }
    result.budgeted_resident_bytes = result.resident_bytes - result.cache_bytes + result.cache_limit_bytes;
    return result;
}

void CurveMemoryBank::write(std::ostream& output) const {
    output.write(magic.data(), static_cast<std::streamsize>(magic.size()));
    write_number(output, 1, 4);
    write_number(output, block_point_capacity, 4);
    write_number(output, curves_.size(), 8);
    for (const auto& curve : curves_) {
        write_number(output, curve.point_count, 8);
        write_number(output, curve.flat_raw ? (curve.point_count == 0 ? 0 : 1) : curve.blocks.size(), 4);
        write_number(output, curve.flat_raw ? 0 : 1, 1);
        if (curve.flat_raw && curve.point_count != 0) {
            const auto payloadBytes = curve.point_count * point_bytes;
            auto crc = checksum_prefix(curve.point_count, false, payloadBytes);
            for (const auto& point : curve.raw_points) {
                for (const auto word : point_bits(point)) {
                    for (unsigned shift = 0; shift < 64; shift += 8) {
                        crc_byte(crc, static_cast<std::uint8_t>(word >> shift));
                    }
                }
            }
            write_number(output, curve.point_count, 4);
            write_number(output, 0, 1);
            write_number(output, payloadBytes, 4);
            write_number(output, crc ^ 0xFFFFFFFFU, 4);
            std::array<char, block_point_capacity * point_bytes> buffer{};
            for (std::size_t begin = 0; begin < curve.point_count; begin += block_point_capacity) {
                const auto end = std::min(begin + block_point_capacity, curve.point_count);
                std::size_t offset = 0;
                for (auto i = begin; i < end; ++i) {
                    for (const auto word : point_bits(curve.raw_points[i])) {
                        for (unsigned shift = 0; shift < 64; shift += 8) {
                            buffer[offset++] = static_cast<char>(word >> shift);
                        }
                    }
                }
                output.write(buffer.data(), static_cast<std::streamsize>(offset));
            }
        }
        for (const auto& block : curve.blocks) {
            write_number(output, block.point_count, 4);
            write_number(output, block.compressed ? 1 : 0, 1);
            write_number(output, block.payload.size(), 4);
            write_number(output, checksum(block.point_count, block.compressed, block.payload), 4);
            output.write(reinterpret_cast<const char*>(block.payload.data()),
                         static_cast<std::streamsize>(block.payload.size()));
        }
    }
    if (!output) {
        throw std::runtime_error("Failed to write CurveMemoryBank archive");
    }
}

CurveMemoryBank CurveMemoryBank::read(std::istream& input, std::size_t cacheBlocks, CurveStoragePolicy policy) {
    CurveMemoryBank result(cacheBlocks, policy);
    std::array<char, magic.size()> signature{};
    input.read(signature.data(), static_cast<std::streamsize>(signature.size()));
    if (!input || signature != magic || read_number(input, 4) != 1
        || read_number(input, 4) != block_point_capacity) {
        invalid_archive();
    }
    const auto curveCount = read_number(input, 8);
    if (curveCount > maximum_curves) {
        invalid_archive();
    }
    // Do not reserve from an untrusted count: a short file cannot force a large
    // allocation merely by claiming to contain millions of empty curves.
    for (std::uint64_t i = 0; i < curveCount; ++i) {
        const auto pointCount = read_number(input, 8);
        const auto blockCount = read_number(input, 4);
        const auto mode = read_number(input, 1);
        if (mode > 1 || pointCount > maximum_points - result.total_points_) {
            invalid_archive();
        }
        const auto expectedBlocks = mode == 0 ? (pointCount == 0 ? 0 : 1)
                                              : (pointCount + block_point_capacity - 1) / block_point_capacity;
        if (blockCount != expectedBlocks || (mode == 1 && pointCount == 0)) {
            invalid_archive();
        }
        std::vector<geometry::Point3D> decoded;
        std::size_t remaining = static_cast<std::size_t>(pointCount);
        for (std::uint64_t j = 0; j < blockCount; ++j) {
            const auto count = read_number(input, 4);
            const auto encoding = read_number(input, 1);
            const auto payloadBytes = read_number(input, 4);
            const auto expectedChecksum = read_number(input, 4);
            const auto expectedCount = mode == 0 ? remaining : std::min(remaining, block_point_capacity);
            if (count != expectedCount || count == 0 || encoding > 1
                || (mode == 0 && encoding != 0)
                || (encoding == 0 && payloadBytes != count * point_bytes)
                || (encoding == 1 && (payloadBytes >= count * point_bytes
                                      || payloadBytes < point_bytes + 3 * (count - 1)))) {
                invalid_archive();
            }
            Block block;
            block.point_count = static_cast<std::size_t>(count);
            block.compressed = encoding == 1;
            // Flat raw v1 blocks can be an entire large curve. Read/decode them
            // in point-aligned chunks without retaining a second byte payload.
            // A short stream never allocates from a large untrusted count.
            std::array<std::uint8_t, block_point_capacity * point_bytes> buffer{};
            auto crc = checksum_prefix(block.point_count, block.compressed,
                                       static_cast<std::size_t>(payloadBytes));
            std::size_t bytesRemaining = static_cast<std::size_t>(payloadBytes);
            while (bytesRemaining != 0) {
                const auto chunk = std::min(bytesRemaining, buffer.size());
                input.read(reinterpret_cast<char*>(buffer.data()), static_cast<std::streamsize>(chunk));
                if (!input) {
                    invalid_archive();
                }
                for (std::size_t k = 0; k < chunk; ++k) crc_byte(crc, buffer[k]);
                if (block.compressed) {
                    block.payload.insert(block.payload.end(), buffer.data(), buffer.data() + chunk);
                } else {
                    const auto bytes = std::span<const std::uint8_t>(buffer.data(), chunk);
                    for (std::size_t k = 0; k < chunk / point_bytes; ++k) {
                        decoded.push_back(raw_point(bytes, k));
                    }
                }
                bytesRemaining -= chunk;
            }
            if ((crc ^ 0xFFFFFFFFU) != expectedChecksum) {
                invalid_archive();
            }
            if (block.compressed) {
                const auto points = decode_block(block);
                decoded.insert(decoded.end(), points.begin(), points.end());
            }
            remaining -= static_cast<std::size_t>(count);
        }
        // Archives are portable data, not a promise that their former cache
        // configuration remains economical under the caller's current policy.
        Curve curve;
        result.select_storage(curve, decoded);
        if (curve.flat_raw) {
            // Remove growth slack after the payload has actually been verified.
            if (decoded.capacity() != decoded.size()) {
                std::vector<geometry::Point3D>(decoded.begin(), decoded.end()).swap(decoded);
            }
            curve.raw_points = std::move(decoded);
        }
        result.curves_.push_back(std::move(curve));
        result.total_points_ += static_cast<std::size_t>(pointCount);
        for (const auto& block : result.curves_.back().blocks) {
            result.compressed_blocks_ += block.compressed ? 1 : 0;
        }
    }
    if (input.peek() != std::char_traits<char>::eof() || input.bad()) {
        invalid_archive();
    }
    result.update_cache_budget();
    return result;
}

} // namespace adaptivecad::tool
