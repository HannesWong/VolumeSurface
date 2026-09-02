#include "volume_surface/BrickArena.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <unordered_map>

namespace volume_surface {
namespace {

using LeafNode = openvdb::FloatGrid::TreeType::LeafNodeType;

struct OriginKey
{
    std::int32_t x = 0;
    std::int32_t y = 0;
    std::int32_t z = 0;

    bool operator==(const OriginKey&) const = default;
};

struct OriginKeyHash
{
    std::size_t operator()(const OriginKey& key) const noexcept
    {
        std::uint64_t value = static_cast<std::uint32_t>(key.x);
        value ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.y)) << 21;
        value ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.z)) << 42;
        value ^= value >> 33;
        value *= 0xff51afd7ed558ccdULL;
        value ^= value >> 33;
        return static_cast<std::size_t>(value);
    }
};

struct LeafRecord
{
    const LeafNode* leaf = nullptr;
    OriginKey origin;
    std::array<std::int64_t, 3> brickCoordinate{};
    std::uint64_t mortonKey = 0;
};

std::uint64_t splitByThree(std::uint32_t value)
{
    std::uint64_t result = value & 0x1fffffU;
    result = (result | result << 32) & 0x001f00000000ffffULL;
    result = (result | result << 16) & 0x001f0000ff0000ffULL;
    result = (result | result << 8) & 0x100f00f00f00f00fULL;
    result = (result | result << 4) & 0x10c30c30c30c30c3ULL;
    result = (result | result << 2) & 0x1249249249249249ULL;
    return result;
}

std::uint64_t mortonKey(std::uint32_t x, std::uint32_t y, std::uint32_t z)
{
    return splitByThree(x) | (splitByThree(y) << 1) | (splitByThree(z) << 2);
}

std::uint32_t popcount64(std::uint64_t value)
{
    return static_cast<std::uint32_t>(std::popcount(value));
}

OriginKey toOriginKey(const openvdb::Coord& origin)
{
    return {origin.x(), origin.y(), origin.z()};
}

openvdb::Coord toCoord(const std::array<std::int32_t, 3>& origin)
{
    return openvdb::Coord(origin[0], origin[1], origin[2]);
}

bool equalFloatBits(float lhs, float rhs)
{
    return std::bit_cast<std::uint32_t>(lhs) == std::bit_cast<std::uint32_t>(rhs);
}

}

bool ValidationReport::ok() const noexcept
{
    return missingBrickCount == 0 &&
        extraBrickCount == 0 &&
        activeStateMismatchCount == 0 &&
        valueMismatchCount == 0;
}

BrickArena BrickArena::fromGrid(const openvdb::FloatGrid& grid)
{
    if (grid.getGridClass() == openvdb::GRID_LEVEL_SET) {
        throw std::invalid_argument(
            "BrickArena currently accepts fog or unclassified float grids, not level sets");
    }
    if (grid.tree().activeTileCount() != 0) {
        throw std::invalid_argument("BrickArena does not expand active VDB tiles");
    }

    BrickArena arena;
    arena.mGridName = grid.getName();
    arena.mBackground = grid.background();
    arena.mGridClass = grid.getGridClass();
    arena.mTransform = grid.transform().copy();

    std::vector<LeafRecord> records;
    records.reserve(static_cast<std::size_t>(grid.tree().leafCount()));

    std::array<std::int64_t, 3> minimumBrickCoordinate{
        std::numeric_limits<std::int64_t>::max(),
        std::numeric_limits<std::int64_t>::max(),
        std::numeric_limits<std::int64_t>::max()
    };

    for (auto leafIterator = grid.tree().cbeginLeaf(); leafIterator; ++leafIterator) {
        const openvdb::Coord origin = leafIterator->origin();
        LeafRecord record;
        record.leaf = &*leafIterator;
        record.origin = toOriginKey(origin);
        record.brickCoordinate = {
            static_cast<std::int64_t>(origin.x()) / static_cast<std::int64_t>(kBrickDimension),
            static_cast<std::int64_t>(origin.y()) / static_cast<std::int64_t>(kBrickDimension),
            static_cast<std::int64_t>(origin.z()) / static_cast<std::int64_t>(kBrickDimension)
        };
        for (std::size_t axis = 0; axis < 3; ++axis) {
            minimumBrickCoordinate[axis] = std::min(
                minimumBrickCoordinate[axis], record.brickCoordinate[axis]);
        }
        records.push_back(record);
    }

    if (!records.empty()) {
        for (LeafRecord& record: records) {
            std::array<std::uint32_t, 3> relative{};
            for (std::size_t axis = 0; axis < 3; ++axis) {
                const std::int64_t coordinate =
                    record.brickCoordinate[axis] - minimumBrickCoordinate[axis];
                if (coordinate < 0 || coordinate > 0x1fffff) {
                    throw std::overflow_error("Brick coordinate span exceeds the Morton key range");
                }
                relative[axis] = static_cast<std::uint32_t>(coordinate);
            }
            record.mortonKey = mortonKey(relative[0], relative[1], relative[2]);
        }

        std::sort(records.begin(), records.end(), [](const LeafRecord& lhs, const LeafRecord& rhs) {
            return std::tie(lhs.mortonKey, lhs.origin.x, lhs.origin.y, lhs.origin.z) <
                std::tie(rhs.mortonKey, rhs.origin.x, rhs.origin.y, rhs.origin.z);
        });
    }

    const std::size_t brickCount = records.size();
    arena.mMetadata.resize(brickCount);
    arena.mValuesA.resize(brickCount * kVoxelsPerBrick);
    arena.mValuesB.resize(brickCount * kVoxelsPerBrick);
    arena.mActiveWords.resize(brickCount * kMaskWordsPerBrick);
    arena.mCurrentValues = arena.mValuesA.data();
    arena.mNextValues = arena.mValuesB.data();

    for (std::size_t brickIndex = 0; brickIndex < brickCount; ++brickIndex) {
        const LeafRecord& record = records[brickIndex];
        BrickMetadata& metadata = arena.mMetadata[brickIndex];
        metadata.origin = {record.origin.x, record.origin.y, record.origin.z};
        metadata.mortonKey = record.mortonKey;
        metadata.neighbors.fill(kInvalidBrick);

        const std::size_t valueOffset = brickIndex * kVoxelsPerBrick;
        std::copy_n(
            record.leaf->buffer().data(),
            kVoxelsPerBrick,
            arena.mValuesA.data() + valueOffset);
        std::copy_n(
            record.leaf->buffer().data(),
            kVoxelsPerBrick,
            arena.mValuesB.data() + valueOffset);

        const auto& valueMask = record.leaf->getValueMask();
        const std::size_t maskOffset = brickIndex * kMaskWordsPerBrick;
        for (std::size_t wordIndex = 0; wordIndex < kMaskWordsPerBrick; ++wordIndex) {
            const std::uint64_t word = valueMask.getWord<std::uint64_t>(
                static_cast<openvdb::Index>(wordIndex));
            arena.mActiveWords[maskOffset + wordIndex] = word;
            metadata.activeVoxelCount += popcount64(word);
        }
        arena.mActiveVoxelCount += metadata.activeVoxelCount;
    }

    arena.buildNeighborTable();
    return arena;
}

openvdb::FloatGrid::Ptr BrickArena::toGrid() const
{
    auto grid = openvdb::FloatGrid::create(mBackground);
    grid->setName(mGridName);
    grid->setGridClass(mGridClass);
    grid->setTransform(mTransform->copy());

    for (std::size_t brickIndex = 0; brickIndex < brickCount(); ++brickIndex) {
        const openvdb::Coord origin = toCoord(mMetadata[brickIndex].origin);
        auto leaf = std::make_unique<LeafNode>(origin, mBackground, false);
        std::copy_n(
            currentBrick(brickIndex),
            kVoxelsPerBrick,
            leaf->buffer().data());

        auto& valueMask = leaf->getValueMask();
        const std::uint64_t* words = activeMask(brickIndex);
        for (std::size_t wordIndex = 0; wordIndex < kMaskWordsPerBrick; ++wordIndex) {
            valueMask.getWord<std::uint64_t>(static_cast<openvdb::Index>(wordIndex)) =
                words[wordIndex];
        }
        grid->tree().addLeaf(leaf.release());
    }

    return grid;
}

ValidationReport BrickArena::validateAgainst(
    const openvdb::FloatGrid& grid,
    float tolerance) const
{
    if (tolerance < 0.0f) {
        throw std::invalid_argument("validation tolerance must not be negative");
    }

    ValidationReport report;
    report.arenaBrickCount = brickCount();
    report.gridBrickCount = static_cast<std::size_t>(grid.tree().leafCount());
    std::size_t matchedBrickCount = 0;

    for (std::size_t brickIndex = 0; brickIndex < brickCount(); ++brickIndex) {
        const LeafNode* leaf = grid.tree().probeConstLeaf(toCoord(mMetadata[brickIndex].origin));
        if (!leaf) {
            ++report.missingBrickCount;
            continue;
        }
        ++matchedBrickCount;

        const std::uint64_t* arenaMask = activeMask(brickIndex);
        const auto& gridMask = leaf->getValueMask();
        for (std::size_t wordIndex = 0; wordIndex < kMaskWordsPerBrick; ++wordIndex) {
            const std::uint64_t difference = arenaMask[wordIndex] ^
                gridMask.getWord<std::uint64_t>(static_cast<openvdb::Index>(wordIndex));
            report.activeStateMismatchCount += popcount64(difference);
        }

        const float* arenaValues = currentBrick(brickIndex);
        const float* gridValues = leaf->buffer().data();
        for (std::size_t voxelOffset = 0; voxelOffset < kVoxelsPerBrick; ++voxelOffset) {
            const float arenaValue = arenaValues[voxelOffset];
            const float gridValue = gridValues[voxelOffset];
            if (equalFloatBits(arenaValue, gridValue)) continue;
            if (std::isnan(arenaValue) && std::isnan(gridValue)) continue;

            const double error = std::abs(
                static_cast<double>(arenaValue) - static_cast<double>(gridValue));
            if (error <= tolerance) continue;

            ++report.valueMismatchCount;
            report.maximumAbsoluteError = std::max(report.maximumAbsoluteError, error);
        }
    }

    if (report.gridBrickCount > matchedBrickCount) {
        report.extraBrickCount = report.gridBrickCount - matchedBrickCount;
    }
    return report;
}

void BrickArena::swapValueBuffers() noexcept
{
    std::swap(mCurrentValues, mNextValues);
}

std::size_t BrickArena::brickCount() const noexcept
{
    return mMetadata.size();
}

std::uint64_t BrickArena::activeVoxelCount() const noexcept
{
    return mActiveVoxelCount;
}

std::size_t BrickArena::hotDataBytes() const noexcept
{
    return mMetadata.size() * sizeof(BrickMetadata) +
        mValuesA.size() * sizeof(float) +
        mValuesB.size() * sizeof(float) +
        mActiveWords.size() * sizeof(std::uint64_t);
}

bool BrickArena::buffersAreAligned() const noexcept
{
    auto isAligned = [](const void* pointer) {
        return pointer == nullptr ||
            reinterpret_cast<std::uintptr_t>(pointer) % 64 == 0;
    };
    return isAligned(mValuesA.data()) &&
        isAligned(mValuesB.data()) &&
        isAligned(mActiveWords.data()) &&
        (mMetadata.empty() || isAligned(mMetadata.data()));
}

const BrickMetadata& BrickArena::metadata(std::size_t brickIndex) const
{
    return mMetadata.at(brickIndex);
}

const float* BrickArena::currentBrick(std::size_t brickIndex) const
{
    if (brickIndex >= brickCount()) throw std::out_of_range("brick index out of range");
    return mCurrentValues + brickIndex * kVoxelsPerBrick;
}

float* BrickArena::currentBrick(std::size_t brickIndex)
{
    if (brickIndex >= brickCount()) throw std::out_of_range("brick index out of range");
    return mCurrentValues + brickIndex * kVoxelsPerBrick;
}

const float* BrickArena::nextBrick(std::size_t brickIndex) const
{
    if (brickIndex >= brickCount()) throw std::out_of_range("brick index out of range");
    return mNextValues + brickIndex * kVoxelsPerBrick;
}

float* BrickArena::nextBrick(std::size_t brickIndex)
{
    if (brickIndex >= brickCount()) throw std::out_of_range("brick index out of range");
    return mNextValues + brickIndex * kVoxelsPerBrick;
}

const std::uint64_t* BrickArena::activeMask(std::size_t brickIndex) const
{
    if (brickIndex >= brickCount()) throw std::out_of_range("brick index out of range");
    return mActiveWords.data() + brickIndex * kMaskWordsPerBrick;
}

bool BrickArena::isActive(std::size_t brickIndex, std::size_t voxelOffset) const
{
    if (voxelOffset >= kVoxelsPerBrick) throw std::out_of_range("voxel offset out of range");
    const std::uint64_t* words = activeMask(brickIndex);
    return (words[voxelOffset >> 6] & (std::uint64_t{1} << (voxelOffset & 63))) != 0;
}

std::uint32_t BrickArena::neighborIndex(
    std::size_t brickIndex,
    int dx,
    int dy,
    int dz) const
{
    return metadata(brickIndex).neighbors[neighborSlot(dx, dy, dz)];
}

float BrickArena::background() const noexcept
{
    return mBackground;
}

const std::string& BrickArena::gridName() const noexcept
{
    return mGridName;
}

openvdb::GridClass BrickArena::gridClass() const noexcept
{
    return mGridClass;
}

const openvdb::math::Transform& BrickArena::transform() const noexcept
{
    return *mTransform;
}

std::size_t BrickArena::neighborSlot(int dx, int dy, int dz)
{
    if (dx < -1 || dx > 1 || dy < -1 || dy > 1 || dz < -1 || dz > 1 ||
        (dx == 0 && dy == 0 && dz == 0)) {
        throw std::invalid_argument("neighbor offset must be in [-1, 1] and not be zero");
    }

    std::size_t slot = 0;
    for (int candidateX = -1; candidateX <= 1; ++candidateX) {
        for (int candidateY = -1; candidateY <= 1; ++candidateY) {
            for (int candidateZ = -1; candidateZ <= 1; ++candidateZ) {
                if (candidateX == 0 && candidateY == 0 && candidateZ == 0) continue;
                if (candidateX == dx && candidateY == dy && candidateZ == dz) return slot;
                ++slot;
            }
        }
    }
    throw std::logic_error("neighbor slot lookup failed");
}

void BrickArena::buildNeighborTable()
{
    std::unordered_map<OriginKey, std::uint32_t, OriginKeyHash> brickLookup;
    brickLookup.reserve(brickCount() * 2);
    for (std::size_t brickIndex = 0; brickIndex < brickCount(); ++brickIndex) {
        const auto& origin = mMetadata[brickIndex].origin;
        brickLookup.emplace(
            OriginKey{origin[0], origin[1], origin[2]},
            static_cast<std::uint32_t>(brickIndex));
    }

    for (std::size_t brickIndex = 0; brickIndex < brickCount(); ++brickIndex) {
        BrickMetadata& metadata = mMetadata[brickIndex];
        std::size_t slot = 0;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    const OriginKey neighborOrigin{
                        metadata.origin[0] + static_cast<std::int32_t>(dx * kBrickDimension),
                        metadata.origin[1] + static_cast<std::int32_t>(dy * kBrickDimension),
                        metadata.origin[2] + static_cast<std::int32_t>(dz * kBrickDimension)
                    };
                    const auto found = brickLookup.find(neighborOrigin);
                    metadata.neighbors[slot] = found == brickLookup.end()
                        ? kInvalidBrick
                        : found->second;
                    ++slot;
                }
            }
        }
    }
}

}
