#pragma once

#include "volume_surface/AlignedBuffer.h"

#include <openvdb/openvdb.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace volume_surface {

inline constexpr std::uint32_t kInvalidBrick = std::numeric_limits<std::uint32_t>::max();
inline constexpr std::size_t kBrickDimension = 8;
inline constexpr std::size_t kVoxelsPerBrick = 512;
inline constexpr std::size_t kMaskWordsPerBrick = 8;
inline constexpr std::size_t kNeighborCount = 26;

struct alignas(64) BrickMetadata
{
    std::array<std::int32_t, 3> origin{};
    std::uint32_t activeVoxelCount = 0;
    std::uint64_t mortonKey = 0;
    std::array<std::uint32_t, kNeighborCount> neighbors{};
};

static_assert(sizeof(BrickMetadata) % 64 == 0);

struct ValidationReport
{
    std::size_t arenaBrickCount = 0;
    std::size_t gridBrickCount = 0;
    std::size_t missingBrickCount = 0;
    std::size_t extraBrickCount = 0;
    std::uint64_t activeStateMismatchCount = 0;
    std::uint64_t valueMismatchCount = 0;
    double maximumAbsoluteError = 0.0;

    [[nodiscard]] bool ok() const noexcept;
};

class BrickArena
{
public:
    static BrickArena fromGrid(const openvdb::FloatGrid& grid);

    [[nodiscard]] openvdb::FloatGrid::Ptr toGrid() const;
    [[nodiscard]] ValidationReport validateAgainst(
        const openvdb::FloatGrid& grid,
        float tolerance = 0.0f) const;

    void swapValueBuffers() noexcept;

    [[nodiscard]] std::size_t brickCount() const noexcept;
    [[nodiscard]] std::uint64_t activeVoxelCount() const noexcept;
    [[nodiscard]] std::size_t hotDataBytes() const noexcept;
    [[nodiscard]] bool buffersAreAligned() const noexcept;

    [[nodiscard]] const BrickMetadata& metadata(std::size_t brickIndex) const;
    [[nodiscard]] const float* currentBrick(std::size_t brickIndex) const;
    [[nodiscard]] float* currentBrick(std::size_t brickIndex);
    [[nodiscard]] const float* nextBrick(std::size_t brickIndex) const;
    [[nodiscard]] float* nextBrick(std::size_t brickIndex);
    [[nodiscard]] const std::uint64_t* activeMask(std::size_t brickIndex) const;
    [[nodiscard]] bool isActive(std::size_t brickIndex, std::size_t voxelOffset) const;
    [[nodiscard]] std::uint32_t neighborIndex(
        std::size_t brickIndex,
        int dx,
        int dy,
        int dz) const;

    [[nodiscard]] float background() const noexcept;
    [[nodiscard]] const std::string& gridName() const noexcept;
    [[nodiscard]] openvdb::GridClass gridClass() const noexcept;
    [[nodiscard]] const openvdb::math::Transform& transform() const noexcept;

private:
    BrickArena() = default;

    static std::size_t neighborSlot(int dx, int dy, int dz);
    void buildNeighborTable();

    std::vector<BrickMetadata> mMetadata;
    AlignedBuffer<float> mValuesA;
    AlignedBuffer<float> mValuesB;
    AlignedBuffer<std::uint64_t> mActiveWords;
    float* mCurrentValues = nullptr;
    float* mNextValues = nullptr;
    openvdb::math::Transform::Ptr mTransform;
    std::string mGridName;
    float mBackground = 0.0f;
    openvdb::GridClass mGridClass = openvdb::GRID_UNKNOWN;
    std::uint64_t mActiveVoxelCount = 0;
};

}
