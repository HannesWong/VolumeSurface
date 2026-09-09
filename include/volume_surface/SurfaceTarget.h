#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <openvdb/openvdb.h>

namespace volume_surface {

enum class SurfaceTargetSampleKind : std::uint8_t {
    Core,
    Transition,
};

struct SurfaceTargetSample
{
    openvdb::Coord coordinate;
    openvdb::Vec3f worldPosition{};
    openvdb::Vec3f normal{};
    float density = 0.0f;
    float planarity = 0.0f;
    float angularDeviationRadians = 0.0f;
    float supportWeight = 0.0f;
    std::uint8_t transitionLayer = 0;
    SurfaceTargetSampleKind kind = SurfaceTargetSampleKind::Core;
};

struct SurfaceTargetSettings
{
    double isoValue = 255.0;
    std::size_t transitionLayers = 2;
    // Kept for cache compatibility; normal smoothing is owned by Surface Normal.
    double normalRadius = 0.002;
    double planarityRadius = 0.002;
    double planarityAngularScaleRadians = 0.2617993877991494;
    // Retained in the cache schema for compatibility; extraction no longer
    // rejects large inputs based on a fixed sample-count limit.
    std::size_t maximumSampleCount = 8'000'000;
};

struct SurfaceTargetCache
{
    std::vector<SurfaceTargetSample> samples;
    std::size_t coreCount = 0;
    std::size_t transitionCount = 0;

    [[nodiscard]] bool empty() const noexcept { return samples.empty(); }
};

struct SurfaceTargetComponentFilterResult
{
    SurfaceTargetCache primary;
    std::size_t componentCount = 0;
    std::size_t excludedCoreCount = 0;
    std::size_t excludedTransitionCount = 0;
};

struct SurfaceTargetCacheMetadata
{
    std::string sourcePath;
    std::string gridName;
    std::uint64_t sourceFileSize = 0;
    std::int64_t sourceWriteTime = 0;
    std::int32_t gridClass = 0;
    bool hasActiveBounds = false;
    std::array<std::int32_t, 3> activeBoundsMinimum{};
    std::array<std::int32_t, 3> activeBoundsMaximum{};
    SurfaceTargetSettings settings{};
};

SurfaceTargetCache extractSurfaceTarget(
    const openvdb::FloatGrid& grid,
    const SurfaceTargetSettings& settings = {});

SurfaceTargetComponentFilterResult retainLargestSurfaceTargetComponent(
    const SurfaceTargetCache& cache);

bool saveSurfaceTargetCache(
    const std::filesystem::path& path,
    const SurfaceTargetCache& cache,
    const SurfaceTargetCacheMetadata& metadata,
    std::string* error = nullptr);

bool loadSurfaceTargetCache(
    const std::filesystem::path& path,
    const SurfaceTargetCacheMetadata& expectedMetadata,
    SurfaceTargetCache& cache,
    std::string* error = nullptr);

} // namespace volume_surface
