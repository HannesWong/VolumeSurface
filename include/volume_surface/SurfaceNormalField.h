#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <openvdb/openvdb.h>

#include "volume_surface/SurfaceTarget.h"

namespace volume_surface {

enum class SurfaceFitNeighborhood : std::uint8_t {
    Grid3x3,
    Grid5x5,
    Grid9x9,
};

enum class SurfaceNormalNeighborhood : std::uint8_t {
    None,
    Grid3x3,
    Grid5x5,
};

struct SurfaceNormalFitSettings
{
    SurfaceFitNeighborhood neighborhood = SurfaceFitNeighborhood::Grid3x3;
    std::size_t robustIterations = 2;
    double isoValue = 255.0;
};

struct SurfaceNormalSmoothingSettings
{
    SurfaceNormalNeighborhood neighborhood = SurfaceNormalNeighborhood::None;
    double strength = 1.0;
    std::size_t robustIterations = 2;
};

struct SurfaceNormalField
{
    std::vector<openvdb::Vec3f> normals;
    std::size_t coreCount = 0;
    std::size_t smoothedCoreCount = 0;
    std::size_t smoothedTransitionCount = 0;

    [[nodiscard]] bool empty() const noexcept { return normals.empty(); }
};

SurfaceNormalField fitSurfaceTargetNormals(
    const SurfaceTargetCache& target,
    const SurfaceNormalFitSettings& settings = {});

SurfaceNormalField fitSurfaceTargetNormals(
    const openvdb::FloatGrid& grid,
    const SurfaceTargetCache& target,
    const SurfaceNormalFitSettings& settings = {});

SurfaceNormalField smoothSurfaceTargetNormals(
    const SurfaceTargetCache& target,
    const SurfaceNormalField& seed,
    const SurfaceNormalSmoothingSettings& settings = {});

SurfaceNormalField smoothSurfaceTargetNormals(
    const SurfaceTargetCache& target,
    const SurfaceNormalSmoothingSettings& settings = {});

} // namespace volume_surface
