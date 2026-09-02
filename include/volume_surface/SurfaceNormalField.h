#pragma once

#include <cstddef>
#include <vector>

#include <openvdb/openvdb.h>

#include "volume_surface/SurfaceTarget.h"

namespace volume_surface {

struct SurfaceNormalSmoothingSettings
{
    double radius = 0.005;
    double strength = 0.5;
    std::size_t iterations = 2;
    double angleSigmaRadians = 0.3490658503988659;
    double sheetThickness = 0.0015;
    std::size_t maximumNeighbors = 64;
};

struct SurfaceNormalField
{
    std::vector<openvdb::Vec3f> normals;
    std::size_t coreCount = 0;
    std::size_t smoothedCoreCount = 0;
    std::size_t smoothedTransitionCount = 0;

    [[nodiscard]] bool empty() const noexcept { return normals.empty(); }
};

SurfaceNormalField smoothSurfaceTargetNormals(
    const SurfaceTargetCache& target,
    const SurfaceNormalSmoothingSettings& settings = {});

} // namespace volume_surface
