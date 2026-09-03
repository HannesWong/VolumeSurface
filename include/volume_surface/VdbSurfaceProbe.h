#pragma once

#include <cstddef>

#include <openvdb/openvdb.h>

namespace volume_surface {

struct VdbSurfaceProbeSettings
{
    double isoValue = 255.0;
    std::size_t maximumIterations = 8;
    double valueTolerance = 0.1;
    double maximumStep = 0.0;
};

struct VdbSurfaceProbeResult
{
    bool valid = false;
    bool converged = false;
    std::size_t iterations = 0;
    openvdb::Vec3d worldPosition{};
    openvdb::Vec3d indexPosition{};
    openvdb::Coord nearestVoxel{};
    openvdb::Vec3d normal{};
    double sampledValue = 0.0;
    double isoResidual = 0.0;
    double displacement = 0.0;
};

VdbSurfaceProbeResult projectVdbSurface(
    const openvdb::FloatGrid& grid,
    const openvdb::Vec3d& initialWorldPosition,
    const VdbSurfaceProbeSettings& settings = {});

} // namespace volume_surface
