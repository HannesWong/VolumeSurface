#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include <openvdb/openvdb.h>

#include "volume_surface/VdbSurfaceProbe.h"
#include "volume_surface/SurfaceTarget.h"

namespace volume_surface::viewer {

struct ReconstructionPickReport
{
    bool hit = false;
    std::size_t slotIndex = std::numeric_limits<std::size_t>::max();
    std::size_t triangleIndex = std::numeric_limits<std::size_t>::max();
    std::array<double, 3> barycentric{};
    openvdb::Vec3d rayDirection{};
    openvdb::Vec3d scenePosition{};
    openvdb::Vec3d meshWorldPosition{};
    volume_surface::VdbSurfaceProbeResult vdb;
    bool targetSampleFound = false;
    std::size_t targetSampleIndex = std::numeric_limits<std::size_t>::max();
    double targetSampleDistance = 0.0;
    SurfaceTargetSample targetSample;
    bool smoothedNormalFound = false;
    openvdb::Vec3d smoothedNormal{};
    std::string normalUsedByReconstruction = "surface_target_seed";
};

std::string formatReconstructionPickReportJsonl(
    const ReconstructionPickReport& report,
    const std::string& sourcePath,
    const std::string& gridName,
    double isoValue);

} // namespace volume_surface::viewer
