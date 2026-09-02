#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <openvdb/openvdb.h>

#include "volume_surface/SurfaceMesh.h"
#include "volume_surface/SurfaceTarget.h"

namespace volume_surface {

struct SurfaceReconstructionSettings
{
    double isoValue = 255.0;
    double mlsRadius = 0.005;
    double targetCellSize = 0.001;
    std::size_t candidatePaddingCells = 2;
    std::size_t maximumCellCount = 4'000'000;
    std::size_t maximumSamplesPerFit = 96;
    double regularization = 1.0e-6;
    double transitionGeometryWeight = 0.05;
    double minimumFogDensityFraction = 0.75;
    double sourceTopologyAdaptivity = 1.0;
};

struct SurfaceReconstructionTimings
{
    double anchorLocalizationMilliseconds = 0.0;
    double spatialIndexMilliseconds = 0.0;
    double fieldSamplingMilliseconds = 0.0;
    double meshExtractionMilliseconds = 0.0;
    double totalMilliseconds = 0.0;
};

struct SurfaceReconstructionResult
{
    SurfaceMesh mesh;
    std::size_t candidateCellCount = 0;
    std::size_t crossingCellCount = 0;
    std::size_t fieldSampleCount = 0;
    std::size_t invalidFieldFallbackCount = 0;
    std::size_t sourceSupportFallbackCount = 0;
    std::size_t sourceCrossingCellCount = 0;
    std::size_t missingNeighborFaceCount = 0;
    std::size_t emittedFaceCount = 0;
    bool usedSourceTopologyFallback = false;
    std::size_t sourceTopologyProjectionFallbackCount = 0;
    SurfaceReconstructionTimings timings;

    [[nodiscard]] bool empty() const noexcept { return mesh.empty(); }
};

SurfaceReconstructionResult reconstructSurfaceMLS(
    const openvdb::FloatGrid& sourceGrid,
    const SurfaceTargetCache& target,
    const SurfaceReconstructionSettings& settings = {});

} // namespace volume_surface
