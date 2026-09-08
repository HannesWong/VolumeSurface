#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <openvdb/openvdb.h>

#include "volume_surface/SurfaceNormalField.h"
#include "volume_surface/SurfaceTarget.h"

namespace volume_surface {

struct SurfaceNormalLocalSeedSettings
{
    // Flip-point propagation remains local and bounded so a preview cannot
    // consume the entire surface when the selected point is ambiguous.
    std::size_t maximumCoreSamples = 250'000;
    double maximumDistanceMillimeters = 0.0;
    double minimumAxisAlignment = 0.15;
    double maximumSurfaceNormalComponent = 0.75;
};

struct SurfaceNormalLocalSeedPreview
{
    bool valid = false;
    std::size_t seedSampleIndex = static_cast<std::size_t>(-1);
    std::size_t flipPointSampleIndex = static_cast<std::size_t>(-1);
    std::size_t affectedCoreSampleCount = 0;
    std::size_t boundaryCoreSampleCount = 0;
    std::size_t invalidNormalBoundaryCount = 0;
    std::size_t surfaceContinuationBoundaryCount = 0;
    std::size_t axisAlignmentBoundaryCount = 0;
    std::size_t distanceBoundaryCount = 0;
    std::size_t alreadyAlignedBoundaryCount = 0;
    bool maximumCoreSamplesReached = false;
    std::int32_t maximumDepth = -1;
    double maximumDistanceMillimeters = 0.0;
    std::vector<std::size_t> affectedCoreSampleIndices;
    std::vector<std::size_t> boundaryCoreSampleIndices;
    // Parent links make the local flip chain explicit for diagnostics and
    // future chain visualization. The selected flip point has parent -1.
    std::vector<std::int32_t> parentBySample;
    std::vector<std::int32_t> depthBySample;
};

// Local points are explicit flip events. The legacy Seed names remain as
// aliases so existing integrations and cache replay code stay compatible.
using SurfaceNormalLocalFlipPointSettings = SurfaceNormalLocalSeedSettings;
using SurfaceNormalLocalFlipPointPreview = SurfaceNormalLocalSeedPreview;

SurfaceNormalLocalFlipPointPreview previewSurfaceNormalLocalFlipPoint(
    const SurfaceTargetCache& target,
    const SurfaceNormalField& fittedAxes,
    const SurfaceNormalField& globalOrientedField,
    std::size_t flipPointSampleIndex,
    const SurfaceNormalLocalFlipPointSettings& settings = {},
    const SurfaceNormalExpansionTrace* orientationTrace = nullptr);

SurfaceNormalField applySurfaceNormalLocalFlipPoint(
    const SurfaceTargetCache& target,
    const SurfaceNormalField& baseField,
    const SurfaceNormalLocalFlipPointPreview& preview);

// Compatibility entry points for previously saved local-seed workflows.
inline SurfaceNormalLocalSeedPreview previewSurfaceNormalLocalSeed(
    const SurfaceTargetCache& target,
    const SurfaceNormalField& fittedAxes,
    const SurfaceNormalField& globalOrientedField,
    std::size_t seedSampleIndex,
    const SurfaceNormalLocalSeedSettings& settings = {},
    const SurfaceNormalExpansionTrace* orientationTrace = nullptr)
{
    return previewSurfaceNormalLocalFlipPoint(
        target,
        fittedAxes,
        globalOrientedField,
        seedSampleIndex,
        settings,
        orientationTrace);
}

inline SurfaceNormalField applySurfaceNormalLocalSeed(
    const SurfaceTargetCache& target,
    const SurfaceNormalField& baseField,
    const SurfaceNormalLocalSeedPreview& preview)
{
    return applySurfaceNormalLocalFlipPoint(target, baseField, preview);
}

} // namespace volume_surface
