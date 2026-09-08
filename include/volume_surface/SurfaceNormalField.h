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

struct SurfaceNormalExpansionTrace;

struct SurfaceNormalFitSettings
{
    SurfaceFitNeighborhood neighborhood = SurfaceFitNeighborhood::Grid9x9;
    std::size_t robustIterations = 2;
    double isoValue = 255.0;
};

// Retained for compatibility with historical diagnostics only. The viewer
// runtime uses explicit local flip points instead of automatic sign-island repair.
struct SurfaceNormalFitSignRepairSettings
{
    double maximumIslandAreaSquareMillimeters = 25.0;
    double minimumOpposingBoundaryFraction = 0.80;
    double minimumHostAreaRatio = 4.0;
    double minimumAlignment = 0.75;
    double minimumRegionConnectivity = 0.35;
    double minimumRepairEnergyMargin = 0.25;
    std::size_t minimumOpposingBoundaryEdges = 8;
    double maximumSurfaceNormalComponent = 0.75;
    std::size_t maximumProtectedSeedDepth = 2;
};

struct SurfaceNormalFitSignRepairPatch
{
    std::size_t representativeSampleIndex = static_cast<std::size_t>(-1);
    std::size_t hostRepresentativeSampleIndex = static_cast<std::size_t>(-1);
    std::size_t sampleCount = 0;
    double areaSquareMillimeters = 0.0;
    double hostAreaSquareMillimeters = 0.0;
    double opposingBoundaryFraction = 0.0;
    double weightedOpposingBoundarySupport = 0.0;
};

struct SurfaceNormalFitSignRepairReport
{
    std::size_t validCoreEdgeCount = 0;
    std::size_t candidateIslandCount = 0;
    std::size_t acceptedIslandCount = 0;
    std::size_t flippedCoreSampleCount = 0;
    std::size_t reorientedTransitionSampleCount = 0;
    double meanPointAxisConfidence = 0.0;
    double weightedOpposingSupportBefore = 0.0;
    double weightedOpposingSupportAfter = 0.0;
    std::vector<SurfaceNormalFitSignRepairPatch> acceptedPatches;
};

struct SurfaceNormalSmoothingSettings
{
    SurfaceNormalNeighborhood neighborhood = SurfaceNormalNeighborhood::None;
    double strength = 1.0;
    std::size_t robustIterations = 2;
};

struct SurfaceNormalOrientationSettings
{
    // Reject weak local links during seed-rooted orientation propagation.
    double minimumAlignment = 0.15;
    // Reject links whose displacement is dominated by the local surface normal.
    double maximumSurfaceNormalComponent = 0.75;
};

struct SurfaceNormalExpansionTrace
{
    std::vector<std::int32_t> parentBySample;
    std::vector<std::int32_t> depthBySample;
    std::vector<std::uint8_t> orientedBySample;

    [[nodiscard]] bool matchesSampleCount(std::size_t count) const noexcept
    {
        return parentBySample.size() == count &&
            depthBySample.size() == count &&
            orientedBySample.size() == count;
    }
};

struct SurfaceNormalExpansionNeighborhood
{
    bool valid = false;
    std::size_t targetSampleIndex = static_cast<std::size_t>(-1);
    std::size_t sourceSampleIndex = static_cast<std::size_t>(-1);
    std::int32_t targetDepth = -1;
    std::vector<std::size_t> targetNextSampleIndices;
    std::vector<std::size_t> sourceBatchSampleIndices;
};

struct SurfaceFitNeighborhoodInspection
{
    bool valid = false;
    std::size_t centerSampleIndex = static_cast<std::size_t>(-1);
    std::vector<std::size_t> sampleIndices;
};

struct SurfaceNormalAdjacencyStatistics
{
    struct Issue
    {
        std::size_t firstSampleIndex = 0;
        std::size_t secondSampleIndex = 0;
        openvdb::Coord firstCoordinate{};
        openvdb::Coord secondCoordinate{};
        openvdb::Vec3f firstNormal{};
        openvdb::Vec3f secondNormal{};
        double signedAlignment = 1.0;
        double absoluteAlignment = 1.0;
        bool transitionLinked = false;
        bool unseededComponent = false;
        bool transitionWithoutCoreSupport = false;
    };

    std::size_t validCoreSampleCount = 0;
    std::size_t validTransitionSampleCount = 0;
    std::size_t adjacencyEdgeCount = 0;
    std::size_t opposingEdgeCount = 0;
    std::size_t weakEdgeCount = 0;
    std::size_t transitionAdjacencyEdgeCount = 0;
    std::size_t transitionOpposingEdgeCount = 0;
    std::size_t opposingUnseededEdgeCount = 0;
    std::size_t opposingTransitionWithoutCoreSupportEdgeCount = 0;
    std::size_t connectedComponentCount = 0;
    std::size_t seededComponentSampleCount = 0;
    std::size_t unseededComponentSampleCount = 0;
    double minimumSignedAlignment = 1.0;
    double meanSignedAlignment = 0.0;
    double minimumAbsoluteAlignment = 1.0;
    double meanAbsoluteAlignment = 0.0;
    std::vector<Issue> worstOpposingEdges;
    std::vector<Issue> worstCoreOpposingEdges;
    std::vector<Issue> worstTransitionOpposingEdges;
};

struct SurfaceNormalField
{
    // Fitted normals are unoriented axes: their sign is intentionally not
    // inferred from the source normal or the VDB scalar field.
    std::vector<openvdb::Vec3f> normals;
    std::size_t coreCount = 0;
    std::size_t smoothedCoreCount = 0;
    std::size_t smoothedTransitionCount = 0;

    [[nodiscard]] bool empty() const noexcept { return normals.empty(); }
};

struct SurfaceNormalFitSignRepairResult
{
    SurfaceNormalField field;
    SurfaceNormalFitSignRepairReport report;
};

SurfaceNormalField fitSurfaceTargetNormals(
    const SurfaceTargetCache& target,
    const SurfaceNormalFitSettings& settings = {});

// Historical API; do not call from the active viewer pipeline.
SurfaceNormalFitSignRepairResult repairSurfaceNormalFitSignIslands(
    const openvdb::FloatGrid& grid,
    const SurfaceTargetCache& target,
    const SurfaceNormalField& rawAxes,
    const SurfaceNormalFitSignRepairSettings& settings = {},
    const SurfaceNormalExpansionTrace* orientationTrace = nullptr);

SurfaceNormalField fitSurfaceTargetNormals(
    // The grid overload is retained for callers that already have a grid;
    // fitting itself does not sample or orient from the grid.
    const openvdb::FloatGrid& grid,
    const SurfaceTargetCache& target,
    const SurfaceNormalFitSettings& settings = {});

SurfaceFitNeighborhoodInspection inspectSurfaceFitNeighborhood(
    const SurfaceTargetCache& target,
    std::size_t centerSampleIndex,
    SurfaceFitNeighborhood neighborhood);

SurfaceNormalField smoothSurfaceTargetNormals(
    const SurfaceTargetCache& target,
    const SurfaceNormalField& seed,
    const SurfaceNormalSmoothingSettings& settings = {});

SurfaceNormalField smoothSurfaceTargetNormals(
    const SurfaceTargetCache& target,
    const SurfaceNormalSmoothingSettings& settings = {});

SurfaceNormalField orientSurfaceTargetNormals(
    const SurfaceTargetCache& target,
    const SurfaceNormalField& axes,
    std::size_t seedSampleIndex,
    const openvdb::Vec3d& seedDirection,
    const SurfaceNormalOrientationSettings& settings = {},
    SurfaceNormalExpansionTrace* expansionTrace = nullptr);

SurfaceNormalExpansionNeighborhood inspectSurfaceNormalExpansion(
    const SurfaceTargetCache& target,
    const SurfaceNormalExpansionTrace& expansionTrace,
    std::size_t targetSampleIndex);

SurfaceNormalAdjacencyStatistics analyzeSurfaceTargetNormalAdjacency(
    const SurfaceTargetCache& target,
    const SurfaceNormalField& field,
    std::size_t seedSampleIndex = static_cast<std::size_t>(-1),
    double weakAlignment = 0.15);

} // namespace volume_surface
