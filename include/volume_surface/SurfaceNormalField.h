#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <openvdb/openvdb.h>

#include "volume_surface/SurfaceMeshContinuity.h"
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
    SurfaceFitNeighborhood neighborhood = SurfaceFitNeighborhood::Grid9x9;
    std::size_t robustIterations = 2;
    double isoValue = 255.0;
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
    // Reject mesh-derived propagation links that have no reliable topology
    // support in the source triangle mesh.
    double minimumMeshContinuity = 0.05;
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
    enum class SampleState : std::uint8_t {
        Kept,
        Downweighted,
        RejectedTopology,
        RejectedResidual,
    };

    struct Sample
    {
        std::size_t sampleIndex = static_cast<std::size_t>(-1);
        double weight = 0.0;
        double topologyWeight = 1.0;
        double planeResidual = 0.0;
        SampleState state = SampleState::Kept;
    };

    bool valid = false;
    std::size_t centerSampleIndex = static_cast<std::size_t>(-1);
    std::vector<std::size_t> sampleIndices;
    std::vector<Sample> samples;
    std::size_t keptSampleCount = 0;
    std::size_t downweightedSampleCount = 0;
    std::size_t rejectedSampleCount = 0;
    double fitResidualScale = 0.0;
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

SurfaceNormalField fitSurfaceTargetNormals(
    const SurfaceTargetCache& target,
    const SurfaceNormalFitSettings& settings = {});

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

SurfaceFitNeighborhoodInspection inspectSurfaceFitNeighborhood(
    const SurfaceTargetCache& target,
    std::size_t centerSampleIndex,
    SurfaceFitNeighborhood neighborhood,
    std::size_t robustIterations,
    const SurfaceMeshContinuityField* meshContinuity,
    double minimumMeshContinuity = 0.05);

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
    SurfaceNormalExpansionTrace* expansionTrace = nullptr,
    const SurfaceMeshContinuityField* meshContinuity = nullptr);

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
