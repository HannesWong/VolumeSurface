#pragma once

#include <openvdb/openvdb.h>

#include <array>
#include <cstddef>
#include <memory>
#include <vector>

namespace volume_surface {

struct SurfaceBrushParameters
{
    double coreRadius = 0.0;
    double falloffRadius = 0.0;
    float strength = 1.0f;
};

struct SurfacePropagationSettings
{
    double isoValue = 0.0;
    double planarityRadius = 0.0;
    double planarityAngularScaleRadians = 0.2617993877991494;
    double planeOffsetPenalty = 16.0;
    double normalChangePenalty = 0.5;
    double normalChangeScaleRadians = 0.3490658503988659;
    double seedSearchDistance = 0.0;
    std::size_t maximumCandidateVoxelCount = 8'000'000;
};

struct SurfaceBrushSample
{
    openvdb::Coord coordinate;
    std::array<float, 3> worldPosition{};
    std::array<float, 3> normal{};
    float angularDeviationRadians = 0.0f;
    float planarity = 0.0f;
    float propagationDistance = 0.0f;
    float weight = 0.0f;
};

struct SurfaceBrushTimings
{
    double hierarchyQueryMilliseconds = 0.0;
    double blockMilliseconds = 0.0;
    double anchorResolveMilliseconds = 0.0;
    double centerlineRouteMilliseconds = 0.0;
    double surfaceSweepMilliseconds = 0.0;
    double totalMilliseconds = 0.0;
};

struct SurfaceBrushResult
{
    openvdb::Coord seedCoordinate;
    std::vector<std::vector<openvdb::Coord>> centerlinePaths;
    std::vector<SurfaceBrushSample> samples;
    std::size_t candidateVoxelCount = 0;
    std::size_t candidateLeafCount = 0;
    SurfaceBrushTimings timings;

    [[nodiscard]] bool empty() const noexcept { return samples.empty(); }
};

struct SurfaceBrushHierarchyQuery
{
    std::vector<openvdb::CoordBBox> leafBounds;
    std::size_t visitedNodeCount = 0;
};

struct SurfaceBrushHierarchyDebugNode
{
    openvdb::Vec3d worldMinimum{};
    openvdb::Vec3d worldMaximum{};
    std::size_t depth = 0;
    std::size_t leafCount = 0;
    bool terminal = false;
};

class SurfaceBrushHierarchy
{
public:
    explicit SurfaceBrushHierarchy(const openvdb::FloatGrid& grid);
    ~SurfaceBrushHierarchy();

    SurfaceBrushHierarchy(const SurfaceBrushHierarchy&) = delete;
    SurfaceBrushHierarchy& operator=(const SurfaceBrushHierarchy&) = delete;
    SurfaceBrushHierarchy(SurfaceBrushHierarchy&&) = delete;
    SurfaceBrushHierarchy& operator=(SurfaceBrushHierarchy&&) = delete;

    [[nodiscard]] std::size_t leafCount() const noexcept;

    [[nodiscard]] std::size_t debugMaxDepth() const noexcept;

    [[nodiscard]] std::vector<SurfaceBrushHierarchyDebugNode> debugNodesAtDepth(
        std::size_t depth) const;

    [[nodiscard]] SurfaceBrushHierarchyQuery queryCapsule(
        const openvdb::Vec3d& startWorld,
        const openvdb::Vec3d& endWorld,
        double radius) const;

private:
    struct Impl;
    std::unique_ptr<Impl> mImpl;
};

SurfaceBrushResult propagateSurfaceBrush(
    const openvdb::FloatGrid& grid,
    const openvdb::Vec3d& brushCenterWorld,
    const SurfaceBrushParameters& brush,
    const SurfacePropagationSettings& settings);

SurfaceBrushResult propagateSurfaceBrushStroke(
    const openvdb::FloatGrid& grid,
    const std::vector<openvdb::Vec3d>& brushPathWorld,
    const SurfaceBrushParameters& brush,
    const SurfacePropagationSettings& settings);

SurfaceBrushResult propagateSurfaceBrushStroke(
    const openvdb::FloatGrid& grid,
    const std::vector<openvdb::Vec3d>& brushPathWorld,
    const SurfaceBrushParameters& brush,
    const SurfacePropagationSettings& settings,
    const SurfaceBrushHierarchy* hierarchy);

} // namespace volume_surface
