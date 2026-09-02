#include "volume_surface/SurfaceBrush.h"

#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetSphere.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

openvdb::FloatGrid::Ptr createPlanarFog(bool addGap)
{
    auto grid = openvdb::FloatGrid::create(0.0f);
    grid->setGridClass(openvdb::GRID_FOG_VOLUME);
    grid->setTransform(openvdb::math::Transform::createLinearTransform(
        openvdb::math::scale<openvdb::math::Mat4d>(openvdb::Vec3d(0.31, 0.31, 1.0))));

    auto accessor = grid->getAccessor();
    for (int z = -5; z <= 5; ++z) {
        for (int y = -10; y <= 10; ++y) {
            if (addGap && y == 0) {
                continue;
            }
            for (int x = -4; x <= 0; ++x) {
                accessor.setValueOn(openvdb::Coord(x, y, z), 1.0f);
            }
        }
    }
    return grid;
}

const volume_surface::SurfaceBrushSample* findSample(
    const volume_surface::SurfaceBrushResult& result,
    const openvdb::Coord& coordinate)
{
    const auto found = std::find_if(
        result.samples.begin(),
        result.samples.end(),
        [&](const volume_surface::SurfaceBrushSample& sample) {
            return sample.coordinate == coordinate;
        });
    return found == result.samples.end() ? nullptr : &*found;
}

volume_surface::SurfacePropagationSettings settingsForFog()
{
    volume_surface::SurfacePropagationSettings settings;
    settings.isoValue = 0.5;
    settings.planarityRadius = 1.5;
    settings.seedSearchDistance = 1.1;
    return settings;
}

void testPlanarPropagationUsesWorldDistance()
{
    const auto grid = createPlanarFog(false);
    volume_surface::SurfaceBrushParameters brush;
    brush.coreRadius = 0.4;
    brush.falloffRadius = 2.0;
    brush.strength = 2.5f;

    const auto result = volume_surface::propagateSurfaceBrush(
        *grid,
        grid->indexToWorld(openvdb::Vec3d(0.0, 0.0, 0.0)),
        brush,
        settingsForFog());
    require(!result.empty(), "planar brush produced no samples");

    const auto* seed = findSample(result, openvdb::Coord(0, 0, 0));
    const auto* yNeighbor = findSample(result, openvdb::Coord(0, 1, 0));
    const auto* zNeighbor = findSample(result, openvdb::Coord(0, 0, 1));
    require(seed && yNeighbor && zNeighbor, "expected planar neighbors were not reached");
    require(std::abs(seed->weight - 2.5f) < 1.0e-6f, "seed did not receive full strength");
    require(std::abs(yNeighbor->propagationDistance - 0.31f) < 1.0e-4f,
        "X/Y voxel spacing was not used");
    require(std::abs(zNeighbor->propagationDistance - 1.0f) < 1.0e-4f,
        "Z voxel spacing was not used");
    require(yNeighbor->planarity > 0.9f, "flat surface did not receive high planarity");
    require(yNeighbor->normal[0] > 0.99f, "fog normal did not point toward lower density");
    require(yNeighbor->weight > zNeighbor->weight,
        "falloff did not decrease with propagation distance");

    for (const auto& sample : result.samples) {
        require(sample.weight >= 0.0f && sample.weight <= brush.strength,
            "brush weight exceeded its strength range");
        require(sample.planarity >= 0.0f && sample.planarity <= 1.0f,
            "planarity left its normalized range");
    }
}

void testPropagationDoesNotJumpAcrossGap()
{
    const auto grid = createPlanarFog(true);
    volume_surface::SurfaceBrushParameters brush;
    brush.coreRadius = 0.2;
    brush.falloffRadius = 2.8;
    brush.strength = 1.0f;

    const auto result = volume_surface::propagateSurfaceBrush(
        *grid,
        grid->indexToWorld(openvdb::Vec3d(0.0, -1.0, 0.0)),
        brush,
        settingsForFog());
    require(!result.empty(), "gapped brush produced no samples");
    require(findSample(result, openvdb::Coord(0, -1, 0)) != nullptr,
        "seed-side component was not reached");
    require(findSample(result, openvdb::Coord(0, 1, 0)) == nullptr,
        "26-neighbor propagation jumped across a disconnected gap");
}

void testStrokePropagatesFromEveryRecordedCenter()
{
    const auto grid = createPlanarFog(false);
    volume_surface::SurfaceBrushParameters brush;
    brush.coreRadius = 0.4;
    brush.falloffRadius = 0.3;
    brush.strength = 1.0f;

    const auto result = volume_surface::propagateSurfaceBrushStroke(
        *grid,
        {
            grid->indexToWorld(openvdb::Vec3d(0.0, -5.0, 0.0)),
            grid->indexToWorld(openvdb::Vec3d(0.0, 5.0, 0.0))},
        brush,
        settingsForFog());
    require(!result.empty(), "stroke brush produced no samples");
    const auto* firstSeed = findSample(result, openvdb::Coord(0, -5, 0));
    const auto* secondSeed = findSample(result, openvdb::Coord(0, 5, 0));
    require(firstSeed && secondSeed,
        "stroke propagation did not include every recorded path center");
    require(std::abs(firstSeed->weight - brush.strength) < 1.0e-6f &&
            std::abs(secondSeed->weight - brush.strength) < 1.0e-6f,
        "stroke path centers did not receive full brush strength");
    require(result.centerlinePaths.size() == 1,
        "connected stroke anchors did not produce one centerline");
    const auto& centerline = result.centerlinePaths.front();
    require(centerline.front() == openvdb::Coord(0, -5, 0) &&
            centerline.back() == openvdb::Coord(0, 5, 0),
        "centerline endpoints did not match the resolved surface anchors");
    for (std::size_t point = 1; point < centerline.size(); ++point) {
        const openvdb::Coord delta = centerline[point] - centerline[point - 1];
        require(std::abs(delta.x()) <= 1 &&
                std::abs(delta.y()) <= 1 &&
                std::abs(delta.z()) <= 1 &&
                delta != openvdb::Coord(0),
            "centerline contains a non-26-connected surface step");
    }
    require(result.timings.blockMilliseconds >= 0.0 &&
            result.timings.anchorResolveMilliseconds >= 0.0 &&
            result.timings.centerlineRouteMilliseconds >= 0.0 &&
            result.timings.surfaceSweepMilliseconds >= 0.0 &&
            result.timings.totalMilliseconds >= 0.0,
        "brush timing values must be nonnegative");
}

void testHierarchyCapsuleMatchesLocalBrush()
{
    const auto grid = createPlanarFog(false);
    volume_surface::SurfaceBrushParameters brush;
    brush.coreRadius = 0.4;
    brush.falloffRadius = 2.0;
    brush.strength = 1.0f;
    const auto settings = settingsForFog();
    const openvdb::Vec3d center = grid->indexToWorld(openvdb::Vec3d(0.0, 0.0, 0.0));

    volume_surface::SurfaceBrushHierarchy hierarchy(*grid);
    const auto query = hierarchy.queryCapsule(center, center, 3.0);
    require(hierarchy.leafCount() > 0, "surface hierarchy contains no VDB leaves");
    require(!query.leafBounds.empty() && query.visitedNodeCount > 0,
        "surface hierarchy capsule did not select the planar surface leaves");

    const auto localResult = volume_surface::propagateSurfaceBrush(
        *grid,
        center,
        brush,
        settings);
    const auto hierarchyResult = volume_surface::propagateSurfaceBrushStroke(
        *grid,
        {center},
        brush,
        settings,
        &hierarchy);
    require(!hierarchyResult.empty(), "hierarchy brush produced no samples");
    require(hierarchyResult.candidateLeafCount > 0,
        "hierarchy brush did not report selected leaves");
    require(hierarchyResult.timings.hierarchyQueryMilliseconds >= 0.0,
        "hierarchy query time must be nonnegative");
    for (const auto& sample : localResult.samples) {
        const auto* hierarchySample = findSample(hierarchyResult, sample.coordinate);
        require(hierarchySample != nullptr,
            "hierarchy capsule omitted a local brush sample");
        require(std::abs(hierarchySample->weight - sample.weight) < 1.0e-6f,
            "hierarchy capsule changed a local brush weight");
    }
}

void testLevelSetNormalOrientation()
{
    const auto grid = openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(
        10.0f,
        openvdb::Vec3f(0.0f),
        0.5f,
        3.0f);
    volume_surface::SurfaceBrushParameters brush;
    brush.coreRadius = 0.5;
    brush.falloffRadius = 2.0;
    brush.strength = 1.0f;
    volume_surface::SurfacePropagationSettings settings;
    settings.isoValue = 0.0;
    settings.planarityRadius = 1.5;

    const auto result = volume_surface::propagateSurfaceBrush(
        *grid,
        openvdb::Vec3d(10.0, 0.0, 0.0),
        brush,
        settings);
    require(!result.empty(), "level set brush produced no samples");
    const auto* seed = findSample(result, result.seedCoordinate);
    require(seed && seed->normal[0] > 0.9f,
        "level set normal did not point toward positive SDF values");
}

} // namespace

int main()
{
    try {
        openvdb::initialize();
        testPlanarPropagationUsesWorldDistance();
        testPropagationDoesNotJumpAcrossGap();
        testStrokePropagatesFromEveryRecordedCenter();
        testHierarchyCapsuleMatchesLocalBrush();
        testLevelSetNormalOrientation();
        std::cout << "surface_brush_tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "surface_brush_tests failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
