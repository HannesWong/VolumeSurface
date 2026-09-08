#include "volume_surface/SurfaceTarget.h"
#include "volume_surface/SurfaceNormalField.h"
#include "volume_surface/SurfaceNormalLocalSeed.h"
#include "volume_surface/VdbSurfaceProbe.h"
#include "volume_surface/viewer/SurfaceNormalLocalSeedStore.h"

#include <openvdb/openvdb.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

openvdb::FloatGrid::Ptr createLayeredFog()
{
    auto grid = openvdb::FloatGrid::create(0.0f);
    grid->setGridClass(openvdb::GRID_FOG_VOLUME);
    grid->setTransform(openvdb::math::Transform::createLinearTransform(
        openvdb::math::scale<openvdb::math::Mat4d>(
            openvdb::Vec3d(0.31, 0.31, 1.0))));

    auto accessor = grid->getAccessor();
    for (int z = -2; z <= 2; ++z) {
        for (int y = -2; y <= 2; ++y) {
            for (int x = -2; x <= 0; ++x) {
                accessor.setValueOn(openvdb::Coord(x, y, z), 255.0f);
            }
            accessor.setValueOn(openvdb::Coord(1, y, z), 128.0f);
            accessor.setValueOn(openvdb::Coord(2, y, z), 64.0f);
        }
    }
    return grid;
}

const volume_surface::SurfaceTargetSample* findSample(
    const volume_surface::SurfaceTargetCache& cache,
    const openvdb::Coord& coordinate)
{
    for (const auto& sample : cache.samples) {
        if (sample.coordinate == coordinate) {
            return &sample;
        }
    }
    return nullptr;
}

openvdb::FloatGrid::Ptr createMetricGrid(const openvdb::Vec3d& voxelSize)
{
    auto grid = openvdb::FloatGrid::create(0.0f);
    grid->setGridClass(openvdb::GRID_FOG_VOLUME);
    grid->setTransform(openvdb::math::Transform::createLinearTransform(
        openvdb::math::scale<openvdb::math::Mat4d>(voxelSize)));
    return grid;
}

volume_surface::SurfaceTargetCache createCoreSheet(
    const openvdb::FloatGrid& grid,
    int firstCount,
    int secondCount,
    bool normalAlongX)
{
    volume_surface::SurfaceTargetCache cache;
    cache.samples.reserve(static_cast<std::size_t>(firstCount * secondCount));
    for (int second = 0; second < secondCount; ++second) {
        for (int first = 0; first < firstCount; ++first) {
            volume_surface::SurfaceTargetSample sample;
            sample.coordinate = normalAlongX
                ? openvdb::Coord(0, first, second)
                : openvdb::Coord(first, second, 0);
            sample.worldPosition = openvdb::Vec3f(
                grid.indexToWorld(sample.coordinate.asVec3d()));
            sample.kind = volume_surface::SurfaceTargetSampleKind::Core;
            cache.samples.push_back(sample);
        }
    }
    cache.coreCount = cache.samples.size();
    return cache;
}

volume_surface::SurfaceNormalField createUniformNormalField(
    const volume_surface::SurfaceTargetCache& cache,
    const openvdb::Vec3f& normal)
{
    volume_surface::SurfaceNormalField field;
    field.normals.assign(cache.samples.size(), normal);
    field.coreCount = cache.coreCount;
    field.smoothedCoreCount = cache.coreCount;
    return field;
}

std::size_t findSampleIndex(
    const volume_surface::SurfaceTargetCache& cache,
    const openvdb::Coord& coordinate)
{
    for (std::size_t index = 0; index < cache.samples.size(); ++index) {
        if (cache.samples[index].coordinate == coordinate) {
            return index;
        }
    }
    throw std::runtime_error("synthetic surface target sample is missing");
}

void testCoreAndTransitionSamples()
{
    const auto grid = createLayeredFog();
    volume_surface::SurfaceTargetSettings settings;
    settings.isoValue = 255.0;
    settings.transitionLayers = 2;
    settings.normalRadius = 2.0;
    settings.planarityRadius = 1.0;

    const auto cache = volume_surface::extractSurfaceTarget(*grid, settings);
    require(!cache.empty(), "surface target cache is empty");
    require(cache.coreCount > 0, "surface target cache has no core samples");
    require(cache.transitionCount > 0, "surface target cache has no transition samples");

    const auto* core = findSample(cache, openvdb::Coord(0, 0, 0));
    const auto* firstTransition = findSample(cache, openvdb::Coord(1, 0, 0));
    const auto* secondTransition = findSample(cache, openvdb::Coord(2, 0, 0));
    require(core && firstTransition && secondTransition,
        "expected layered samples were not extracted");
    require(core->kind == volume_surface::SurfaceTargetSampleKind::Core,
        "alpha=255 boundary was not classified as core");
    require(firstTransition->kind == volume_surface::SurfaceTargetSampleKind::Transition &&
            secondTransition->kind == volume_surface::SurfaceTargetSampleKind::Transition,
        "transparent layers were not classified as transition samples");
    require(firstTransition->transitionLayer == 1 &&
            secondTransition->transitionLayer == 2,
        "transition layer distance was not measured from the nearest core");
    require(core->normal.x() > 0.9f,
        "fog-volume boundary normal did not point toward transparent space");
    require(firstTransition->supportWeight > secondTransition->supportWeight,
        "transition support did not decrease with distance");
    require(core->planarity >= 0.0f && core->planarity <= 1.0f,
        "core planarity left its normalized range");

    const auto repeated = volume_surface::extractSurfaceTarget(*grid, settings);
    require(repeated.samples.size() == cache.samples.size() &&
            repeated.coreCount == cache.coreCount &&
            repeated.transitionCount == cache.transitionCount,
        "parallel extraction changed cache cardinality");
    for (std::size_t index = 0; index < cache.samples.size(); ++index) {
        const auto& lhs = cache.samples[index];
        const auto& rhs = repeated.samples[index];
        require(lhs.coordinate == rhs.coordinate && lhs.kind == rhs.kind &&
                lhs.transitionLayer == rhs.transitionLayer,
            "parallel extraction changed deterministic sample ordering");
        require((lhs.worldPosition - rhs.worldPosition).lengthSqr() < 1.0e-12f &&
                (lhs.normal - rhs.normal).lengthSqr() < 1.0e-12f &&
                std::abs(lhs.planarity - rhs.planarity) < 1.0e-6f,
            "parallel extraction changed sample attributes");
    }
}

void testCacheSaveLoadRoundTrip()
{
    const auto grid = createLayeredFog();
    volume_surface::SurfaceTargetSettings settings;
    settings.isoValue = 255.0;
    settings.transitionLayers = 2;
    settings.normalRadius = 2.0;
    settings.planarityRadius = 1.0;
    const auto cache = volume_surface::extractSurfaceTarget(*grid, settings);

    volume_surface::SurfaceTargetCacheMetadata metadata;
    metadata.sourcePath = "synthetic/layered.vdb";
    metadata.gridName = "density";
    metadata.sourceFileSize = 1234;
    metadata.sourceWriteTime = 5678;
    metadata.gridClass = static_cast<std::int32_t>(grid->getGridClass());
    metadata.hasActiveBounds = true;
    metadata.activeBoundsMinimum = {-2, -2, -2};
    metadata.activeBoundsMaximum = {2, 2, 2};
    metadata.settings = settings;

    const auto path = std::filesystem::current_path() /
        "surface-target-roundtrip-test.bin";
    std::error_code errorCode;
    std::filesystem::remove(path, errorCode);
    std::string error;
    require(volume_surface::saveSurfaceTargetCache(
                path,
                cache,
                metadata,
                &error),
        "surface target cache save failed");

    volume_surface::SurfaceTargetCache loaded;
    require(volume_surface::loadSurfaceTargetCache(
                path,
                metadata,
                loaded,
                &error),
        "surface target cache load failed");
    require(loaded.samples.size() == cache.samples.size() &&
            loaded.coreCount == cache.coreCount &&
            loaded.transitionCount == cache.transitionCount,
        "surface target cache round trip changed counts");
    for (std::size_t index = 0; index < cache.samples.size(); ++index) {
        const auto& lhs = cache.samples[index];
        const auto& rhs = loaded.samples[index];
        require(lhs.coordinate == rhs.coordinate && lhs.kind == rhs.kind &&
                lhs.transitionLayer == rhs.transitionLayer,
            "surface target cache round trip changed sample identity");
        require((lhs.worldPosition - rhs.worldPosition).lengthSqr() < 1.0e-12f &&
                (lhs.normal - rhs.normal).lengthSqr() < 1.0e-12f &&
                std::abs(lhs.supportWeight - rhs.supportWeight) < 1.0e-6f,
            "surface target cache round trip changed sample values");
    }

    auto mismatchedMetadata = metadata;
    mismatchedMetadata.settings.isoValue = 254.0;
    require(!volume_surface::loadSurfaceTargetCache(
                path,
                mismatchedMetadata,
                loaded,
                &error),
        "surface target cache accepted mismatched metadata");
    std::filesystem::remove(path, errorCode);
}

void testLocalNormalSeedStoreRoundTrip()
{
    volume_surface::SurfaceTargetCacheMetadata metadata;
    metadata.sourcePath = "synthetic/layered.vdb";
    metadata.gridName = "density";
    metadata.sourceFileSize = 1234;
    metadata.sourceWriteTime = 5678;
    metadata.settings.isoValue = 255.0;
    metadata.settings.transitionLayers = 2;

    const std::vector<volume_surface::viewer::SurfaceNormalLocalSeedRecord> seeds{
        {openvdb::Coord(1, 2, 3), 1},
        {openvdb::Coord(-4, 5, -6), 0},
        {openvdb::Coord(7, 8, 9), 1, false}};
    const auto path = std::filesystem::current_path() /
        "surface-target-local-seeds-roundtrip.jsonl";
    std::error_code errorCode;
    std::filesystem::remove(path, errorCode);
    std::string error;
    require(volume_surface::viewer::SurfaceNormalLocalSeedStore::save(
                path,
                seeds,
                metadata,
                error),
        "local normal seed cache save failed");

    std::vector<volume_surface::viewer::SurfaceNormalLocalSeedRecord> loaded;
    require(volume_surface::viewer::SurfaceNormalLocalSeedStore::load(
                path,
                metadata,
                loaded,
                error),
        "local normal seed cache load failed");
    require(loaded.size() == seeds.size() &&
            loaded[0].coordinate == seeds[0].coordinate &&
            loaded[0].targetSign == seeds[0].targetSign &&
            loaded[1].coordinate == seeds[1].coordinate &&
            loaded[1].targetSign == seeds[1].targetSign &&
            loaded[0].activeFlipPoint && loaded[1].activeFlipPoint &&
            loaded[2].coordinate == seeds[2].coordinate &&
            !loaded[2].activeFlipPoint,
        "local normal seed cache round trip changed records");

    auto mismatchedMetadata = metadata;
    mismatchedMetadata.settings.isoValue = 254.0;
    require(!volume_surface::viewer::SurfaceNormalLocalSeedStore::load(
                path,
                mismatchedMetadata,
                loaded,
                error),
        "local normal seed cache accepted mismatched metadata");
    std::filesystem::remove(path, errorCode);
}

void testConnectedNormalTrend()
{
    const auto grid = createLayeredFog();
    volume_surface::SurfaceTargetSettings targetSettings;
    targetSettings.isoValue = 255.0;
    targetSettings.transitionLayers = 2;
    targetSettings.normalRadius = 2.0;
    targetSettings.planarityRadius = 1.0;
    const auto cache = volume_surface::extractSurfaceTarget(*grid, targetSettings);

    volume_surface::SurfaceNormalSmoothingSettings normalSettings;
    normalSettings.strength = 1.0;
    normalSettings.robustIterations = 2;
    normalSettings.neighborhood = volume_surface::SurfaceNormalNeighborhood::Grid3x3;
    const auto small = volume_surface::smoothSurfaceTargetNormals(
        cache,
        normalSettings);
    require(small.normals.size() == cache.samples.size(),
        "3x3 normal trend changed sample cardinality");
    require(small.smoothedCoreCount > 0,
        "3x3 normal trend produced no core normals");

    normalSettings.neighborhood = volume_surface::SurfaceNormalNeighborhood::Grid5x5;
    const auto large = volume_surface::smoothSurfaceTargetNormals(
        cache,
        normalSettings);
    require(large.normals.size() == cache.samples.size(),
        "5x5 normal trend changed sample cardinality");
    require(large.smoothedCoreCount > 0,
        "5x5 normal trend produced no core normals");
    const auto* core = findSample(cache, openvdb::Coord(0, 0, 0));
    require(core != nullptr, "normal trend test core sample is missing");
    std::size_t coreIndex = 0;
    while (coreIndex < cache.samples.size() && &cache.samples[coreIndex] != core) {
        ++coreIndex;
    }
    require(coreIndex < cache.samples.size(), "normal trend core index is invalid");
    require(openvdb::Vec3d(small.normals[coreIndex]).lengthSqr() > 0.99,
        "3x3 normal trend did not produce a unit normal axis");
    require(openvdb::Vec3d(large.normals[coreIndex]).lengthSqr() > 0.99,
        "5x5 normal trend did not produce a unit normal axis");

    volume_surface::SurfaceNormalFitSettings fitSettings;
    fitSettings.neighborhood = volume_surface::SurfaceFitNeighborhood::Grid9x9;
    const auto fitted = volume_surface::fitSurfaceTargetNormals(
        cache,
        fitSettings);
    require(fitted.normals.size() == cache.samples.size() &&
            fitted.smoothedCoreCount > 0,
        "9x9 surface fit produced no fitted core normals");
    require(openvdb::Vec3d(fitted.normals[coreIndex]).lengthSqr() > 0.99,
        "9x9 surface fit did not produce a unit normal axis");
    auto geometryOnlyCache = cache;
    for (auto& sample : geometryOnlyCache.samples) {
        sample.normal = openvdb::Vec3f(0.0f);
    }
    const auto fittedWithoutSourceNormals = volume_surface::fitSurfaceTargetNormals(
        geometryOnlyCache,
        fitSettings);
    require(fittedWithoutSourceNormals.normals.size() == cache.samples.size() &&
            fittedWithoutSourceNormals.smoothedCoreCount > 0,
        "surface fit still depended on source normal values");
    require(std::abs(openvdb::Vec3d(fittedWithoutSourceNormals.normals[coreIndex]).dot(
        openvdb::Vec3d(fitted.normals[coreIndex]))) > 0.99,
        "geometry-only fit changed the fitted normal axis");
    const auto gridOverload = volume_surface::fitSurfaceTargetNormals(
        *grid,
        cache,
        fitSettings);
    require(gridOverload.normals[coreIndex].lengthSqr() > 0.99f,
        "grid overload did not produce a unit normal axis");
    normalSettings.neighborhood = volume_surface::SurfaceNormalNeighborhood::None;
    const auto unsmoothed = volume_surface::smoothSurfaceTargetNormals(
        cache,
        fitted,
        normalSettings);
    require(unsmoothed.normals.size() == fitted.normals.size() &&
            unsmoothed.smoothedCoreCount == 0 &&
            (openvdb::Vec3d(unsmoothed.normals[coreIndex]) -
                openvdb::Vec3d(fitted.normals[coreIndex])).lengthSqr() < 1.0e-12,
        "None normal trend changed the fitted seed field");

    const openvdb::Vec3d fittedAxis =
        openvdb::Vec3d(fitted.normals[coreIndex]).unit();
    const auto oriented = volume_surface::orientSurfaceTargetNormals(
        cache,
        fitted,
        coreIndex,
        -fittedAxis);
    require(oriented.normals.size() == fitted.normals.size() &&
            oriented.smoothedCoreCount > 0,
        "orientation seed produced no oriented core normals");
    require(openvdb::Vec3d(oriented.normals[coreIndex]).dot(-fittedAxis) > 0.99,
        "orientation seed did not set the selected seed direction");
    std::size_t firstTransitionIndex = 0;
    std::size_t secondTransitionIndex = 0;
    bool foundFirstTransition = false;
    bool foundSecondTransition = false;
    for (std::size_t index = 0; index < cache.samples.size(); ++index) {
        if (cache.samples[index].coordinate == openvdb::Coord(1, 0, 0)) {
            firstTransitionIndex = index;
            foundFirstTransition = true;
        } else if (cache.samples[index].coordinate == openvdb::Coord(2, 0, 0)) {
            secondTransitionIndex = index;
            foundSecondTransition = true;
        }
    }
    require(foundFirstTransition && foundSecondTransition,
        "orientation transition samples are missing");
    require(openvdb::Vec3d(oriented.normals[firstTransitionIndex]).dot(
                openvdb::Vec3d(oriented.normals[coreIndex])) > 0.99 &&
            openvdb::Vec3d(oriented.normals[secondTransitionIndex]).dot(
                openvdb::Vec3d(oriented.normals[firstTransitionIndex])) > 0.99,
        "transition orientation did not propagate through multiple layers");
    const auto orientationStats =
        volume_surface::analyzeSurfaceTargetNormalAdjacency(
            cache,
            oriented,
            coreIndex);
    require(orientationStats.validCoreSampleCount > 0 &&
            orientationStats.adjacencyEdgeCount > 0 &&
            orientationStats.connectedComponentCount > 0,
        "normal adjacency analysis did not find the connected core surface");
    require(orientationStats.seededComponentSampleCount > 0 &&
            orientationStats.unseededComponentSampleCount == 0,
        "normal adjacency analysis misclassified the seeded component");
}

void testFitSignIslandRepair()
{
    const auto grid = createMetricGrid(openvdb::Vec3d(0.00031, 0.00031, 0.001));
    const auto cache = createCoreSheet(*grid, 11, 11, false);
    auto raw = createUniformNormalField(cache, openvdb::Vec3f(0.0f, 0.0f, 1.0f));
    for (int y = 4; y <= 6; ++y) {
        for (int x = 4; x <= 6; ++x) {
            raw.normals[findSampleIndex(cache, openvdb::Coord(x, y, 0))] *= -1.0f;
        }
    }

    volume_surface::SurfaceNormalFitSignRepairSettings settings;
    settings.maximumIslandAreaSquareMillimeters = 2.0;
    const auto repaired = volume_surface::repairSurfaceNormalFitSignIslands(
        *grid,
        cache,
        raw,
        settings);
    require(repaired.report.validCoreEdgeCount > 0,
        "fit sign repair did not evaluate core adjacency edges");
    require(repaired.report.candidateIslandCount > 0 &&
            repaired.report.acceptedIslandCount == 1 &&
            repaired.report.flippedCoreSampleCount == 9,
        "fit sign repair did not accept the reversed small island");
    require(repaired.report.weightedOpposingSupportAfter <
            repaired.report.weightedOpposingSupportBefore,
        "fit sign repair did not reduce opposing boundary support");
    for (const auto& normal : repaired.field.normals) {
        require(normal.z() > 0.99f,
            "fit sign repair left a reversed sample in the small island");
    }
}

void testFitSignRepairLeavesComparableRegionsUntouched()
{
    const auto grid = createMetricGrid(openvdb::Vec3d(0.00031, 0.00031, 0.001));
    const auto cache = createCoreSheet(*grid, 10, 6, false);
    auto raw = createUniformNormalField(cache, openvdb::Vec3f(0.0f, 0.0f, 1.0f));
    for (int y = 0; y < 6; ++y) {
        for (int x = 0; x < 5; ++x) {
            raw.normals[findSampleIndex(cache, openvdb::Coord(x, y, 0))] *= -1.0f;
        }
    }

    volume_surface::SurfaceNormalFitSignRepairSettings settings;
    settings.maximumIslandAreaSquareMillimeters = 100.0;
    const auto repaired = volume_surface::repairSurfaceNormalFitSignIslands(
        *grid,
        cache,
        raw,
        settings);
    require(repaired.report.acceptedIslandCount == 0,
        "fit sign repair changed similarly sized opposing regions");
    require(repaired.field.normals[findSampleIndex(cache, openvdb::Coord(0, 0, 0))].z() < -0.99f,
        "fit sign repair changed the comparable region orientation");
}

void testFitSignRepairLeavesDisconnectedIslandUntouched()
{
    const auto grid = createMetricGrid(openvdb::Vec3d(0.00031, 0.00031, 0.001));
    auto cache = createCoreSheet(*grid, 8, 8, false);
    for (int y = 0; y < 2; ++y) {
        for (int x = 0; x < 2; ++x) {
            volume_surface::SurfaceTargetSample sample;
            sample.coordinate = openvdb::Coord(30 + x, y, 0);
            sample.worldPosition = openvdb::Vec3f(
                grid->indexToWorld(sample.coordinate.asVec3d()));
            sample.kind = volume_surface::SurfaceTargetSampleKind::Core;
            cache.samples.push_back(sample);
        }
    }
    cache.coreCount = cache.samples.size();
    auto raw = createUniformNormalField(cache, openvdb::Vec3f(0.0f, 0.0f, 1.0f));
    for (int y = 0; y < 2; ++y) {
        for (int x = 0; x < 2; ++x) {
            raw.normals[findSampleIndex(cache, openvdb::Coord(30 + x, y, 0))] *= -1.0f;
        }
    }

    const auto repaired = volume_surface::repairSurfaceNormalFitSignIslands(
        *grid,
        cache,
        raw);
    require(repaired.report.acceptedIslandCount == 0,
        "fit sign repair changed a disconnected island");
    require(repaired.field.normals[findSampleIndex(cache, openvdb::Coord(30, 0, 0))].z() < -0.99f,
        "fit sign repair changed the disconnected island orientation");
}

void testFitSignRepairUsesPhysicalArea()
{
    const auto grid = createMetricGrid(openvdb::Vec3d(0.00031, 0.00031, 0.004));
    const auto cache = createCoreSheet(*grid, 11, 11, true);
    auto raw = createUniformNormalField(cache, openvdb::Vec3f(1.0f, 0.0f, 0.0f));
    for (int z = 4; z <= 6; ++z) {
        for (int y = 4; y <= 6; ++y) {
            raw.normals[findSampleIndex(cache, openvdb::Coord(0, y, z))] *= -1.0f;
        }
    }

    volume_surface::SurfaceNormalFitSignRepairSettings settings;
    settings.maximumIslandAreaSquareMillimeters = 2.0;
    const auto rejected = volume_surface::repairSurfaceNormalFitSignIslands(
        *grid,
        cache,
        raw,
        settings);
    require(rejected.report.acceptedIslandCount == 0,
        "fit sign repair ignored the anisotropic physical area");

    settings.maximumIslandAreaSquareMillimeters = 20.0;
    const auto accepted = volume_surface::repairSurfaceNormalFitSignIslands(
        *grid,
        cache,
        raw,
        settings);
    require(accepted.report.acceptedIslandCount == 1 &&
            accepted.report.acceptedPatches.front().areaSquareMillimeters > 10.0,
        "fit sign repair did not report anisotropic physical island area");
}

void testFitSignRepairProtectsSeedNeighborhood()
{
    const auto grid = createMetricGrid(openvdb::Vec3d(0.00031, 0.00031, 0.001));
    const auto cache = createCoreSheet(*grid, 11, 11, false);
    auto primary = createUniformNormalField(cache, openvdb::Vec3f(0.0f, 0.0f, 1.0f));
    for (int y = 4; y <= 6; ++y) {
        for (int x = 4; x <= 6; ++x) {
            primary.normals[findSampleIndex(cache, openvdb::Coord(x, y, 0))] *= -1.0f;
        }
    }

    volume_surface::SurfaceNormalExpansionTrace trace;
    trace.parentBySample.assign(cache.samples.size(), -1);
    trace.depthBySample.assign(cache.samples.size(), 10);
    trace.orientedBySample.assign(cache.samples.size(), 1);
    trace.depthBySample[findSampleIndex(cache, openvdb::Coord(5, 5, 0))] = 0;

    volume_surface::SurfaceNormalFitSignRepairSettings settings;
    settings.maximumIslandAreaSquareMillimeters = 100.0;
    const auto repaired = volume_surface::repairSurfaceNormalFitSignIslands(
        *grid,
        cache,
        primary,
        settings,
        &trace);
    require(repaired.report.acceptedIslandCount == 0 &&
            repaired.report.flippedCoreSampleCount == 0,
        "fit sign repair changed a region containing the protected seed neighborhood");
}

void testLocalNormalSeedPreviewAndApply()
{
    const auto grid = createMetricGrid(openvdb::Vec3d(0.00031, 0.00031, 0.001));
    const auto cache = createCoreSheet(*grid, 11, 11, false);
    const auto axes = createUniformNormalField(cache, openvdb::Vec3f(0.0f, 0.0f, 1.0f));
    auto global = axes;
    for (int y = 4; y <= 6; ++y) {
        for (int x = 4; x <= 6; ++x) {
            global.normals[findSampleIndex(cache, openvdb::Coord(x, y, 0))] *= -1.0f;
        }
    }
    const std::size_t seedIndex = findSampleIndex(cache, openvdb::Coord(5, 5, 0));
    const auto preview = volume_surface::previewSurfaceNormalLocalSeed(
        cache,
        axes,
        global,
        seedIndex);
    require(preview.valid && preview.affectedCoreSampleCount == 9,
        "local seed preview did not isolate the reversed region");
    require(preview.boundaryCoreSampleCount > 0,
        "local seed preview did not record a stopping boundary");
    const auto applied = volume_surface::applySurfaceNormalLocalSeed(
        cache,
        global,
        preview);
    for (const auto& normal : applied.normals) {
        require(normal.z() > 0.99f,
            "local seed apply left a reversed sample");
    }
}

void testLocalFlipPointChainAndCap()
{
    const auto grid = createMetricGrid(openvdb::Vec3d(0.001, 0.001, 0.001));
    const auto cache = createCoreSheet(*grid, 7, 1, false);
    const auto axes = createUniformNormalField(cache, openvdb::Vec3f(0.0f, 0.0f, 1.0f));
    auto global = axes;
    for (int x = 2; x <= 4; ++x) {
        global.normals[findSampleIndex(cache, openvdb::Coord(x, 0, 0))] *= -1.0f;
    }
    const std::size_t flipPoint = findSampleIndex(cache, openvdb::Coord(3, 0, 0));
    const auto preview = volume_surface::previewSurfaceNormalLocalFlipPoint(
        cache,
        axes,
        global,
        flipPoint);
    require(preview.valid && preview.affectedCoreSampleCount == 3,
        "flip point propagation did not follow the consecutive opposing chain");
    require(preview.parentBySample[findSampleIndex(cache, openvdb::Coord(2, 0, 0))] ==
                static_cast<std::int32_t>(flipPoint),
        "flip point chain did not retain the parent link");

    auto cappedSettings = volume_surface::SurfaceNormalLocalSeedSettings{};
    cappedSettings.maximumCoreSamples = 2;
    const auto capped = volume_surface::previewSurfaceNormalLocalFlipPoint(
        cache,
        axes,
        global,
        flipPoint,
        cappedSettings);
    require(capped.maximumCoreSamplesReached,
        "flip point preview did not report the sample cap");
    const auto unchanged = volume_surface::applySurfaceNormalLocalFlipPoint(
        cache,
        global,
        capped);
    require(unchanged.normals[flipPoint].z() < -0.99f,
        "capped flip point preview was committed as a partial correction");
}

void testLocalFlipPointFollowsExpansionChildrenOnly()
{
    const auto grid = createMetricGrid(openvdb::Vec3d(0.001, 0.001, 0.001));
    const auto cache = createCoreSheet(*grid, 7, 1, false);
    const auto axes = createUniformNormalField(cache, openvdb::Vec3f(0.0f, 0.0f, 1.0f));
    auto global = axes;
    for (int x = 2; x <= 4; ++x) {
        global.normals[findSampleIndex(cache, openvdb::Coord(x, 0, 0))] *= -1.0f;
    }

    const std::size_t upstream = findSampleIndex(cache, openvdb::Coord(2, 0, 0));
    const std::size_t flipPoint = findSampleIndex(cache, openvdb::Coord(3, 0, 0));
    const std::size_t downstream = findSampleIndex(cache, openvdb::Coord(4, 0, 0));
    volume_surface::SurfaceNormalExpansionTrace trace;
    trace.parentBySample.assign(cache.samples.size(), -1);
    trace.depthBySample.assign(cache.samples.size(), -1);
    trace.orientedBySample.assign(cache.samples.size(), 1);
    trace.parentBySample[flipPoint] = static_cast<std::int32_t>(upstream);
    trace.parentBySample[downstream] = static_cast<std::int32_t>(flipPoint);
    trace.depthBySample[upstream] = 0;
    trace.depthBySample[flipPoint] = 1;
    trace.depthBySample[downstream] = 2;

    const auto preview = volume_surface::previewSurfaceNormalLocalFlipPoint(
        cache,
        axes,
        global,
        flipPoint,
        {},
        &trace);
    require(preview.valid && preview.affectedCoreSampleCount == 2,
        "directed flip point propagation did not isolate the downstream chain");
    require(std::find(
                preview.affectedCoreSampleIndices.begin(),
                preview.affectedCoreSampleIndices.end(),
                upstream) == preview.affectedCoreSampleIndices.end(),
        "directed flip point propagation changed an upstream sample");
    require(std::find(
                preview.affectedCoreSampleIndices.begin(),
                preview.affectedCoreSampleIndices.end(),
                downstream) != preview.affectedCoreSampleIndices.end(),
        "directed flip point propagation skipped a downstream sample");
}

void testVdbSurfaceProbe()
{
    const auto grid = createLayeredFog();
    volume_surface::VdbSurfaceProbeSettings settings;
    settings.isoValue = 255.0;
    settings.valueTolerance = 0.5;
    const auto result = volume_surface::projectVdbSurface(
        *grid,
        grid->indexToWorld(openvdb::Vec3d(0.5, 0.0, 0.0)),
        settings);
    require(result.valid && result.converged,
        "VDB surface probe did not converge on a layered fog boundary");
    require(std::abs(result.indexPosition.x()) < 0.01,
        "VDB surface probe converged to the wrong index position");
    require(result.nearestVoxel == openvdb::Coord(0, 0, 0),
        "VDB surface probe returned the wrong nearest voxel");
}

} // namespace

int main()
{
    try {
        openvdb::initialize();
        testCoreAndTransitionSamples();
        testCacheSaveLoadRoundTrip();
        testLocalNormalSeedStoreRoundTrip();
        testConnectedNormalTrend();
        testFitSignIslandRepair();
        testFitSignRepairLeavesComparableRegionsUntouched();
        testFitSignRepairLeavesDisconnectedIslandUntouched();
        testFitSignRepairUsesPhysicalArea();
        testFitSignRepairProtectsSeedNeighborhood();
        testLocalNormalSeedPreviewAndApply();
        testLocalFlipPointChainAndCap();
        testLocalFlipPointFollowsExpansionChildrenOnly();
        testVdbSurfaceProbe();
        std::cout << "surface_target_tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "surface_target_tests failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
