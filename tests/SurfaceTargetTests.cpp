#include "volume_surface/SurfaceTarget.h"

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

} // namespace

int main()
{
    try {
        openvdb::initialize();
        testCoreAndTransitionSamples();
        testCacheSaveLoadRoundTrip();
        std::cout << "surface_target_tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "surface_target_tests failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
