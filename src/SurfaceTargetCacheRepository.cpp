#include "volume_surface/viewer/SurfaceTargetCacheRepository.h"

#include <system_error>

namespace volume_surface::viewer {

std::filesystem::path SurfaceTargetCacheRepository::pathForInput(
    const std::filesystem::path& input,
    const std::string& gridName)
{
    const std::string sourceStem = input.stem().empty()
        ? std::string("surface")
        : input.stem().string();
    const std::string gridSuffix = gridName.empty()
        ? std::string("grid")
        : gridName;
    return input.parent_path() /
        (sourceStem + "." + gridSuffix + ".surface-target.bin");
}

volume_surface::SurfaceTargetCacheMetadata
SurfaceTargetCacheRepository::metadataForInput(
    const std::filesystem::path& input,
    const std::string& gridName,
    const openvdb::FloatGrid* grid,
    const volume_surface::SurfaceTargetSettings& settings)
{
    volume_surface::SurfaceTargetCacheMetadata metadata;
    metadata.sourcePath = std::filesystem::absolute(input)
        .lexically_normal()
        .generic_string();
    metadata.gridName = gridName;
    std::error_code errorCode;
    metadata.sourceFileSize = std::filesystem::file_size(input, errorCode);
    if (errorCode) {
        metadata.sourceFileSize = 0;
        errorCode.clear();
    }
    const auto writeTime = std::filesystem::last_write_time(input, errorCode);
    metadata.sourceWriteTime = errorCode
        ? 0
        : static_cast<std::int64_t>(writeTime.time_since_epoch().count());
    if (grid) {
        metadata.gridClass = static_cast<std::int32_t>(grid->getGridClass());
        const auto bounds = grid->evalActiveVoxelBoundingBox();
        if (!bounds.empty()) {
            metadata.hasActiveBounds = true;
            metadata.activeBoundsMinimum = {
                bounds.min().x(), bounds.min().y(), bounds.min().z()};
            metadata.activeBoundsMaximum = {
                bounds.max().x(), bounds.max().y(), bounds.max().z()};
        }
    }
    metadata.settings = settings;
    return metadata;
}

bool SurfaceTargetCacheRepository::save(
    const std::filesystem::path& path,
    const volume_surface::SurfaceTargetCache& cache,
    const volume_surface::SurfaceTargetCacheMetadata& metadata,
    std::string& error)
{
    error.clear();
    if (!volume_surface::saveSurfaceTargetCache(path, cache, metadata, &error)) {
        return false;
    }
    return true;
}

bool SurfaceTargetCacheRepository::load(
    const std::filesystem::path& path,
    const volume_surface::SurfaceTargetCacheMetadata& expectedMetadata,
    volume_surface::SurfaceTargetCache& cache,
    std::string& error)
{
    error.clear();
    if (!volume_surface::loadSurfaceTargetCache(
            path,
            expectedMetadata,
            cache,
            &error)) {
        return false;
    }
    return true;
}

} // namespace volume_surface::viewer
