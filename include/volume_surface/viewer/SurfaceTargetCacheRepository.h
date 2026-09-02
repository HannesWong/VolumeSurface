#pragma once

#include <filesystem>
#include <string>

#include "volume_surface/SurfaceTarget.h"

namespace volume_surface::viewer {

class SurfaceTargetCacheRepository final {
public:
    static std::filesystem::path pathForInput(
        const std::filesystem::path& input,
        const std::string& gridName);

    static volume_surface::SurfaceTargetCacheMetadata metadataForInput(
        const std::filesystem::path& input,
        const std::string& gridName,
        const openvdb::FloatGrid* grid,
        const volume_surface::SurfaceTargetSettings& settings);

    static bool save(
        const std::filesystem::path& path,
        const volume_surface::SurfaceTargetCache& cache,
        const volume_surface::SurfaceTargetCacheMetadata& metadata,
        std::string& error);

    static bool load(
        const std::filesystem::path& path,
        const volume_surface::SurfaceTargetCacheMetadata& expectedMetadata,
        volume_surface::SurfaceTargetCache& cache,
        std::string& error);
};

} // namespace volume_surface::viewer
