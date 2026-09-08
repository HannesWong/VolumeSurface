#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include <openvdb/openvdb.h>

#include "volume_surface/SurfaceTarget.h"

namespace volume_surface::viewer {

struct SurfaceNormalLocalSeedRecord
{
    openvdb::Coord coordinate{};
    std::uint8_t targetSign = 1;
    // False is retained only for legacy seed records. New records default to
    // the explicit flip-point semantic.
    bool activeFlipPoint = true;
    std::array<std::uint8_t, 4> color{255, 255, 255, 255};
    bool hasColor = false;
    bool accepted = true;
    bool hasAccepted = false;
};

class SurfaceNormalLocalSeedStore final
{
public:
    static std::filesystem::path pathForInput(
        const std::filesystem::path& input,
        const std::string& gridName);

    static bool save(
        const std::filesystem::path& path,
        const std::vector<SurfaceNormalLocalSeedRecord>& seeds,
        const volume_surface::SurfaceTargetCacheMetadata& metadata,
        std::string& error);

    static bool load(
        const std::filesystem::path& path,
        const volume_surface::SurfaceTargetCacheMetadata& expectedMetadata,
        std::vector<SurfaceNormalLocalSeedRecord>& seeds,
        std::string& error);
};

} // namespace volume_surface::viewer
