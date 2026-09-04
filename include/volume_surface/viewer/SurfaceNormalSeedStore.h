#pragma once

#include <filesystem>
#include <string>

#include <openvdb/openvdb.h>

namespace volume_surface::viewer {

struct SurfaceNormalSeed
{
    bool valid = false;
    openvdb::Coord coordinate{};
    openvdb::Vec3d worldPosition{};
    openvdb::Vec3d normal{};
    openvdb::Vec3d rayDirection{};
};

class SurfaceNormalSeedStore final
{
public:
    static std::filesystem::path pathForInput(
        const std::filesystem::path& input,
        const std::string& gridName);

    static bool save(
        const std::filesystem::path& path,
        const SurfaceNormalSeed& seed,
        const std::filesystem::path& sourcePath,
        const std::string& gridName,
        double isoValue,
        std::string& error);

    static bool load(
        const std::filesystem::path& path,
        const std::filesystem::path& sourcePath,
        const std::string& gridName,
        double isoValue,
        SurfaceNormalSeed& seed,
        std::string& error);
};

} // namespace volume_surface::viewer
