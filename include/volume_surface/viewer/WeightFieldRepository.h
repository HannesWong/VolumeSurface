#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <openvdb/openvdb.h>

namespace volume_surface::viewer {

class WeightFieldRepository final {
public:
    static openvdb::FloatGrid::Ptr createEmpty(
        const openvdb::FloatGrid& referenceGrid);

    static std::filesystem::path directoryForInput(
        const std::filesystem::path& input);
    static std::filesystem::path pathForName(
        const std::filesystem::path& directory,
        const std::string& name);
    static bool isValidName(const std::string& name);
    static std::string nameFromPath(const std::filesystem::path& path);

    static std::vector<std::filesystem::path> list(
        const std::filesystem::path& directory,
        std::string& error);

    static bool save(
        const std::filesystem::path& directory,
        const std::filesystem::path& currentPath,
        const std::string& name,
        bool saveAsNew,
        const openvdb::FloatGrid::Ptr& weightGrid,
        const openvdb::FloatGrid& sourceGrid,
        const std::string& sourceGridName,
        std::filesystem::path& savedPath,
        std::string& error);

    static bool load(
        const std::filesystem::path& path,
        const openvdb::FloatGrid& sourceGrid,
        const std::string& sourceGridName,
        openvdb::FloatGrid::Ptr& loadedGrid,
        std::string& error);
};

} // namespace volume_surface::viewer
