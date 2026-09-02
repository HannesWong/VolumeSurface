#include "volume_surface/viewer/WeightFieldRepository.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <string_view>
#include <system_error>

#include <openvdb/io/File.h>

namespace volume_surface::viewer {

namespace {

constexpr std::string_view gridName = "surface_weight";
constexpr std::string_view fileSuffix = ".brushmask.vdb";
constexpr std::string_view schemaKey = "volume_surface.weight_field_schema";
constexpr std::string_view sourceGridKey = "volume_surface.source_grid";
constexpr std::string_view sourceBoundsMinimumKey =
    "volume_surface.source_bounds_minimum";
constexpr std::string_view sourceBoundsMaximumKey =
    "volume_surface.source_bounds_maximum";
constexpr std::int32_t schemaVersion = 1;

void writeMetadata(
    openvdb::FloatGrid& weightGrid,
    const openvdb::FloatGrid& sourceGrid,
    const std::string& sourceGridName)
{
    const openvdb::CoordBBox sourceBounds = sourceGrid.evalActiveVoxelBoundingBox();
    const std::array<std::string_view, 4> metadataKeys{
        schemaKey,
        sourceGridKey,
        sourceBoundsMinimumKey,
        sourceBoundsMaximumKey};
    for (const std::string_view key : metadataKeys) {
        weightGrid.removeMeta(std::string(key));
    }
    weightGrid.setName(std::string(gridName));
    weightGrid.insertMeta(
        std::string(schemaKey),
        openvdb::Int32Metadata(schemaVersion));
    weightGrid.insertMeta(
        std::string(sourceGridKey),
        openvdb::StringMetadata(sourceGridName));
    weightGrid.insertMeta(
        std::string(sourceBoundsMinimumKey),
        openvdb::Vec3IMetadata(sourceBounds.min().asVec3i()));
    weightGrid.insertMeta(
        std::string(sourceBoundsMaximumKey),
        openvdb::Vec3IMetadata(sourceBounds.max().asVec3i()));
}

bool validateMetadata(
    const openvdb::FloatGrid& weightGrid,
    const openvdb::FloatGrid& sourceGrid,
    const std::string& sourceGridName,
    std::string& error)
{
    if (weightGrid.getName() != gridName) {
        error = "The VDB does not contain a surface_weight grid";
        return false;
    }
    const auto schema = weightGrid.getMetadata<openvdb::Int32Metadata>(
        std::string(schemaKey));
    const auto storedSourceGrid = weightGrid.getMetadata<openvdb::StringMetadata>(
        std::string(sourceGridKey));
    const auto storedBoundsMinimum = weightGrid.getMetadata<openvdb::Vec3IMetadata>(
        std::string(sourceBoundsMinimumKey));
    const auto storedBoundsMaximum = weightGrid.getMetadata<openvdb::Vec3IMetadata>(
        std::string(sourceBoundsMaximumKey));
    if (!schema || schema->value() != schemaVersion || !storedSourceGrid ||
        !storedBoundsMinimum || !storedBoundsMaximum) {
        error = "The weight field metadata is incomplete or uses an unsupported schema";
        return false;
    }
    if (storedSourceGrid->value() != sourceGridName) {
        error = "The weight field was created for a different grid";
        return false;
    }
    const openvdb::CoordBBox sourceBounds = sourceGrid.evalActiveVoxelBoundingBox();
    if (storedBoundsMinimum->value() != sourceBounds.min().asVec3i() ||
        storedBoundsMaximum->value() != sourceBounds.max().asVec3i()) {
        error = "The weight field source bounds do not match the current VDB";
        return false;
    }
    if (!(weightGrid.transform() == sourceGrid.transform())) {
        error = "The weight field transform does not match the current VDB";
        return false;
    }
    return true;
}

} // namespace

openvdb::FloatGrid::Ptr WeightFieldRepository::createEmpty(
    const openvdb::FloatGrid& referenceGrid)
{
    auto weightGrid = openvdb::FloatGrid::create(0.0f);
    weightGrid->setTransform(referenceGrid.transform().copy());
    weightGrid->setName(std::string(gridName));
    weightGrid->setGridClass(openvdb::GRID_UNKNOWN);
    return weightGrid;
}

std::filesystem::path WeightFieldRepository::directoryForInput(
    const std::filesystem::path& input)
{
    return input.parent_path() / (input.stem().string() + ".weight-fields");
}

std::filesystem::path WeightFieldRepository::pathForName(
    const std::filesystem::path& directory,
    const std::string& name)
{
    return directory / (name + std::string(fileSuffix));
}

bool WeightFieldRepository::isValidName(const std::string& name)
{
    if (name.empty() || name == "." || name == "..") {
        return false;
    }
    constexpr std::string_view forbiddenCharacters = "\\/:*?\"<>|";
    return std::all_of(
        name.begin(),
        name.end(),
        [forbiddenCharacters](const unsigned char character) {
            return character >= 0x20 && forbiddenCharacters.find(
                static_cast<char>(character)) == std::string_view::npos;
        });
}

std::string WeightFieldRepository::nameFromPath(
    const std::filesystem::path& path)
{
    std::string name = path.filename().string();
    if (name.ends_with(fileSuffix)) {
        name.resize(name.size() - fileSuffix.size());
    }
    return name;
}

std::vector<std::filesystem::path> WeightFieldRepository::list(
    const std::filesystem::path& directory,
    std::string& error)
{
    error.clear();
    std::vector<std::filesystem::path> files;
    std::error_code errorCode;
    if (!std::filesystem::exists(directory, errorCode)) {
        if (errorCode) {
            error = "Unable to inspect weight field library: " + errorCode.message();
        }
        return files;
    }

    for (std::filesystem::directory_iterator iterator(
             directory,
             std::filesystem::directory_options::skip_permission_denied,
             errorCode);
         !errorCode && iterator != std::filesystem::directory_iterator();
         iterator.increment(errorCode)) {
        if (!iterator->is_regular_file(errorCode)) {
            continue;
        }
        const std::string filename = iterator->path().filename().string();
        if (filename.ends_with(fileSuffix)) {
            files.push_back(iterator->path());
        }
    }
    if (errorCode) {
        error = "Unable to scan weight field library: " + errorCode.message();
        files.clear();
        return files;
    }
    std::sort(
        files.begin(),
        files.end(),
        [](const std::filesystem::path& left, const std::filesystem::path& right) {
            return left.filename().string() < right.filename().string();
        });
    return files;
}

bool WeightFieldRepository::save(
    const std::filesystem::path& directory,
    const std::filesystem::path& currentPath,
    const std::string& name,
    bool saveAsNew,
    const openvdb::FloatGrid::Ptr& weightGrid,
    const openvdb::FloatGrid& sourceGrid,
    const std::string& sourceGridName,
    std::filesystem::path& savedPath,
    std::string& error)
{
    error.clear();
    if (!weightGrid) {
        error = "The weight field is not initialized";
        return false;
    }
    if (!isValidName(name)) {
        error = "Weight field names cannot contain \\ / : * ? \" < > or |";
        return false;
    }
    const std::filesystem::path targetPath = saveAsNew || currentPath.empty()
        ? pathForName(directory, name)
        : currentPath;
    std::error_code errorCode;
    if ((saveAsNew || currentPath.empty()) &&
        targetPath != currentPath &&
        std::filesystem::exists(targetPath, errorCode)) {
        error = "A field with this name already exists; load it before overwriting";
        return false;
    }
    if (errorCode) {
        error = "Unable to inspect target field: " + errorCode.message();
        return false;
    }
    std::filesystem::create_directories(directory, errorCode);
    if (errorCode) {
        error = "Unable to create weight field library: " + errorCode.message();
        return false;
    }

    try {
        writeMetadata(*weightGrid, sourceGrid, sourceGridName);
        openvdb::GridPtrVec grids;
        grids.push_back(weightGrid);
        openvdb::io::File file(targetPath.string());
        file.write(grids);
        file.close();
        savedPath = targetPath;
        return true;
    } catch (const std::exception& exception) {
        error = std::string("Failed to save weight field: ") + exception.what();
        return false;
    }
}

bool WeightFieldRepository::load(
    const std::filesystem::path& path,
    const openvdb::FloatGrid& sourceGrid,
    const std::string& sourceGridName,
    openvdb::FloatGrid::Ptr& loadedGrid,
    std::string& error)
{
    error.clear();
    try {
        openvdb::io::File file(path.string());
        file.open();
        const openvdb::GridBase::Ptr baseGrid = file.readGrid(std::string(gridName));
        file.close();
        auto weightGrid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);
        if (!weightGrid) {
            error = "The selected VDB has no FloatGrid named surface_weight";
            return false;
        }
        if (!validateMetadata(*weightGrid, sourceGrid, sourceGridName, error)) {
            return false;
        }
        loadedGrid = std::move(weightGrid);
        return true;
    } catch (const std::exception& exception) {
        error = std::string("Failed to load weight field: ") + exception.what();
        return false;
    }
}

} // namespace volume_surface::viewer
