#include "volume_surface/viewer/SurfaceNormalSeedStore.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace volume_surface::viewer {
namespace {

constexpr int kSchemaVersion = 1;

std::string normalizedSourcePath(const std::filesystem::path& path)
{
    return path.lexically_normal().generic_string();
}

std::string escapeJsonString(const std::string& value)
{
    std::string escaped;
    escaped.reserve(value.size() + 8);
    for (const char character : value) {
        switch (character) {
            case '\\': escaped += "\\\\"; break;
            case '"': escaped += "\\\""; break;
            case '\n': escaped += "\\n"; break;
            case '\r': escaped += "\\r"; break;
            case '\t': escaped += "\\t"; break;
            default: escaped += character; break;
        }
    }
    return escaped;
}

bool readNumber(const std::string& json, const char* key, double& value)
{
    const std::string marker = std::string("\"") + key + "\"";
    const std::size_t markerPosition = json.find(marker);
    if (markerPosition == std::string::npos) return false;
    const std::size_t colonPosition = json.find(':', markerPosition + marker.size());
    if (colonPosition == std::string::npos) return false;
    const char* begin = json.c_str() + colonPosition + 1;
    char* end = nullptr;
    const double parsed = std::strtod(begin, &end);
    if (end == begin || !std::isfinite(parsed)) return false;
    value = parsed;
    return true;
}

bool readInteger(const std::string& json, const char* key, int& value)
{
    double parsed = 0.0;
    if (!readNumber(json, key, parsed) || std::floor(parsed) != parsed) {
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

bool readBoolean(const std::string& json, const char* key, bool& value)
{
    const std::string marker = std::string("\"") + key + "\"";
    const std::size_t markerPosition = json.find(marker);
    if (markerPosition == std::string::npos) return false;
    const std::size_t colonPosition = json.find(':', markerPosition + marker.size());
    if (colonPosition == std::string::npos) return false;
    const std::size_t truePosition = json.find("true", colonPosition + 1);
    const std::size_t falsePosition = json.find("false", colonPosition + 1);
    if (truePosition != std::string::npos &&
        (falsePosition == std::string::npos || truePosition < falsePosition)) {
        value = true;
        return true;
    }
    if (falsePosition != std::string::npos) {
        value = false;
        return true;
    }
    return false;
}

bool readString(const std::string& json, const char* key, std::string& value)
{
    const std::string marker = std::string("\"") + key + "\"";
    const std::size_t markerPosition = json.find(marker);
    if (markerPosition == std::string::npos) return false;
    const std::size_t colonPosition = json.find(':', markerPosition + marker.size());
    if (colonPosition == std::string::npos) return false;
    const std::size_t begin = json.find('"', colonPosition + 1);
    if (begin == std::string::npos) return false;
    std::string parsed;
    bool escaped = false;
    for (std::size_t index = begin + 1; index < json.size(); ++index) {
        const char character = json[index];
        if (escaped) {
            switch (character) {
                case '\\': parsed += '\\'; break;
                case '"': parsed += '"'; break;
                case 'n': parsed += '\n'; break;
                case 'r': parsed += '\r'; break;
                case 't': parsed += '\t'; break;
                default: parsed += character; break;
            }
            escaped = false;
        } else if (character == '\\') {
            escaped = true;
        } else if (character == '"') {
            value = parsed;
            return true;
        } else {
            parsed += character;
        }
    }
    return false;
}

bool readVector(const std::string& json, const char* key, openvdb::Vec3d& value)
{
    const std::string marker = std::string("\"") + key + "\"";
    const std::size_t markerPosition = json.find(marker);
    if (markerPosition == std::string::npos) return false;
    const std::size_t begin = json.find('[', markerPosition + marker.size());
    const std::size_t end = json.find(']', begin == std::string::npos ? markerPosition : begin);
    if (begin == std::string::npos || end == std::string::npos) return false;
    std::stringstream values(json.substr(begin + 1, end - begin - 1));
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    char separator = 0;
    if (!(values >> x >> separator >> y >> separator >> z) ||
        !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        return false;
    }
    value = openvdb::Vec3d{x, y, z};
    return true;
}

bool readCoord(const std::string& json, const char* key, openvdb::Coord& value)
{
    openvdb::Vec3d vector;
    if (!readVector(json, key, vector) ||
        std::floor(vector.x()) != vector.x() ||
        std::floor(vector.y()) != vector.y() ||
        std::floor(vector.z()) != vector.z()) {
        return false;
    }
    value = openvdb::Coord(
        static_cast<int>(vector.x()),
        static_cast<int>(vector.y()),
        static_cast<int>(vector.z()));
    return true;
}

bool sameIsoValue(double left, double right)
{
    return std::isfinite(left) && std::isfinite(right) &&
        std::abs(left - right) <= 1.0e-6 * std::max({1.0, std::abs(left), std::abs(right)});
}

} // namespace

std::filesystem::path SurfaceNormalSeedStore::pathForInput(
    const std::filesystem::path& input,
    const std::string& gridName)
{
    return input.parent_path() /
        (input.stem().string() + "." + gridName + ".surface-normal-seed.jsonl");
}

bool SurfaceNormalSeedStore::save(
    const std::filesystem::path& path,
    const SurfaceNormalSeed& seed,
    const std::filesystem::path& sourcePath,
    const std::string& gridName,
    double isoValue,
    std::string& error)
{
    error.clear();
    if (!seed.valid || !seed.worldPosition.isFinite() ||
        !seed.normal.isFinite() || seed.normal.lengthSqr() <= 1.0e-20 ||
        !seed.rayDirection.isFinite()) {
        error = "The orientation seed is incomplete";
        return false;
    }
    std::ofstream output(path, std::ios::trunc);
    if (!output) {
        error = "Failed to open " + path.string() + " for writing";
        return false;
    }
    output << std::setprecision(17)
           << "{\"schema\": " << kSchemaVersion
           << ", \"source\": \"" << escapeJsonString(normalizedSourcePath(sourcePath))
           << "\", \"grid\": \"" << escapeJsonString(gridName)
           << "\", \"iso\": " << isoValue
           << ", \"valid\": true"
           << ", \"coordinate\": [" << seed.coordinate.x() << ", "
           << seed.coordinate.y() << ", " << seed.coordinate.z() << "]"
           << ", \"world\": [" << seed.worldPosition.x() << ", "
           << seed.worldPosition.y() << ", " << seed.worldPosition.z() << "]"
           << ", \"normal\": [" << seed.normal.x() << ", "
           << seed.normal.y() << ", " << seed.normal.z() << "]"
           << ", \"ray_direction\": [" << seed.rayDirection.x() << ", "
           << seed.rayDirection.y() << ", " << seed.rayDirection.z() << "]}\n";
    if (!output) {
        error = "Failed while writing " + path.string();
        return false;
    }
    return true;
}

bool SurfaceNormalSeedStore::load(
    const std::filesystem::path& path,
    const std::filesystem::path& sourcePath,
    const std::string& gridName,
    double isoValue,
    SurfaceNormalSeed& seed,
    std::string& error)
{
    error.clear();
    std::ifstream input(path);
    if (!input) return false;
    std::stringstream buffer;
    buffer << input.rdbuf();
    const std::string json = buffer.str();
    int schema = 0;
    std::string storedSource;
    std::string storedGrid;
    double storedIso = 0.0;
    bool valid = false;
    SurfaceNormalSeed loaded;
    if (!readInteger(json, "schema", schema) || schema != kSchemaVersion ||
        !readString(json, "source", storedSource) ||
        !readString(json, "grid", storedGrid) ||
        !readNumber(json, "iso", storedIso) ||
        !readBoolean(json, "valid", valid) || !valid ||
        !readCoord(json, "coordinate", loaded.coordinate) ||
        !readVector(json, "world", loaded.worldPosition) ||
        !readVector(json, "normal", loaded.normal) ||
        !readVector(json, "ray_direction", loaded.rayDirection)) {
        error = "The orientation seed file is incomplete or unsupported";
        return false;
    }
    if (storedSource != normalizedSourcePath(sourcePath) ||
        storedGrid != gridName || !sameIsoValue(storedIso, isoValue)) {
        error = "The orientation seed belongs to a different source or iso value";
        return false;
    }
    loaded.valid = loaded.worldPosition.isFinite() && loaded.normal.isFinite() &&
        loaded.normal.lengthSqr() > 1.0e-20 && loaded.rayDirection.isFinite();
    if (!loaded.valid) {
        error = "The orientation seed contains invalid vectors";
        return false;
    }
    seed = loaded;
    return true;
}

} // namespace volume_surface::viewer
