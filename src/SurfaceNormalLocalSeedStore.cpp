#include "volume_surface/viewer/SurfaceNormalLocalSeedStore.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace volume_surface::viewer {
namespace {

constexpr int kSchemaVersion = 1;

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

bool readUnsignedInteger(
    const std::string& json,
    const char* key,
    std::uint64_t& value)
{
    const std::string marker = std::string("\"") + key + "\"";
    const std::size_t markerPosition = json.find(marker);
    if (markerPosition == std::string::npos) return false;
    const std::size_t colonPosition = json.find(':', markerPosition + marker.size());
    if (colonPosition == std::string::npos) return false;
    const char* begin = json.c_str() + colonPosition + 1;
    char* end = nullptr;
    const unsigned long long parsed = std::strtoull(begin, &end, 10);
    if (end == begin) return false;
    value = static_cast<std::uint64_t>(parsed);
    return true;
}

bool readSignedInteger(
    const std::string& json,
    const char* key,
    std::int64_t& value)
{
    const std::string marker = std::string("\"") + key + "\"";
    const std::size_t markerPosition = json.find(marker);
    if (markerPosition == std::string::npos) return false;
    const std::size_t colonPosition = json.find(':', markerPosition + marker.size());
    if (colonPosition == std::string::npos) return false;
    const char* begin = json.c_str() + colonPosition + 1;
    char* end = nullptr;
    const long long parsed = std::strtoll(begin, &end, 10);
    if (end == begin) return false;
    value = static_cast<std::int64_t>(parsed);
    return true;
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

bool readCoord(const std::string& json, const char* key, openvdb::Coord& value)
{
    const std::string marker = std::string("\"") + key + "\"";
    const std::size_t markerPosition = json.find(marker);
    if (markerPosition == std::string::npos) return false;
    const std::size_t begin = json.find('[', markerPosition + marker.size());
    const std::size_t end = json.find(
        ']', begin == std::string::npos ? markerPosition : begin);
    if (begin == std::string::npos || end == std::string::npos) return false;
    std::stringstream values(json.substr(begin + 1, end - begin - 1));
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    char separator = 0;
    if (!(values >> x >> separator >> y >> separator >> z) ||
        !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        std::floor(x) != x || std::floor(y) != y || std::floor(z) != z) {
        return false;
    }
    value = openvdb::Coord(
        static_cast<int>(x),
        static_cast<int>(y),
        static_cast<int>(z));
    return true;
}

bool readColor(
    const std::string& json,
    const char* key,
    std::array<std::uint8_t, 4>& value)
{
    const std::string marker = std::string("\"") + key + "\"";
    const std::size_t markerPosition = json.find(marker);
    if (markerPosition == std::string::npos) return false;
    const std::size_t begin = json.find('[', markerPosition + marker.size());
    const std::size_t end = json.find(
        ']', begin == std::string::npos ? markerPosition : begin);
    if (begin == std::string::npos || end == std::string::npos) return false;
    std::stringstream values(json.substr(begin + 1, end - begin - 1));
    std::array<std::uint8_t, 4> parsed{};
    for (std::size_t index = 0; index < parsed.size(); ++index) {
        unsigned int component = 0;
        if (!(values >> component) || component > 255) {
            return false;
        }
        parsed[index] = static_cast<std::uint8_t>(component);
        if (index + 1 < parsed.size()) {
            char separator = 0;
            if (!(values >> separator) || separator != ',') {
                return false;
            }
        }
    }
    value = parsed;
    return true;
}

bool sameIsoValue(double left, double right)
{
    return std::isfinite(left) && std::isfinite(right) &&
        std::abs(left - right) <=
            1.0e-6 * std::max({1.0, std::abs(left), std::abs(right)});
}

bool hasKind(const std::string& json, const char* kind)
{
    std::string storedKind;
    return readString(json, "kind", storedKind) && storedKind == kind;
}

bool readContext(
    const std::string& json,
    std::string& source,
    std::string& grid,
    std::uint64_t& sourceFileSize,
    std::int64_t& sourceWriteTime,
    double& isoValue,
    std::uint64_t& transitionLayers)
{
    return readString(json, "source", source) &&
        readString(json, "grid", grid) &&
        readUnsignedInteger(json, "source_file_size", sourceFileSize) &&
        readSignedInteger(json, "source_write_time", sourceWriteTime) &&
        readNumber(json, "iso", isoValue) &&
        readUnsignedInteger(json, "transition_layers", transitionLayers);
}

} // namespace

std::filesystem::path SurfaceNormalLocalSeedStore::pathForInput(
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
        (sourceStem + "." + gridSuffix + ".surface-target.local-seeds.jsonl");
}

bool SurfaceNormalLocalSeedStore::save(
    const std::filesystem::path& path,
    const std::vector<SurfaceNormalLocalSeedRecord>& seeds,
    const volume_surface::SurfaceTargetCacheMetadata& metadata,
    std::string& error)
{
    error.clear();
    std::ofstream output(path, std::ios::trunc);
    if (!output) {
        error = "Failed to open " + path.string() + " for writing";
        return false;
    }
    output << std::setprecision(17)
           << "{\"schema\": " << kSchemaVersion
           << ", \"kind\": \"metadata\""
           << ", \"source\": \"" << escapeJsonString(metadata.sourcePath)
           << "\", \"grid\": \"" << escapeJsonString(metadata.gridName)
           << "\", \"source_file_size\": " << metadata.sourceFileSize
           << ", \"source_write_time\": " << metadata.sourceWriteTime
           << ", \"iso\": " << metadata.settings.isoValue
           << ", \"transition_layers\": " << metadata.settings.transitionLayers
           << "}\n";
    for (const auto& seed : seeds) {
        output << "{\"schema\": " << kSchemaVersion
               << ", \"kind\": \""
               << (seed.activeFlipPoint ? "flip_point" : "seed")
               << "\""
               << ", \"coordinate\": [" << seed.coordinate.x() << ", "
               << seed.coordinate.y() << ", " << seed.coordinate.z() << "]"
               << ", \"target_sign\": "
               << static_cast<unsigned int>(seed.targetSign)
               << ", \"color\": ["
               << static_cast<unsigned int>(seed.color[0]) << ", "
               << static_cast<unsigned int>(seed.color[1]) << ", "
               << static_cast<unsigned int>(seed.color[2]) << ", "
               << static_cast<unsigned int>(seed.color[3]) << "]"
               << ", \"accepted\": "
               << (seed.accepted ? 1 : 0) << "}\n";
    }
    if (!output) {
        error = "Failed while writing " + path.string();
        return false;
    }
    return true;
}

bool SurfaceNormalLocalSeedStore::load(
    const std::filesystem::path& path,
    const volume_surface::SurfaceTargetCacheMetadata& expectedMetadata,
    std::vector<SurfaceNormalLocalSeedRecord>& seeds,
    std::string& error)
{
    error.clear();
    seeds.clear();
    std::ifstream input(path);
    if (!input) return false;

    std::string line;
    if (!std::getline(input, line) || line.empty() || !hasKind(line, "metadata")) {
        error = "The local flip-point cache has no valid metadata record";
        return false;
    }
    double schemaNumber = 0.0;
    std::string source;
    std::string grid;
    std::uint64_t sourceFileSize = 0;
    std::int64_t sourceWriteTime = 0;
    double isoValue = 0.0;
    std::uint64_t transitionLayers = 0;
    if (!readNumber(line, "schema", schemaNumber) ||
        schemaNumber != kSchemaVersion ||
        !readContext(
            line,
            source,
            grid,
            sourceFileSize,
            sourceWriteTime,
            isoValue,
            transitionLayers)) {
        error = "The local flip-point cache metadata is incomplete or unsupported";
        return false;
    }
    if (source != expectedMetadata.sourcePath ||
        grid != expectedMetadata.gridName ||
        sourceFileSize != expectedMetadata.sourceFileSize ||
        sourceWriteTime != expectedMetadata.sourceWriteTime ||
        !sameIsoValue(isoValue, expectedMetadata.settings.isoValue) ||
        transitionLayers != expectedMetadata.settings.transitionLayers) {
        error = "The local flip-point cache belongs to a different surface target";
        return false;
    }

    while (std::getline(input, line)) {
        if (line.empty()) continue;
        // Accept the historical "seed" record so existing sidecars remain
        // usable after the UI/API semantic change to explicit flip points.
        const bool activeFlipPoint = hasKind(line, "flip_point");
        const bool legacySeed = hasKind(line, "seed");
        if (!activeFlipPoint && !legacySeed) {
            error = "The local flip-point cache contains an unsupported record";
            seeds.clear();
            return false;
        }
        double seedSchema = 0.0;
        openvdb::Coord coordinate;
        std::uint64_t targetSign = 0;
        if (!readNumber(line, "schema", seedSchema) ||
            seedSchema != kSchemaVersion ||
            !readCoord(line, "coordinate", coordinate) ||
            !readUnsignedInteger(line, "target_sign", targetSign) ||
            targetSign > 255) {
            error = "The local flip-point cache contains an invalid record";
            seeds.clear();
            return false;
        }
        std::array<std::uint8_t, 4> color{255, 255, 255, 255};
        const bool hasColor = readColor(line, "color", color);
        std::uint64_t acceptedValue = 1;
        const bool hasAccepted = line.find("\"accepted\"") != std::string::npos;
        if (hasAccepted &&
            (!readUnsignedInteger(line, "accepted", acceptedValue) ||
             acceptedValue > 1)) {
            error = "The local flip-point cache contains an invalid accepted state";
            seeds.clear();
            return false;
        }
        seeds.push_back({
            coordinate,
            static_cast<std::uint8_t>(targetSign),
            activeFlipPoint,
            color,
            hasColor,
            acceptedValue != 0,
            hasAccepted});
    }
    return true;
}

} // namespace volume_surface::viewer
