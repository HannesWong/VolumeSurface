#include "volume_surface/viewer/BrushSettingsStore.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace volume_surface::viewer {

namespace {

bool readNumber(const std::string& json, const char* key, float& value)
{
    const std::string marker = std::string("\"") + key + "\"";
    const std::size_t markerPosition = json.find(marker);
    if (markerPosition == std::string::npos) {
        return false;
    }
    const std::size_t colonPosition = json.find(':', markerPosition + marker.size());
    if (colonPosition == std::string::npos) {
        return false;
    }
    const char* begin = json.c_str() + colonPosition + 1;
    char* end = nullptr;
    const float parsed = std::strtof(begin, &end);
    if (end == begin || !std::isfinite(parsed)) {
        return false;
    }
    value = parsed;
    return true;
}

bool readBoolean(const std::string& json, const char* key, bool& value)
{
    const std::string marker = std::string("\"") + key + "\"";
    const std::size_t markerPosition = json.find(marker);
    if (markerPosition == std::string::npos) {
        return false;
    }
    const std::size_t colonPosition = json.find(':', markerPosition + marker.size());
    if (colonPosition == std::string::npos) {
        return false;
    }
    const std::string remainder = json.substr(colonPosition + 1);
    const std::size_t truePosition = remainder.find("true");
    const std::size_t falsePosition = remainder.find("false");
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

void clampSettings(WeightPaintingSettings& settings)
{
    settings.coreRadiusMillimeters = std::clamp(
        settings.coreRadiusMillimeters, 0.0f, 50.0f);
    const float minimumFalloff = std::max(
        0.1f,
        settings.coreRadiusMillimeters + 0.1f);
    settings.falloffRadiusMillimeters = minimumFalloff >= 50.0f
        ? 50.0f
        : std::clamp(settings.falloffRadiusMillimeters, minimumFalloff, 50.0f);
    settings.strength = std::clamp(settings.strength, 0.0f, 100.0f);
    settings.planarityRadiusMillimeters = std::clamp(
        settings.planarityRadiusMillimeters, 0.1f, 20.0f);
    settings.planarityAngleDegrees = std::clamp(
        settings.planarityAngleDegrees, 1.0f, 90.0f);
    settings.planeOffsetPenalty = std::clamp(
        settings.planeOffsetPenalty, 0.0f, 100.0f);
    settings.normalChangePenalty = std::clamp(
        settings.normalChangePenalty, 0.0f, 20.0f);
    settings.normalChangeAngleDegrees = std::clamp(
        settings.normalChangeAngleDegrees, 1.0f, 90.0f);
}

} // namespace

bool BrushSettingsStore::save(
    const std::filesystem::path& path,
    const BrushSettingsDocument& document,
    std::string& error)
{
    error.clear();
    std::ofstream output(path, std::ios::trunc);
    if (!output) {
        error = "Failed to open " + path.string() + " for writing";
        return false;
    }
    const WeightPaintingSettings& settings = document.settings;
    output << std::boolalpha
           << "{\n"
           << "  \"brushPreviewEnabled\": " << document.brushPreviewEnabled << ",\n"
           << "  \"coreRadiusMillimeters\": " << settings.coreRadiusMillimeters << ",\n"
           << "  \"falloffRadiusMillimeters\": " << settings.falloffRadiusMillimeters << ",\n"
           << "  \"strength\": " << settings.strength << ",\n"
           << "  \"planarityRadiusMillimeters\": " << settings.planarityRadiusMillimeters << ",\n"
           << "  \"planarityAngleDegrees\": " << settings.planarityAngleDegrees << ",\n"
           << "  \"planeOffsetPenalty\": " << settings.planeOffsetPenalty << ",\n"
           << "  \"normalChangePenalty\": " << settings.normalChangePenalty << ",\n"
           << "  \"normalChangeAngleDegrees\": " << settings.normalChangeAngleDegrees << "\n"
           << "}\n";
    if (!output) {
        error = "Failed while writing " + path.string();
        return false;
    }
    return true;
}

bool BrushSettingsStore::load(
    const std::filesystem::path& path,
    BrushSettingsDocument& document,
    std::string& error)
{
    error.clear();
    std::ifstream input(path);
    if (!input) {
        return false;
    }
    std::stringstream buffer;
    buffer << input.rdbuf();
    const std::string json = buffer.str();
    if (json.empty()) {
        error = "Brush settings file is empty; using built-in values";
        return false;
    }

    readBoolean(json, "brushPreviewEnabled", document.brushPreviewEnabled);
    WeightPaintingSettings& settings = document.settings;
    readNumber(json, "coreRadiusMillimeters", settings.coreRadiusMillimeters);
    readNumber(json, "falloffRadiusMillimeters", settings.falloffRadiusMillimeters);
    readNumber(json, "strength", settings.strength);
    readNumber(json, "planarityRadiusMillimeters", settings.planarityRadiusMillimeters);
    readNumber(json, "planarityAngleDegrees", settings.planarityAngleDegrees);
    readNumber(json, "planeOffsetPenalty", settings.planeOffsetPenalty);
    readNumber(json, "normalChangePenalty", settings.normalChangePenalty);
    readNumber(json, "normalChangeAngleDegrees", settings.normalChangeAngleDegrees);
    clampSettings(settings);
    return true;
}

} // namespace volume_surface::viewer
