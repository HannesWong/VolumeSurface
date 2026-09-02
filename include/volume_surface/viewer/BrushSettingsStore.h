#pragma once

#include <filesystem>
#include <string>

#include "volume_surface/viewer/WeightPaintingStage.h"

namespace volume_surface::viewer {

struct BrushSettingsDocument {
    bool brushPreviewEnabled = true;
    WeightPaintingSettings settings;
};

class BrushSettingsStore final {
public:
    static bool save(
        const std::filesystem::path& path,
        const BrushSettingsDocument& document,
        std::string& error);

    static bool load(
        const std::filesystem::path& path,
        BrushSettingsDocument& document,
        std::string& error);
};

} // namespace volume_surface::viewer
