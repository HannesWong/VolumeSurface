#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace volume_surface::viewer {

struct ViewerOptions {
    std::filesystem::path input;
    std::string gridName = "density";
    double isoValue = 255.0;
    double adaptivity = 0.1;
    bool inspectOnly = false;
    bool headlessSmoke = false;
    std::filesystem::path replayBrushProfile;
};

ViewerOptions parseViewerOptions(int argc, char** argv);

struct ViewerInputEntry {
    std::string label;
    std::filesystem::path path;
    bool defaultSelected = false;
};

std::filesystem::path viewerInputCatalogPath();

std::vector<ViewerInputEntry> loadViewerInputCatalog(
    const std::filesystem::path& catalogPath,
    std::string& status);

std::filesystem::path selectVdbInputFromCatalog(
    const std::filesystem::path& catalogPath = {});

} // namespace volume_surface::viewer
