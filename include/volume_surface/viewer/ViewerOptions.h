#pragma once

#include <filesystem>
#include <string>

namespace volume_surface::viewer {

struct ViewerOptions {
    std::filesystem::path input = R"(G:\transformed\rightArm\rightArm.0224.vdb)";
    std::string gridName = "density";
    double isoValue = 255.0;
    double adaptivity = 0.1;
    bool inspectOnly = false;
    bool headlessSmoke = false;
    std::filesystem::path replayBrushProfile;
};

ViewerOptions parseViewerOptions(int argc, char** argv);

} // namespace volume_surface::viewer
