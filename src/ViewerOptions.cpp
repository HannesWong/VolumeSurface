#include "volume_surface/viewer/ViewerOptions.h"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace volume_surface::viewer {

ViewerOptions parseViewerOptions(int argc, char** argv)
{
    ViewerOptions options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        auto requireValue = [&](const char* option) -> std::string {
            if (++index >= argc) {
                throw std::invalid_argument(
                    std::string("Missing value for ") + option);
            }
            return argv[index];
        };

        if (argument == "--input") {
            options.input = requireValue("--input");
        } else if (argument == "--grid") {
            options.gridName = requireValue("--grid");
        } else if (argument == "--iso") {
            options.isoValue = std::stod(requireValue("--iso"));
        } else if (argument == "--adaptivity") {
            options.adaptivity = std::stod(requireValue("--adaptivity"));
        } else if (argument == "--inspect-only") {
            options.inspectOnly = true;
        } else if (argument == "--headless-smoke") {
            options.headlessSmoke = true;
        } else if (argument == "--replay-brush-profile") {
            options.replayBrushProfile = requireValue("--replay-brush-profile");
        } else if (argument == "--help") {
            std::cout
                << "volume_surface_viewer [--input file.vdb] [--grid density] "
                   "[--iso 255] [--adaptivity 0.1] [--inspect-only] [--headless-smoke] "
                   "[--replay-brush-profile brush_profile.jsonl]\n";
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::invalid_argument("Unknown argument: " + argument);
        }
    }
    return options;
}

} // namespace volume_surface::viewer
