#include "volume_surface/BrickArena.h"

#include <openvdb/io/File.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

namespace fs = std::filesystem;

struct Options
{
    fs::path inputFile;
    fs::path outputFile;
    std::string gridName = "density";
    float tolerance = 0.0f;
    bool validate = true;
};

void usage(const char* program, int status)
{
    std::cerr <<
        "Usage: " << program << " INPUT.vdb [options]\n"
        "Options:\n"
        "    --grid NAME       float grid to import (default: density)\n"
        "    --output FILE     write the arena back to a VDB file\n"
        "    --tolerance N     validation value tolerance (default: 0)\n"
        "    --no-validate     skip comparison with the source grid\n"
        "    -h, --help        print this message\n";
    std::exit(status);
}

float parseFloat(const std::string& text, const std::string& option)
{
    std::size_t parsed = 0;
    const float value = std::stof(text, &parsed);
    if (parsed != text.size()) throw std::runtime_error("invalid value for " + option + ": " + text);
    return value;
}

Options parseOptions(int argc, char* argv[])
{
    Options options;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        auto requireValue = [&]() -> std::string {
            if (++index >= argc) throw std::runtime_error("missing value for " + argument);
            return argv[index];
        };

        if (argument == "-h" || argument == "--help") {
            usage(argv[0], EXIT_SUCCESS);
        } else if (argument == "--grid") {
            options.gridName = requireValue();
        } else if (argument == "--output") {
            options.outputFile = requireValue();
        } else if (argument == "--tolerance") {
            options.tolerance = parseFloat(requireValue(), argument);
        } else if (argument == "--no-validate") {
            options.validate = false;
        } else if (!argument.empty() && argument[0] == '-') {
            throw std::runtime_error("unknown option: " + argument);
        } else if (options.inputFile.empty()) {
            options.inputFile = argument;
        } else {
            throw std::runtime_error("unexpected positional argument: " + argument);
        }
    }

    if (options.inputFile.empty()) throw std::runtime_error("an input VDB file is required");
    if (options.gridName.empty()) throw std::runtime_error("grid name must not be empty");
    if (options.tolerance < 0.0f) throw std::runtime_error("tolerance must not be negative");
    return options;
}

double mebibytes(std::size_t bytes)
{
    return static_cast<double>(bytes) / (1024.0 * 1024.0);
}

}

int main(int argc, char* argv[])
{
    try {
        const Options options = parseOptions(argc, argv);
        openvdb::initialize();

        openvdb::io::File input(options.inputFile.string());
        input.open();
        openvdb::GridBase::Ptr baseGrid = input.readGrid(options.gridName);
        input.close();
        if (!baseGrid || !baseGrid->isType<openvdb::FloatGrid>()) {
            throw std::runtime_error("selected grid is not a FloatGrid: " + options.gridName);
        }
        auto sourceGrid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);

        const auto importStart = std::chrono::steady_clock::now();
        volume_surface::BrickArena arena = volume_surface::BrickArena::fromGrid(*sourceGrid);
        const auto importEnd = std::chrono::steady_clock::now();
        const double importSeconds = std::chrono::duration<double>(importEnd - importStart).count();

        const openvdb::Vec3d voxelSize = arena.transform().voxelSize();
        std::cout << "grid: " << arena.gridName() << '\n'
            << std::fixed << std::setprecision(9)
            << "voxel size: [" << voxelSize.x() << ", " << voxelSize.y() << ", "
            << voxelSize.z() << "]\n"
            << std::setprecision(3)
            << "bricks: " << arena.brickCount() << '\n'
            << "active voxels: " << arena.activeVoxelCount() << '\n'
            << "hot data: " << mebibytes(arena.hotDataBytes()) << " MiB\n"
            << "64-byte aligned: " << (arena.buffersAreAligned() ? "yes" : "no") << '\n'
            << "import time: " << importSeconds << " s\n";

        if (options.validate) {
            const auto validationStart = std::chrono::steady_clock::now();
            const volume_surface::ValidationReport report =
                arena.validateAgainst(*sourceGrid, options.tolerance);
            const auto validationEnd = std::chrono::steady_clock::now();
            const double validationSeconds =
                std::chrono::duration<double>(validationEnd - validationStart).count();

            std::cout
                << "validation: " << (report.ok() ? "passed" : "failed") << '\n'
                << "missing bricks: " << report.missingBrickCount << '\n'
                << "extra bricks: " << report.extraBrickCount << '\n'
                << "active mismatches: " << report.activeStateMismatchCount << '\n'
                << "value mismatches: " << report.valueMismatchCount << '\n'
                << "maximum absolute error: " << report.maximumAbsoluteError << '\n'
                << "validation time: " << validationSeconds << " s\n";
            if (!report.ok()) return EXIT_FAILURE;
        }

        if (!options.outputFile.empty()) {
            openvdb::GridPtrVec outputGrids;
            outputGrids.push_back(arena.toGrid());
            openvdb::io::File output(options.outputFile.string());
            output.write(outputGrids);
            output.close();
            std::cout << "wrote: " << options.outputFile.string() << '\n';
        }

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
