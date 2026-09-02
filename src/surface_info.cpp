#include "volume_surface/SurfaceMesh.h"

#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include <openvdb/io/File.h>

namespace {

openvdb::FloatGrid::Ptr loadFloatGrid(
    const std::filesystem::path& path,
    const std::string& gridName)
{
    openvdb::io::File file(path.string());
    file.open();
    auto baseGrid = file.readGrid(gridName);
    file.close();
    auto grid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);
    if (!grid) {
        throw std::runtime_error("Grid is missing or is not a FloatGrid: " + gridName);
    }
    return grid;
}

} // namespace

int main(int argc, char** argv)
{
    try {
        if (argc < 2 || argc > 5) {
            std::cerr << "Usage: vdb_surface_info <file.vdb> [grid=density] [iso=255] [adaptivity=0]\n";
            return EXIT_FAILURE;
        }

        openvdb::initialize();
        const std::filesystem::path path = argv[1];
        const std::string gridName = argc >= 3 ? argv[2] : "density";
        const double isoValue = argc >= 4 ? std::stod(argv[3]) : 255.0;
        const double adaptivity = argc >= 5 ? std::stod(argv[4]) : 0.0;

        const auto grid = loadFloatGrid(path, gridName);
        const auto mesh = volume_surface::extractIsoSurface(*grid, isoValue, adaptivity);

        std::cout << std::fixed << std::setprecision(6)
                  << "grid=" << gridName << '\n'
                  << "class=" << grid->getGridClass() << '\n'
                  << "iso=" << isoValue << '\n'
                  << "adaptivity=" << adaptivity << '\n'
                  << "vertices=" << mesh.vertices.size() << '\n'
                  << "triangles=" << mesh.triangleCount() << '\n'
                  << "bounds.min=[" << mesh.bounds.minimum[0] << ", "
                  << mesh.bounds.minimum[1] << ", " << mesh.bounds.minimum[2] << "]\n"
                  << "bounds.max=[" << mesh.bounds.maximum[0] << ", "
                  << mesh.bounds.maximum[1] << ", " << mesh.bounds.maximum[2] << "]\n";
        return mesh.empty() ? EXIT_FAILURE : EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
