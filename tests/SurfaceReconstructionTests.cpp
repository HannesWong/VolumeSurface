#include "volume_surface/SurfaceReconstruction.h"

#include <openvdb/openvdb.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

openvdb::FloatGrid::Ptr createLayeredFog()
{
    auto grid = openvdb::FloatGrid::create(0.0f);
    grid->setGridClass(openvdb::GRID_FOG_VOLUME);
    grid->setTransform(openvdb::math::Transform::createLinearTransform(
        openvdb::math::scale<openvdb::math::Mat4d>(
            openvdb::Vec3d(0.31, 0.31, 1.0))));

    auto accessor = grid->getAccessor();
    for (int z = -4; z <= 4; ++z) {
        for (int y = -4; y <= 4; ++y) {
            for (int x = -3; x <= 0; ++x) {
                accessor.setValueOn(openvdb::Coord(x, y, z), 255.0f);
            }
            accessor.setValueOn(openvdb::Coord(1, y, z), 128.0f);
            accessor.setValueOn(openvdb::Coord(2, y, z), 64.0f);
        }
    }
    return grid;
}

} // namespace

int main()
{
    try {
        openvdb::initialize();
        const auto grid = createLayeredFog();
        volume_surface::SurfaceTargetSettings targetSettings;
        targetSettings.isoValue = 255.0;
        targetSettings.transitionLayers = 2;
        targetSettings.planarityRadius = 1.0;
        const auto target = volume_surface::extractSurfaceTarget(*grid, targetSettings);

        volume_surface::SurfaceReconstructionSettings settings;
        settings.isoValue = 255.0;
        settings.mlsRadius = 0.005;
        settings.targetCellSize = 0.001;
        settings.candidatePaddingCells = 2;
        settings.maximumCellCount = 200'000;
        const auto result = volume_surface::reconstructSurfaceMLS(*grid, target, settings);
        require(!result.mesh.empty(), "MLS reconstruction returned an empty mesh");
        require(result.mesh.indices.size() % 3 == 0,
            "MLS reconstruction returned invalid triangle indices");
        require(result.crossingCellCount > 0 && result.fieldSampleCount > 0,
            "MLS reconstruction did not sample a crossing field");
        for (const auto& vertex : result.mesh.vertices) {
            const double length = std::sqrt(
                static_cast<double>(vertex.normal[0]) * vertex.normal[0] +
                static_cast<double>(vertex.normal[1]) * vertex.normal[1] +
                static_cast<double>(vertex.normal[2]) * vertex.normal[2]);
            require(std::isfinite(length) && std::abs(length - 1.0) < 1.0e-3,
                "MLS reconstruction returned an invalid normal");
        }
        std::cout << "surface_reconstruction_tests passed vertices="
                  << result.mesh.vertices.size()
                  << " triangles=" << result.mesh.triangleCount() << '\n';
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "surface_reconstruction_tests failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
