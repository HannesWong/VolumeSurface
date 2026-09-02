#include "volume_surface/SurfaceMesh.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/LevelSetUtil.h>

namespace {

bool hasOutwardNormals(const volume_surface::SurfaceMesh& mesh)
{
    double radialAlignment = 0.0;
    for (const auto& vertex : mesh.vertices) {
        radialAlignment +=
            static_cast<double>(vertex.position[0]) * vertex.normal[0] +
            static_cast<double>(vertex.position[1]) * vertex.normal[1] +
            static_cast<double>(vertex.position[2]) * vertex.normal[2];
    }
    return radialAlignment > 0.0;
}

} // namespace

int main()
{
    openvdb::initialize();
    const auto grid = openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(
        10.0f,
        openvdb::Vec3f(0.0f),
        0.5f,
        3.0f);
    const auto mesh = volume_surface::extractIsoSurface(*grid, 0.0, 0.0);

    if (mesh.vertices.empty() || mesh.indices.empty() || mesh.indices.size() % 3 != 0) {
        std::cerr << "Expected a non-empty triangle mesh\n";
        return EXIT_FAILURE;
    }

    for (const auto& vertex : mesh.vertices) {
        const double length = std::sqrt(
            static_cast<double>(vertex.normal[0]) * vertex.normal[0] +
            static_cast<double>(vertex.normal[1]) * vertex.normal[1] +
            static_cast<double>(vertex.normal[2]) * vertex.normal[2]);
        if (!std::isfinite(length) || std::abs(length - 1.0) > 1.0e-4) {
            std::cerr << "Expected finite unit normals\n";
            return EXIT_FAILURE;
        }
    }

    for (const auto index : mesh.indices) {
        if (index >= mesh.vertices.size()) {
            std::cerr << "Expected every index to address a vertex\n";
            return EXIT_FAILURE;
        }
    }

    if (!hasOutwardNormals(mesh)) {
        std::cerr << "Expected level set normals to point outward\n";
        return EXIT_FAILURE;
    }

    const auto fogGrid = grid->deepCopy();
    openvdb::tools::sdfToFogVolume(*fogGrid);
    const auto fogMesh = volume_surface::extractIsoSurface(*fogGrid, 0.5, 0.0);
    if (fogMesh.vertices.empty() || !hasOutwardNormals(fogMesh)) {
        std::cerr << "Expected fog volume normals to point outward\n";
        return EXIT_FAILURE;
    }

    std::cout << "vertices=" << mesh.vertices.size()
              << " triangles=" << mesh.triangleCount() << '\n';
    return EXIT_SUCCESS;
}
