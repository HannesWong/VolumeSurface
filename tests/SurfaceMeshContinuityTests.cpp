#include "volume_surface/SurfaceMeshContinuity.h"

#include <cassert>
#include <cmath>

int main()
{
    openvdb::initialize();
    auto grid = openvdb::FloatGrid::create(0.0f);

    volume_surface::SurfaceMesh mesh;
    mesh.vertices = {
        {{{0.0f, 0.0f, 0.0f}}, {{0.0f, 0.0f, 1.0f}}},
        {{{1.0f, 0.0f, 0.0f}}, {{0.0f, 0.0f, 1.0f}}},
        {{{1.0f, 1.0f, 0.0f}}, {{0.0f, 0.0f, 1.0f}}},
        {{{0.0f, 1.0f, 0.0f}}, {{0.0f, 0.0f, 1.0f}}},
        {{{4.0f, 0.0f, 0.0f}}, {{0.0f, 0.0f, 1.0f}}}};
    mesh.indices = {0, 1, 2, 0, 2, 3};

    volume_surface::SurfaceTargetCache target;
    target.samples.resize(3);
    target.samples[0].coordinate = openvdb::Coord(0, 0, 0);
    target.samples[0].worldPosition = openvdb::Vec3f(0.0f, 0.0f, 0.0f);
    target.samples[1].coordinate = openvdb::Coord(1, 0, 0);
    target.samples[1].worldPosition = openvdb::Vec3f(1.0f, 0.0f, 0.0f);
    target.samples[2].coordinate = openvdb::Coord(4, 0, 0);
    target.samples[2].worldPosition = openvdb::Vec3f(4.0f, 0.0f, 0.0f);

    const auto field = volume_surface::buildSurfaceMeshContinuity(
        mesh,
        target,
        *grid);
    assert(field.matchesSampleCount(target.samples.size()));
    assert(field.supportedSampleCount == 3);
    assert(volume_surface::surfaceMeshContinuityWeight(field, 0, 1) > 0.9);
    assert(volume_surface::surfaceMeshContinuityWeight(field, 0, 2) == 0.0);
    assert(std::isfinite(volume_surface::surfaceMeshContinuityWeight(field, 0, 1)));
    return 0;
}
