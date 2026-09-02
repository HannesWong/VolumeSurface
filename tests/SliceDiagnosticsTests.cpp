#include "volume_surface/SliceDiagnostics.h"

#include <array>
#include <cstdlib>
#include <iostream>

int main()
{
    openvdb::initialize();
    auto grid = openvdb::FloatGrid::create(0.0f);
    auto transform = openvdb::math::Transform::createLinearTransform(1.0);
    transform->preScale(openvdb::Vec3d(0.25, 0.5, 1.0));
    grid->setTransform(transform);
    auto accessor = grid->getAccessor();
    for (int y = 0; y < 4; ++y) {
        for (int x = 0; x < 5; ++x) {
            accessor.setValue(openvdb::Coord(x, y, 2), static_cast<float>(x + y));
        }
    }

    const openvdb::CoordBBox bounds(openvdb::Coord(0, 0, 1), openvdb::Coord(4, 3, 3));
    const std::array<float, 2> levels{2.0f, 4.0f};
    const auto slice = volume_surface::extractDensitySlice(
        *grid,
        bounds,
        volume_surface::SliceAxis::Z,
        2,
        8.0f,
        levels);

    if (slice.width != 5 || slice.height != 4) {
        std::cerr << "Unexpected XY slice dimensions\n";
        return EXIT_FAILURE;
    }
    if (slice.horizontalSpacing != 0.25 || slice.verticalSpacing != 0.5) {
        std::cerr << "Unexpected physical slice spacing\n";
        return EXIT_FAILURE;
    }
    if (slice.values[3 * slice.width + 4] != 7.0f || slice.contours.empty()) {
        std::cerr << "Unexpected slice samples or contour output\n";
        return EXIT_FAILURE;
    }
    if (slice.rgba.size() != slice.width * slice.height * 4) {
        std::cerr << "Unexpected RGBA buffer size\n";
        return EXIT_FAILURE;
    }

    std::cout << "slice=" << slice.width << 'x' << slice.height
              << " contours=" << slice.contours.size() << '\n';
    return EXIT_SUCCESS;
}
