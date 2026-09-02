#include "volume_surface/BrickArena.h"

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

std::size_t findBrick(
    const volume_surface::BrickArena& arena,
    int x,
    int y,
    int z)
{
    for (std::size_t index = 0; index < arena.brickCount(); ++index) {
        const auto& origin = arena.metadata(index).origin;
        if (origin[0] == x && origin[1] == y && origin[2] == z) return index;
    }
    throw std::runtime_error("test brick was not found");
}

openvdb::FloatGrid::Ptr createGrid()
{
    auto grid = openvdb::FloatGrid::create(0.0f);
    grid->setName("density");
    grid->setGridClass(openvdb::GRID_FOG_VOLUME);
    grid->setTransform(openvdb::math::Transform::createLinearTransform(
        openvdb::math::scale<openvdb::math::Mat4d>(openvdb::Vec3d(0.31, 0.31, 1.0))));

    auto accessor = grid->getAccessor();
    accessor.setValueOn(openvdb::Coord(-8, 0, 0), 1.25f);
    accessor.setValueOn(openvdb::Coord(-1, 7, 7), 2.5f);
    accessor.setValueOn(openvdb::Coord(0, 0, 0), 3.75f);
    accessor.setValueOn(openvdb::Coord(7, 7, 7), 5.0f);
    accessor.setValueOn(openvdb::Coord(0, 8, 8), 6.25f);
    return grid;
}

void runTests()
{
    auto source = createGrid();
    volume_surface::BrickArena arena = volume_surface::BrickArena::fromGrid(*source);

    require(arena.brickCount() == 3, "unexpected brick count");
    require(arena.activeVoxelCount() == 5, "unexpected active voxel count");
    require(arena.buffersAreAligned(), "arena buffers are not aligned");
    require(arena.gridName() == "density", "grid name was not retained");
    require(arena.gridClass() == openvdb::GRID_FOG_VOLUME, "grid class was not retained");

    const std::size_t negativeX = findBrick(arena, -8, 0, 0);
    const std::size_t origin = findBrick(arena, 0, 0, 0);
    const std::size_t diagonal = findBrick(arena, 0, 8, 8);
    require(
        arena.neighborIndex(negativeX, 1, 0, 0) == origin,
        "positive X neighbor lookup failed");
    require(
        arena.neighborIndex(origin, -1, 0, 0) == negativeX,
        "negative X neighbor lookup failed");
    require(
        arena.neighborIndex(origin, 0, 1, 1) == diagonal,
        "diagonal neighbor lookup failed");
    require(
        arena.neighborIndex(origin, 1, 0, 0) == volume_surface::kInvalidBrick,
        "missing neighbor was not marked invalid");

    const volume_surface::ValidationReport sourceReport = arena.validateAgainst(*source);
    require(sourceReport.ok(), "source validation failed");

    openvdb::FloatGrid::Ptr roundTrip = arena.toGrid();
    const volume_surface::ValidationReport roundTripReport = arena.validateAgainst(*roundTrip);
    require(roundTripReport.ok(), "round-trip validation failed");
    require(roundTrip->transform() == source->transform(), "transform was not retained");

    float* nextValues = arena.nextBrick(origin);
    const std::size_t zeroOffset = openvdb::FloatGrid::TreeType::LeafNodeType::coordToOffset(
        openvdb::Coord(0, 0, 0));
    nextValues[zeroOffset] = 9.5f;
    arena.swapValueBuffers();
    require(std::abs(arena.currentBrick(origin)[zeroOffset] - 9.5f) < 1.0e-6f, "buffer swap failed");
}

}

int main()
{
    try {
        openvdb::initialize();
        runTests();
        std::cout << "brick_arena_tests passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "brick_arena_tests failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
