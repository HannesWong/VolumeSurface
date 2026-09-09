#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <openvdb/openvdb.h>

namespace volume_surface {

struct SurfaceVertex {
    std::array<float, 3> position{};
    std::array<float, 3> normal{};
};

struct SurfaceBounds {
    std::array<float, 3> minimum{};
    std::array<float, 3> maximum{};
};

struct SurfaceMesh {
    std::vector<SurfaceVertex> vertices;
    std::vector<std::uint32_t> indices;
    SurfaceBounds bounds;

    [[nodiscard]] bool empty() const noexcept { return indices.empty(); }
    [[nodiscard]] std::size_t triangleCount() const noexcept { return indices.size() / 3; }
};

struct SurfaceMeshComponentSplit {
    SurfaceMesh primary;
    SurfaceMesh excluded;
    std::size_t componentCount = 0;
    std::size_t primaryTriangleCount = 0;
    std::size_t excludedTriangleCount = 0;
};

SurfaceMesh extractIsoSurface(
    const openvdb::FloatGrid& grid,
    double isoValue,
    double adaptivity = 0.0);

SurfaceMeshComponentSplit splitSurfaceMeshComponents(const SurfaceMesh& mesh);

// Builds a sparse narrow-band mask around a surface mesh using the supplied VDB transform.
openvdb::BoolGrid::Ptr buildSurfaceMeshMask(
    const SurfaceMesh& mesh,
    const openvdb::math::Transform& transform,
    float halfWidthVoxels = 3.0f);

} // namespace volume_surface
