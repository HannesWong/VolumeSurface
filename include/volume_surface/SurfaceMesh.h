#pragma once

#include <array>
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

SurfaceMesh extractIsoSurface(
    const openvdb::FloatGrid& grid,
    double isoValue,
    double adaptivity = 0.0);

} // namespace volume_surface
