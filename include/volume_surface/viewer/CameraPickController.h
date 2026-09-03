#pragma once

#include <array>
#include <cstddef>
#include <memory>

#include <openvdb/openvdb.h>

#include "volume_surface/SurfaceMesh.h"
#include "volume_surface/viewer/SceneCoordinateMapper.h"

namespace volume_surface::viewer {

struct CameraPickRay
{
    openvdb::Vec3d origin{};
    openvdb::Vec3d direction{};
};

struct CameraPickHit
{
    bool hit = false;
    std::size_t slotIndex = 0;
    std::size_t triangleIndex = 0;
    std::array<double, 3> barycentric{};
    double rayDistance = 0.0;
    openvdb::Vec3d scenePosition{};
};

class CameraPickController final
{
public:
    CameraPickController();
    ~CameraPickController();

    CameraPickController(const CameraPickController&) = delete;
    CameraPickController& operator=(const CameraPickController&) = delete;
    CameraPickController(CameraPickController&&) noexcept;
    CameraPickController& operator=(CameraPickController&&) noexcept;

    void rebuildSlot(
        std::size_t slotIndex,
        const SurfaceMesh& mesh,
        const SceneCoordinateMapper& mapper);

    void clearSlot(std::size_t slotIndex) noexcept;

    [[nodiscard]] CameraPickHit pick(
        const CameraPickRay& ray,
        const std::array<bool, 4>& visibleSlots) const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> mImpl;
};

} // namespace volume_surface::viewer
