#pragma once

#include <cstdint>

#include <openvdb/openvdb.h>

#include "volume_surface/viewer/CameraPickController.h"

namespace filament {
class Camera;
}

namespace volume_surface::viewer {

struct SurfaceAwareCameraPose
{
    openvdb::Vec3d eye{};
    openvdb::Vec3d target{};
    openvdb::Vec3d up{0.0, 1.0, 0.0};
};

struct SurfaceAwareCameraWheelResult
{
    bool handled = false;
    bool usedAdaptiveStep = false;
    CameraPickHit hit;
    float projectedDistanceMillimeters = 0.0f;
    float appliedStepMillimeters = 0.0f;
};

class SurfaceAwareCameraController final
{
public:
    enum class DragMode : std::uint8_t
    {
        None,
        Rotate,
        Pan
    };

    SurfaceAwareCameraController() = default;

    void reset(
        const openvdb::Vec3d& eye,
        const openvdb::Vec3d& target,
        const openvdb::Vec3d& up) noexcept;

    void cancelDrag() noexcept;

    [[nodiscard]] bool hasPose() const noexcept { return mHasPose; }

    [[nodiscard]] bool dragging() const noexcept
    {
        return mDragMode != DragMode::None;
    }

    [[nodiscard]] DragMode dragMode() const noexcept { return mDragMode; }

    bool beginDrag(
        int button,
        int pointerX,
        int pointerY,
        const CameraPickHit& hit) noexcept;

    bool updateDrag(
        int pointerX,
        int pointerY,
        const CameraPickRay& ray) noexcept;

    void endDrag() noexcept;

    [[nodiscard]] SurfaceAwareCameraWheelResult applyWheel(
        std::int32_t rawDelta,
        const CameraPickHit& hit,
        float displayScale,
        float wheelZoomMultiplier,
        float wheelHitDistanceRatio,
        float wheelMinimumDistanceMillimeters,
        float wheelSurfaceClearanceMillimeters) noexcept;

    void applyTo(filament::Camera& camera) const noexcept;

private:
    static openvdb::Vec3d normalizedOr(
        const openvdb::Vec3d& value,
        const openvdb::Vec3d& fallback) noexcept;

    static openvdb::Vec3d rotateAroundAxis(
        const openvdb::Vec3d& value,
        const openvdb::Vec3d& axis,
        double angle) noexcept;

    [[nodiscard]] bool intersectPanPlane(
        const CameraPickRay& ray,
        const openvdb::Vec3d& rayOrigin,
        openvdb::Vec3d& point) const noexcept;

    SurfaceAwareCameraPose mPose;
    bool mHasPose = false;

    DragMode mDragMode = DragMode::None;
    int mDragStartX = 0;
    int mDragStartY = 0;
    SurfaceAwareCameraPose mDragStartPose;
    openvdb::Vec3d mDragPivot{};
    openvdb::Vec3d mDragUp{};
    openvdb::Vec3d mDragRight{};
    openvdb::Vec3d mPanPlaneNormal{};
    openvdb::Vec3d mPanAnchor{};
};

} // namespace volume_surface::viewer
