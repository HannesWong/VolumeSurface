#pragma once

#include <openvdb/openvdb.h>

#include <math/vec3.h>

namespace volume_surface::viewer {

class SceneCoordinateMapper final {
public:
    SceneCoordinateMapper() = default;

    SceneCoordinateMapper(
        const openvdb::Vec3d& referenceCenter,
        float displayScale) noexcept
        : mReferenceCenter(referenceCenter),
          mDisplayScale(displayScale)
    {
    }

    void reset(
        const openvdb::Vec3d& referenceCenter,
        float displayScale) noexcept
    {
        mReferenceCenter = referenceCenter;
        mDisplayScale = displayScale;
    }

    [[nodiscard]] filament::math::float3 toScene(
        const openvdb::Vec3d& worldPosition) const noexcept
    {
        return (filament::math::float3{
                    static_cast<float>(worldPosition.x()),
                    static_cast<float>(worldPosition.y()),
                    static_cast<float>(worldPosition.z())} -
                filament::math::float3{
                    static_cast<float>(mReferenceCenter.x()),
                    static_cast<float>(mReferenceCenter.y()),
                    static_cast<float>(mReferenceCenter.z())}) *
                mDisplayScale +
            filament::math::float3{0.0f, 0.0f, -4.0f};
    }

    [[nodiscard]] filament::math::float3 toScene(
        const filament::math::float3& worldPosition) const noexcept
    {
        return (worldPosition -
                filament::math::float3{
                    static_cast<float>(mReferenceCenter.x()),
                    static_cast<float>(mReferenceCenter.y()),
                    static_cast<float>(mReferenceCenter.z())}) *
                mDisplayScale +
            filament::math::float3{0.0f, 0.0f, -4.0f};
    }

    [[nodiscard]] float displayScale() const noexcept
    {
        return mDisplayScale;
    }

private:
    openvdb::Vec3d mReferenceCenter{};
    float mDisplayScale = 1.0f;
};

} // namespace volume_surface::viewer
