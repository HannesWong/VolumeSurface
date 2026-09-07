#include "volume_surface/viewer/SurfaceAwareCameraController.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <filament/Camera.h>
#include <math/vec3.h>

namespace volume_surface::viewer {
namespace {

constexpr double kEpsilon = 1.0e-12;
constexpr double kRotationSensitivity = 0.005;
constexpr double kMaximumPitch = 1.50;
constexpr double kMillimetersToMeters = 0.001;
constexpr double kManipulatorZoomSpeed = 0.01;

bool finiteVector(const openvdb::Vec3d& value) noexcept
{
    return std::isfinite(value.x()) &&
        std::isfinite(value.y()) &&
        std::isfinite(value.z());
}

} // namespace

void SurfaceAwareCameraController::reset(
    const openvdb::Vec3d& eye,
    const openvdb::Vec3d& target,
    const openvdb::Vec3d& up) noexcept
{
    mPose.eye = finiteVector(eye) ? eye : openvdb::Vec3d{0.0, 0.0, 1.0};
    mPose.target = finiteVector(target) ? target : openvdb::Vec3d{0.0, 0.0, 0.0};
    mPose.up = normalizedOr(up, openvdb::Vec3d{0.0, 1.0, 0.0});
    mHasPose = true;
    cancelDrag();
}

void SurfaceAwareCameraController::cancelDrag() noexcept
{
    mDragMode = DragMode::None;
}

bool SurfaceAwareCameraController::beginDrag(
    int button,
    int pointerX,
    int pointerY,
    const CameraPickHit& hit) noexcept
{
    if (!mHasPose || (button != 1 && button != 3)) {
        return false;
    }

    mDragMode = button == 1 ? DragMode::Rotate : DragMode::Pan;
    mDragStartX = pointerX;
    mDragStartY = pointerY;
    mDragStartPose = mPose;

    const openvdb::Vec3d forward = normalizedOr(
        mDragStartPose.target - mDragStartPose.eye,
        openvdb::Vec3d{0.0, 0.0, -1.0});
    mDragUp = normalizedOr(mDragStartPose.up, openvdb::Vec3d{0.0, 1.0, 0.0});
    mDragRight = normalizedOr(
        forward.cross(mDragUp),
        openvdb::Vec3d{1.0, 0.0, 0.0});

    mDragPivot = hit.hit ? hit.scenePosition : mDragStartPose.target;
    if (!finiteVector(mDragPivot)) {
        mDragPivot = mDragStartPose.target;
    }

    if (mDragMode == DragMode::Pan) {
        mPanPlaneNormal = forward;
        mPanAnchor = hit.hit ? hit.scenePosition : mDragStartPose.target;
        if (!finiteVector(mPanAnchor)) {
            mPanAnchor = mDragStartPose.target;
        }
    }
    return true;
}

bool SurfaceAwareCameraController::updateDrag(
    int pointerX,
    int pointerY,
    const CameraPickRay& ray) noexcept
{
    if (!mHasPose || mDragMode == DragMode::None) {
        return false;
    }

    if (mDragMode == DragMode::Pan) {
        openvdb::Vec3d currentPoint;
        if (!intersectPanPlane(ray, mDragStartPose.eye, currentPoint)) {
            return false;
        }
        const openvdb::Vec3d translation = mPanAnchor - currentPoint;
        if (!finiteVector(translation)) {
            return false;
        }
        mPose.eye = mDragStartPose.eye + translation;
        mPose.target = mDragStartPose.target + translation;
        mPose.up = mDragStartPose.up;
        return true;
    }

    const double horizontalDelta =
        static_cast<double>(pointerX - mDragStartX);
    const double verticalDelta =
        static_cast<double>(pointerY - mDragStartY);
    const double yaw = -horizontalDelta * kRotationSensitivity;
    const double pitch = std::clamp(
        -verticalDelta * kRotationSensitivity,
        -kMaximumPitch,
        kMaximumPitch);

    const openvdb::Vec3d yawedRight = rotateAroundAxis(
        mDragRight,
        mDragUp,
        yaw);
    const auto rotateDragVector = [this, yaw, pitch, &yawedRight](
        const openvdb::Vec3d& value) {
        return rotateAroundAxis(
            rotateAroundAxis(value, mDragUp, yaw),
            yawedRight,
            pitch);
    };

    const openvdb::Vec3d eyeOffset = rotateDragVector(
        mDragStartPose.eye - mDragPivot);
    const openvdb::Vec3d targetOffset = rotateDragVector(
        mDragStartPose.target - mDragPivot);
    mPose.eye = mDragPivot + eyeOffset;
    mPose.target = mDragPivot + targetOffset;
    mPose.up = normalizedOr(
        rotateDragVector(mDragStartPose.up),
        openvdb::Vec3d{0.0, 1.0, 0.0});
    return finiteVector(mPose.eye) &&
        finiteVector(mPose.target) &&
        finiteVector(mPose.up);
}

void SurfaceAwareCameraController::endDrag() noexcept
{
    cancelDrag();
}

SurfaceAwareCameraWheelResult SurfaceAwareCameraController::applyWheel(
    std::int32_t rawDelta,
    const CameraPickHit& hit,
    float displayScale,
    float wheelZoomMultiplier,
    float wheelHitDistanceRatio,
    float wheelMinimumDistanceMillimeters,
    float wheelSurfaceClearanceMillimeters) noexcept
{
    SurfaceAwareCameraWheelResult result;
    result.hit = hit;
    if (!mHasPose || rawDelta == 0) {
        return result;
    }

    const openvdb::Vec3d forward = normalizedOr(
        mPose.target - mPose.eye,
        openvdb::Vec3d{0.0, 0.0, -1.0});
    const double scale = std::max(
        static_cast<double>(displayScale),
        1.0e-9);
    const double minimumStep = std::max(
        0.0,
        static_cast<double>(wheelMinimumDistanceMillimeters)) *
        kMillimetersToMeters * scale;
    const double clearance = std::max(
        0.0,
        static_cast<double>(wheelSurfaceClearanceMillimeters)) *
        kMillimetersToMeters * scale;

    const double projectedDistance = hit.hit
        ? (hit.scenePosition - mPose.eye).dot(forward)
        : std::numeric_limits<double>::quiet_NaN();
    const bool useAdaptiveStep = std::isfinite(projectedDistance) &&
        projectedDistance > 0.0;

    double sceneStep = 0.0;
    if (useAdaptiveStep) {
        const double hitRatio = std::clamp(
            static_cast<double>(wheelHitDistanceRatio),
            0.001,
            1.0);
        const double speedScale = std::clamp(
            static_cast<double>(wheelZoomMultiplier) / 12.0,
            0.1,
            4.0);
        const double wheelMagnitude = std::abs(static_cast<double>(rawDelta));
        sceneStep = std::max(
            minimumStep,
            projectedDistance * hitRatio * speedScale) * wheelMagnitude;
        if (rawDelta > 0) {
            const double maximumApproach = projectedDistance - clearance;
            if (!std::isfinite(maximumApproach) || maximumApproach <= 0.0) {
                result.handled = true;
                result.usedAdaptiveStep = true;
                result.projectedDistanceMillimeters = static_cast<float>(
                    projectedDistance / (kMillimetersToMeters * scale));
                return result;
            }
            sceneStep = std::min(sceneStep, maximumApproach);
        }
        result.usedAdaptiveStep = true;
        result.projectedDistanceMillimeters = static_cast<float>(
            projectedDistance / (kMillimetersToMeters * scale));
    } else {
        sceneStep = std::abs(static_cast<double>(rawDelta)) *
            std::max(0.0, static_cast<double>(wheelZoomMultiplier)) *
            kManipulatorZoomSpeed;
    }

    if (!std::isfinite(sceneStep) || sceneStep <= 0.0) {
        result.handled = true;
        return result;
    }

    const double direction = rawDelta > 0 ? 1.0 : -1.0;
    const openvdb::Vec3d movement = forward * (direction * sceneStep);
    mPose.eye += movement;
    mPose.target += movement;
    if (!finiteVector(mPose.eye) || !finiteVector(mPose.target)) {
        return SurfaceAwareCameraWheelResult{};
    }

    result.handled = true;
    result.appliedStepMillimeters = static_cast<float>(
        sceneStep / (kMillimetersToMeters * scale));
    return result;
}

void SurfaceAwareCameraController::applyTo(filament::Camera& camera) const noexcept
{
    if (!mHasPose) {
        return;
    }
    camera.lookAt(
        filament::math::double3{mPose.eye.x(), mPose.eye.y(), mPose.eye.z()},
        filament::math::double3{
            mPose.target.x(), mPose.target.y(), mPose.target.z()},
        filament::math::double3{mPose.up.x(), mPose.up.y(), mPose.up.z()});
}

openvdb::Vec3d SurfaceAwareCameraController::normalizedOr(
    const openvdb::Vec3d& value,
    const openvdb::Vec3d& fallback) noexcept
{
    const double lengthSquared = value.dot(value);
    if (finiteVector(value) && std::isfinite(lengthSquared) &&
        lengthSquared > kEpsilon) {
        return value / std::sqrt(lengthSquared);
    }
    const double fallbackLengthSquared = fallback.dot(fallback);
    if (finiteVector(fallback) && std::isfinite(fallbackLengthSquared) &&
        fallbackLengthSquared > kEpsilon) {
        return fallback / std::sqrt(fallbackLengthSquared);
    }
    return openvdb::Vec3d{0.0, 0.0, -1.0};
}

openvdb::Vec3d SurfaceAwareCameraController::rotateAroundAxis(
    const openvdb::Vec3d& value,
    const openvdb::Vec3d& axis,
    double angle) noexcept
{
    const openvdb::Vec3d unitAxis = normalizedOr(
        axis,
        openvdb::Vec3d{0.0, 1.0, 0.0});
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return value * cosine +
        unitAxis.cross(value) * sine +
        unitAxis * (unitAxis.dot(value) * (1.0 - cosine));
}

bool SurfaceAwareCameraController::intersectPanPlane(
    const CameraPickRay& ray,
    const openvdb::Vec3d& rayOrigin,
    openvdb::Vec3d& point) const noexcept
{
    const double denominator = mPanPlaneNormal.dot(ray.direction);
    if (!std::isfinite(denominator) || std::abs(denominator) <= kEpsilon) {
        return false;
    }
    const double distance = (mPanAnchor - rayOrigin).dot(mPanPlaneNormal) /
        denominator;
    if (!std::isfinite(distance) || distance <= 0.0) {
        return false;
    }
    point = rayOrigin + ray.direction * distance;
    return finiteVector(point);
}

} // namespace volume_surface::viewer
