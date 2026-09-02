#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <math/mat4.h>
#include <math/vec3.h>
#include <openvdb/openvdb.h>

namespace volume_surface::viewer {

enum class BrushParameterId : std::uint8_t {
    None,
    CoreRadius,
    FalloffRadius,
    Strength,
    PlanarityRadius,
    PlanarityAngle,
    PlaneOffsetPenalty,
    NormalChangePenalty,
    NormalChangeAngle,
};

struct BrushInteractionState {
    bool brushPickPending = false;
    std::uint64_t brushPickSerial = 0;
    bool brushControlDown = false;
    bool brushLeftButtonDown = false;
    bool brushPointerDirty = false;
    bool brushStrokeSampleRequested = false;
    bool brushStrokeFinalizeRequested = false;
    bool brushHasLastPathCenter = false;
    bool brushStrokeHasAppliedPreview = false;
    openvdb::Vec3d brushLastPathCenterWorld{};
    std::vector<openvdb::Vec3d> brushStrokePathWorld;
    std::size_t brushStrokeFittedCenterCount = 0;
    std::chrono::steady_clock::time_point brushStrokeNextFitTime{};
    int brushPointerX = 0;
    int brushPointerY = 0;
    bool brushCenterPending = false;
    openvdb::Vec3d pendingBrushCenterWorld{};
    filament::math::mat4 brushPickClipToScene;
    std::uint32_t brushPickViewportWidth = 0;
    std::uint32_t brushPickViewportHeight = 0;
    bool brushCursorVisible = false;
    filament::math::double3 brushCursorScenePosition{};

    bool brushParameterAdjustActive = false;
    BrushParameterId brushParameterAdjustField = BrushParameterId::None;
    float brushParameterAdjustInitialValue = 0.0f;
    float brushParameterAdjustLastX = 0.0f;
    float brushParameterAdjustAnchorX = 0.0f;
    float brushParameterAdjustAnchorY = 0.0f;
    int brushParameterAdjustPrecisionIndex = 1;
    bool brushParameterAdjustExitedPrecisionZone = false;
};

} // namespace volume_surface::viewer
