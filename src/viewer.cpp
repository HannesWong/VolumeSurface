#include "volume_surface/SliceDiagnostics.h"
#include "volume_surface/SurfaceBrush.h"
#include "volume_surface/SurfaceMesh.h"
#include "volume_surface/SurfaceReconstruction.h"
#include "volume_surface/SurfaceTarget.h"
#include "volume_surface/SurfaceTargetPreview.h"
#include "volume_surface/viewer/BrushProfileRecorder.h"
#include "volume_surface/viewer/MeshRenderer.h"
#include "volume_surface/viewer/PresentationController.h"
#include "volume_surface/viewer/SliceRenderer.h"
#include "volume_surface/viewer/SurfaceTargetCacheRepository.h"
#include "volume_surface/viewer/ViewerContext.h"
#include "volume_surface/viewer/ViewerState.h"
#include "volume_surface/viewer/WorkflowController.h"
#include "volume_surface/viewer/WeightFieldRepository.h"
#include "volume_surface/viewer/WeightPaintingStage.h"
#include "volume_surface/viewer/ViewerOptions.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <filament/Engine.h>
#include <filament/Camera.h>
#include <filament/Scene.h>
#include <filament/View.h>
#include <filamentapp/FilamentApp2.h>
#include <filamentapp/SDLDisplayManager.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <math/mat4.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <openvdb/io/File.h>
#include <openvdb/Metadata.h>

using filament::Engine;
using filament::Scene;
using filament::View;
using filament::math::float3;
using MeshRenderer = volume_surface::viewer::MeshRenderer;
using MeshSlot = volume_surface::viewer::MeshSlot;
using BrushProfileRecorder = volume_surface::viewer::BrushProfileRecorder;
using WorkflowStage = volume_surface::viewer::WorkflowStage;
using WeightPaintingStage = volume_surface::viewer::WeightPaintingStage;
using volume_surface::viewer::workflowStageName;
using PresentationController = volume_surface::viewer::PresentationController;
using SliceRenderer = volume_surface::viewer::SliceRenderer;
using ViewerContext = volume_surface::viewer::ViewerContext;
using ViewerState = volume_surface::viewer::ViewerState;
using WeightFieldRepository = volume_surface::viewer::WeightFieldRepository;
using SurfaceTargetCacheRepository =
    volume_surface::viewer::SurfaceTargetCacheRepository;
using ViewerOptions = volume_surface::viewer::ViewerOptions;
using CameraPickHit = volume_surface::viewer::CameraPickHit;
using CameraPickRay = volume_surface::viewer::CameraPickRay;
using WorkflowPanel = volume_surface::viewer::WorkflowPanel;
using ReconstructionPanelAction =
    volume_surface::viewer::ReconstructionPanelAction;
using volume_surface::viewer::parseViewerOptions;

namespace {

using BrushProfileStroke = volume_surface::viewer::BrushProfileStroke;

constexpr float kCameraNearMeters = 0.0001f;
constexpr float kCameraFarMeters = 10.0f;

void applyWorkflowPresentation(ViewerState& state, Scene& scene);
void setWorkflowStage(ViewerState& state, Scene& scene, WorkflowStage stage);
bool rebuildSurfaceTargetCache(
    ViewerState& state,
    bool tryLoadExisting = true);
bool saveSurfaceTargetCacheToDisk(ViewerState& state);
bool loadSurfaceTargetCacheFromDisk(ViewerState& state);
bool rebuildSurfaceNormalField(ViewerState& state);
bool rebuildSurfaceReconstruction(ViewerState& state, Engine& engine, Scene& scene);

bool buildCameraPickRay(
    const View& view,
    int pointerX,
    int pointerY,
    CameraPickRay& ray)
{
    const auto viewport = view.getViewport();
    if (viewport.width == 0 || viewport.height == 0) {
        return false;
    }

    const ImGuiIO& io = ImGui::GetIO();
    const float scaleX = io.DisplaySize.x > 0.0f
        ? static_cast<float>(viewport.width) / io.DisplaySize.x
        : io.DisplayFramebufferScale.x;
    const float scaleY = io.DisplaySize.y > 0.0f
        ? static_cast<float>(viewport.height) / io.DisplaySize.y
        : io.DisplayFramebufferScale.y;
    if (!std::isfinite(scaleX) || !std::isfinite(scaleY) ||
        scaleX <= 0.0f || scaleY <= 0.0f) {
        return false;
    }

    const float drawableX = static_cast<float>(pointerX) * scaleX;
    const float drawableY = static_cast<float>(pointerY) * scaleY;
    const float localX = drawableX - static_cast<float>(viewport.left);
    const float localY = drawableY;
    if (localX < 0.0f || localY < 0.0f ||
        localX >= static_cast<float>(viewport.width) ||
        localY >= static_cast<float>(viewport.height)) {
        return false;
    }

    const double normalizedX =
        (static_cast<double>(localX) + 0.5) / viewport.width * 2.0 - 1.0;
    const double normalizedY =
        1.0 - (static_cast<double>(localY) + 0.5) / viewport.height * 2.0;
    const auto& camera = view.getCamera();
    // The rendering projection uses an infinite far plane. Use the finite culling
    // projection so that unprojecting the far clip point remains well-defined.
    const filament::math::mat4 clipToScene = camera.getModelMatrix() *
        inverse(camera.getCullingProjectionMatrix());
    const filament::math::double3 nearPoint = filament::math::mat4::project(
        clipToScene,
        filament::math::double3{normalizedX, normalizedY, -1.0});
    const filament::math::double3 farPoint = filament::math::mat4::project(
        clipToScene,
        filament::math::double3{normalizedX, normalizedY, 1.0});

    ray.origin = openvdb::Vec3d{
        camera.getPosition().x,
        camera.getPosition().y,
        camera.getPosition().z};
    ray.direction = openvdb::Vec3d{
        farPoint.x - nearPoint.x,
        farPoint.y - nearPoint.y,
        farPoint.z - nearPoint.z};
    const double directionLength = ray.direction.length();
    if (!std::isfinite(directionLength) || directionLength <= 1.0e-12) {
        return false;
    }
    ray.direction /= directionLength;
    return true;
}

class ViewerDisplayManager final : public filament::app::SDLDisplayManager {
public:
    ViewerDisplayManager(
        Engine::Backend backend,
        float* wheelZoomMultiplier,
        ViewerState* state)
        : SDLDisplayManager(backend),
          mWheelZoomMultiplier(wheelZoomMultiplier),
          mState(state)
    {
    }

    void pollEvents(std::vector<filament::app::AppEvent>& events) override
    {
        SDLDisplayManager::pollEvents(events);
        for (auto& event : events) {
            if (event.type == filament::app::AppEvent::Type::MOUSE_MOVE) {
                const bool pointerChanged =
                    mState->brushPointerX != event.mouseMove.x ||
                    mState->brushPointerY != event.mouseMove.y;
                mState->brushPointerX = event.mouseMove.x;
                mState->brushPointerY = event.mouseMove.y;
                if (mState->brushControlDown && pointerChanged) {
                    mState->brushPointerDirty = true;
                    if (mState->brushLeftButtonDown) {
                        mState->brushStrokeSampleRequested = true;
                    }
                }
            } else if (event.type == filament::app::AppEvent::Type::MOUSE_BUTTON_DOWN) {
                mState->brushPointerX = event.mouseButton.x;
                mState->brushPointerY = event.mouseButton.y;
                const bool pointerOverUi = isPointerOverUi(
                    event.mouseButton.x,
                    event.mouseButton.y);
                if (!mState->brushControlDown &&
                    !pointerOverUi &&
                    (event.mouseButton.button == 1 ||
                        event.mouseButton.button == 3)) {
                    mCameraGrabActive = true;
                    ImGui::GetIO().WantCaptureMouse = false;
                }
                if (mState->brushControlDown &&
                    !pointerOverUi &&
                    event.mouseButton.button == 1) {
                    mState->brushLeftButtonDown = true;
                    mState->brushStrokeFinalizeRequested = false;
                    mState->brushHasLastPathCenter = false;
                    mState->brushStrokeHasAppliedPreview = false;
                    mState->brushStrokePathWorld.clear();
                    mState->brushStrokeFittedCenterCount = 0;
                    mState->brushStrokeNextFitTime = {};
                    mState->brushInteractionController.beginProfileStroke(*mState);
                    mState->brushStrokeSampleRequested = true;
                    mState->brushPointerDirty = true;
                }
            } else if (event.type == filament::app::AppEvent::Type::MOUSE_BUTTON_UP) {
                if (event.mouseButton.button == 1 || event.mouseButton.button == 3) {
                    mCameraGrabActive = false;
                }
                if (event.mouseButton.button == 1 && mState->brushLeftButtonDown) {
                    mState->brushPointerX = event.mouseButton.x;
                    mState->brushPointerY = event.mouseButton.y;
                    mState->brushLeftButtonDown = false;
                    mState->brushPointerDirty = true;
                    mState->brushStrokeSampleRequested = true;
                    mState->brushStrokeFinalizeRequested = true;
                }
            } else if (event.type == filament::app::AppEvent::Type::MOUSE_WHEEL) {
                int pointerX = 0;
                int pointerY = 0;
                SDLDisplayManager::getMouseState(&pointerX, &pointerY);
                mState->brushPointerX = pointerX;
                mState->brushPointerY = pointerY;
            }
            if (event.type == filament::app::AppEvent::Type::KEYDOWN &&
                (event.key.code == filament::app::AppKey::LEFT_CTRL ||
                    event.key.code == filament::app::AppKey::RIGHT_CTRL)) {
                const bool wasThisControlDown = event.key.code ==
                    filament::app::AppKey::LEFT_CTRL ? mLeftControlDown : mRightControlDown;
                if (event.key.code == filament::app::AppKey::LEFT_CTRL) {
                    mLeftControlDown = true;
                } else {
                    mRightControlDown = true;
                }
                mState->brushControlDown = mLeftControlDown || mRightControlDown;
                if (!wasThisControlDown) {
                    mState->brushPointerDirty = true;
                    mState->brushCursorVisible = false;
                }
            } else if (event.type == filament::app::AppEvent::Type::KEYUP &&
                (event.key.code == filament::app::AppKey::LEFT_CTRL ||
                    event.key.code == filament::app::AppKey::RIGHT_CTRL)) {
                if (event.key.code == filament::app::AppKey::LEFT_CTRL) {
                    mLeftControlDown = false;
                } else {
                    mRightControlDown = false;
                }
                mState->brushControlDown = mLeftControlDown || mRightControlDown;
                if (!mState->brushControlDown) {
                    if (mState->brushLeftButtonDown) {
                        mState->brushStrokeFinalizeRequested = true;
                    }
                    mState->brushLeftButtonDown = false;
                    mState->brushStrokeSampleRequested = false;
                    mState->brushCursorVisible = false;
                }
            }
        }
        if (mState->brushParameterAdjustActive) {
            return;
        }

        double pendingWheelDistance = std::numeric_limits<double>::quiet_NaN();
        for (auto& event : events) {
            if (mState->brushPreviewEnabled && mState->brushControlDown &&
                (event.type == filament::app::AppEvent::Type::MOUSE_BUTTON_DOWN ||
                    event.type == filament::app::AppEvent::Type::MOUSE_MOVE ||
                    event.type == filament::app::AppEvent::Type::MOUSE_WHEEL)) {
                event.type = filament::app::AppEvent::Type::TEXTINPUT;
                event.text.text[0] = '\0';
                continue;
            }
            if (event.type == filament::app::AppEvent::Type::MOUSE_WHEEL) {
                const std::int32_t rawDelta = event.mouseWheel.delta;
                event.mouseWheel.delta = adaptWheelDelta(
                    rawDelta,
                    pendingWheelDistance);
            }
        }
    }

    std::uint32_t getMouseState(int* x, int* y) const override
    {
        const std::uint32_t buttons = SDLDisplayManager::getMouseState(x, y);
        if (mCameraGrabActive) {
            // Keep ImGui from stealing a camera drag when the pointer crosses a panel.
            *x = -1;
            *y = -1;
        }
        return buttons;
    }

private:
    std::int32_t adaptWheelDelta(
        std::int32_t rawDelta,
        double& pendingWheelDistance)
    {
        const std::int32_t fallback = static_cast<std::int32_t>(std::lround(
            -static_cast<double>(rawDelta) * *mWheelZoomMultiplier));
        if (rawDelta == 0 || !mState->context.view ||
            isPointerOverUi(mState->brushPointerX, mState->brushPointerY)) {
            mState->cameraWheelHit = {};
            mState->cameraWheelProjectedDistanceMillimeters = 0.0f;
            mState->cameraWheelAppliedStepMillimeters = 0.0f;
            mState->cameraWheelUsedAdaptiveStep = false;
            pendingWheelDistance = std::numeric_limits<double>::quiet_NaN();
            return fallback;
        }

        CameraPickRay ray;
        if (!buildCameraPickRay(
                *mState->context.view,
                mState->brushPointerX,
                mState->brushPointerY,
                ray)) {
            mState->cameraWheelHit = {};
            mState->cameraWheelProjectedDistanceMillimeters = 0.0f;
            mState->cameraWheelAppliedStepMillimeters = 0.0f;
            mState->cameraWheelUsedAdaptiveStep = false;
            pendingWheelDistance = std::numeric_limits<double>::quiet_NaN();
            return fallback;
        }

        std::array<bool, 4> visibleSlots{};
        for (std::size_t index = 0; index < visibleSlots.size(); ++index) {
            visibleSlots[index] = mState->slots[index].visible;
        }
        const CameraPickHit hit = mState->cameraPickController.pick(ray, visibleSlots);
        mState->cameraWheelHit = hit;
        if (!hit.hit) {
            mState->cameraWheelProjectedDistanceMillimeters = 0.0f;
            mState->cameraWheelAppliedStepMillimeters = 0.0f;
            mState->cameraWheelUsedAdaptiveStep = false;
            pendingWheelDistance = std::numeric_limits<double>::quiet_NaN();
            return fallback;
        }

        const auto& camera = mState->context.view->getCamera();
        const openvdb::Vec3d cameraPosition{
            camera.getPosition().x,
            camera.getPosition().y,
            camera.getPosition().z};
        const auto cameraForward = camera.getForwardVector();
        openvdb::Vec3d forward{
            cameraForward.x,
            cameraForward.y,
            cameraForward.z};
        const double forwardLength = forward.length();
        if (!std::isfinite(forwardLength) || forwardLength <= 1.0e-12) {
            mState->cameraWheelProjectedDistanceMillimeters = 0.0f;
            mState->cameraWheelAppliedStepMillimeters = 0.0f;
            mState->cameraWheelUsedAdaptiveStep = false;
            return fallback;
        }
        forward /= forwardLength;
        const double projectedDistance =
            (hit.scenePosition - cameraPosition).dot(forward);
        if (!std::isfinite(projectedDistance) || projectedDistance <= 0.0) {
            mState->cameraWheelProjectedDistanceMillimeters = 0.0f;
            mState->cameraWheelAppliedStepMillimeters = 0.0f;
            mState->cameraWheelUsedAdaptiveStep = false;
            pendingWheelDistance = std::numeric_limits<double>::quiet_NaN();
            return fallback;
        }

        if (!std::isfinite(pendingWheelDistance)) {
            pendingWheelDistance = projectedDistance;
        }
        const double distanceForStep = std::max(pendingWheelDistance, 0.0);

        constexpr double millimetersToWorldUnits = 0.001;
        constexpr double manipulatorZoomSpeed = 0.01;
        const double displayScale = std::max(
            static_cast<double>(mState->displayScale), 1.0e-9);
        const double minimumStep = std::max(
            0.0,
            static_cast<double>(mState->wheelMinimumDistanceMillimeters)) *
            millimetersToWorldUnits * displayScale;
        const double clearance = std::max(
            0.0,
            static_cast<double>(mState->wheelSurfaceClearanceMillimeters)) *
            millimetersToWorldUnits * displayScale;
        const double sceneUnitsPerMillimeter =
            millimetersToWorldUnits * displayScale;
        mState->cameraWheelProjectedDistanceMillimeters = static_cast<float>(
            projectedDistance / sceneUnitsPerMillimeter);
        const double hitRatio = std::clamp(
            static_cast<double>(mState->wheelHitDistanceRatio),
            0.001,
            1.0);
        const double speedScale = std::clamp(
            static_cast<double>(*mWheelZoomMultiplier) / 12.0,
            0.1,
            4.0);
        const double wheelMagnitude = std::abs(static_cast<double>(rawDelta));
        double sceneStep = std::max(
            minimumStep,
            distanceForStep * hitRatio * speedScale) * wheelMagnitude;
        if (rawDelta > 0) {
            const double maximumApproach = distanceForStep - clearance;
            if (!std::isfinite(maximumApproach) || maximumApproach <= 0.0) {
                mState->cameraWheelAppliedStepMillimeters = 0.0f;
                mState->cameraWheelUsedAdaptiveStep = true;
                return 0;
            }
            sceneStep = std::min(sceneStep, maximumApproach);
        }
        if (!std::isfinite(sceneStep) || sceneStep <= 0.0) {
            mState->cameraWheelAppliedStepMillimeters = 0.0f;
            mState->cameraWheelUsedAdaptiveStep = true;
            return 0;
        }

        pendingWheelDistance += rawDelta > 0 ? -sceneStep : sceneStep;
        mState->cameraWheelAppliedStepMillimeters = static_cast<float>(
            sceneStep / sceneUnitsPerMillimeter);
        mState->cameraWheelUsedAdaptiveStep = true;

        const auto scrollMagnitude = static_cast<std::int64_t>(std::lround(
            sceneStep / manipulatorZoomSpeed));
        if (scrollMagnitude <= 0) {
            return 0;
        }
        const auto boundedMagnitude = std::min<std::int64_t>(
            scrollMagnitude,
            std::numeric_limits<std::int32_t>::max());
        return rawDelta > 0
            ? -static_cast<std::int32_t>(boundedMagnitude)
            : static_cast<std::int32_t>(boundedMagnitude);
    }

    bool isPointerOverUi(int x, int y) const
    {
        ImGuiContext* context = ImGui::GetCurrentContext();
        if (!context) {
            return false;
        }
        const ImVec2 point{static_cast<float>(x), static_cast<float>(y)};
        for (int index = context->Windows.Size - 1; index >= 0; --index) {
            ImGuiWindow* window = context->Windows[index];
            if (!window || !window->WasActive || window->Hidden || window->Collapsed ||
                (window->Flags & ImGuiWindowFlags_NoInputs) != 0) {
                continue;
            }
            if (window->Rect().Contains(point)) {
                return true;
            }
        }
        return false;
    }

    float* mWheelZoomMultiplier;
    ViewerState* mState;
    bool mLeftControlDown = false;
    bool mRightControlDown = false;
    bool mCameraGrabActive = false;
};


openvdb::FloatGrid::Ptr loadFloatGrid(
    const std::filesystem::path& path,
    const std::string& gridName)
{
    openvdb::io::File file(path.string());
    file.open();
    auto baseGrid = file.readGrid(gridName);
    file.close();
    auto grid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);
    if (!grid) {
        throw std::runtime_error("Grid is missing or is not a FloatGrid: " + gridName);
    }
    return grid;
}

int axisIndex(volume_surface::SliceAxis axis)
{
    return static_cast<int>(axis);
}

const char* axisName(int axis)
{
    constexpr std::array<const char*, 3> names{"X", "Y", "Z"};
    return names.at(static_cast<std::size_t>(axis));
}

const char* planeName(volume_surface::SliceAxis axis)
{
    switch (axis) {
        case volume_surface::SliceAxis::X:
            return "YZ";
        case volume_surface::SliceAxis::Y:
            return "XZ";
        case volume_surface::SliceAxis::Z:
            return "XY";
    }
    return "Unknown";
}

bool nearlyEqual(float a, float b)
{
    return std::abs(a - b) <= 1.0e-4f;
}

struct RasterContourStyle {
    bool visible = false;
    std::array<std::uint8_t, 4> color{};
    int radius = 0;
};

RasterContourStyle rasterContourStyle(const ViewerState& state, float level)
{
    if (nearlyEqual(level, state.referenceIsoValue)) {
        return {
            state.referenceContour,
            {255, 235, 40, 255},
            1};
    }

    constexpr std::array<float, 4> levels{64.0f, 128.0f, 192.0f, 255.0f};
    constexpr std::array<std::array<std::uint8_t, 4>, 4> colors{
        std::array<std::uint8_t, 4>{35, 190, 255, 255},
        std::array<std::uint8_t, 4>{50, 230, 120, 255},
        std::array<std::uint8_t, 4>{255, 145, 35, 255},
        std::array<std::uint8_t, 4>{255, 60, 75, 255}};
    for (std::size_t i = 0; i < levels.size(); ++i) {
        if (nearlyEqual(level, levels[i])) {
            return {state.auxiliaryContours[i], colors[i], 0};
        }
    }
    return {};
}

void paintSlicePixel(
    volume_surface::DensitySlice& slice,
    int x,
    int y,
    const std::array<std::uint8_t, 4>& color,
    int radius)
{
    for (int offsetY = -radius; offsetY <= radius; ++offsetY) {
        for (int offsetX = -radius; offsetX <= radius; ++offsetX) {
            const int pixelX = x + offsetX;
            const int pixelY = y + offsetY;
            if (pixelX < 0 || pixelY < 0
                || pixelX >= static_cast<int>(slice.width)
                || pixelY >= static_cast<int>(slice.height)) {
                continue;
            }
            const std::size_t offset =
                (static_cast<std::size_t>(pixelY) * slice.width + pixelX) * 4;
            std::copy(color.begin(), color.end(), slice.rgba.begin() + offset);
        }
    }
}

void rasterizeSliceContours(ViewerState& state, volume_surface::DensitySlice& slice)
{
    for (const auto& segment : slice.contours) {
        const auto style = rasterContourStyle(state, segment.level);
        if (!style.visible) {
            continue;
        }

        const int startX = static_cast<int>(std::lround(
            segment.start[0] * static_cast<float>(slice.width - 1)));
        const int startY = static_cast<int>(std::lround(
            segment.start[1] * static_cast<float>(slice.height - 1)));
        const int endX = static_cast<int>(std::lround(
            segment.end[0] * static_cast<float>(slice.width - 1)));
        const int endY = static_cast<int>(std::lround(
            segment.end[1] * static_cast<float>(slice.height - 1)));

        int x = startX;
        int y = startY;
        const int deltaX = std::abs(endX - startX);
        const int stepX = startX < endX ? 1 : -1;
        const int deltaY = -std::abs(endY - startY);
        const int stepY = startY < endY ? 1 : -1;
        int error = deltaX + deltaY;
        while (true) {
            paintSlicePixel(slice, x, y, style.color, style.radius);
            if (x == endX && y == endY) {
                break;
            }
            const int doubledError = 2 * error;
            if (doubledError >= deltaY) {
                error += deltaY;
                x += stepX;
            }
            if (doubledError <= deltaX) {
                error += deltaX;
                y += stepY;
            }
        }
    }
}

void rasterizeSurfaceTargetSamples(
    const ViewerState& state,
    volume_surface::DensitySlice& slice)
{
    if (!state.surfaceTargetCache || state.surfaceTargetCache->empty() ||
        (!state.surfaceTargetCoreSamples && !state.surfaceTargetTransitionSamples)) {
        return;
    }

    const int fixedAxis = axisIndex(state.sliceAxis);
    const int fixedIndex = state.sliceIndices[fixedAxis];
    const std::size_t pixelCount =
        static_cast<std::size_t>(slice.width) * slice.height;
    std::vector<std::uint8_t> pixelPriority(pixelCount, 0);

    for (const auto& sample : state.surfaceTargetCache->samples) {
        const bool core =
            sample.kind == volume_surface::SurfaceTargetSampleKind::Core;
        if ((core && !state.surfaceTargetCoreSamples) ||
            (!core && !state.surfaceTargetTransitionSamples) ||
            sample.coordinate[fixedAxis] != fixedIndex) {
            continue;
        }

        const int horizontal = sample.coordinate[slice.horizontalAxis] -
            slice.horizontalMinimum;
        const int vertical = sample.coordinate[slice.verticalAxis] -
            slice.verticalMinimum;
        if (horizontal < 0 || vertical < 0 ||
            horizontal >= static_cast<int>(slice.width) ||
            vertical >= static_cast<int>(slice.height)) {
            continue;
        }

        const std::size_t pixelIndex =
            static_cast<std::size_t>(slice.height - 1 - vertical) * slice.width +
            static_cast<std::size_t>(horizontal);
        const std::uint8_t priority = core ? 2 : 1;
        if (priority <= pixelPriority[pixelIndex]) {
            continue;
        }
        pixelPriority[pixelIndex] = priority;
    }

    for (std::size_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex) {
        if (pixelPriority[pixelIndex] == 0) {
            continue;
        }
        const int x = static_cast<int>(pixelIndex % slice.width);
        const int y = static_cast<int>(pixelIndex / slice.width);
        const auto color = pixelPriority[pixelIndex] == 2
            ? std::array<std::uint8_t, 4>{255, 235, 40, 255}
            : std::array<std::uint8_t, 4>{255, 130, 70, 230};
        paintSlicePixel(
            slice,
            x,
            y,
            color,
            pixelPriority[pixelIndex] == 2 ? 1 : 0);
    }
}

void refreshDensitySlice(ViewerState& state, Engine& engine)
{
    try {
        std::vector<float> levels{state.referenceIsoValue};
        for (const float level : {64.0f, 128.0f, 192.0f, 255.0f}) {
            if (std::abs(level - state.referenceIsoValue) > 1.0e-4f) {
                levels.push_back(level);
            }
        }
        const int selectedIndex = state.sliceIndices[axisIndex(state.sliceAxis)];
        auto slice = volume_surface::extractDensitySlice(
            *state.grid,
            state.sliceBounds,
            state.sliceAxis,
            selectedIndex,
            state.sliceDisplayMaximum,
            levels);
        rasterizeSliceContours(state, slice);
        rasterizeSurfaceTargetSamples(state, slice);

        state.sliceRenderer.rebuild(engine, slice);
        state.densitySlice = std::move(slice);
        state.sliceStatus = "Slice ready";
        state.sliceDirty = false;
    } catch (const std::exception& error) {
        state.sliceStatus = std::string("Slice failed: ") + error.what();
        state.sliceDirty = false;
    }
}

void updateReferenceTransform(ViewerState& state)
{
    const auto& bounds = state.slots[0].mesh.bounds;
    const float3 minimum{bounds.minimum[0], bounds.minimum[1], bounds.minimum[2]};
    const float3 maximum{bounds.maximum[0], bounds.maximum[1], bounds.maximum[2]};
    state.referenceCenter = (minimum + maximum) * 0.5f;
    const float3 extent = maximum - minimum;
    const float maximumExtent = std::max(extent.x, std::max(extent.y, extent.z));
    state.displayScale = maximumExtent > 0.0f ? 2.0f / maximumExtent : 1.0f;
    state.context.coordinates.reset(
        openvdb::Vec3d{
            state.referenceCenter.x,
            state.referenceCenter.y,
            state.referenceCenter.z},
        state.displayScale);
}

std::filesystem::path brushWeightFieldDirectoryForInput(
    const std::filesystem::path& input)
{
    return WeightFieldRepository::directoryForInput(input);
}

bool isValidBrushWeightFieldName(const std::string& name)
{
    return WeightFieldRepository::isValidName(name);
}

std::vector<BrushProfileStroke> loadBrushProfileDocument(
    const std::filesystem::path& path)
{
    return BrushProfileRecorder::load(path);
}

void applySlotStyle(
    ViewerState& state,
    Engine& engine,
    std::size_t slotIndex)
{
    if (slotIndex >= state.slots.size()) {
        return;
    }
    state.meshRenderer.applyStyle(engine, slotIndex, state.slots[slotIndex]);
}

void setSlotVisible(
    ViewerState& state,
    Scene& scene,
    std::size_t slotIndex,
    bool visible)
{
    if (slotIndex >= state.slots.size()) {
        return;
    }
    auto& slot = state.slots[slotIndex];
    if (!slot.available()) {
        slot.visible = false;
        state.meshRenderer.setVisible(scene, slotIndex, false);
        return;
    }
    slot.visible = visible;
    state.meshRenderer.setVisible(scene, slotIndex, visible);
}

void applyWorkflowPresentation(ViewerState& state, Scene& scene)
{
    const bool showHeatmap = WeightPaintingStage::isActive(
        state.workflowController.stage());
    PresentationController::State presentationState;
    presentationState.surfaceTargetStage =
        state.workflowController.stage() == WorkflowStage::SurfaceTarget ||
        state.workflowController.stage() == WorkflowStage::NormalField;
    presentationState.weightPaintingStage = showHeatmap;
    presentationState.showResults =
        state.workflowController.stage() == WorkflowStage::Reconstruction ||
        state.workflowController.stage() == WorkflowStage::Review;
    presentationState.showReferenceMesh = state.showReferenceMesh;
    presentationState.showSurfaceTargetMesh = state.showSurfaceTargetMesh;
    state.presentationController.apply(
        scene,
        state.meshRenderer,
        state.slots,
        state.surfaceTargetPreview,
        state.brushHeatmap,
        presentationState);
    if (!showHeatmap) {
        state.brushCursorVisible = false;
    }
}

void setWorkflowStage(ViewerState& state, Scene& scene, WorkflowStage stage)
{
    if (!state.workflowController.setStage(stage)) {
        applyWorkflowPresentation(state, scene);
        return;
    }
    state.normalFieldPreviewActive = stage == WorkflowStage::NormalField;
    state.normalFieldPreviewDirty = true;
    applyWorkflowPresentation(state, scene);
    state.status = std::string("Active stage: ") + workflowStageName(stage);
}

void destroyGpuSlot(
    ViewerState& state,
    Engine& engine,
    Scene& scene,
    std::size_t slotIndex)
{
    if (slotIndex >= state.slots.size()) {
        return;
    }
    state.meshRenderer.destroy(engine, scene, slotIndex);
    state.cameraPickController.clearSlot(slotIndex);
    state.slots[slotIndex].visible = false;
}

void createGpuSlot(
    ViewerState& state,
    Engine& engine,
    Scene& scene,
    std::size_t slotIndex)
{
    if (slotIndex >= state.slots.size()) {
        return;
    }
    auto& slot = state.slots[slotIndex];
    if (!slot.available()) {
        return;
    }
    const auto& mapper = state.context.coordinates;
    state.meshRenderer.create(
        engine,
        scene,
        slotIndex,
        slot,
        mapper,
        state.context.app->getDefaultMaterial(),
        state.context.app->getTransparentMaterial());
    state.meshRenderer.applyStyle(engine, slotIndex, slot);
    state.meshRenderer.setVisible(scene, slotIndex, slot.visible);
    state.cameraPickController.rebuildSlot(
        slotIndex,
        slot.mesh,
        state.context.coordinates);
}

void destroyBrushHeatmapGeometry(
    ViewerState& state,
    Engine& engine,
    Scene& scene)
{
    state.brushHeatmap.clearGeometry(engine, scene);
}

void destroyBrushHeatmapResources(
    ViewerState& state,
    Engine& engine,
    Scene& scene)
{
    state.brushHeatmap.destroy(engine, scene);
    state.brushStrokeCount = 0;
}

void createBrushHeatmapResources(
    ViewerState& state,
    Engine& engine,
    Scene& scene)
{
    state.brushStrokeCount = 0;
    const auto& mesh = state.slots[0].mesh;
    if (mesh.empty() || !state.grid) {
        state.brushHeatmap.destroy(engine, scene);
        return;
    }
    state.brushHeatmap.create(
        engine,
        scene,
        mesh,
        *state.grid,
        state.referenceCenter,
        static_cast<float>(state.displayScale));
}

void destroyBrushCursorResources(
    ViewerState& state,
    Engine& engine,
    Scene& scene)
{
    state.brushCursorRenderer.destroy(engine, scene);
}

void createBrushCursorResources(
    ViewerState& state,
    Engine& engine,
    Scene& scene)
{
    state.brushCursorRenderer.create(engine, scene);
}

std::filesystem::path surfaceTargetCachePathForInput(const ViewerState& state)
{
    return SurfaceTargetCacheRepository::pathForInput(state.input, state.gridName);
}

volume_surface::SurfaceTargetCacheMetadata surfaceTargetCacheMetadata(
    const ViewerState& state)
{
    return SurfaceTargetCacheRepository::metadataForInput(
        state.input,
        state.gridName,
        state.grid.get(),
        state.surfaceTargetSettings);
}

bool saveSurfaceTargetCacheToDisk(ViewerState& state)
{
    if (!state.surfaceTargetCache || state.surfaceTargetCache->empty()) {
        state.surfaceTargetCacheStatus = "No surface target cache is available to save";
        return false;
    }
    state.surfaceTargetCachePath = surfaceTargetCachePathForInput(state);
    std::string error;
    if (!SurfaceTargetCacheRepository::save(
            state.surfaceTargetCachePath,
            *state.surfaceTargetCache,
            surfaceTargetCacheMetadata(state),
            error)) {
        state.surfaceTargetCacheStatus = "Cache save failed: " + error;
        return false;
    }
    state.surfaceTargetCacheStatus =
        "Saved " + state.surfaceTargetCachePath.filename().string();
    return true;
}

bool loadSurfaceTargetCacheFromDisk(ViewerState& state)
{
    if (!state.grid) {
        state.surfaceTargetCacheStatus = "Cannot load cache without a source grid";
        return false;
    }
    state.surfaceTargetCachePath = surfaceTargetCachePathForInput(state);
    const auto start = std::chrono::steady_clock::now();
    volume_surface::SurfaceTargetCache loaded;
    std::string error;
    if (!SurfaceTargetCacheRepository::load(
            state.surfaceTargetCachePath,
            surfaceTargetCacheMetadata(state),
            loaded,
            error)) {
        state.surfaceTargetCacheStatus = "Cache load skipped: " + error;
        return false;
    }
    state.surfaceTargetBuildMilliseconds =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
    state.surfaceTargetCache = std::make_shared<volume_surface::SurfaceTargetCache>(
        std::move(loaded));
    state.normalFieldReady = false;
    state.normalField = {};
    state.normalFieldStatus = "Normal field has not been built for this cache";
    state.normalFieldPreviewDirty = true;
    state.surfaceTargetCacheStatus =
        "Loaded " + state.surfaceTargetCachePath.filename().string();
    state.surfaceTargetStatus =
        "Surface target loaded: " +
        std::to_string(state.surfaceTargetCache->coreCount) +
        " core / " +
        std::to_string(state.surfaceTargetCache->transitionCount) +
        " transition samples";
    state.sliceDirty = true;
    return true;
}

bool rebuildSurfaceTargetCache(
    ViewerState& state,
    bool tryLoadExisting)
{
    if (!state.grid) {
        state.surfaceTargetStatus = "Surface target requires a loaded grid";
        return false;
    }

    state.surfaceTargetSettings.isoValue = state.isoValue;
    constexpr double millimetersToMeters = 0.001;
    state.surfaceTargetSettings.normalRadius =
        std::max(0.0001, static_cast<double>(state.brushPlanarityRadiusMillimeters) *
            millimetersToMeters);
    state.surfaceTargetSettings.planarityRadius =
        std::max(0.0001, static_cast<double>(state.brushPlanarityRadiusMillimeters) *
            millimetersToMeters);
    state.surfaceTargetSettings.planarityAngularScaleRadians =
        std::max(0.001, static_cast<double>(state.brushPlanarityAngleDegrees) *
            3.14159265358979323846 / 180.0);
    state.surfaceTargetCachePath = surfaceTargetCachePathForInput(state);
    if (tryLoadExisting && loadSurfaceTargetCacheFromDisk(state)) {
        return true;
    }
    const auto start = std::chrono::steady_clock::now();
    try {
        auto cache = std::make_shared<volume_surface::SurfaceTargetCache>(
            volume_surface::extractSurfaceTarget(
                *state.grid,
                state.surfaceTargetSettings));
        state.surfaceTargetBuildMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
        state.surfaceTargetCache = std::move(cache);
        state.normalFieldReady = false;
        state.normalField = {};
        state.normalFieldStatus = "Normal field has not been built for this cache";
        state.normalFieldPreviewDirty = true;
        state.surfaceTargetStatus =
            "Surface target ready: " +
            std::to_string(state.surfaceTargetCache->coreCount) +
            " core / " +
            std::to_string(state.surfaceTargetCache->transitionCount) +
            " transition samples";
        state.sliceDirty = true;
        saveSurfaceTargetCacheToDisk(state);
        return true;
    } catch (const std::exception& error) {
        state.surfaceTargetBuildMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
        state.surfaceTargetCache.reset();
        state.normalFieldReady = false;
        state.normalField = {};
        state.normalFieldStatus = "Normal field is unavailable";
        state.normalFieldPreviewDirty = true;
        state.surfaceTargetStatus = std::string("Surface target failed: ") + error.what();
        state.sliceDirty = true;
        return false;
    }
}

bool rebuildSurfaceNormalField(ViewerState& state)
{
    if (!state.surfaceTargetCache || state.surfaceTargetCache->empty()) {
        state.normalFieldReady = false;
        state.normalField = {};
        state.normalFieldStatus = "Normal field requires a SurfaceTarget cache";
        return false;
    }

    try {
        const auto start = std::chrono::steady_clock::now();
        state.normalField = volume_surface::smoothSurfaceTargetNormals(
            *state.surfaceTargetCache,
            state.normalSmoothingSettings);
        state.normalFieldReady = !state.normalField.empty();
        const double elapsedMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
        state.normalFieldStatus = state.normalFieldReady
            ? "Normal field ready: " +
                std::to_string(state.normalField.smoothedCoreCount) +
                " core / " +
                std::to_string(state.normalField.smoothedTransitionCount) +
                " transition samples (" +
                std::to_string(elapsedMilliseconds) + " ms)"
            : "Normal field produced no valid normals";
        state.normalFieldPreviewDirty = true;
        return state.normalFieldReady;
    } catch (const std::exception& error) {
        state.normalFieldReady = false;
        state.normalField = {};
        state.normalFieldStatus = std::string("Normal field failed: ") + error.what();
        state.normalFieldPreviewDirty = true;
        return false;
    }
}

void rebuildSurfaceTargetPreview(ViewerState& state, Engine& engine, Scene& scene)
{
    volume_surface::SurfaceTargetPreview::Inputs inputs;
    inputs.cache = state.surfaceTargetCache.get();
    inputs.fallbackMesh = &state.slots[0].mesh;
    inputs.hierarchy = state.brushHierarchy.get();
    inputs.referenceCenter = openvdb::Vec3d{
        state.referenceCenter.x,
        state.referenceCenter.y,
        state.referenceCenter.z};
    inputs.voxelSize = state.grid
        ? state.grid->voxelSize()
        : openvdb::Vec3d{1.0, 1.0, 1.0};
    inputs.displayScale = state.displayScale;
    if (state.surfaceTargetCache &&
        state.normalFieldPreviewActive &&
        state.normalFieldPreviewSmoothed &&
        state.normalFieldReady &&
        state.normalField.normals.size() == state.surfaceTargetCache->samples.size()) {
        inputs.normalOverrides = &state.normalField.normals;
    }
    state.surfaceTargetPreview.rebuild(engine, scene, inputs);
    state.normalFieldPreviewDirty = false;
    applyWorkflowPresentation(state, scene);
}

void updateBrushCursorGeometry(
    ViewerState& state,
    Engine& engine,
    Scene& scene,
    View& view)
{
    const bool shouldBeVisible = WeightPaintingStage::isActive(
        state.workflowController.stage()) &&
        state.brushPreviewEnabled &&
        state.brushControlDown && state.brushCursorVisible;
    const float3 center{
        static_cast<float>(state.brushCursorScenePosition.x),
        static_cast<float>(state.brushCursorScenePosition.y),
        static_cast<float>(state.brushCursorScenePosition.z)};
    state.brushCursorRenderer.update(
        engine,
        scene,
        view,
        shouldBeVisible,
        center,
        static_cast<float>(state.displayScale),
        state.brushCoreRadiusMillimeters,
        state.brushFalloffRadiusMillimeters);
}

void rebuildReference(ViewerState& state, Engine& engine, Scene& scene)
{
    try {
        state.status = "Extracting density iso-surface...";
        auto mesh = volume_surface::extractIsoSurface(
            *state.grid,
            state.isoValue,
            state.adaptivity);
        if (mesh.empty()) {
            state.status = "No surface found at the selected density value";
            return;
        }

        auto& reference = state.slots[0];
        destroyBrushHeatmapResources(state, engine, scene);
        destroyGpuSlot(state, engine, scene, 0);
        reference.mesh = std::move(mesh);
        reference.visible = true;
        for (std::size_t index = 1; index < state.slots.size(); ++index) {
            auto& resultSlot = state.slots[index];
            destroyGpuSlot(state, engine, scene, index);
            resultSlot.mesh = {};
            resultSlot.visible = false;
        }
        state.reconstructionStatus = "Reference changed; regenerate Result A";
        state.reconstructionCandidateCellCount = 0;
        state.reconstructionCrossingCellCount = 0;
        state.reconstructionFieldSampleCount = 0;
        state.reconstructionSourceSupportFallbackCount = 0;
        state.referenceIsoValue = state.isoValue;
        state.sliceDirty = true;
        updateReferenceTransform(state);
        createGpuSlot(state, engine, scene, 0);
        createBrushHeatmapResources(state, engine, scene);
        state.brushPaintingPanel.rebuildHeatmapFromWeightField(state, engine, scene);
        rebuildSurfaceTargetCache(state);
        rebuildSurfaceTargetPreview(state, engine, scene);
        applyWorkflowPresentation(state, scene);
        state.brushResult = {};
        state.brushStrokePathWorld.clear();
        state.brushStrokeFittedCenterCount = 0;
        state.brushStrokeFinalizeRequested = false;
        state.brushInteractionController.discardProfileStroke(state);
        state.brushStatus = "Reference rebuilt; Ctrl + left-drag to paint brush weights";
        state.status = "Reference rebuilt";
    } catch (const std::exception& error) {
        state.status = std::string("Rebuild failed: ") + error.what();
    }
}

bool rebuildSurfaceReconstruction(ViewerState& state, Engine& engine, Scene& scene)
{
    if (!state.grid || !state.surfaceTargetCache ||
        state.surfaceTargetCache->empty()) {
        state.reconstructionStatus =
            "Result A requires a loaded SurfaceTarget cache";
        return false;
    }

    state.reconstructionSettings.isoValue = state.isoValue;
    const auto start = std::chrono::steady_clock::now();
    try {
        auto result = volume_surface::reconstructSurfaceMLS(
            *state.grid,
            *state.surfaceTargetCache,
            state.reconstructionSettings);
        if (result.mesh.empty()) {
            state.reconstructionStatus =
                "MLS reconstruction produced no triangles";
            return false;
        }

        auto& resultSlot = state.slots[1];
        destroyGpuSlot(state, engine, scene, 1);
        resultSlot.mesh = std::move(result.mesh);
        resultSlot.opacity = 1.0f;
        resultSlot.visible = false;
        createGpuSlot(state, engine, scene, 1);
        state.reconstructionTimings = result.timings;
        state.reconstructionCandidateCellCount = result.candidateCellCount;
        state.reconstructionCrossingCellCount = result.crossingCellCount;
        state.reconstructionFieldSampleCount = result.fieldSampleCount;
        state.reconstructionSourceSupportFallbackCount =
            result.sourceSupportFallbackCount;
        state.reconstructionStatus =
            "Result A ready: " +
            std::to_string(resultSlot.mesh.vertices.size()) +
            " vertices / " +
            std::to_string(resultSlot.mesh.triangleCount()) +
            " triangles" +
            (result.usedSourceTopologyFallback
                ? " (closed source topology + MLS projection)"
                : " (MLS/DC topology)");
        state.status = "Surface Reconstruction Result A ready";
        applyWorkflowPresentation(state, scene);
        return true;
    } catch (const std::exception& error) {
        state.reconstructionTimings.totalMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
        state.reconstructionStatus =
            std::string("MLS reconstruction failed: ") + error.what();
        return false;
    }
}

void drawSlotControls(ViewerState& state, Engine& engine, Scene& scene, std::size_t index)
{
    auto& slot = state.slots[index];
    ImGui::PushID(static_cast<int>(index));
    ImGui::BeginDisabled(!slot.available());

    bool visible = index == 0 ? state.showReferenceMesh : slot.visible;
    if (ImGui::Checkbox(slot.name.c_str(), &visible)) {
        if (index == 0) {
            state.showReferenceMesh = visible;
        }
        setSlotVisible(state, scene, index, visible);
    }
    ImGui::SameLine();
    if (ImGui::Button("Solo")) {
        for (std::size_t candidateIndex = 0;
             candidateIndex < state.slots.size();
             ++candidateIndex) {
            setSlotVisible(
                state,
                scene,
                candidateIndex,
                candidateIndex == index);
        }
        if (index == 0) {
            state.showReferenceMesh = true;
        } else {
            state.showReferenceMesh = false;
        }
    }

    bool styleChanged = ImGui::ColorEdit3("Color", slot.color.data());
    styleChanged |= ImGui::SliderFloat("Opacity", &slot.opacity, 0.05f, 1.0f, "%.2f");
    if (styleChanged) {
        applySlotStyle(state, engine, index);
    }

    if (slot.available()) {
        ImGui::Text(
            "%zu vertices / %zu triangles",
            slot.mesh.vertices.size(),
            slot.mesh.triangleCount());
    } else {
        ImGui::TextUnformatted("No result assigned");
    }

    ImGui::EndDisabled();
    ImGui::Separator();
    ImGui::PopID();
}

void drawSliceWindow(ViewerState& state, Engine& engine)
{
    if (state.sliceDirty) {
        refreshDensitySlice(state, engine);
    }

    ImGui::SetNextWindowPos(ImVec2(400.0f, 10.0f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(700.0f, 760.0f), ImGuiCond_Once);
    ImGui::SetNextWindowCollapsed(true, ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(1.0f);
    ImGui::Begin("Slice comparison");
    state.mouseOverUi |= ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);

    const std::array<volume_surface::SliceAxis, 3> axes{
        volume_surface::SliceAxis::X,
        volume_surface::SliceAxis::Y,
        volume_surface::SliceAxis::Z};
    for (std::size_t i = 0; i < axes.size(); ++i) {
        if (i > 0) {
            ImGui::SameLine();
        }
        const bool selected = state.sliceAxis == axes[i];
        if (ImGui::RadioButton(planeName(axes[i]), selected)) {
            state.sliceAxis = axes[i];
            state.sliceDirty = true;
        }
    }

    const int fixedAxis = axisIndex(state.sliceAxis);
    const int minimumIndex = state.sliceBounds.min()[fixedAxis];
    const int maximumIndex = state.sliceBounds.max()[fixedAxis];
    if (ImGui::SliderInt(
            "Slice index",
            &state.sliceIndices[fixedAxis],
            minimumIndex,
            maximumIndex)) {
        state.sliceDirty = true;
    }

    openvdb::Vec3d indexPosition(0.0);
    indexPosition[fixedAxis] = static_cast<double>(state.sliceIndices[fixedAxis]);
    const auto worldPosition = state.grid->transform().indexToWorld(indexPosition);
    ImGui::Text(
        "%s = %d, world %s = %.3f mm",
        axisName(fixedAxis),
        state.sliceIndices[fixedAxis],
        axisName(fixedAxis),
        worldPosition[fixedAxis] * 1000.0);

    ImGui::DragFloat(
        "Density display max",
        &state.sliceDisplayMaximum,
        1.0f,
        1.0f,
        1020.0f,
        "%.1f");
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        state.sliceDisplayMaximum = std::max(state.sliceDisplayMaximum, 1.0f);
        state.sliceDirty = true;
    }

    if (ImGui::Checkbox("Raw iso surface", &state.referenceContour)) {
        state.sliceDirty = true;
    }
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0f, 0.92f, 0.16f, 1.0f), "%.1f", state.referenceIsoValue);
    if (ImGui::Checkbox("Core samples", &state.surfaceTargetCoreSamples)) {
        state.sliceDirty = true;
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("Transition samples", &state.surfaceTargetTransitionSamples)) {
        state.sliceDirty = true;
    }
    constexpr std::array<const char*, 4> contourLabels{"64", "128", "192", "255"};
    constexpr std::array<float, 4> auxiliaryLevels{64.0f, 128.0f, 192.0f, 255.0f};
    constexpr std::array<ImVec4, 4> contourColors{
        ImVec4(0.14f, 0.75f, 1.0f, 1.0f),
        ImVec4(0.20f, 0.90f, 0.47f, 1.0f),
        ImVec4(1.0f, 0.57f, 0.14f, 1.0f),
        ImVec4(1.0f, 0.24f, 0.30f, 1.0f)};
    for (std::size_t i = 0; i < state.auxiliaryContours.size(); ++i) {
        ImGui::PushID(static_cast<int>(i));
        const float level = auxiliaryLevels[i];
        ImGui::BeginDisabled(nearlyEqual(level, state.referenceIsoValue));
        if (ImGui::Checkbox("##auxiliary", &state.auxiliaryContours[i])) {
            state.sliceDirty = true;
        }
        ImGui::SameLine();
        ImGui::TextColored(contourColors[i], "%s", contourLabels[i]);
        ImGui::EndDisabled();
        if (i + 1 < state.auxiliaryContours.size()) {
            ImGui::SameLine();
        }
        ImGui::PopID();
    }

    ImGui::Text(
        "Slice density range: %.3f to %.3f",
        state.densitySlice.minimumValue,
        state.densitySlice.maximumValue);
    ImGui::TextWrapped("Status: %s", state.sliceStatus.c_str());

    if (state.sliceRenderer.texture() &&
        state.densitySlice.width > 0 &&
        state.densitySlice.height > 0) {
        const double physicalWidth =
            state.densitySlice.width * state.densitySlice.horizontalSpacing;
        const double physicalHeight =
            state.densitySlice.height * state.densitySlice.verticalSpacing;
        const float aspect = static_cast<float>(physicalWidth / physicalHeight);
        const float maximumWidth = std::max(200.0f, ImGui::GetContentRegionAvail().x);
        constexpr float maximumHeight = 520.0f;
        ImVec2 imageSize(maximumWidth, maximumWidth / aspect);
        if (imageSize.y > maximumHeight) {
            imageSize.y = maximumHeight;
            imageSize.x = maximumHeight * aspect;
        }

        const ImVec2 imageMinimum = ImGui::GetCursorScreenPos();
        const ImTextureID textureId = static_cast<ImTextureID>(
            reinterpret_cast<std::uintptr_t>(state.sliceRenderer.texture()));
        ImGui::Image(ImTextureRef(textureId), imageSize);
        if (ImGui::IsItemHovered()) {
            const ImVec2 mouse = ImGui::GetMousePos();
            const float normalizedX = std::clamp(
                (mouse.x - imageMinimum.x) / imageSize.x, 0.0f, 1.0f);
            const float normalizedY = std::clamp(
                (mouse.y - imageMinimum.y) / imageSize.y, 0.0f, 1.0f);
            const auto x = static_cast<std::uint32_t>(std::lround(
                normalizedX * (state.densitySlice.width - 1)));
            const auto y = static_cast<std::uint32_t>(std::lround(
                (1.0f - normalizedY) * (state.densitySlice.height - 1)));
            const float density = state.densitySlice.values[
                static_cast<std::size_t>(y) * state.densitySlice.width + x];

            openvdb::Vec3d indexCoordinate(0.0);
            indexCoordinate[fixedAxis] = state.sliceIndices[fixedAxis];
            indexCoordinate[state.densitySlice.horizontalAxis] =
                state.densitySlice.horizontalMinimum + x;
            indexCoordinate[state.densitySlice.verticalAxis] =
                state.densitySlice.verticalMinimum + y;
            const auto world = state.grid->transform().indexToWorld(indexCoordinate);
            ImGui::BeginTooltip();
            ImGui::Text("density = %.3f", density);
            ImGui::Text(
                "index = [%.0f, %.0f, %.0f]",
                indexCoordinate.x(), indexCoordinate.y(), indexCoordinate.z());
            ImGui::Text(
                "world = [%.3f, %.3f, %.3f] mm",
                world.x() * 1000.0,
                world.y() * 1000.0,
                world.z() * 1000.0);
            ImGui::EndTooltip();
        }

        ImGui::Text(
            "Horizontal: +%s, vertical: +%s",
            axisName(state.densitySlice.horizontalAxis),
            axisName(state.densitySlice.verticalAxis));
    }

    ImGui::End();
}

void drawSourceWindow(ViewerState& state)
{
    ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(390.0f, 460.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(1.0f);
    ImGui::Begin("Source VDB");
    state.mouseOverUi |= ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);
    ImGui::TextWrapped("File: %s", state.input.string().c_str());
    ImGui::Text("Grid: %s", state.gridName.c_str());
    if (state.grid) {
        const auto voxelSize = state.grid->voxelSize();
        ImGui::Text(
            "Voxel size: %.3f x %.3f x %.3f mm",
            voxelSize.x() * 1000.0,
            voxelSize.y() * 1000.0,
            voxelSize.z() * 1000.0);
        ImGui::Text(
            "Active voxels: %lld",
            static_cast<long long>(state.grid->activeVoxelCount()));
        const auto bounds = state.grid->evalActiveVoxelBoundingBox();
        ImGui::Text(
            "Active bbox: [%d, %d, %d] - [%d, %d, %d]",
            bounds.min().x(),
            bounds.min().y(),
            bounds.min().z(),
            bounds.max().x(),
            bounds.max().y(),
            bounds.max().z());
    }
    ImGui::Separator();
    ImGui::TextWrapped(
        "The source stage is read-only. Use Surface Validation to inspect density levels before editing downstream data.");
    ImGui::TextWrapped("Status: %s", state.status.c_str());
    ImGui::End();
}

void drawSurfaceTargetWindow(ViewerState& state, Engine& engine, Scene& scene)
{
    ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(390.0f, 520.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(1.0f);
    ImGui::Begin("Surface Target");
    state.mouseOverUi |= ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);
    ImGui::TextWrapped(
        "This stage previews the surface representation that will feed point-based smoothing.");
    const auto& mesh = state.slots[0].mesh;
    ImGui::Text("Reference vertices: %zu", mesh.vertices.size());
    ImGui::Text("Reference triangles: %zu", mesh.triangleCount());
    if (state.surfaceTargetCache) {
        ImGui::Text(
            "Target samples: %zu (core %zu / transition %zu)",
            state.surfaceTargetCache->samples.size(),
            state.surfaceTargetCache->coreCount,
            state.surfaceTargetCache->transitionCount);
        ImGui::Text(
            "Target extraction: %.1f ms",
            state.surfaceTargetBuildMilliseconds);
    }
    if (state.brushHierarchy) {
        ImGui::Text(
            "Surface BVH leaves: %zu",
            state.brushHierarchy->leafCount());
        ImGui::Text(
            "BVH build time: %.1f ms",
            state.brushHierarchyBuildMilliseconds);
    }
    ImGui::Separator();
    auto& preview = state.surfaceTargetPreview;
    auto& previewSettings = preview.settings();
    bool previewChanged = ImGui::Checkbox(
        "Surface mesh",
        &state.showSurfaceTargetMesh);
    previewChanged |= ImGui::Checkbox("Point cloud", &previewSettings.showPoints);
    ImGui::SameLine();
    previewChanged |= ImGui::Checkbox("Core", &previewSettings.showCore);
    ImGui::SameLine();
    previewChanged |= ImGui::Checkbox("Transition", &previewSettings.showTransition);
    previewChanged |= ImGui::Checkbox("Surface normals", &previewSettings.showNormals);
    previewChanged |= ImGui::Checkbox("BVH bounds", &previewSettings.showBvh);
    bool materialSettingsChanged = ImGui::Checkbox(
        "Front-facing points / normals only",
        &previewSettings.frontFacingOnly);
    float pointScale = previewSettings.pointScale;
    if (ImGui::SliderFloat("Point render scale", &pointScale, 0.25f, 4.0f, "%.2f")) {
        previewSettings.pointScale = pointScale;
        materialSettingsChanged = true;
    }
    int pointStride = static_cast<int>(std::clamp<std::size_t>(
        previewSettings.pointStride,
        1,
        512));
    if (ImGui::SliderInt("Point sample stride", &pointStride, 1, 512)) {
        previewSettings.pointStride = static_cast<std::size_t>(pointStride);
        rebuildSurfaceTargetPreview(state, engine, scene);
    }
    if (state.brushHierarchy) {
        const int maximumDepth = static_cast<int>(state.brushHierarchy->debugMaxDepth());
        int bvhDepth = static_cast<int>(std::min(
            previewSettings.bvhDepth,
            static_cast<std::size_t>(maximumDepth)));
        if (ImGui::SliderInt("BVH depth", &bvhDepth, 0, maximumDepth)) {
            previewSettings.bvhDepth = static_cast<std::size_t>(bvhDepth);
            rebuildSurfaceTargetPreview(state, engine, scene);
        }
        ImGui::Text(
            "BVH depth %d / %d",
            bvhDepth,
            maximumDepth);
    }
    if (ImGui::Button("Rebuild surface target")) {
        rebuildSurfaceTargetCache(state, false);
        rebuildSurfaceTargetPreview(state, engine, scene);
    }
    ImGui::SameLine();
    ImGui::TextWrapped("%s", state.surfaceTargetStatus.c_str());
    if (ImGui::Button("Save cache")) {
        saveSurfaceTargetCacheToDisk(state);
    }
    ImGui::SameLine();
    if (ImGui::Button("Load cache")) {
        if (loadSurfaceTargetCacheFromDisk(state)) {
            rebuildSurfaceTargetPreview(state, engine, scene);
        }
    }
    ImGui::TextWrapped(
        "Cache file: %s",
        state.surfaceTargetCachePath.empty()
            ? "not assigned"
            : state.surfaceTargetCachePath.string().c_str());
    ImGui::TextWrapped("Cache status: %s", state.surfaceTargetCacheStatus.c_str());
    if (materialSettingsChanged) {
        preview.updateMaterialSettings(engine);
    }
    if (previewChanged) {
        applyWorkflowPresentation(state, scene);
    }
    const auto& previewStats = preview.statistics();
    ImGui::Text(
        "Preview: %zu points / %zu normal segments / %zu BVH segments",
        previewStats.corePointCount + previewStats.transitionPointCount,
        previewStats.normalSegmentCount,
        previewStats.bvhSegmentCount);
    ImGui::TextWrapped("Status: %s", state.status.c_str());
    ImGui::End();
}

void drawNormalFieldWindow(ViewerState& state, Engine& engine, Scene& scene)
{
    ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(390.0f, 560.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(1.0f);
    ImGui::Begin("Normal Field");
    state.mouseOverUi |= ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);
    ImGui::TextWrapped(
        "Smooth the normals attached to SurfaceTarget samples without moving the samples.");
    ImGui::TextWrapped(
        "The filtered field is preview-only in this first pass; Reconstruction still uses the raw field.");
    ImGui::Separator();

    auto& settings = state.normalSmoothingSettings;
    float radiusMillimeters = static_cast<float>(settings.radius * 1000.0);
    if (ImGui::SliderFloat(
            "Smoothing radius",
            &radiusMillimeters,
            0.5f,
            20.0f,
            "%.2f mm",
            ImGuiSliderFlags_AlwaysClamp)) {
        settings.radius = std::max(0.0005, static_cast<double>(radiusMillimeters) * 0.001);
    }
    float strength = static_cast<float>(settings.strength);
    if (ImGui::SliderFloat("Strength", &strength, 0.0f, 1.0f, "%.2f")) {
        settings.strength = std::clamp(static_cast<double>(strength), 0.0, 1.0);
    }
    int iterations = static_cast<int>(std::clamp<std::size_t>(settings.iterations, 1, 6));
    if (ImGui::SliderInt("Iterations", &iterations, 1, 6)) {
        settings.iterations = static_cast<std::size_t>(iterations);
    }
    float angleSigmaDegrees = static_cast<float>(
        settings.angleSigmaRadians * 180.0 / 3.14159265358979323846);
    if (ImGui::SliderFloat(
            "Angle sigma",
            &angleSigmaDegrees,
            5.0f,
            60.0f,
            "%.1f deg")) {
        settings.angleSigmaRadians = std::max(
            1.0e-3,
            static_cast<double>(angleSigmaDegrees) *
                3.14159265358979323846 / 180.0);
    }
    float sheetThicknessMillimeters = static_cast<float>(settings.sheetThickness * 1000.0);
    if (ImGui::SliderFloat(
            "Sheet thickness",
            &sheetThicknessMillimeters,
            0.2f,
            5.0f,
            "%.2f mm")) {
        settings.sheetThickness = std::max(
            0.0001,
            static_cast<double>(sheetThicknessMillimeters) * 0.001);
    }
    int maximumNeighbors = static_cast<int>(std::clamp<std::size_t>(
        settings.maximumNeighbors,
        8,
        128));
    if (ImGui::SliderInt("Maximum neighbors", &maximumNeighbors, 8, 128)) {
        settings.maximumNeighbors = static_cast<std::size_t>(maximumNeighbors);
    }

    if (ImGui::Button("Build / update normal field")) {
        rebuildSurfaceNormalField(state);
        state.normalFieldPreviewDirty = true;
    }
    ImGui::SameLine();
    ImGui::TextWrapped("%s", state.normalFieldStatus.c_str());
    bool previewChanged = ImGui::Checkbox(
        "Use smoothed normals",
        &state.normalFieldPreviewSmoothed);
    ImGui::SameLine();
    previewChanged |= ImGui::Checkbox(
        "Show normal vectors",
        &state.surfaceTargetPreview.settings().showNormals);
    previewChanged |= ImGui::Checkbox(
        "Show points",
        &state.surfaceTargetPreview.settings().showPoints);
    if (previewChanged) {
        state.normalFieldPreviewDirty = true;
    }
    ImGui::Text(
        "Samples: %zu / %zu",
        state.normalField.smoothedCoreCount,
        state.normalField.smoothedTransitionCount);
    ImGui::TextWrapped(
        "Changing parameters only updates the preview after Build / update is pressed.");
    ImGui::TextWrapped("Status: %s", state.status.c_str());
    if (state.normalFieldPreviewDirty) {
        rebuildSurfaceTargetPreview(state, engine, scene);
    }
    ImGui::End();
}

void drawReviewWindow(ViewerState& state, Engine& engine, Scene& scene)
{
    ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(390.0f, 760.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(1.0f);
    ImGui::Begin("Review / Export");
    state.mouseOverUi |= ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);
    ImGui::TextWrapped(
        "Compare the reference and generated result slots here. Export controls will be added with the reconstruction backend.");
    for (std::size_t index = 0; index < state.slots.size(); ++index) {
        drawSlotControls(state, engine, scene, index);
    }
    ImGui::End();
}

void drawUi(
    const std::shared_ptr<ViewerState>& statePointer,
    Engine& engine,
    View& view,
    Scene& scene)
{
    ViewerState& state = *statePointer;
    if (state.workflowController.stage() == WorkflowStage::WeightPainting) {
        state.brushInteractionController.processPendingCenter(state, engine, scene);
        state.brushInteractionController.processPendingStroke(state, engine, scene);
    } else {
        state.brushCenterPending = false;
        state.brushStrokeSampleRequested = false;
        state.brushStrokeFinalizeRequested = false;
    }
    if (state.brushControlDown) {
        ImGui::GetIO().MousePos = ImVec2(
            static_cast<float>(state.brushPointerX),
            static_cast<float>(state.brushPointerY));
    }
    state.mouseOverUi = false;
    switch (state.workflowController.stage()) {
        case WorkflowStage::Source:
            drawSourceWindow(state);
            break;
        case WorkflowStage::Validation: {
            ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Once);
            ImGui::SetNextWindowSize(ImVec2(390.0f, 760.0f), ImGuiCond_Once);
            ImGui::SetNextWindowBgAlpha(1.0f);
            ImGui::Begin("Surface Validation");
            state.mouseOverUi |= ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);
            ImGui::TextUnformatted("Data mode: Fog Volume");
            ImGui::TextWrapped(
                "Surface: density = isoValue (not an SDF zero level set)");
            ImGui::Text("Grid: %s", state.gridName.c_str());
            ImGui::TextWrapped("File: %s", state.input.string().c_str());
            const auto& referenceBounds = state.slots[0].mesh.bounds;
            ImGui::Text(
                "Reference size: %.1f x %.1f x %.1f mm",
                (referenceBounds.maximum[0] - referenceBounds.minimum[0]) * 1000.0f,
                (referenceBounds.maximum[1] - referenceBounds.minimum[1]) * 1000.0f,
                (referenceBounds.maximum[2] - referenceBounds.minimum[2]) * 1000.0f);
            ImGui::Separator();
            ImGui::InputFloat("Density iso", &state.isoValue, 1.0f, 10.0f, "%.3f");
            ImGui::SliderFloat("Mesh adaptivity", &state.adaptivity, 0.0f, 1.0f, "%.3f");
            if (ImGui::Button("Rebuild Reference")) {
                rebuildReference(state, engine, scene);
            }
            ImGui::TextWrapped("Status: %s", state.status.c_str());
            ImGui::Separator();
            ImGui::TextWrapped(
                "Inspect density slices, 255-level surface coverage, and reference bounds here. SDF conversion is not required.");
            ImGui::End();
            break;
        }
        case WorkflowStage::SurfaceTarget:
            drawSurfaceTargetWindow(state, engine, scene);
            break;
        case WorkflowStage::NormalField:
            drawNormalFieldWindow(state, engine, scene);
            break;
        case WorkflowStage::WeightPainting:
            state.brushPaintingPanel.draw(state, engine, scene);
            break;
        case WorkflowStage::Reconstruction: {
            const auto action = state.reconstructionPanel.draw(state);
            if (action == ReconstructionPanelAction::ReferenceVisibilityChanged) {
                applyWorkflowPresentation(state, scene);
            } else if (action == ReconstructionPanelAction::GenerateResultA) {
                rebuildSurfaceReconstruction(state, engine, scene);
            } else if (action == ReconstructionPanelAction::ClearResultA) {
                auto& resultSlot = state.slots[1];
                destroyGpuSlot(state, engine, scene, 1);
                resultSlot.mesh = {};
                resultSlot.visible = false;
                state.reconstructionStatus = "Result A cleared";
                state.reconstructionCandidateCellCount = 0;
                state.reconstructionCrossingCellCount = 0;
                state.reconstructionFieldSampleCount = 0;
                state.reconstructionSourceSupportFallbackCount = 0;
                applyWorkflowPresentation(state, scene);
            }
            break;
        }
        case WorkflowStage::Review:
            drawReviewWindow(state, engine, scene);
            break;
    }
    if (state.normalFieldPreviewDirty) {
        rebuildSurfaceTargetPreview(state, engine, scene);
    }
    if (const auto requestedStage = state.workflowPanel.draw(state, engine)) {
        setWorkflowStage(state, scene, *requestedStage);
    }
    drawSliceWindow(state, engine);
    state.brushInteractionController.requestPick(statePointer, view);
    updateBrushCursorGeometry(state, engine, scene, view);
}

std::unique_ptr<FilamentApp2> createViewer(
    const std::shared_ptr<ViewerState>& state,
    filament::app::DisplayManager* displayManager)
{
    auto setup = [state](Engine* engine, View* view, Scene* scene) {
        state->context.engine = engine;
        state->context.scene = scene;
        state->context.view = view;
        view->getCamera().lookAt(
            {0.0f, 0.0f, 1.0f},
            {0.0f, 0.0f, -4.0f},
            {0.0f, 1.0f, 0.0f});
        view->setPostProcessingEnabled(true);
        state->context.lighting.create(
            *engine,
            *scene,
            state->globalLightIntensity,
            WorkflowPanel::directionalLightDirection(*state));

        for (std::size_t index = 0; index < state->slots.size(); ++index) {
            createGpuSlot(*state, *engine, *scene, index);
        }
        rebuildSurfaceTargetPreview(*state, *engine, *scene);
        createBrushHeatmapResources(*state, *engine, *scene);
        createBrushCursorResources(*state, *engine, *scene);
        if (state->replayBrushProfile) {
            state->brushInteractionController.runProfileReplay(*state, *engine, *scene);
        } else if (state->headlessSmoke) {
            const auto& firstPosition = state->slots[0].mesh.vertices.front().position;
            state->pendingBrushCenterWorld = openvdb::Vec3d{
                firstPosition[0], firstPosition[1], firstPosition[2]};
            state->brushCenterPending = true;
            state->brushInteractionController.processPendingCenter(*state, *engine, *scene);
            const std::size_t firstTriangleCount =
                state->brushHeatmap.stats().visibleTriangleCount;

            const auto& secondPosition = state->slots[0].mesh.vertices.back().position;
            state->pendingBrushCenterWorld = openvdb::Vec3d{
                secondPosition[0], secondPosition[1], secondPosition[2]};
            state->brushCenterPending = true;
            state->brushInteractionController.processPendingCenter(*state, *engine, *scene);
            if (state->brushStrokeCount != 2 ||
                state->brushHeatmap.stats().visibleTriangleCount < firstTriangleCount) {
                throw std::runtime_error("Brush stroke accumulation smoke test failed");
            }
            const std::size_t savedWeightVoxelCount =
                static_cast<std::size_t>(state->brushWeightGrid->activeVoxelCount());
            const std::size_t savedWeightVertexCount =
                state->brushHeatmap.stats().affectedVertexCount;
            const std::size_t savedStrokeCount = state->brushStrokeCount;
            if (savedWeightVoxelCount == 0 || savedWeightVertexCount == 0) {
                throw std::runtime_error("Brush weight field accumulation smoke test failed");
            }
            state->brushWeightFieldDirectory =
                std::filesystem::current_path() / "headless-weight-field-smoke";
            state->brushPaintingPanel.setWeightFieldName(*state, "roundtrip");
            state->brushWeightFieldPath =
                state->brushPaintingPanel.weightFieldPathForName(*state, "roundtrip");
            if (!state->brushPaintingPanel.saveWeightField(*state, false)) {
                throw std::runtime_error(
                    "Brush weight field save smoke test failed: " +
                    state->brushWeightFieldStatus);
            }
            state->brushWeightGrid = state->brushPaintingPanel.createWeightGrid(*state->grid);
            state->brushPaintingPanel.clearHeatmap(*state, *engine, *scene);
            if (!state->brushPaintingPanel.loadWeightField(
                    *state,
                    *engine,
                    *scene,
                    state->brushWeightFieldPath) ||
                state->brushWeightGrid->activeVoxelCount() != savedWeightVoxelCount ||
                state->brushHeatmap.stats().affectedVertexCount != savedWeightVertexCount) {
                throw std::runtime_error(
                    "Brush weight field load smoke test failed: " +
                    state->brushWeightFieldStatus);
            }
            state->brushStrokeCount = savedStrokeCount;
            state->brushCursorScenePosition = filament::math::double3{
                (firstPosition[0] - state->referenceCenter.x) * state->displayScale,
                (firstPosition[1] - state->referenceCenter.y) * state->displayScale,
                (firstPosition[2] - state->referenceCenter.z) * state->displayScale - 4.0};
            state->workflowController.setStage(WorkflowStage::WeightPainting);
            state->brushControlDown = true;
            state->brushCursorVisible = true;
            updateBrushCursorGeometry(*state, *engine, *scene, *view);
            if (!state->brushCursorRenderer.addedToScene() ||
                state->brushCursorRenderer.vertexCount() != 384) {
                throw std::runtime_error("Brush cursor renderable smoke test failed");
            }
            state->brushControlDown = false;
            state->workflowController.setStage(WorkflowStage::Validation);
            updateBrushCursorGeometry(*state, *engine, *scene, *view);
            std::cout << "brush.samples=" << state->brushResult.samples.size()
                      << " candidates=" << state->brushResult.candidateVoxelCount
                      << " strokes=" << state->brushStrokeCount
                      << " cursor.vertices=" << state->brushCursorRenderer.vertexCount()
                      << " weight_field.voxels=" << savedWeightVoxelCount
                      << " heatmap.triangles="
                      << state->brushHeatmap.stats().visibleTriangleCount << '\n';
        }
        applyWorkflowPresentation(*state, *scene);
        refreshDensitySlice(*state, *engine);
    };

    auto cleanup = [state](Engine* engine, View*, Scene* scene) {
        destroyBrushCursorResources(*state, *engine, *scene);
        destroyBrushHeatmapResources(*state, *engine, *scene);
        state->surfaceTargetPreview.destroy(*engine, *scene);
        state->meshRenderer.destroyAll(*engine, *scene);
        for (auto& slot : state->slots) {
            slot.visible = false;
        }
        state->context.lighting.destroy(*engine, *scene);
        state->sliceRenderer.destroy(*engine);
    };

    auto imgui = [state](Engine* engine, View*) {
        if (!state->context.view) {
            return;
        }
        drawUi(
            state,
            *engine,
            *state->context.view,
            *state->context.view->getScene());
    };

    auto viewer = FilamentApp2::Builder()
        .displayManager(displayManager)
        .title("VolumeSurface diagnostics")
        .size(1280, 800)
        .backend(Engine::Backend::OPENGL)
        .headless(state->headlessSmoke || state->replayBrushProfile)
        .samples(4)
        .setup(std::move(setup))
        .cleanup(std::move(cleanup))
        .imgui(std::move(imgui))
        .animation([state](Engine*, View*, double) {
            if ((state->headlessSmoke || state->replayBrushProfile) &&
                --state->smokeFramesRemaining <= 0) {
                state->context.app->close();
            }
        })
        .build();
    state->context.app = viewer.get();
    state->context.app->setSidebarWidth(0);
    // Filament receives scene units, while the VDB and the UI use meters. The
    // viewer normalizes the reference mesh by displayScale, so convert the
    // requested physical clip planes into the current scene coordinate system.
    const float sceneUnitsPerMeter = std::max(state->displayScale, 1.0e-6f);
    state->context.app->setCameraNearFar(
        kCameraNearMeters * sceneUnitsPerMeter,
        kCameraFarMeters * sceneUnitsPerMeter);
    return viewer;
}

void printMeshInfo(const ViewerState& state)
{
    const auto& mesh = state.slots[0].mesh;
    std::cout << "mode=fog\n"
              << "grid=" << state.gridName << '\n'
              << "iso=" << state.isoValue << '\n'
              << "adaptivity=" << state.adaptivity << '\n'
              << "vertices=" << mesh.vertices.size() << '\n'
              << "triangles=" << mesh.triangleCount() << '\n';
}

void printSliceInfo(const ViewerState& state)
{
    std::vector<float> levels{state.referenceIsoValue};
    for (const float level : {64.0f, 128.0f, 192.0f, 255.0f}) {
        if (std::abs(level - state.referenceIsoValue) > 1.0e-4f) {
            levels.push_back(level);
        }
    }
    const std::array<volume_surface::SliceAxis, 3> axes{
        volume_surface::SliceAxis::X,
        volume_surface::SliceAxis::Y,
        volume_surface::SliceAxis::Z};
    for (const auto axis : axes) {
        const auto slice = volume_surface::extractDensitySlice(
            *state.grid,
            state.sliceBounds,
            axis,
            state.sliceIndices[axisIndex(axis)],
            state.sliceDisplayMaximum,
            levels);
        std::cout << "slice." << planeName(axis)
                  << "=" << slice.width << 'x' << slice.height
                  << " index=" << slice.fixedIndex
                  << " range=[" << slice.minimumValue << ',' << slice.maximumValue << ']'
                  << " contours=" << slice.contours.size() << '\n';
    }
}

} // namespace

int main(int argc, char** argv)
{
    try {
        const ViewerOptions options = parseViewerOptions(argc, argv);
        openvdb::initialize();

        auto state = std::make_shared<ViewerState>();
        state->input = options.input;
        state->gridName = options.gridName;
        state->isoValue = static_cast<float>(options.isoValue);
        state->referenceIsoValue = static_cast<float>(options.isoValue);
        state->adaptivity = static_cast<float>(options.adaptivity);
        state->headlessSmoke = options.headlessSmoke;
        state->replayBrushProfile = !options.replayBrushProfile.empty();
        if (state->replayBrushProfile) {
            state->brushProfileReplayStrokes =
                loadBrushProfileDocument(options.replayBrushProfile);
        }
        state->brushPaintingPanel.loadSettings(*state);
        state->grid = loadFloatGrid(options.input, options.gridName);
        state->brushWeightGrid = state->brushPaintingPanel.createWeightGrid(*state->grid);
        state->brushWeightFieldDirectory = brushWeightFieldDirectoryForInput(options.input);
        state->brushPaintingPanel.refreshWeightFieldLibrary(*state);
        state->sliceBounds = state->grid->evalActiveVoxelBoundingBox();
        state->sliceBounds.expand(1);
        for (int axis = 0; axis < 3; ++axis) {
            state->sliceIndices[axis] =
                (state->sliceBounds.min()[axis] + state->sliceBounds.max()[axis]) / 2;
        }
        state->slots[0].mesh = volume_surface::extractIsoSurface(
            *state->grid,
            options.isoValue,
            options.adaptivity);
        if (state->slots[0].mesh.empty()) {
            throw std::runtime_error("No surface found at the selected density value");
        }
        updateReferenceTransform(*state);
        if (!options.inspectOnly) {
            rebuildSurfaceTargetCache(*state);
            if (!state->surfaceTargetCache) {
                throw std::runtime_error(state->surfaceTargetStatus);
            }
            std::cout << "surface_target.samples="
                      << state->surfaceTargetCache->samples.size()
                      << " core=" << state->surfaceTargetCache->coreCount
                      << " transition=" << state->surfaceTargetCache->transitionCount
                      << " build_ms=" << state->surfaceTargetBuildMilliseconds
                      << " cache_status=" << state->surfaceTargetCacheStatus << '\n';
        }
        printMeshInfo(*state);
        if (options.inspectOnly) {
            printSliceInfo(*state);
            return EXIT_SUCCESS;
        }

        const auto hierarchyBuildStart = std::chrono::steady_clock::now();
        state->brushHierarchy = std::make_unique<volume_surface::SurfaceBrushHierarchy>(
            *state->grid);
        state->brushHierarchyBuildMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - hierarchyBuildStart).count();
        std::cout << "brush_bvh.leaves=" << state->brushHierarchy->leafCount()
                  << " build_ms=" << state->brushHierarchyBuildMilliseconds << '\n';

        auto displayManager = std::make_unique<ViewerDisplayManager>(
            Engine::Backend::OPENGL,
            &state->wheelZoomMultiplier,
            state.get());
        auto viewer = createViewer(state, displayManager.get());
        viewer->run();
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
