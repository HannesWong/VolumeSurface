#include "volume_surface/SliceDiagnostics.h"
#include "volume_surface/SurfaceBrush.h"
#include "volume_surface/SurfaceMesh.h"
#include "volume_surface/SurfaceMeshContinuity.h"
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
using SurfaceFitPlaneRenderer = volume_surface::viewer::SurfaceFitPlaneRenderer;
using SurfaceFitExpansionDebugRenderer =
    volume_surface::viewer::SurfaceFitExpansionDebugRenderer;
using SurfaceFitNeighborhoodDebugRenderer =
    volume_surface::viewer::SurfaceFitNeighborhoodDebugRenderer;
using SurfaceFitNeighborhoodDisplayMode =
    volume_surface::viewer::SurfaceFitNeighborhoodDisplayMode;
using SurfaceTargetPointPicker =
    volume_surface::viewer::SurfaceTargetPointPicker;
using SurfaceNormalSeed = volume_surface::viewer::SurfaceNormalSeed;
using SurfaceNormalSeedStore = volume_surface::viewer::SurfaceNormalSeedStore;
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
bool rebuildSurfaceMeshContinuity(ViewerState& state);
bool loadSurfaceNormalSeedFromDisk(ViewerState& state);
bool saveSurfaceNormalSeedToDisk(ViewerState& state);
void processOrientationSeedPick(
    ViewerState& state,
    const View& view);
bool rebuildOrientedNormalField(ViewerState& state);
bool rebuildSurfaceNormalFit(ViewerState& state);
bool rebuildSurfaceNormalField(ViewerState& state);
bool rebuildSurfaceReconstruction(ViewerState& state, Engine& engine, Scene& scene);
void processSurfaceFitPick(
    ViewerState& state,
    Engine& engine,
    Scene& scene,
    const View& view);
void processSurfaceFitExpansionPick(
    ViewerState& state,
    Engine& engine,
    Scene& scene,
    const View& view);
void rebuildSurfaceFitExpansionDebug(
    ViewerState& state,
    Engine& engine,
    Scene& scene);
void rebuildPickedSurfaceFitPlane(
    ViewerState& state,
    Engine& engine,
    Scene& scene);

void clearSurfaceFitExpansionSelection(ViewerState& state)
{
    state.surfaceFitExpansionNeighborhood = {};
    state.surfaceFitExpansionPickArmed = false;
    state.surfaceFitExpansionPickRequested = false;
    state.surfaceFitExpansionParentDepthThreshold = 0;
    state.surfaceFitExpansionDebugRenderer.settings().showNeighborhood = false;
    state.surfaceFitExpansionStatus = "No debug Core point selected";
    state.surfaceFitNeighborhoodInspection = {};
    state.surfaceFitNeighborhoodPickArmed = false;
    state.surfaceFitNeighborhoodPickRequested = false;
    state.surfaceFitNeighborhoodStatus = "No fit neighborhood center selected";
}

bool hasUsableNormalField(const ViewerState& state)
{
    return state.normalFieldReady &&
        state.surfaceTargetCache &&
        state.normalField.normals.size() == state.surfaceTargetCache->samples.size();
}

bool hasUsableOrientedNormalField(const ViewerState& state)
{
    return state.orientedNormalFieldReady &&
        state.surfaceTargetCache &&
        state.orientedNormalField.normals.size() ==
            state.surfaceTargetCache->samples.size();
}

const char* normalFitNeighborhoodName(const ViewerState& state)
{
    switch (state.normalFitSettings.neighborhood) {
        case volume_surface::SurfaceFitNeighborhood::Grid5x5:
            return "5x5";
        case volume_surface::SurfaceFitNeighborhood::Grid9x9:
            return "9x9";
        case volume_surface::SurfaceFitNeighborhood::Grid3x3:
            return "3x3";
    }
    return "3x3";
}

std::size_t normalFitNeighborhoodSide(const ViewerState& state)
{
    switch (state.normalFitSettings.neighborhood) {
        case volume_surface::SurfaceFitNeighborhood::Grid5x5:
            return 5;
        case volume_surface::SurfaceFitNeighborhood::Grid9x9:
            return 9;
        case volume_surface::SurfaceFitNeighborhood::Grid3x3:
            return 3;
    }
    return 3;
}

const char* normalTrendNeighborhoodName(const ViewerState& state)
{
    switch (state.normalSmoothingSettings.neighborhood) {
        case volume_surface::SurfaceNormalNeighborhood::None:
            return "none";
        case volume_surface::SurfaceNormalNeighborhood::Grid5x5:
            return "5x5";
        case volume_surface::SurfaceNormalNeighborhood::Grid3x3:
            return "3x3";
    }
    return "none";
}

std::string reconstructionNormalSourceName(const ViewerState& state)
{
    if (hasUsableOrientedNormalField(state)) {
        return std::string("seed_oriented_") + normalFitNeighborhoodName(state) +
            "+surface_normal_" + normalTrendNeighborhoodName(state);
    }
    return hasUsableNormalField(state)
        ? std::string("surface_fit_") + normalFitNeighborhoodName(state) +
            "+surface_normal_" + normalTrendNeighborhoodName(state)
        : std::string("surface_target_seed");
}

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
                if (mSurfaceCameraGrabActive && !mState->brushControlDown) {
                    updateSurfaceCameraDrag(event.mouseMove.x, event.mouseMove.y);
                    event.type = filament::app::AppEvent::Type::TEXTINPUT;
                    event.text.text[0] = '\0';
                    continue;
                }
            } else if (event.type == filament::app::AppEvent::Type::MOUSE_BUTTON_DOWN) {
                mState->brushPointerX = event.mouseButton.x;
                mState->brushPointerY = event.mouseButton.y;
                const bool pointerOverUi = isPointerOverUi(
                    event.mouseButton.x,
                    event.mouseButton.y);
                const bool surfaceFitPick = mState->reconstructionPickArmed &&
                    mState->workflowController.stage() == WorkflowStage::SurfaceFit;
                const bool orientationSeedPick = mState->orientationSeedPickArmed &&
                    mState->workflowController.stage() == WorkflowStage::SurfaceFit;
                const bool expansionDebugPick = mState->surfaceFitExpansionPickArmed &&
                    mState->workflowController.stage() == WorkflowStage::SurfaceFit;
                const bool neighborhoodDebugPick =
                    mState->surfaceFitNeighborhoodPickArmed &&
                    mState->workflowController.stage() == WorkflowStage::SurfaceFit;
                if ((surfaceFitPick || orientationSeedPick || expansionDebugPick ||
                     neighborhoodDebugPick) &&
                    !mState->brushControlDown &&
                    !pointerOverUi &&
                    event.mouseButton.button == 1) {
                    if (surfaceFitPick) {
                        mState->reconstructionPickRequested = true;
                        mState->reconstructionPickX = event.mouseButton.x;
                        mState->reconstructionPickY = event.mouseButton.y;
                        mState->reconstructionPickArmed = false;
                    } else if (orientationSeedPick) {
                        mState->orientationSeedPickRequested = true;
                        mState->orientationSeedPickX = event.mouseButton.x;
                        mState->orientationSeedPickY = event.mouseButton.y;
                        mState->orientationSeedPickArmed = false;
                    } else {
                        mState->surfaceFitExpansionPickRequested = true;
                        mState->surfaceFitExpansionPickX = event.mouseButton.x;
                        mState->surfaceFitExpansionPickY = event.mouseButton.y;
                        if (neighborhoodDebugPick) {
                            mState->surfaceFitNeighborhoodPickRequested = true;
                            mState->surfaceFitNeighborhoodPickArmed = false;
                        } else {
                            mState->surfaceFitExpansionPickArmed = false;
                        }
                    }
                    mCameraGrabActive = false;
                    event.type = filament::app::AppEvent::Type::TEXTINPUT;
                    event.text.text[0] = '\0';
                    continue;
                }
                if (!mState->brushControlDown &&
                    !pointerOverUi &&
                    (event.mouseButton.button == 1 ||
                        event.mouseButton.button == 3)) {
                    mSurfaceCameraGrabActive = beginSurfaceCameraDrag(
                        event.mouseButton.button,
                        event.mouseButton.x,
                        event.mouseButton.y);
                    mCameraGrabActive = mSurfaceCameraGrabActive;
                    if (mSurfaceCameraGrabActive) {
                        ImGui::GetIO().WantCaptureMouse = false;
                        event.type = filament::app::AppEvent::Type::TEXTINPUT;
                        event.text.text[0] = '\0';
                        continue;
                    }
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
                if (mSurfaceCameraGrabActive &&
                    (event.mouseButton.button == 1 ||
                        event.mouseButton.button == 3)) {
                    mState->surfaceAwareCameraController.endDrag();
                    mSurfaceCameraGrabActive = false;
                    mCameraGrabActive = false;
                    event.type = filament::app::AppEvent::Type::TEXTINPUT;
                    event.text.text[0] = '\0';
                    continue;
                }
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
                if (!mState->brushControlDown &&
                    !mState->brushParameterAdjustActive &&
                    applySurfaceCameraWheel(event.mouseWheel.delta)) {
                    event.type = filament::app::AppEvent::Type::TEXTINPUT;
                    event.text.text[0] = '\0';
                    continue;
                }
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
                    if (mState->brushControlDown && mSurfaceCameraGrabActive) {
                        mState->surfaceAwareCameraController.cancelDrag();
                        mSurfaceCameraGrabActive = false;
                        mCameraGrabActive = false;
                    }
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
    bool pickAtPointer(
        int pointerX,
        int pointerY,
        CameraPickRay& ray,
        CameraPickHit& hit) const
    {
        if (!mState->context.view ||
            !buildCameraPickRay(*mState->context.view, pointerX, pointerY, ray)) {
            return false;
        }
        std::array<bool, 4> visibleSlots{};
        for (std::size_t index = 0; index < visibleSlots.size(); ++index) {
            visibleSlots[index] = mState->slots[index].visible;
        }
        hit = mState->cameraPickController.pick(ray, visibleSlots);
        return true;
    }

    bool beginSurfaceCameraDrag(int button, int pointerX, int pointerY)
    {
        CameraPickRay ray;
        CameraPickHit hit;
        if (pickAtPointer(pointerX, pointerY, ray, hit)) {
            return mState->surfaceAwareCameraController.beginDrag(
                button,
                pointerX,
                pointerY,
                hit);
        }
        // Keep the SurfaceAware controller as the sole drag owner even when
        // the pointer is not over a pickable mesh.
        hit = {};
        return mState->surfaceAwareCameraController.beginDrag(
            button,
            pointerX,
            pointerY,
            hit);
    }

    bool updateSurfaceCameraDrag(int pointerX, int pointerY)
    {
        CameraPickRay ray;
        if (!mState->context.view ||
            !buildCameraPickRay(*mState->context.view, pointerX, pointerY, ray)) {
            return true;
        }
        mState->surfaceAwareCameraController.updateDrag(pointerX, pointerY, ray);
        // Apply immediately so consecutive motion events in one poll cycle
        // build their rays from the latest controller pose.
        mState->surfaceAwareCameraController.applyTo(
            mState->context.view->getCamera());
        return true;
    }

    bool applySurfaceCameraWheel(std::int32_t rawDelta)
    {
        if (rawDelta == 0 || !mState->context.view ||
            isPointerOverUi(mState->brushPointerX, mState->brushPointerY)) {
            return false;
        }
        CameraPickRay ray;
        CameraPickHit hit;
        if (!pickAtPointer(
                mState->brushPointerX,
                mState->brushPointerY,
                ray,
                hit)) {
            hit = {};
        }
        const auto result = mState->surfaceAwareCameraController.applyWheel(
            rawDelta,
            hit,
            mState->displayScale,
            mState->wheelZoomMultiplier,
            mState->wheelHitDistanceRatio,
            mState->wheelMinimumDistanceMillimeters,
            mState->wheelSurfaceClearanceMillimeters);
        if (!result.handled) {
            return false;
        }
        mState->cameraWheelHit = result.hit;
        mState->cameraWheelProjectedDistanceMillimeters =
            result.projectedDistanceMillimeters;
        mState->cameraWheelAppliedStepMillimeters = result.appliedStepMillimeters;
        mState->cameraWheelUsedAdaptiveStep = result.usedAdaptiveStep;
        return true;
    }

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
    bool mSurfaceCameraGrabActive = false;
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
        state.workflowController.stage() == WorkflowStage::SurfaceFit ||
        state.workflowController.stage() == WorkflowStage::NormalField;
    presentationState.weightPaintingStage = showHeatmap;
    presentationState.showResults =
        state.workflowController.stage() == WorkflowStage::Reconstruction ||
        state.workflowController.stage() == WorkflowStage::Review;
    presentationState.surfaceFitStage =
        state.workflowController.stage() == WorkflowStage::SurfaceFit;
    presentationState.surfaceFitPlaneAvailable =
        state.reconstructionPickReport.hit;
    presentationState.showSurfaceFitMesh = state.showSurfaceTargetMesh;
    presentationState.showReferenceMesh = state.showReferenceMesh;
    presentationState.showSurfaceTargetMesh = state.showSurfaceTargetMesh;
    state.presentationController.apply(
        scene,
        state.meshRenderer,
        state.slots,
        state.surfaceTargetPreview,
        state.surfaceFitPlaneRenderer,
        state.brushHeatmap,
        presentationState);
    state.surfaceFitExpansionDebugRenderer.setVisible(
        scene,
        state.workflowController.stage() == WorkflowStage::SurfaceFit);
    state.surfaceFitNeighborhoodDebugRenderer.setVisible(
        scene,
        state.workflowController.stage() == WorkflowStage::SurfaceFit);
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
    if (stage != WorkflowStage::SurfaceFit) {
        state.reconstructionPickArmed = false;
        state.reconstructionPickRequested = false;
    }
    if (stage != WorkflowStage::SurfaceFit) {
        state.orientationSeedPickArmed = false;
        state.orientationSeedPickRequested = false;
    }
    if (stage != WorkflowStage::SurfaceFit) {
        state.surfaceFitExpansionPickArmed = false;
        state.surfaceFitExpansionPickRequested = false;
        state.surfaceFitNeighborhoodPickArmed = false;
        state.surfaceFitNeighborhoodPickRequested = false;
    }
    state.normalFieldPreviewActive = stage == WorkflowStage::NormalField;
    state.normalFitPreviewActive = stage == WorkflowStage::SurfaceFit;
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

std::filesystem::path surfaceNormalSeedPathForInput(const ViewerState& state)
{
    return SurfaceNormalSeedStore::pathForInput(state.input, state.gridName);
}

bool loadSurfaceNormalSeedFromDisk(ViewerState& state)
{
    state.surfaceNormalSeedPath = surfaceNormalSeedPathForInput(state);
    state.surfaceNormalSeed = {};
    std::string error;
    if (!SurfaceNormalSeedStore::load(
            state.surfaceNormalSeedPath,
            state.input,
            state.gridName,
            state.isoValue,
            state.surfaceNormalSeed,
            error)) {
        state.surfaceNormalSeedStatus = error.empty()
            ? "No saved orientation seed"
            : "Seed load skipped: " + error;
        return false;
    }
    state.surfaceNormalSeedStatus =
        "Loaded " + state.surfaceNormalSeedPath.filename().string();
    const bool orientedReady = rebuildOrientedNormalField(state);
    if (orientedReady) {
        state.reconstructionStatus = state.slots[1].available()
            ? "Orientation seed updated; regenerate Result A"
            : "Orientation seed applied; Result A will use it on generation";
    }
    return true;
}

bool saveSurfaceNormalSeedToDisk(ViewerState& state)
{
    if (!state.surfaceNormalSeed.valid) {
        state.surfaceNormalSeedStatus = "No orientation seed is available to save";
        return false;
    }
    state.surfaceNormalSeedPath = surfaceNormalSeedPathForInput(state);
    std::string error;
    if (!SurfaceNormalSeedStore::save(
            state.surfaceNormalSeedPath,
            state.surfaceNormalSeed,
            state.input,
            state.gridName,
            state.isoValue,
            error)) {
        state.surfaceNormalSeedStatus = "Seed save failed: " + error;
        return false;
    }
    state.surfaceNormalSeedStatus =
        "Saved " + state.surfaceNormalSeedPath.filename().string();
    const bool orientedReady = rebuildOrientedNormalField(state);
    if (orientedReady) {
        state.reconstructionStatus = state.slots[1].available()
            ? "Orientation seed updated; regenerate Result A"
            : "Orientation seed applied; Result A will use it on generation";
    }
    return true;
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

bool rebuildSurfaceMeshContinuity(ViewerState& state)
{
    state.surfaceMeshContinuity = {};
    state.surfaceMeshContinuityReady = false;
    state.surfaceMeshContinuityBuildMilliseconds = 0.0;
    if (!state.grid || !state.surfaceTargetCache ||
        state.surfaceTargetCache->empty() || state.slots[0].mesh.empty()) {
        return false;
    }

    const auto start = std::chrono::steady_clock::now();
    try {
        state.surfaceMeshContinuity = volume_surface::buildSurfaceMeshContinuity(
            state.slots[0].mesh,
            *state.surfaceTargetCache,
            *state.grid);
        state.surfaceMeshContinuityBuildMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
        state.surfaceMeshContinuityReady =
            !state.surfaceMeshContinuity.empty() &&
            state.surfaceMeshContinuity.supportedSampleCount > 0;
        return state.surfaceMeshContinuityReady;
    } catch (const std::exception&) {
        state.surfaceMeshContinuity = {};
        state.surfaceMeshContinuityBuildMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
        return false;
    }
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
    rebuildSurfaceMeshContinuity(state);
    clearSurfaceFitExpansionSelection(state);
    state.normalFitReady = false;
    state.normalFitField = {};
    state.normalFitStatus = "Fitted normal seeds have not been built for this cache";
    state.normalFieldReady = false;
    state.normalField = {};
    state.normalFieldStatus = "Normal field has not been built for this cache";
    state.orientedNormalFieldReady = false;
    state.orientedNormalField = {};
    state.orientedNormalExpansionTrace = {};
    state.normalFitAdjacencyStatistics = {};
    state.orientedNormalAdjacencyStatistics = {};
    state.orientedNormalStatus = "Orientation seed must be reapplied for this cache";
    state.normalFieldPreviewDirty = true;
    state.reconstructionPickArmed = false;
    state.reconstructionPickRequested = false;
    state.reconstructionPickReport = {};
    state.reconstructionPickStatus = "No reconstruction point has been picked";
    state.orientationSeedPickArmed = false;
    state.orientationSeedPickRequested = false;
    state.surfaceTargetCacheStatus =
        "Loaded " + state.surfaceTargetCachePath.filename().string();
    state.surfaceTargetStatus =
        "Surface target loaded: " +
        std::to_string(state.surfaceTargetCache->coreCount) +
        " core / " +
        std::to_string(state.surfaceTargetCache->transitionCount) +
        " transition samples";
    loadSurfaceNormalSeedFromDisk(state);
    state.sliceDirty = true;
    return true;
}

bool rebuildSurfaceTargetCache(
    ViewerState& state,
    bool tryLoadExisting)
{
    state.surfaceMeshContinuity = {};
    state.surfaceMeshContinuityReady = false;
    state.surfaceMeshContinuityBuildMilliseconds = 0.0;
    if (!state.grid) {
        state.surfaceTargetStatus = "Surface target requires a loaded grid";
        return false;
    }

    state.surfaceTargetSettings.isoValue = state.isoValue;
    constexpr double millimetersToMeters = 0.001;
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
        rebuildSurfaceMeshContinuity(state);
        clearSurfaceFitExpansionSelection(state);
        state.normalFitReady = false;
        state.normalFitField = {};
        state.normalFitStatus = "Fitted normal seeds have not been built for this cache";
        state.normalFieldReady = false;
        state.normalField = {};
        state.normalFieldStatus = "Normal field has not been built for this cache";
        state.orientedNormalFieldReady = false;
        state.orientedNormalField = {};
        state.orientedNormalExpansionTrace = {};
        state.normalFitAdjacencyStatistics = {};
        state.orientedNormalAdjacencyStatistics = {};
        state.orientedNormalStatus = "Orientation seed must be reapplied for this cache";
        state.normalFieldPreviewDirty = true;
        state.reconstructionPickArmed = false;
        state.reconstructionPickRequested = false;
        state.reconstructionPickReport = {};
        state.reconstructionPickStatus = "No reconstruction point has been picked";
        state.orientationSeedPickArmed = false;
        state.orientationSeedPickRequested = false;
        state.surfaceTargetStatus =
            "Surface target ready: " +
            std::to_string(state.surfaceTargetCache->coreCount) +
            " core / " +
            std::to_string(state.surfaceTargetCache->transitionCount) +
            " transition samples";
        loadSurfaceNormalSeedFromDisk(state);
        state.sliceDirty = true;
        saveSurfaceTargetCacheToDisk(state);
        return true;
    } catch (const std::exception& error) {
        state.surfaceTargetBuildMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
        state.surfaceTargetCache.reset();
        state.surfaceMeshContinuity = {};
        state.surfaceMeshContinuityReady = false;
        state.surfaceMeshContinuityBuildMilliseconds = 0.0;
        clearSurfaceFitExpansionSelection(state);
        state.normalFitReady = false;
        state.normalFitField = {};
        state.normalFitStatus = "Fitted normal seeds are unavailable";
        state.normalFieldReady = false;
        state.normalField = {};
        state.normalFieldStatus = "Normal field is unavailable";
        state.orientedNormalFieldReady = false;
        state.orientedNormalField = {};
        state.orientedNormalExpansionTrace = {};
        state.normalFitAdjacencyStatistics = {};
        state.orientedNormalAdjacencyStatistics = {};
        state.orientedNormalStatus = "Orientation seed is unavailable";
        state.normalFieldPreviewDirty = true;
        state.reconstructionPickArmed = false;
        state.reconstructionPickRequested = false;
        state.reconstructionPickReport = {};
        state.reconstructionPickStatus = "No reconstruction point has been picked";
        state.orientationSeedPickArmed = false;
        state.orientationSeedPickRequested = false;
        state.surfaceNormalSeed = {};
        state.surfaceNormalSeedStatus = "No orientation seed is available";
        state.surfaceTargetStatus = std::string("Surface target failed: ") + error.what();
        state.sliceDirty = true;
        return false;
    }
}

bool rebuildSurfaceNormalFit(ViewerState& state)
{
    clearSurfaceFitExpansionSelection(state);
    state.orientedNormalFieldReady = false;
    state.orientedNormalField = {};
    state.orientedNormalExpansionTrace = {};
    state.normalFitAdjacencyStatistics = {};
    state.orientedNormalAdjacencyStatistics = {};
    state.orientedNormalStatus = "Orientation seed must be reapplied after fitting";
    if (!state.grid || !state.surfaceTargetCache || state.surfaceTargetCache->empty()) {
        state.normalFitReady = false;
        state.normalFitField = {};
        state.normalFitStatus = "Surface fit requires the source grid and SurfaceTarget cache";
        return false;
    }

    try {
        const auto start = std::chrono::steady_clock::now();
        auto fitSettings = state.normalFitSettings;
        fitSettings.isoValue = state.surfaceTargetSettings.isoValue;
        state.normalFitField = volume_surface::fitSurfaceTargetNormals(
            *state.grid,
            *state.surfaceTargetCache,
            fitSettings);
        state.normalFitReady = !state.normalFitField.empty();
        if (state.normalFitReady) {
            state.normalFitAdjacencyStatistics =
                volume_surface::analyzeSurfaceTargetNormalAdjacency(
                    *state.surfaceTargetCache,
                    state.normalFitField);
        }
        const double elapsedMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
        state.normalFitStatus = state.normalFitReady
            ? "Surface fit ready (" + std::string(normalFitNeighborhoodName(state)) + "): " +
                std::to_string(state.normalFitField.smoothedCoreCount) +
                " core / " +
                std::to_string(state.normalFitField.smoothedTransitionCount) +
                " transition samples (" +
                std::to_string(elapsedMilliseconds) + " ms)"
            : "Surface fit produced no valid normals";
        state.normalFieldReady = false;
        state.normalField = {};
        state.normalFieldStatus = "Normal field requires a fitted seed field";
        state.normalFieldPreviewDirty = true;
        return state.normalFitReady;
    } catch (const std::exception& error) {
        state.normalFitReady = false;
        state.normalFitField = {};
        state.normalFitAdjacencyStatistics = {};
        state.orientedNormalAdjacencyStatistics = {};
        state.normalFitStatus = std::string("Surface fit failed: ") + error.what();
        state.normalFieldReady = false;
        state.normalField = {};
        state.normalFieldStatus = "Normal field is unavailable";
        state.normalFieldPreviewDirty = true;
        return false;
    }
}

bool rebuildSurfaceNormalField(ViewerState& state)
{
    clearSurfaceFitExpansionSelection(state);
    state.orientedNormalFieldReady = false;
    state.orientedNormalField = {};
    state.orientedNormalExpansionTrace = {};
    state.orientedNormalAdjacencyStatistics = {};
    state.orientedNormalStatus = "Orientation seed must be reapplied after smoothing";
    if (!state.surfaceTargetCache || state.surfaceTargetCache->empty()) {
        state.normalFieldReady = false;
        state.normalField = {};
        state.normalFieldStatus = "Normal field requires a SurfaceTarget cache";
        return false;
    }
    if (!state.normalFitReady ||
        state.normalFitField.normals.size() != state.surfaceTargetCache->samples.size()) {
        state.normalFieldReady = false;
        state.normalField = {};
        state.normalFieldStatus = "Build Surface Fit / Normal Seed first";
        return false;
    }

    try {
        const auto start = std::chrono::steady_clock::now();
        state.normalField = volume_surface::smoothSurfaceTargetNormals(
            *state.surfaceTargetCache,
            state.normalFitField,
            state.normalSmoothingSettings);
        state.normalFieldReady = !state.normalField.empty();
        const double elapsedMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
        state.normalFieldStatus = state.normalFieldReady
            ? "Normal field ready (" + std::string(normalTrendNeighborhoodName(state)) +"): " +
                std::to_string(state.normalField.smoothedCoreCount) +
                " averaged core / " +
                std::to_string(state.normalField.smoothedTransitionCount) +
                " averaged transition samples (" +
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

bool rebuildOrientedNormalField(ViewerState& state)
{
    clearSurfaceFitExpansionSelection(state);
    state.orientedNormalFieldReady = false;
    state.orientedNormalField = {};
    state.orientedNormalAdjacencyStatistics = {};
    if (!state.surfaceNormalSeed.valid || !state.surfaceTargetCache ||
        state.surfaceTargetCache->empty()) {
        state.orientedNormalStatus =
            "Apply a saved orientation seed after building Surface Fit";
        return false;
    }

    const volume_surface::SurfaceNormalField* baseField = nullptr;
    if (hasUsableNormalField(state)) {
        baseField = &state.normalField;
    } else if (state.normalFitReady &&
               state.normalFitField.normals.size() == state.surfaceTargetCache->samples.size()) {
        baseField = &state.normalFitField;
    }
    if (!baseField) {
        state.orientedNormalStatus =
            "Build fitted or smoothed normals before applying the seed";
        return false;
    }

    std::size_t seedSampleIndex = std::numeric_limits<std::size_t>::max();
    for (std::size_t index = 0; index < state.surfaceTargetCache->samples.size(); ++index) {
        if (state.surfaceTargetCache->samples[index].coordinate ==
            state.surfaceNormalSeed.coordinate) {
            seedSampleIndex = index;
            break;
        }
    }
    if (seedSampleIndex == std::numeric_limits<std::size_t>::max()) {
        state.orientedNormalStatus =
            "Saved seed coordinate is not present in the current SurfaceTarget cache";
        return false;
    }

    try {
        state.orientedNormalField = volume_surface::orientSurfaceTargetNormals(
            *state.surfaceTargetCache,
            *baseField,
            seedSampleIndex,
            state.surfaceNormalSeed.normal,
            {},
            &state.orientedNormalExpansionTrace,
            state.surfaceMeshContinuityReady
                ? &state.surfaceMeshContinuity
                : nullptr);
        state.normalFitAdjacencyStatistics =
            volume_surface::analyzeSurfaceTargetNormalAdjacency(
                *state.surfaceTargetCache,
                *baseField,
                seedSampleIndex);
        state.orientedNormalFieldReady = !state.orientedNormalField.empty();
        if (state.orientedNormalFieldReady) {
            state.orientedNormalAdjacencyStatistics =
                volume_surface::analyzeSurfaceTargetNormalAdjacency(
                    *state.surfaceTargetCache,
                    state.orientedNormalField,
                    seedSampleIndex);
        }
        state.orientedNormalStatus = state.orientedNormalFieldReady
            ? "Seed orientation applied: " +
                std::to_string(state.orientedNormalField.smoothedCoreCount) +
                " core / " +
                std::to_string(state.orientedNormalField.smoothedTransitionCount) +
                " transition samples"
            : "Seed orientation produced no valid normals";
        state.normalFieldPreviewDirty = true;
        return state.orientedNormalFieldReady;
    } catch (const std::exception& error) {
        state.orientedNormalStatus =
            std::string("Seed orientation failed: ") + error.what();
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
    if (state.surfaceTargetCache) {
        if (state.normalFitPreviewActive &&
            state.normalFitReady &&
            state.normalFitField.normals.size() == state.surfaceTargetCache->samples.size()) {
            inputs.normalOverrides = hasUsableOrientedNormalField(state)
                ? &state.orientedNormalField.normals
                : &state.normalFitField.normals;
        } else if (state.normalFieldPreviewActive &&
                   state.normalFieldPreviewSmoothed) {
            if (hasUsableOrientedNormalField(state)) {
                inputs.normalOverrides = &state.orientedNormalField.normals;
            } else if (state.normalFieldReady &&
                       state.normalField.normals.size() == state.surfaceTargetCache->samples.size()) {
                inputs.normalOverrides = &state.normalField.normals;
            }
        }
    }
    state.surfaceTargetPreview.rebuild(engine, scene, inputs);
    SurfaceTargetPointPicker::Inputs pickerInputs;
    pickerInputs.cache = state.surfaceTargetCache.get();
    pickerInputs.referenceCenter = inputs.referenceCenter;
    pickerInputs.voxelSize = inputs.voxelSize;
    pickerInputs.displayScale = inputs.displayScale;
    pickerInputs.pointScale = state.surfaceTargetPreview.settings().pointScale;
    pickerInputs.pointStride = state.surfaceTargetPreview.settings().pointStride;
    // Debug picking remains available even when the display-only point cloud is hidden.
    pickerInputs.includeCore = true;
    state.surfaceTargetPointPicker.rebuild(engine, pickerInputs);
    rebuildSurfaceFitExpansionDebug(state, engine, scene);
    state.normalFieldPreviewDirty = false;
    applyWorkflowPresentation(state, scene);
}

void rebuildSurfaceFitExpansionDebug(
    ViewerState& state,
    Engine& engine,
    Scene& scene)
{
    SurfaceFitExpansionDebugRenderer::Inputs inputs;
    volume_surface::SurfaceFitNeighborhoodInspection fitNeighborhood;
    inputs.cache = state.surfaceTargetCache.get();
    inputs.neighborhood = &state.surfaceFitExpansionNeighborhood;
    inputs.expansionTrace = &state.orientedNormalExpansionTrace;
    inputs.parentDepthThreshold = state.surfaceFitExpansionParentDepthThreshold;
    inputs.referenceCenter = openvdb::Vec3d{
        state.referenceCenter.x,
        state.referenceCenter.y,
        state.referenceCenter.z};
    inputs.voxelSize = state.grid
        ? state.grid->voxelSize()
        : openvdb::Vec3d{1.0, 1.0, 1.0};
    inputs.displayScale = state.displayScale;
    inputs.sourcePointScale = state.surfaceTargetPreview.settings().pointScale;
    if (state.surfaceTargetCache) {
        if (state.normalFitPreviewActive &&
            state.normalFitReady &&
            state.normalFitField.normals.size() == state.surfaceTargetCache->samples.size()) {
            inputs.normalOverrides = hasUsableOrientedNormalField(state)
                ? &state.orientedNormalField.normals
                : &state.normalFitField.normals;
        } else if (state.normalFieldPreviewActive &&
                   state.normalFieldPreviewSmoothed) {
            if (hasUsableOrientedNormalField(state)) {
                inputs.normalOverrides = &state.orientedNormalField.normals;
            } else if (state.normalFieldReady &&
                       state.normalField.normals.size() == state.surfaceTargetCache->samples.size()) {
                inputs.normalOverrides = &state.normalField.normals;
            }
        }
    }
    const bool needFitNeighborhoodInspection =
        state.surfaceFitExpansionDebugRenderer.settings().showUnifiedNeighborhood ||
        state.surfaceFitNeighborhoodDebugRenderer.settings().displayMode !=
            SurfaceFitNeighborhoodDisplayMode::None;
    if (needFitNeighborhoodInspection &&
        state.surfaceTargetCache &&
        state.surfaceFitNeighborhoodInspection.valid) {
        fitNeighborhood = state.surfaceFitNeighborhoodInspection;
        inputs.fitNeighborhood = &fitNeighborhood;
    } else if (state.surfaceFitExpansionDebugRenderer.settings().showUnifiedNeighborhood &&
               state.surfaceTargetCache &&
               state.surfaceFitExpansionNeighborhood.valid) {
        fitNeighborhood = volume_surface::inspectSurfaceFitNeighborhood(
            *state.surfaceTargetCache,
            state.surfaceFitExpansionNeighborhood.targetSampleIndex,
            state.normalFitSettings.neighborhood,
            state.normalFitSettings.robustIterations,
            state.surfaceMeshContinuityReady
                ? &state.surfaceMeshContinuity
                : nullptr);
        inputs.fitNeighborhood = &fitNeighborhood;
    }
    state.surfaceFitExpansionDebugRenderer.rebuild(engine, scene, inputs);
    state.surfaceFitExpansionDebugRenderer.setVisible(
        scene,
        state.workflowController.stage() == WorkflowStage::SurfaceFit);
    SurfaceFitNeighborhoodDebugRenderer::Inputs neighborhoodInputs;
    neighborhoodInputs.cache = state.surfaceTargetCache.get();
    neighborhoodInputs.inspection = inputs.fitNeighborhood;
    neighborhoodInputs.referenceCenter = inputs.referenceCenter;
    neighborhoodInputs.voxelSize = inputs.voxelSize;
    neighborhoodInputs.displayScale = inputs.displayScale;
    neighborhoodInputs.sourcePointScale = inputs.sourcePointScale;
    state.surfaceFitNeighborhoodDebugRenderer.rebuild(
        engine,
        scene,
        neighborhoodInputs);
    state.surfaceFitNeighborhoodDebugRenderer.setVisible(
        scene,
        state.workflowController.stage() == WorkflowStage::SurfaceFit);
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
        state.reconstructionProjectionVertexCount = 0;
        state.reconstructionProjectionRejectedCount = 0;
        state.reconstructionProjectionDensityRejectedCount = 0;
        state.reconstructionProjectionMaximumDisplacement = 0.0;
        state.reconstructionPickArmed = false;
        state.reconstructionPickRequested = false;
        state.reconstructionPickReport = {};
        state.reconstructionPickStatus = "No reconstruction point has been picked";
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
        const auto* normalOverrides = hasUsableOrientedNormalField(state)
            ? &state.orientedNormalField.normals
            : hasUsableNormalField(state)
                ? &state.normalField.normals
                : nullptr;
        auto result = volume_surface::reconstructSurfaceMLS(
            *state.grid,
            *state.surfaceTargetCache,
            state.reconstructionSettings,
            normalOverrides);
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
        state.reconstructionProjectionVertexCount = result.projectionVertexCount;
        state.reconstructionProjectionRejectedCount = result.projectionRejectedCount;
        state.reconstructionProjectionDensityRejectedCount =
            result.projectionDensityRejectedCount;
        state.reconstructionProjectionMaximumDisplacement =
            result.projectionMaximumDisplacement;
        state.reconstructionPickArmed = false;
        state.reconstructionPickRequested = false;
        state.reconstructionPickReport = {};
        state.reconstructionPickStatus = "No reconstruction point has been picked";
        state.reconstructionStatus =
            "Result A ready: " +
            std::to_string(resultSlot.mesh.vertices.size()) +
            " vertices / " +
            std::to_string(resultSlot.mesh.triangleCount()) +
            " triangles" +
            " (source topology + MLS projection)" +
            "; normals: " + reconstructionNormalSourceName(state);
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

bool surfaceTargetCoordinateLess(
    const openvdb::Coord& left,
    const openvdb::Coord& right)
{
    if (left.x() != right.x()) return left.x() < right.x();
    if (left.y() != right.y()) return left.y() < right.y();
    return left.z() < right.z();
}

bool findNearestSurfaceTargetSample(
    const volume_surface::SurfaceTargetCache& cache,
    const openvdb::FloatGrid& grid,
    const openvdb::Vec3d& worldPosition,
    volume_surface::viewer::ReconstructionPickReport& report)
{
    if (cache.samples.empty() || !worldPosition.isFinite()) {
        return false;
    }
    const openvdb::Vec3d indexPosition = grid.worldToIndex(worldPosition);
    if (!indexPosition.isFinite()) {
        return false;
    }
    const openvdb::Coord center = openvdb::Coord::round(indexPosition);
    constexpr int kSearchHalfExtent = 4;
    double nearestDistanceSquared = std::numeric_limits<double>::infinity();
    std::size_t nearestIndex = std::numeric_limits<std::size_t>::max();
    for (int dz = -kSearchHalfExtent; dz <= kSearchHalfExtent; ++dz) {
        for (int dy = -kSearchHalfExtent; dy <= kSearchHalfExtent; ++dy) {
            for (int dx = -kSearchHalfExtent; dx <= kSearchHalfExtent; ++dx) {
                const openvdb::Coord coordinate = center.offsetBy(dx, dy, dz);
                const auto iterator = std::lower_bound(
                    cache.samples.begin(),
                    cache.samples.end(),
                    coordinate,
                    [](const volume_surface::SurfaceTargetSample& sample,
                       const openvdb::Coord& value) {
                        return surfaceTargetCoordinateLess(sample.coordinate, value);
                    });
                if (iterator == cache.samples.end() ||
                    iterator->coordinate != coordinate) {
                    continue;
                }
                const std::size_t sampleIndex = static_cast<std::size_t>(
                    iterator - cache.samples.begin());
                const openvdb::Vec3d delta =
                    openvdb::Vec3d(iterator->worldPosition) - worldPosition;
                const double distanceSquared = delta.lengthSqr();
                if (distanceSquared < nearestDistanceSquared) {
                    nearestDistanceSquared = distanceSquared;
                    nearestIndex = sampleIndex;
                }
            }
        }
    }
    if (nearestIndex == std::numeric_limits<std::size_t>::max()) {
        return false;
    }
    report.targetSampleFound = true;
    report.targetSampleIndex = nearestIndex;
    report.targetSampleDistance = std::sqrt(nearestDistanceSquared);
    report.targetSample = cache.samples[nearestIndex];
    return true;
}

bool findNearestCoreSurfaceTargetSample(
    const volume_surface::SurfaceTargetCache& cache,
    const openvdb::FloatGrid& grid,
    const openvdb::Vec3d& worldPosition,
    std::size_t& sampleIndex,
    double& distance)
{
    if (cache.samples.empty() || !worldPosition.isFinite()) {
        return false;
    }
    const openvdb::Vec3d indexPosition = grid.worldToIndex(worldPosition);
    if (!indexPosition.isFinite()) {
        return false;
    }
    const openvdb::Coord center = openvdb::Coord::round(indexPosition);
    constexpr int kSearchHalfExtent = 4;
    double nearestDistanceSquared = std::numeric_limits<double>::infinity();
    std::size_t nearestIndex = std::numeric_limits<std::size_t>::max();
    for (int dz = -kSearchHalfExtent; dz <= kSearchHalfExtent; ++dz) {
        for (int dy = -kSearchHalfExtent; dy <= kSearchHalfExtent; ++dy) {
            for (int dx = -kSearchHalfExtent; dx <= kSearchHalfExtent; ++dx) {
                const openvdb::Coord coordinate = center.offsetBy(dx, dy, dz);
                const auto iterator = std::lower_bound(
                    cache.samples.begin(),
                    cache.samples.end(),
                    coordinate,
                    [](const volume_surface::SurfaceTargetSample& sample,
                       const openvdb::Coord& value) {
                        return surfaceTargetCoordinateLess(sample.coordinate, value);
                    });
                if (iterator == cache.samples.end() ||
                    iterator->coordinate != coordinate ||
                    iterator->kind != volume_surface::SurfaceTargetSampleKind::Core) {
                    continue;
                }
                const std::size_t index = static_cast<std::size_t>(
                    iterator - cache.samples.begin());
                const openvdb::Vec3d delta =
                    openvdb::Vec3d(iterator->worldPosition) - worldPosition;
                const double distanceSquared = delta.lengthSqr();
                if (distanceSquared < nearestDistanceSquared) {
                    nearestDistanceSquared = distanceSquared;
                    nearestIndex = index;
                }
            }
        }
    }
    if (nearestIndex == std::numeric_limits<std::size_t>::max()) {
        return false;
    }
    sampleIndex = nearestIndex;
    distance = std::sqrt(nearestDistanceSquared);
    return true;
}

void rebuildPickedSurfaceFitPlane(
    ViewerState& state,
    Engine& engine,
    Scene& scene)
{
    SurfaceFitPlaneRenderer::Inputs inputs;
    const auto& report = state.reconstructionPickReport;
    if (!report.hit) {
        state.surfaceFitPlaneRenderer.rebuild(engine, scene, inputs);
        return;
    }

    if (!report.targetSampleFound ||
        !state.surfaceTargetCache ||
        !state.normalFitReady ||
        state.normalFitField.normals.size() != state.surfaceTargetCache->samples.size()) {
        state.surfaceFitPlaneRenderer.rebuild(engine, scene, inputs);
        return;
    }

    inputs.valid = true;
    inputs.centerWorld = report.targetSampleFound
        ? openvdb::Vec3d(report.targetSample.worldPosition)
        : report.vdb.valid
            ? report.vdb.worldPosition
            : report.meshWorldPosition;
    inputs.normalWorld = report.targetSampleFound
        ? openvdb::Vec3d(report.targetSample.normal)
        : report.vdb.normal;
    const auto& displayedNormals = hasUsableOrientedNormalField(state)
        ? state.orientedNormalField.normals
        : state.normalFitField.normals;
    if (report.targetSampleIndex >= displayedNormals.size()) {
        state.surfaceFitPlaneRenderer.rebuild(engine, scene, {});
        return;
    }
    inputs.normalWorld = openvdb::Vec3d(displayedNormals[report.targetSampleIndex]);
    inputs.referenceCenter = openvdb::Vec3d{
        state.referenceCenter.x,
        state.referenceCenter.y,
        state.referenceCenter.z};
    inputs.voxelSize = state.grid
        ? state.grid->voxelSize()
        : openvdb::Vec3d{1.0, 1.0, 1.0};
    inputs.neighborhoodSide = normalFitNeighborhoodSide(state);
    inputs.displayScale = state.displayScale;
    state.surfaceFitPlaneRenderer.rebuild(engine, scene, inputs);
}

void processSurfaceFitPick(
    ViewerState& state,
    Engine& engine,
    Scene& scene,
    const View& view)
{
    if (!state.reconstructionPickRequested) {
        return;
    }
    state.reconstructionPickRequested = false;
    state.reconstructionPickReport = {};
    rebuildPickedSurfaceFitPlane(state, engine, scene);
    applyWorkflowPresentation(state, scene);
    if (state.workflowController.stage() != WorkflowStage::SurfaceFit ||
        !state.grid || !state.surfaceTargetCache ||
        state.surfaceTargetCache->empty()) {
        state.reconstructionPickStatus =
            "Pick requires the Surface Fit stage and a SurfaceTarget cache";
        return;
    }

    CameraPickRay ray;
    if (!buildCameraPickRay(
            view,
            state.reconstructionPickX,
            state.reconstructionPickY,
            ray)) {
        state.reconstructionPickStatus = "Unable to build a pick ray";
        return;
    }
    std::array<bool, 4> visibleSlots{};
    for (std::size_t index = 0; index < visibleSlots.size(); ++index) {
        visibleSlots[index] = state.slots[index].visible;
    }
    const CameraPickHit hit = state.cameraPickController.pick(ray, visibleSlots);
    if (!hit.hit) {
        state.reconstructionPickStatus = "No visible mesh was hit";
        return;
    }

    auto& report = state.reconstructionPickReport;
    report.hit = true;
    report.slotIndex = hit.slotIndex;
    report.triangleIndex = hit.triangleIndex;
    report.barycentric = hit.barycentric;
    report.rayDirection = ray.direction;
    report.scenePosition = hit.scenePosition;
    report.meshWorldPosition = state.context.coordinates.toWorld(
        filament::math::float3{
            static_cast<float>(hit.scenePosition.x()),
            static_cast<float>(hit.scenePosition.y()),
            static_cast<float>(hit.scenePosition.z())});

    volume_surface::VdbSurfaceProbeSettings probeSettings;
    probeSettings.isoValue = state.isoValue;
    report.vdb = volume_surface::projectVdbSurface(
        *state.grid,
        report.meshWorldPosition,
        probeSettings);
    report.normalUsedByReconstruction = reconstructionNormalSourceName(state);
    const openvdb::Vec3d targetPosition = report.vdb.valid
        ? report.vdb.worldPosition
        : report.meshWorldPosition;
    findNearestSurfaceTargetSample(
        *state.surfaceTargetCache,
        *state.grid,
        targetPosition,
        report);
    if (report.targetSampleFound &&
        (hasUsableOrientedNormalField(state) || hasUsableNormalField(state))) {
        const auto& normals = hasUsableOrientedNormalField(state)
            ? state.orientedNormalField.normals
            : state.normalField.normals;
        report.smoothedNormal = openvdb::Vec3d(normals[report.targetSampleIndex]);
        report.smoothedNormalFound = report.smoothedNormal.lengthSqr() > 1.0e-20;
    }
    state.reconstructionPickStatus = report.vdb.converged
        ? "Picked mesh and projected it to the VDB iso-surface"
        : "Picked mesh; VDB projection did not converge, original point retained";
    rebuildPickedSurfaceFitPlane(state, engine, scene);
    applyWorkflowPresentation(state, scene);
}

void processSurfaceFitExpansionPick(
    ViewerState& state,
    Engine& engine,
    Scene& scene,
    const View& view)
{
    const auto gpuResult = state.surfaceTargetPointPicker.consumeResult();
    if (!gpuResult.ready && !state.surfaceFitExpansionPickRequested) {
        return;
    }
    const bool neighborhoodPick = state.surfaceFitNeighborhoodPickArmed ||
        state.surfaceFitNeighborhoodPickRequested;
    if (state.workflowController.stage() != WorkflowStage::SurfaceFit ||
        !state.grid || !state.surfaceTargetCache ||
        state.surfaceTargetCache->empty() ||
        (!neighborhoodPick &&
            !state.orientedNormalExpansionTrace.matchesSampleCount(
                state.surfaceTargetCache->samples.size())) ||
        (neighborhoodPick && !state.normalFitReady)) {
        state.surfaceFitExpansionPickRequested = false;
        if (neighborhoodPick) {
            state.surfaceFitNeighborhoodPickArmed = false;
            state.surfaceFitNeighborhoodPickRequested = false;
            state.surfaceFitNeighborhoodStatus =
                "Fit neighborhood debug requires completed Surface Fit";
        } else {
            state.surfaceFitExpansionStatus =
                "Debug pick requires completed fitted normal expansion";
        }
        return;
    }

    auto completePick = [&](std::size_t targetSampleIndex, double targetDistance,
                            bool gpu) {
        if (targetSampleIndex >= state.surfaceTargetCache->samples.size() ||
            state.surfaceTargetCache->samples[targetSampleIndex].kind !=
                volume_surface::SurfaceTargetSampleKind::Core) {
            if (neighborhoodPick) {
                state.surfaceFitNeighborhoodPickArmed = false;
                state.surfaceFitNeighborhoodPickRequested = false;
                state.surfaceFitNeighborhoodStatus =
                    "GPU pick returned an invalid Core sample";
            } else {
                state.surfaceFitExpansionStatus =
                    "GPU pick returned an invalid Core sample";
            }
            return;
        }
        if (neighborhoodPick) {
            state.surfaceFitNeighborhoodInspection =
                volume_surface::inspectSurfaceFitNeighborhood(
                    *state.surfaceTargetCache,
                    targetSampleIndex,
                    state.normalFitSettings.neighborhood,
                    state.normalFitSettings.robustIterations,
                    state.surfaceMeshContinuityReady
                        ? &state.surfaceMeshContinuity
                        : nullptr);
            state.surfaceFitNeighborhoodPickArmed = false;
            state.surfaceFitNeighborhoodPickRequested = false;
            if (!state.surfaceFitNeighborhoodInspection.valid) {
                state.surfaceFitNeighborhoodStatus =
                    "The selected Core point has no fit neighborhood";
                rebuildSurfaceFitExpansionDebug(state, engine, scene);
                applyWorkflowPresentation(state, scene);
                return;
            }
            const auto& sample = state.surfaceTargetCache->samples[targetSampleIndex];
            state.surfaceFitNeighborhoodStatus =
                "Fit center selected at (" +
                std::to_string(sample.coordinate.x()) + "," +
                std::to_string(sample.coordinate.y()) + "," +
                std::to_string(sample.coordinate.z()) + ") " +
                (gpu ? "via GPU point pick" :
                    "distance " + std::to_string(targetDistance * 1000.0) + " mm");
            rebuildSurfaceFitExpansionDebug(state, engine, scene);
            applyWorkflowPresentation(state, scene);
            return;
        }
        state.surfaceFitExpansionNeighborhood =
            volume_surface::inspectSurfaceNormalExpansion(
                *state.surfaceTargetCache,
                state.orientedNormalExpansionTrace,
                targetSampleIndex);
        if (!state.surfaceFitExpansionNeighborhood.valid) {
            state.surfaceFitExpansionStatus =
                "The selected Core point has no recorded expansion state";
            return;
        }
        state.surfaceFitExpansionDebugRenderer.settings().showNeighborhood = true;
        state.surfaceFitExpansionParentDepthThreshold = 0;
        const auto& sample = state.surfaceTargetCache->samples[targetSampleIndex];
        state.surfaceFitExpansionStatus =
            "Debug Core selected at (" +
            std::to_string(sample.coordinate.x()) + "," +
            std::to_string(sample.coordinate.y()) + "," +
            std::to_string(sample.coordinate.z()) + ") " +
            (gpu
                ? "via GPU point pick"
                : "distance " + std::to_string(targetDistance * 1000.0) + " mm");
        rebuildSurfaceFitExpansionDebug(state, engine, scene);
        applyWorkflowPresentation(state, scene);
    };

    if (gpuResult.ready && gpuResult.hit) {
        state.surfaceFitExpansionPickRequested = false;
        completePick(gpuResult.sampleIndex, 0.0, true);
        return;
    }

    if (!gpuResult.ready && state.surfaceFitExpansionPickRequested) {
        if (state.surfaceTargetPointPicker.hasPendingRequest()) {
            state.surfaceFitExpansionPickRequested = false;
            if (neighborhoodPick) {
                state.surfaceFitNeighborhoodStatus = "GPU point pick pending";
            } else {
                state.surfaceFitExpansionStatus = "GPU point pick pending";
            }
            return;
        }
        const auto viewport = view.getViewport();
        const ImGuiIO& io = ImGui::GetIO();
        const double scaleX = io.DisplaySize.x > 0.0f
            ? static_cast<double>(viewport.width) / io.DisplaySize.x
            : static_cast<double>(io.DisplayFramebufferScale.x);
        const double scaleY = io.DisplaySize.y > 0.0f
            ? static_cast<double>(viewport.height) / io.DisplaySize.y
            : static_cast<double>(io.DisplayFramebufferScale.y);
        if (std::isfinite(scaleX) && std::isfinite(scaleY) &&
            scaleX > 0.0 && scaleY > 0.0 &&
            state.surfaceTargetPointPicker.request(
                state.surfaceFitExpansionPickX,
                state.surfaceFitExpansionPickY,
                scaleX,
                scaleY)) {
            state.surfaceFitExpansionPickRequested = false;
            if (neighborhoodPick) {
                state.surfaceFitNeighborhoodStatus = "GPU point pick pending";
            } else {
                state.surfaceFitExpansionStatus = "GPU point pick pending";
            }
            return;
        }
    }

    state.surfaceFitExpansionPickRequested = false;
    CameraPickRay ray;
    if (!buildCameraPickRay(
            view,
            state.surfaceFitExpansionPickX,
            state.surfaceFitExpansionPickY,
            ray)) {
        if (neighborhoodPick) {
            state.surfaceFitNeighborhoodPickArmed = false;
            state.surfaceFitNeighborhoodPickRequested = false;
            state.surfaceFitNeighborhoodStatus =
                "Unable to build the fit neighborhood pick ray";
        } else {
            state.surfaceFitExpansionStatus = "Unable to build the debug pick ray";
        }
        return;
    }

    std::size_t targetSampleIndex = std::numeric_limits<std::size_t>::max();
    double targetDistance = 0.0;
    std::array<bool, 4> visibleSlots{};
    for (std::size_t index = 0; index < visibleSlots.size(); ++index) {
        visibleSlots[index] = state.slots[index].visible;
    }
    const CameraPickHit hit = state.cameraPickController.pick(ray, visibleSlots);
    if (!hit.hit) {
        if (neighborhoodPick) {
            state.surfaceFitNeighborhoodPickArmed = false;
            state.surfaceFitNeighborhoodPickRequested = false;
            state.surfaceFitNeighborhoodStatus =
                "No Core point or visible surface was hit for the fit neighborhood";
        } else {
            state.surfaceFitExpansionStatus =
                "No GPU Core point or visible surface was hit for the debug point";
        }
        return;
    }

    const openvdb::Vec3d meshWorldPosition = state.context.coordinates.toWorld(
        float3{
            static_cast<float>(hit.scenePosition.x()),
            static_cast<float>(hit.scenePosition.y()),
            static_cast<float>(hit.scenePosition.z())});
    volume_surface::VdbSurfaceProbeSettings probeSettings;
    probeSettings.isoValue = state.isoValue;
    const auto vdbSurface = volume_surface::projectVdbSurface(
        *state.grid,
        meshWorldPosition,
        probeSettings);
    const openvdb::Vec3d targetPosition = vdbSurface.valid
        ? vdbSurface.worldPosition
        : meshWorldPosition;
    if (!findNearestCoreSurfaceTargetSample(
            *state.surfaceTargetCache,
            *state.grid,
            targetPosition,
            targetSampleIndex,
            targetDistance)) {
        if (neighborhoodPick) {
            state.surfaceFitNeighborhoodPickArmed = false;
            state.surfaceFitNeighborhoodPickRequested = false;
            state.surfaceFitNeighborhoodStatus =
                "The pick is not near a Core SurfaceTarget sample";
        } else {
            state.surfaceFitExpansionStatus =
                "The debug pick is not near a Core SurfaceTarget sample";
        }
        return;
    }
    completePick(targetSampleIndex, targetDistance, false);
}

void processOrientationSeedPick(
    ViewerState& state,
    const View& view)
{
    if (!state.orientationSeedPickRequested) {
        return;
    }
    state.orientationSeedPickRequested = false;
    state.orientationSeedPickArmed = false;
    if (state.workflowController.stage() != WorkflowStage::SurfaceFit ||
        !state.grid || !state.surfaceTargetCache ||
        state.surfaceTargetCache->empty()) {
        state.surfaceNormalSeedStatus =
            "Seed pick requires a SurfaceTarget cache";
        return;
    }

    CameraPickRay ray;
    if (!buildCameraPickRay(
            view,
            state.orientationSeedPickX,
            state.orientationSeedPickY,
            ray)) {
        state.surfaceNormalSeedStatus = "Unable to build the seed pick ray";
        return;
    }
    std::array<bool, 4> visibleSlots{};
    for (std::size_t index = 0; index < visibleSlots.size(); ++index) {
        visibleSlots[index] = state.slots[index].visible;
    }
    const CameraPickHit hit = state.cameraPickController.pick(ray, visibleSlots);
    if (!hit.hit) {
        state.surfaceNormalSeedStatus = "No visible mesh was hit for the seed";
        return;
    }

    const openvdb::Vec3d meshWorldPosition = state.context.coordinates.toWorld(
        filament::math::float3{
            static_cast<float>(hit.scenePosition.x()),
            static_cast<float>(hit.scenePosition.y()),
            static_cast<float>(hit.scenePosition.z())});
    volume_surface::VdbSurfaceProbeSettings probeSettings;
    probeSettings.isoValue = state.isoValue;
    const auto vdbSurface = volume_surface::projectVdbSurface(
        *state.grid,
        meshWorldPosition,
        probeSettings);
    const openvdb::Vec3d targetPosition = vdbSurface.valid
        ? vdbSurface.worldPosition
        : meshWorldPosition;
    volume_surface::viewer::ReconstructionPickReport targetReport;
    if (!findNearestSurfaceTargetSample(
            *state.surfaceTargetCache,
            *state.grid,
            targetPosition,
            targetReport)) {
        state.surfaceNormalSeedStatus =
            "The picked point is not near a SurfaceTarget sample";
        return;
    }

    const double rayLength = ray.direction.length();
    const openvdb::Vec3d outward = rayLength > 1.0e-12 &&
            std::isfinite(rayLength)
        ? -ray.direction / rayLength
        : openvdb::Vec3d(0.0);
    if (outward.lengthSqr() <= 1.0e-20) {
        state.surfaceNormalSeedStatus = "The seed pick ray has no valid direction";
        return;
    }
    state.surfaceNormalSeed = {};
    state.surfaceNormalSeed.valid = true;
    state.surfaceNormalSeed.coordinate = targetReport.targetSample.coordinate;
    state.surfaceNormalSeed.worldPosition =
        openvdb::Vec3d(targetReport.targetSample.worldPosition);
    state.surfaceNormalSeed.normal = outward;
    state.surfaceNormalSeed.rayDirection = ray.direction;
    if (saveSurfaceNormalSeedToDisk(state)) {
        state.surfaceNormalSeedStatus =
            "Seed selected from outside ray and saved automatically";
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
        "The source stage is read-only. Use Surface Target to inspect the extracted surface data before editing downstream data.");
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
        if (state.surfaceMeshContinuityReady) {
            ImGui::Text(
                "Mesh continuity: %zu / %zu samples (%.1f ms)",
                state.surfaceMeshContinuity.supportedSampleCount,
                state.surfaceTargetCache->samples.size(),
                state.surfaceMeshContinuityBuildMilliseconds);
        } else {
            ImGui::TextUnformatted("Mesh continuity: unavailable");
        }
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
    bool materialSettingsChanged = false;
    float pointScale = previewSettings.pointScale;
    if (ImGui::SliderFloat("Point render scale", &pointScale, 0.25f, 4.0f, "%.2f")) {
        previewSettings.pointScale = pointScale;
        materialSettingsChanged = true;
    }
    int pointStride = static_cast<int>(std::clamp<std::size_t>(
        previewSettings.pointStride,
        1,
        32));
    if (ImGui::SliderInt("Point sample stride", &pointStride, 1, 32)) {
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
        state.surfaceTargetPointPicker.updatePointScale(previewSettings.pointScale);
        rebuildSurfaceFitExpansionDebug(state, engine, scene);
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

void drawSurfaceFitWindow(ViewerState& state, Engine& engine, Scene& scene)
{
    ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(390.0f, 860.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(1.0f);
    ImGui::Begin("Surface Fit / Normal Seed");
    state.mouseOverUi |= ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);
    ImGui::TextWrapped(
        "Fit a local surface trend from the connected SurfaceTarget samples and generate seed normals.");
    ImGui::TextWrapped(
        "The neighborhood is selected in index space; no physical-radius resampling is used.");
    ImGui::Separator();

    auto& settings = state.normalFitSettings;
    const char* neighborhoodLabels[] = {"3 x 3", "5 x 5", "9 x 9"};
    int neighborhoodIndex = settings.neighborhood ==
            volume_surface::SurfaceFitNeighborhood::Grid5x5
        ? 1
        : settings.neighborhood == volume_surface::SurfaceFitNeighborhood::Grid9x9
            ? 2
            : 0;
    if (ImGui::Combo(
            "Fit neighborhood",
            &neighborhoodIndex,
            neighborhoodLabels,
            3)) {
        settings.neighborhood = neighborhoodIndex == 2
            ? volume_surface::SurfaceFitNeighborhood::Grid9x9
            : neighborhoodIndex == 1
                ? volume_surface::SurfaceFitNeighborhood::Grid5x5
                : volume_surface::SurfaceFitNeighborhood::Grid3x3;
        clearSurfaceFitExpansionSelection(state);
        state.normalFitReady = false;
        state.normalFitField = {};
        state.normalFieldReady = false;
        state.normalField = {};
        state.orientedNormalFieldReady = false;
        state.orientedNormalField = {};
        state.orientedNormalStatus = "Orientation seed must be reapplied after fitting";
        state.normalFitStatus = "Surface fit settings changed; build the selected mode";
        state.normalFieldStatus = "Normal field requires the updated fitted seed field";
        state.normalFieldPreviewDirty = true;
        rebuildPickedSurfaceFitPlane(state, engine, scene);
        applyWorkflowPresentation(state, scene);
    }
    int robustIterations = static_cast<int>(std::clamp<std::size_t>(
        settings.robustIterations,
        1,
        5));
    if (ImGui::SliderInt("Fit robust iterations", &robustIterations, 1, 5)) {
        settings.robustIterations = static_cast<std::size_t>(robustIterations);
        clearSurfaceFitExpansionSelection(state);
        state.normalFitReady = false;
        state.normalFitField = {};
        state.normalFieldReady = false;
        state.normalField = {};
        state.orientedNormalFieldReady = false;
        state.orientedNormalField = {};
        state.orientedNormalStatus = "Orientation seed must be reapplied after fitting";
        state.normalFitStatus = "Surface fit settings changed; build the selected mode";
        state.normalFieldStatus = "Normal field requires the updated fitted seed field";
        state.normalFieldPreviewDirty = true;
        rebuildPickedSurfaceFitPlane(state, engine, scene);
        applyWorkflowPresentation(state, scene);
    }
    ImGui::Text(
        "Support: %s connected samples",
        neighborhoodIndex == 2 ? "up to 81" : neighborhoodIndex == 1 ? "up to 25" : "up to 9");

    ImGui::Separator();
    ImGui::TextUnformatted("Pick fitted surface plane");
    if (ImGui::Button(
            state.reconstructionPickArmed
                ? "Click a surface point"
                : "Pick surface point")) {
        state.reconstructionPickArmed = true;
        state.reconstructionPickStatus =
            "Click a visible mesh in the viewer";
    }
    ImGui::SameLine();
    ImGui::TextWrapped("%s", state.reconstructionPickStatus.c_str());

    auto& planeSettings = state.surfaceFitPlaneRenderer.settings();
    bool planeSettingsChanged = ImGui::Checkbox(
        "Show fitted plane depth test",
        &planeSettings.showPlane);
    float planeDepthBias = planeSettings.depthBias;
    if (ImGui::SliderFloat(
            "Plane depth bias",
            &planeDepthBias,
            -4.0f,
            4.0f,
            "%.2f",
            ImGuiSliderFlags_AlwaysClamp)) {
        planeSettings.depthBias = planeDepthBias;
        planeSettingsChanged = true;
    }
    ImGui::Text(
        "Plane size follows the selected %s neighborhood",
        neighborhoodLabels[neighborhoodIndex]);
    if (planeSettingsChanged) {
        rebuildPickedSurfaceFitPlane(state, engine, scene);
        state.surfaceFitPlaneRenderer.updateMaterialSettings(engine);
        applyWorkflowPresentation(state, scene);
    }
    if (state.reconstructionPickReport.hit) {
        const auto& report = state.reconstructionPickReport;
        ImGui::Text(
            "Pick: slot %zu / triangle %zu",
            report.slotIndex,
            report.triangleIndex);
        ImGui::Text(
            "Mesh world: %.3f, %.3f, %.3f mm",
            report.meshWorldPosition.x() * 1000.0,
            report.meshWorldPosition.y() * 1000.0,
            report.meshWorldPosition.z() * 1000.0);
        if (report.vdb.valid) {
            ImGui::Text(
                "VDB iso: %.3f, %.3f, %.3f mm (residual %.3f)",
                report.vdb.worldPosition.x() * 1000.0,
                report.vdb.worldPosition.y() * 1000.0,
                report.vdb.worldPosition.z() * 1000.0,
                report.vdb.isoResidual);
            ImGui::Text(
                "Index: %.3f, %.3f, %.3f | displacement %.3f mm",
                report.vdb.indexPosition.x(),
                report.vdb.indexPosition.y(),
                report.vdb.indexPosition.z(),
                report.vdb.displacement * 1000.0);
            ImGui::Text(
                "Nearest voxel: (%d, %d, %d)",
                report.vdb.nearestVoxel.x(),
                report.vdb.nearestVoxel.y(),
                report.vdb.nearestVoxel.z());
        } else {
            ImGui::TextUnformatted("VDB iso projection: unavailable");
        }
        if (report.targetSampleFound) {
            ImGui::Text(
                "Target: %s (%d, %d, %d), %.3f mm away",
                report.targetSample.kind == volume_surface::SurfaceTargetSampleKind::Core
                    ? "Core"
                    : "Transition",
                report.targetSample.coordinate.x(),
                report.targetSample.coordinate.y(),
                report.targetSample.coordinate.z(),
                report.targetSampleDistance * 1000.0);
        } else {
            ImGui::TextUnformatted("Target sample: not found in local cache window");
        }
        if (ImGui::Button("Copy VDB location")) {
            const std::string reportText =
                volume_surface::viewer::formatReconstructionPickReportJsonl(
                    report,
                    state.input.string(),
                    state.gridName,
                    state.isoValue);
            ImGui::SetClipboardText(reportText.c_str());
            state.reconstructionPickStatus = "VDB location copied as JSONL";
        }
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Orientation seed");
    ImGui::TextWrapped(
        "Pick from outside the model. The seed normal defaults to the opposite of the camera ray.");
    if (ImGui::Button(
            state.orientationSeedPickArmed
                ? "Click an external surface point"
                : "Pick orientation seed")) {
        state.orientationSeedPickArmed = true;
        state.reconstructionPickArmed = false;
        state.surfaceNormalSeedStatus =
            "Click a visible mesh from outside the model";
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!state.surfaceNormalSeed.valid);
    if (ImGui::Button("Save seed")) {
        saveSurfaceNormalSeedToDisk(state);
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Load seed")) {
        loadSurfaceNormalSeedFromDisk(state);
    }
    ImGui::TextWrapped("Seed file: %s",
        state.surfaceNormalSeedPath.empty()
            ? "not assigned"
            : state.surfaceNormalSeedPath.string().c_str());
    ImGui::TextWrapped("Seed status: %s", state.surfaceNormalSeedStatus.c_str());
    if (state.surfaceNormalSeed.valid) {
        ImGui::Text(
            "Seed index: (%d, %d, %d)",
            state.surfaceNormalSeed.coordinate.x(),
            state.surfaceNormalSeed.coordinate.y(),
            state.surfaceNormalSeed.coordinate.z());
        ImGui::Text(
            "Seed normal: %.3f, %.3f, %.3f",
            state.surfaceNormalSeed.normal.x(),
            state.surfaceNormalSeed.normal.y(),
            state.surfaceNormalSeed.normal.z());
        ImGui::TextWrapped("Orientation: %s", state.orientedNormalStatus.c_str());
        if (hasUsableOrientedNormalField(state) && state.surfaceTargetCache) {
            for (std::size_t index = 0;
                 index < state.surfaceTargetCache->samples.size();
                 ++index) {
                if (state.surfaceTargetCache->samples[index].coordinate !=
                    state.surfaceNormalSeed.coordinate) {
                    continue;
                }
                const openvdb::Vec3d appliedNormal(
                    state.orientedNormalField.normals[index]);
                const openvdb::Vec3d requestedNormal =
                    state.surfaceNormalSeed.normal.unit();
                ImGui::Text(
                    "Applied at seed: %.3f, %.3f, %.3f (dot %.3f)",
                    appliedNormal.x(),
                    appliedNormal.y(),
                    appliedNormal.z(),
                    appliedNormal.dot(requestedNormal));
                break;
            }
        }
        const auto& beforeStats = state.normalFitAdjacencyStatistics;
        const auto& afterStats = state.orientedNormalAdjacencyStatistics;
        if (afterStats.adjacencyEdgeCount > 0) {
            ImGui::Text(
                "Valid normals: %zu core / %zu transition",
                afterStats.validCoreSampleCount,
                afterStats.validTransitionSampleCount);
            ImGui::Text(
                "Core adjacency edges: %zu | opposing: %zu -> %zu",
                afterStats.adjacencyEdgeCount,
                beforeStats.opposingEdgeCount,
                afterStats.opposingEdgeCount);
            ImGui::Text(
                "Weak edges: %zu | components: %zu | unseeded samples: %zu",
                afterStats.weakEdgeCount,
                afterStats.connectedComponentCount,
                afterStats.unseededComponentSampleCount);
            ImGui::Text(
                "Transition-linked edges: %zu | opposing: %zu",
                afterStats.transitionAdjacencyEdgeCount,
                afterStats.transitionOpposingEdgeCount);
            ImGui::Text(
                "Signed dot mean/min: %.3f / %.3f",
                afterStats.meanSignedAlignment,
                afterStats.minimumSignedAlignment);
        }
    }

    if (ImGui::Button("Build / update fitted normals")) {
        rebuildSurfaceNormalFit(state);
        rebuildOrientedNormalField(state);
        rebuildPickedSurfaceFitPlane(state, engine, scene);
        state.normalFieldPreviewDirty = true;
    }
    ImGui::SameLine();
    ImGui::TextWrapped("%s", state.normalFitStatus.c_str());
    bool previewChanged = ImGui::Checkbox(
        "Show fitted normal vectors",
        &state.surfaceTargetPreview.settings().showNormals);
    previewChanged |= ImGui::Checkbox(
        "Show points",
        &state.surfaceTargetPreview.settings().showPoints);
    if (previewChanged) {
        state.normalFieldPreviewDirty = true;
    }
    ImGui::Separator();
    ImGui::TextUnformatted("Fit neighborhood filter debug");
    if (ImGui::Button(
            state.surfaceFitNeighborhoodPickArmed
                ? "Click a Core point"
                : "Pick fit center")) {
        state.surfaceFitNeighborhoodPickArmed = true;
        state.surfaceFitNeighborhoodPickRequested = false;
        state.surfaceFitExpansionPickArmed = false;
        state.reconstructionPickArmed = false;
        state.orientationSeedPickArmed = false;
        state.surfaceFitNeighborhoodStatus =
            "Click a visible Core point in the viewer";
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear fit debug")) {
        state.surfaceFitNeighborhoodInspection = {};
        state.surfaceFitNeighborhoodPickArmed = false;
        state.surfaceFitNeighborhoodPickRequested = false;
        state.surfaceFitNeighborhoodStatus = "No fit neighborhood center selected";
        rebuildSurfaceFitExpansionDebug(state, engine, scene);
        applyWorkflowPresentation(state, scene);
    }
    const char* fitDebugModeLabels[] = {
        "None",
        "All candidates",
        "Kept / weighted",
        "Rejected",
        "Weight heatmap"};
    int fitDebugMode = static_cast<int>(
        state.surfaceFitNeighborhoodDebugRenderer.settings().displayMode);
    if (ImGui::Combo(
            "Display filter result",
            &fitDebugMode,
            fitDebugModeLabels,
            static_cast<int>(sizeof(fitDebugModeLabels) / sizeof(fitDebugModeLabels[0])))) {
        fitDebugMode = std::clamp(fitDebugMode, 0, 4);
        state.surfaceFitNeighborhoodDebugRenderer.settings().displayMode =
            static_cast<SurfaceFitNeighborhoodDisplayMode>(fitDebugMode);
        rebuildSurfaceFitExpansionDebug(state, engine, scene);
        applyWorkflowPresentation(state, scene);
    }
    ImGui::TextWrapped(
        "Blue: candidates. Green/orange: kept and downweighted. Red/purple: rejected by topology or plane residual.");
    ImGui::TextWrapped("Status: %s", state.surfaceFitNeighborhoodStatus.c_str());
    const auto& fitInspection = state.surfaceFitNeighborhoodInspection;
    if (fitInspection.valid && state.surfaceTargetCache) {
        const auto& centerSample = state.surfaceTargetCache->samples[
            fitInspection.centerSampleIndex];
        ImGui::Text(
            "Center: (%d, %d, %d)",
            centerSample.coordinate.x(),
            centerSample.coordinate.y(),
            centerSample.coordinate.z());
        ImGui::Text(
            "Candidates: %zu | kept: %zu | downweighted: %zu | rejected: %zu",
            fitInspection.samples.size(),
            fitInspection.keptSampleCount,
            fitInspection.downweightedSampleCount,
            fitInspection.rejectedSampleCount);
        ImGui::Text(
            "Plane residual scale: %.6f mm",
            fitInspection.fitResidualScale * 1000.0);
    }
    ImGui::Separator();
    ImGui::TextUnformatted("26-neighborhood expansion debug");
    if (ImGui::Button(
            state.surfaceFitExpansionPickArmed
                ? "Click a Core point"
                : "\xE9\x80\x89\xE5\x8F\x96 debug point")) {
        state.surfaceFitExpansionPickArmed = true;
        state.reconstructionPickArmed = false;
        state.orientationSeedPickArmed = false;
        state.surfaceFitExpansionStatus = "Click a visible Core point in the viewer";
    }
    ImGui::SameLine();
    if (ImGui::Button("\xE6\xB8\x85\xE9\x99\xA4 pick")) {
        clearSurfaceFitExpansionSelection(state);
        rebuildSurfaceFitExpansionDebug(state, engine, scene);
        applyWorkflowPresentation(state, scene);
    }
    const bool showUnifiedNeighborhood =
        state.surfaceFitExpansionDebugRenderer.settings().showUnifiedNeighborhood;
    const bool canToggleNeighborhoodColors =
        state.surfaceFitExpansionNeighborhood.valid;
    if (!canToggleNeighborhoodColors) {
        ImGui::BeginDisabled();
    }
    if (ImGui::Button(
        showUnifiedNeighborhood
            ? "\xE6\x81\xA2\xE5\xA4\x8D\xE5\x88\x86\xE7\xBB\x84\xE9\xA2\x9C\xE8\x89\xB2"
            : "\xE6\x98\xBE\xE7\xA4\xBA\xE7\xBB\x9F\xE4\xB8\x80\xE9\x82\xBB\xE5\x9F\x9F")) {
        state.surfaceFitExpansionDebugRenderer.settings().showUnifiedNeighborhood =
            !showUnifiedNeighborhood;
        rebuildSurfaceFitExpansionDebug(state, engine, scene);
        applyWorkflowPresentation(state, scene);
    }
    if (!canToggleNeighborhoodColors) {
        ImGui::EndDisabled();
    }
    const bool showCurrentPointNormals =
        state.surfaceFitExpansionDebugRenderer.settings().showCurrentPointNormals;
    if (!canToggleNeighborhoodColors) {
        ImGui::BeginDisabled();
    }
    if (ImGui::Button(
            showCurrentPointNormals
                ? "Hide current point normals"
                : "Show current point normals")) {
        state.surfaceFitExpansionDebugRenderer.settings().showCurrentPointNormals =
            !showCurrentPointNormals;
        rebuildSurfaceFitExpansionDebug(state, engine, scene);
        applyWorkflowPresentation(state, scene);
    }
    if (!canToggleNeighborhoodColors) {
        ImGui::EndDisabled();
    }
    ImGui::TextWrapped("%s", state.surfaceFitExpansionStatus.c_str());
    const auto& expansionNeighborhood = state.surfaceFitExpansionNeighborhood;
    if (expansionNeighborhood.valid && state.surfaceTargetCache) {
        const int maximumParentDepth = std::max(0, expansionNeighborhood.targetDepth);
        int parentDepthThreshold = std::clamp(
            state.surfaceFitExpansionParentDepthThreshold,
            0,
            maximumParentDepth);
        if (maximumParentDepth > 0) {
            if (ImGui::SliderInt(
                    "Parent depth >=",
                    &parentDepthThreshold,
                    0,
                    maximumParentDepth,
                    "%d")) {
                state.surfaceFitExpansionParentDepthThreshold = parentDepthThreshold;
                rebuildSurfaceFitExpansionDebug(state, engine, scene);
                applyWorkflowPresentation(state, scene);
            } else {
                state.surfaceFitExpansionParentDepthThreshold = parentDepthThreshold;
            }
        } else {
            state.surfaceFitExpansionParentDepthThreshold = 0;
            ImGui::TextUnformatted("Parent depth >= 0 (seed point)");
        }
        const auto& targetSample = state.surfaceTargetCache->samples[
            expansionNeighborhood.targetSampleIndex];
        ImGui::Text(
            "Target index: (%d, %d, %d), depth %d",
            targetSample.coordinate.x(),
            targetSample.coordinate.y(),
            targetSample.coordinate.z(),
            expansionNeighborhood.targetDepth);
        ImGui::Text(
            "Target world: %.3f, %.3f, %.3f mm",
            targetSample.worldPosition.x() * 1000.0,
            targetSample.worldPosition.y() * 1000.0,
            targetSample.worldPosition.z() * 1000.0);
        if (expansionNeighborhood.sourceSampleIndex !=
            std::numeric_limits<std::size_t>::max()) {
            const auto& sourceSample = state.surfaceTargetCache->samples[
                expansionNeighborhood.sourceSampleIndex];
            ImGui::Text(
                "Source index: (%d, %d, %d)",
                sourceSample.coordinate.x(),
                sourceSample.coordinate.y(),
                sourceSample.coordinate.z());
            ImGui::Text(
                "Source world: %.3f, %.3f, %.3f mm",
                sourceSample.worldPosition.x() * 1000.0,
                sourceSample.worldPosition.y() * 1000.0,
                sourceSample.worldPosition.z() * 1000.0);
        } else {
            ImGui::TextUnformatted("Source: seed / none");
        }
        ImGui::Text(
            "Next batch: %zu | Source batch: %zu",
            expansionNeighborhood.targetNextSampleIndices.size(),
            expansionNeighborhood.sourceBatchSampleIndices.size());
        ImGui::Text(
            "Parent chain depth >= %d: %zu",
            state.surfaceFitExpansionParentDepthThreshold,
            state.surfaceFitExpansionDebugRenderer.statistics().parentPointCount);
    }
    ImGui::Text(
        "Fitted samples: %zu core / %zu transition",
        state.normalFitField.smoothedCoreCount,
        state.normalFitField.smoothedTransitionCount);
    ImGui::TextWrapped(
        "This stage produces the seed field consumed by Surface Normal.");
    ImGui::TextWrapped(
        "Orientation propagation uses the source mesh topology continuity cache when available.");
    ImGui::TextWrapped(
        "Fitted normals use only sample positions and index connectivity; their sign is not corrected from the source normal or VDB values.");
    ImGui::TextWrapped("Status: %s", state.status.c_str());
    if (state.normalFieldPreviewDirty) {
        rebuildPickedSurfaceFitPlane(state, engine, scene);
        rebuildSurfaceTargetPreview(state, engine, scene);
    }
    ImGui::End();
}

void drawNormalFieldWindow(ViewerState& state, Engine& engine, Scene& scene)
{
    ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(390.0f, 560.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(1.0f);
    ImGui::Begin("Surface Normal");
    state.mouseOverUi |= ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);
    ImGui::TextWrapped(
        "Optionally average the fitted surface normals without moving the SurfaceTarget samples.");
    ImGui::TextWrapped(
        "None keeps the Surface Fit / Normal Seed result unchanged.");
    ImGui::TextWrapped(
        "A saved orientation seed is applied to a separate oriented field for this stage and reconstruction.");
    ImGui::Separator();

    auto& settings = state.normalSmoothingSettings;
    const char* neighborhoodLabels[] = {
        "None",
        "3 x 3 connected",
        "5 x 5 connected"};
    int neighborhoodIndex = settings.neighborhood ==
            volume_surface::SurfaceNormalNeighborhood::Grid3x3
        ? 1
        : settings.neighborhood == volume_surface::SurfaceNormalNeighborhood::Grid5x5
            ? 2
            : 0;
    if (ImGui::Combo(
            "Trend neighborhood",
            &neighborhoodIndex,
            neighborhoodLabels,
            3)) {
        settings.neighborhood = neighborhoodIndex == 2
            ? volume_surface::SurfaceNormalNeighborhood::Grid5x5
            : neighborhoodIndex == 1
                ? volume_surface::SurfaceNormalNeighborhood::Grid3x3
                : volume_surface::SurfaceNormalNeighborhood::None;
        clearSurfaceFitExpansionSelection(state);
        state.normalFieldReady = false;
        state.orientedNormalFieldReady = false;
        state.orientedNormalField = {};
        state.orientedNormalExpansionTrace = {};
        state.orientedNormalStatus = "Orientation seed must be reapplied after smoothing";
        state.normalFieldPreviewDirty = true;
        state.normalFieldStatus = "Normal field settings changed; build the selected mode";
    }
    ImGui::BeginDisabled(settings.neighborhood == volume_surface::SurfaceNormalNeighborhood::None);
    float strength = static_cast<float>(settings.strength);
    if (ImGui::SliderFloat("Trend strength", &strength, 0.0f, 1.0f, "%.2f")) {
        settings.strength = std::clamp(static_cast<double>(strength), 0.0, 1.0);
        clearSurfaceFitExpansionSelection(state);
        state.normalFieldReady = false;
        state.orientedNormalFieldReady = false;
        state.orientedNormalField = {};
        state.orientedNormalExpansionTrace = {};
        state.orientedNormalStatus = "Orientation seed must be reapplied after smoothing";
        state.normalFieldPreviewDirty = true;
        state.normalFieldStatus = "Normal field settings changed; build the selected mode";
    }
    int robustIterations = static_cast<int>(std::clamp<std::size_t>(
        settings.robustIterations,
        1,
        5));
    if (ImGui::SliderInt("Robust iterations", &robustIterations, 1, 5)) {
        settings.robustIterations = static_cast<std::size_t>(robustIterations);
        clearSurfaceFitExpansionSelection(state);
        state.normalFieldReady = false;
        state.orientedNormalFieldReady = false;
        state.orientedNormalField = {};
        state.orientedNormalExpansionTrace = {};
        state.orientedNormalStatus = "Orientation seed must be reapplied after smoothing";
        state.normalFieldPreviewDirty = true;
        state.normalFieldStatus = "Normal field settings changed; build the selected mode";
    }
    ImGui::EndDisabled();
    ImGui::Text(
        "Trend support: %s",
        neighborhoodIndex == 2 ? "up to 25" : neighborhoodIndex == 1 ? "up to 9" : "disabled");

    if (ImGui::Button("Build / update normal field")) {
        rebuildSurfaceNormalField(state);
        rebuildOrientedNormalField(state);
        state.normalFieldPreviewDirty = true;
    }
    ImGui::SameLine();
    ImGui::TextWrapped("%s", state.normalFieldStatus.c_str());
    if (state.surfaceNormalSeed.valid) {
        ImGui::TextWrapped("Seed orientation: %s", state.orientedNormalStatus.c_str());
    }
    bool previewChanged = ImGui::Checkbox(
        "Preview with smoothed normals",
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
        "Averaged samples: %zu core / %zu transition",
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
    processSurfaceFitPick(state, engine, scene, view);
    processSurfaceFitExpansionPick(state, engine, scene, view);
    processOrientationSeedPick(state, view);
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
        case WorkflowStage::SurfaceTarget:
            drawSurfaceTargetWindow(state, engine, scene);
            break;
        case WorkflowStage::SurfaceFit:
            drawSurfaceFitWindow(state, engine, scene);
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
                state.reconstructionProjectionVertexCount = 0;
                state.reconstructionProjectionRejectedCount = 0;
                state.reconstructionProjectionDensityRejectedCount = 0;
                state.reconstructionProjectionMaximumDisplacement = 0.0;
                state.reconstructionPickArmed = false;
                state.reconstructionPickRequested = false;
                state.reconstructionPickReport = {};
                state.reconstructionPickStatus = "No reconstruction point has been picked";
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
        if (!state->headlessSmoke && !state->replayBrushProfile) {
            view->setStencilBufferEnabled(true);
        }
        state->surfaceAwareCameraController.reset(
            openvdb::Vec3d{0.0, 0.0, 1.0},
            openvdb::Vec3d{0.0, 0.0, -4.0},
            openvdb::Vec3d{0.0, 1.0, 0.0});
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
            state->workflowController.setStage(WorkflowStage::Source);
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
        state->surfaceTargetPointPicker.destroy(*engine);
        state->surfaceTargetPreview.destroy(*engine, *scene);
        state->surfaceFitPlaneRenderer.destroy(*engine, *scene);
        state->surfaceFitExpansionDebugRenderer.destroy(*engine, *scene);
        state->surfaceFitNeighborhoodDebugRenderer.destroy(*engine, *scene);
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
        .preRender([state](Engine* engine, View* view, Scene*, filament::Renderer*) {
            if (engine && view) {
                state->surfaceAwareCameraController.applyTo(view->getCamera());
                state->surfaceTargetPointPicker.prepare(*engine, *view);
            }
        })
        .postRender([state](Engine* engine, View* view, Scene*, filament::Renderer* renderer) {
            if (engine && view && renderer) {
                state->surfaceTargetPointPicker.finish(*renderer, *view);
            }
        })
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
            std::cout << "mesh_continuity.samples="
                      << state->surfaceMeshContinuity.supportedSampleCount
                      << " build_ms="
                      << state->surfaceMeshContinuityBuildMilliseconds
                      << " ready=" << (state->surfaceMeshContinuityReady ? 1 : 0)
                      << '\n';
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
