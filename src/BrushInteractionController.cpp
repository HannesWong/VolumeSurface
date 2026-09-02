#include "volume_surface/viewer/BrushInteractionController.h"

#include "volume_surface/SurfaceBrush.h"
#include "volume_surface/viewer/BrushProfileRecorder.h"
#include "volume_surface/viewer/BrushHeatmapRenderer.h"
#include "volume_surface/viewer/MeshRenderer.h"
#include "volume_surface/viewer/ViewerState.h"
#include "volume_surface/viewer/WeightPaintingStage.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <filament/Camera.h>
#include <filament/Engine.h>
#include <filament/Scene.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <imgui.h>
#include <math/mat4.h>

using filament::Engine;
using filament::Scene;
using filament::View;
using filament::math::mat4;
using utils::Entity;

namespace volume_surface::viewer {
namespace {

using BrushProfileSettings = volume_surface::viewer::BrushProfileSettings;
using BrushProfileFit = volume_surface::viewer::BrushProfileFit;
using BrushProfileStroke = volume_surface::viewer::BrushProfileStroke;
using WeightPaintingSettings = volume_surface::viewer::WeightPaintingSettings;

BrushProfileSettings captureProfileSettings(const ViewerState& state)
{
    return {
        state.brushCoreRadiusMillimeters,
        state.brushFalloffRadiusMillimeters,
        state.brushStrength,
        state.brushPlanarityRadiusMillimeters,
        state.brushPlanarityAngleDegrees,
        state.brushPlaneOffsetPenalty,
        state.brushNormalChangePenalty,
        state.brushNormalChangeAngleDegrees};
}

void applyProfileSettings(
    ViewerState& state,
    const BrushProfileSettings& settings)
{
    state.brushCoreRadiusMillimeters = settings.coreRadiusMillimeters;
    state.brushFalloffRadiusMillimeters = settings.falloffRadiusMillimeters;
    state.brushStrength = settings.strength;
    state.brushPlanarityRadiusMillimeters = settings.planarityRadiusMillimeters;
    state.brushPlanarityAngleDegrees = settings.planarityAngleDegrees;
    state.brushPlaneOffsetPenalty = settings.planeOffsetPenalty;
    state.brushNormalChangePenalty = settings.normalChangePenalty;
    state.brushNormalChangeAngleDegrees = settings.normalChangeAngleDegrees;
}

void beginProfileStrokeImpl(ViewerState& state)
{
    state.brushProfileRecorder.begin(
        state.input,
        state.gridName,
        state.referenceIsoValue,
        state.adaptivity,
        captureProfileSettings(state));
}

void discardProfileStrokeImpl(ViewerState& state)
{
    state.brushProfileRecorder.discard();
}

void recordProfilePointImpl(
    ViewerState& state,
    const openvdb::Vec3d& worldPosition)
{
    state.brushProfileRecorder.addPoint(worldPosition);
}

void appendProfileStrokeImpl(ViewerState& state)
{
    const std::size_t pointCount = state.brushProfileRecorder.pointCount();
    const std::size_t fitCount = state.brushProfileRecorder.fitCount();
    std::string error;
    if (state.brushProfileRecorder.append(state.brushProfilePath, error)) {
        state.brushProfileStatus = "Recorded " +
            std::to_string(pointCount) + " points / " +
            std::to_string(fitCount) + " fits to " +
            state.brushProfilePath.string();
    } else if (!error.empty()) {
        state.brushProfileStatus = error;
    }
}

void recordProfileFitImpl(
    ViewerState& state,
    std::size_t anchorCount,
    bool finalFit,
    double heatmapMilliseconds,
    double endToEndMilliseconds)
{
    state.brushProfileRecorder.addFit(
        anchorCount,
        finalFit,
        state.brushResult,
        heatmapMilliseconds,
        endToEndMilliseconds,
        state.brushHeatmap.stats().visibleTriangleCount);
}

bool applyHeatmapSamplesImpl(
    ViewerState& state,
    Engine& engine,
    Scene& scene,
    const std::vector<volume_surface::SurfaceBrushSample>& samples,
    bool countStroke)
{
    const bool strokeApplied = state.brushHeatmap.applySamples(engine, scene, samples);
    if (!strokeApplied) {
        return false;
    }
    if (countStroke) {
        ++state.brushStrokeCount;
    }
    return true;
}

bool applyHeatmapImpl(
    ViewerState& state,
    Engine& engine,
    Scene& scene,
    bool countStroke)
{
    return applyHeatmapSamplesImpl(
        state,
        engine,
        scene,
        state.brushResult.samples,
        countStroke);
}

bool mergeResultIntoWeightFieldImpl(ViewerState& state)
{
    if (!state.brushWeightGrid || state.brushResult.empty()) {
        return false;
    }

    bool changed = false;
    auto accessor = state.brushWeightGrid->getAccessor();
    for (const volume_surface::SurfaceBrushSample& sample : state.brushResult.samples) {
        if (!std::isfinite(sample.weight) || sample.weight <= 0.0f) {
            continue;
        }
        const float previousWeight = accessor.getValue(sample.coordinate);
        if (sample.weight > previousWeight) {
            accessor.setValueOn(sample.coordinate, sample.weight);
            changed = true;
        }
    }
    if (changed) {
        state.brushWeightFieldDirty = true;
        state.brushWeightFieldStatus = "Weight field changed";
    }
    return changed;
}

void processPendingCenterImpl(
    ViewerState& state,
    Engine& engine,
    Scene& scene)
{
    if (!state.brushCenterPending) {
        return;
    }
    state.brushCenterPending = false;

    try {
        const WeightPaintingSettings brushSettings{
            state.brushCoreRadiusMillimeters,
            state.brushFalloffRadiusMillimeters,
            state.brushStrength,
            state.brushPlanarityRadiusMillimeters,
            state.brushPlanarityAngleDegrees,
            state.brushPlaneOffsetPenalty,
            state.brushNormalChangePenalty,
            state.brushNormalChangeAngleDegrees};
        const auto request = WeightPaintingStage::makeRequest(
            brushSettings,
            state.referenceIsoValue);

        const auto start = std::chrono::steady_clock::now();
        state.brushResult = volume_surface::propagateSurfaceBrush(
            *state.grid,
            state.pendingBrushCenterWorld,
            request.brush,
            request.propagation);
        const auto stop = std::chrono::steady_clock::now();
        state.brushComputeMilliseconds =
            std::chrono::duration<double, std::milli>(stop - start).count();

        state.brushAveragePlanarity = 0.0f;
        for (const auto& sample : state.brushResult.samples) {
            state.brushAveragePlanarity += sample.planarity;
        }
        if (!state.brushResult.samples.empty()) {
            state.brushAveragePlanarity /=
                static_cast<float>(state.brushResult.samples.size());
        }
        mergeResultIntoWeightFieldImpl(state);
        const auto heatmapStart = std::chrono::steady_clock::now();
        applyHeatmapImpl(state, engine, scene, true);
        state.brushHeatmapMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - heatmapStart).count();
        state.brushStatus = state.brushResult.empty()
            ? "No connected surface voxel found near the picked point"
            : "Brush stroke added to the weight overlay";
    } catch (const std::exception& error) {
        state.brushStatus = std::string("Brush preview failed: ") + error.what();
        state.brushResult = {};
        state.brushHeatmapMilliseconds = 0.0;
    }
}

void processPendingStrokeImpl(
    ViewerState& state,
    Engine& engine,
    Scene& scene)
{
    if (state.brushStrokePathWorld.empty()) {
        if (state.brushStrokeFinalizeRequested &&
            !state.brushPickPending &&
            !state.brushStrokeSampleRequested) {
            state.brushStrokeFinalizeRequested = false;
            state.brushHasLastPathCenter = false;
            state.brushStatus = "The brush stroke did not hit the Reference surface";
            discardProfileStrokeImpl(state);
        }
        return;
    }

    const bool hasUnfittedCenters =
        state.brushStrokeFittedCenterCount < state.brushStrokePathWorld.size();
    const bool waitingForFinalPick =
        state.brushPickPending || state.brushStrokeSampleRequested;
    const bool finalizeNow =
        state.brushStrokeFinalizeRequested && !waitingForFinalPick;
    if (!hasUnfittedCenters) {
        if (finalizeNow) {
            if (state.brushProfileRecorder.active() &&
                state.brushProfileRecorder.hasFits()) {
                state.brushProfileRecorder.markLastFitFinal();
            }
            appendProfileStrokeImpl(state);
            state.brushStrokeFinalizeRequested = false;
            state.brushHasLastPathCenter = false;
            state.brushStrokePathWorld.clear();
            state.brushStrokeFittedCenterCount = 0;
            state.brushStrokeNextFitTime = {};
        }
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!finalizeNow && now < state.brushStrokeNextFitTime) {
        return;
    }

    try {
        volume_surface::SurfaceBrushParameters brush;
        brush.coreRadius = state.brushCoreRadiusMillimeters * 0.001;
        brush.falloffRadius = std::max(
            state.brushFalloffRadiusMillimeters - state.brushCoreRadiusMillimeters,
            0.1f) * 0.001;
        brush.strength = state.brushStrength;

        constexpr double degreesToRadians = 0.017453292519943295;
        volume_surface::SurfacePropagationSettings settings;
        settings.isoValue = state.referenceIsoValue;
        settings.planarityRadius = state.brushPlanarityRadiusMillimeters * 0.001;
        settings.planarityAngularScaleRadians =
            state.brushPlanarityAngleDegrees * degreesToRadians;
        settings.planeOffsetPenalty = state.brushPlaneOffsetPenalty;
        settings.normalChangePenalty = state.brushNormalChangePenalty;
        settings.normalChangeScaleRadians =
            state.brushNormalChangeAngleDegrees * degreesToRadians;

        const std::size_t pathCenterCount = state.brushStrokePathWorld.size();
        const std::size_t firstNewCenter = state.brushStrokeFittedCenterCount;
        const std::size_t incrementalStart = firstNewCenter == 0
            ? 0
            : firstNewCenter - 1;
        std::vector<openvdb::Vec3d> incrementalPath(
            state.brushStrokePathWorld.begin() + static_cast<std::ptrdiff_t>(incrementalStart),
            state.brushStrokePathWorld.end());
        const auto start = std::chrono::steady_clock::now();
        state.brushResult = volume_surface::propagateSurfaceBrushStroke(
            *state.grid,
            incrementalPath,
            brush,
            settings,
            state.brushHierarchy.get());
        const auto stop = std::chrono::steady_clock::now();
        state.brushComputeMilliseconds =
            std::chrono::duration<double, std::milli>(stop - start).count();

        state.brushAveragePlanarity = 0.0f;
        for (const auto& sample : state.brushResult.samples) {
            state.brushAveragePlanarity += sample.planarity;
        }
        if (!state.brushResult.samples.empty()) {
            state.brushAveragePlanarity /=
                static_cast<float>(state.brushResult.samples.size());
        }
        mergeResultIntoWeightFieldImpl(state);
        const auto heatmapStart = std::chrono::steady_clock::now();
        if (applyHeatmapImpl(
                state,
                engine,
                scene,
                !state.brushStrokeHasAppliedPreview)) {
            state.brushStrokeHasAppliedPreview = true;
        }
        state.brushHeatmapMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - heatmapStart).count();
        recordProfileFitImpl(
            state,
            pathCenterCount,
            finalizeNow,
            state.brushHeatmapMilliseconds,
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count());
        state.brushStatus = state.brushResult.empty()
            ? "No connected surface voxel found near the brush path"
            : (finalizeNow ? "Brush path fitted from " : "Brush path preview fitted from ") +
                std::to_string(pathCenterCount) + " surface centers";
    } catch (const std::exception& error) {
        state.brushStatus = std::string("Brush path fitting failed: ") + error.what();
        state.brushResult = {};
        state.brushHeatmapMilliseconds = 0.0;
    }
    state.brushStrokeFittedCenterCount = state.brushStrokePathWorld.size();
    state.brushStrokeNextFitTime =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(120);
    if (finalizeNow) {
        appendProfileStrokeImpl(state);
        state.brushStrokeFinalizeRequested = false;
        state.brushHasLastPathCenter = false;
        state.brushStrokePathWorld.clear();
        state.brushStrokeFittedCenterCount = 0;
        state.brushStrokeNextFitTime = {};
    }
}

void runProfileReplayImpl(
    ViewerState& state,
    Engine& engine,
    Scene& scene)
{
    std::size_t replayFitCount = 0;
    for (std::size_t strokeIndex = 0;
         strokeIndex < state.brushProfileReplayStrokes.size();
         ++strokeIndex) {
        const BrushProfileStroke& stroke = state.brushProfileReplayStrokes[strokeIndex];
        if (stroke.gridName != state.gridName ||
            std::abs(stroke.isoValue - state.referenceIsoValue) > 1.0e-4f ||
            std::abs(stroke.adaptivity - state.adaptivity) > 1.0e-4f) {
            throw std::runtime_error(
                "Brush replay requires the same --iso and --adaptivity as its profile");
        }
        applyProfileSettings(state, stroke.settings);
        std::size_t previousAnchorCount = 0;
        for (std::size_t fitIndex = 0; fitIndex < stroke.fits.size(); ++fitIndex) {
            const BrushProfileFit& recordedFit = stroke.fits[fitIndex];
            if (recordedFit.anchorCount < previousAnchorCount) {
                throw std::runtime_error("Brush profile anchor counts must be nondecreasing");
            }
            if (recordedFit.anchorCount == previousAnchorCount) {
                continue;
            }
            const std::size_t incrementalStart = previousAnchorCount == 0
                ? 0
                : previousAnchorCount - 1;
            state.brushStrokePathWorld.clear();
            state.brushStrokePathWorld.reserve(recordedFit.anchorCount - incrementalStart);
            for (std::size_t pointIndex = incrementalStart;
                 pointIndex < recordedFit.anchorCount;
                 ++pointIndex) {
                state.brushStrokePathWorld.push_back(
                    stroke.points[pointIndex].worldPosition);
            }

            volume_surface::SurfaceBrushParameters brush;
            brush.coreRadius = state.brushCoreRadiusMillimeters * 0.001;
            brush.falloffRadius = std::max(
                state.brushFalloffRadiusMillimeters - state.brushCoreRadiusMillimeters,
                0.1f) * 0.001;
            brush.strength = state.brushStrength;

            constexpr double degreesToRadians = 0.017453292519943295;
            volume_surface::SurfacePropagationSettings settings;
            settings.isoValue = state.referenceIsoValue;
            settings.planarityRadius = state.brushPlanarityRadiusMillimeters * 0.001;
            settings.planarityAngularScaleRadians =
                state.brushPlanarityAngleDegrees * degreesToRadians;
            settings.planeOffsetPenalty = state.brushPlaneOffsetPenalty;
            settings.normalChangePenalty = state.brushNormalChangePenalty;
            settings.normalChangeScaleRadians =
                state.brushNormalChangeAngleDegrees * degreesToRadians;

            const auto endToEndStart = std::chrono::steady_clock::now();
            state.brushResult = volume_surface::propagateSurfaceBrushStroke(
                *state.grid,
                state.brushStrokePathWorld,
                brush,
                settings,
                state.brushHierarchy.get());
            const auto heatmapStart = std::chrono::steady_clock::now();
            applyHeatmapImpl(state, engine, scene, false);
            const double heatmapMilliseconds =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - heatmapStart).count();
            const double endToEndMilliseconds =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - endToEndStart).count();

            std::size_t centerlineNodeCount = 0;
            for (const auto& path : state.brushResult.centerlinePaths) {
                centerlineNodeCount += path.size();
            }
            std::cout << std::fixed << std::setprecision(3)
                      << "replay.fit stroke=" << (strokeIndex + 1)
                      << " fit=" << (fitIndex + 1)
                      << " input_ms=" << recordedFit.inputMilliseconds
                      << " anchors=" << recordedFit.anchorCount
                      << " segment_anchors=" << state.brushStrokePathWorld.size()
                      << " hierarchy_ms="
                      << state.brushResult.timings.hierarchyQueryMilliseconds
                      << " block_ms=" << state.brushResult.timings.blockMilliseconds
                      << " anchor_ms="
                      << state.brushResult.timings.anchorResolveMilliseconds
                      << " route_ms="
                      << state.brushResult.timings.centerlineRouteMilliseconds
                      << " sweep_ms="
                      << state.brushResult.timings.surfaceSweepMilliseconds
                      << " core_ms=" << state.brushResult.timings.totalMilliseconds
                      << " heatmap_ms=" << heatmapMilliseconds
                      << " total_ms=" << endToEndMilliseconds
                      << " candidates=" << state.brushResult.candidateVoxelCount
                      << " leaves=" << state.brushResult.candidateLeafCount
                      << " centerline_nodes=" << centerlineNodeCount
                      << " samples=" << state.brushResult.samples.size()
                      << " heatmap_triangles="
                      << state.brushHeatmap.stats().visibleTriangleCount << '\n';
            ++replayFitCount;
            previousAnchorCount = recordedFit.anchorCount;
        }
    }
    state.brushProfileStatus = "Replayed " +
        std::to_string(state.brushProfileReplayStrokes.size()) + " strokes / " +
        std::to_string(replayFitCount) + " fits";
}

bool shouldAppendPathCenter(
    const ViewerState& state,
    const openvdb::Vec3d& centerWorld)
{
    if (!state.brushHasLastPathCenter) {
        return true;
    }
    const double minimumSpacing = std::max(
        0.00025,
        static_cast<double>(state.brushCoreRadiusMillimeters) * 0.0005);
    const openvdb::Vec3d difference =
        centerWorld - state.brushLastPathCenterWorld;
    return difference.lengthSqr() >= minimumSpacing * minimumSpacing;
}

void requestPickImpl(
    const std::shared_ptr<ViewerState>& state,
    View& view)
{
    if (!WeightPaintingStage::isActive(state->workflowController.stage()) ||
        !state->brushPreviewEnabled || !state->brushControlDown) {
        state->brushStrokeSampleRequested = false;
        state->brushCursorVisible = false;
        return;
    }
    if (state->mouseOverUi) {
        state->brushStrokeSampleRequested = false;
        state->brushCursorVisible = false;
        return;
    }
    const auto viewport = view.getViewport();
    const ImGuiIO& io = ImGui::GetIO();
    ImVec2 logicalPosition{
        static_cast<float>(state->brushPointerX),
        static_cast<float>(state->brushPointerY)};
    const float coordinateScaleX = io.DisplaySize.x > 0.0f
        ? static_cast<float>(viewport.width) / io.DisplaySize.x
        : io.DisplayFramebufferScale.x;
    const float coordinateScaleY = io.DisplaySize.y > 0.0f
        ? static_cast<float>(viewport.height) / io.DisplaySize.y
        : io.DisplayFramebufferScale.y;
    ImVec2 drawablePosition{
        logicalPosition.x * coordinateScaleX,
        logicalPosition.y * coordinateScaleY};
    const float pickCoordinateX = drawablePosition.x -
        static_cast<float>(viewport.left);
    const float pickCoordinateY = static_cast<float>(viewport.height) -
        1.0f - drawablePosition.y;
    if (pickCoordinateX < 0.0f || pickCoordinateY < 0.0f ||
        pickCoordinateX >= static_cast<float>(viewport.width) ||
        pickCoordinateY >= static_cast<float>(viewport.height)) {
        state->brushCursorVisible = false;
        return;
    }

    if (state->brushPickPending ||
        (!state->brushPointerDirty && !state->brushStrokeSampleRequested)) {
        return;
    }

    const bool commitBrush = state->brushStrokeSampleRequested;
    state->brushStrokeSampleRequested = false;
    state->brushPointerDirty = false;
    const std::uint32_t pickX = static_cast<std::uint32_t>(pickCoordinateX);
    const std::uint32_t pickY = static_cast<std::uint32_t>(pickCoordinateY);
    state->brushPickViewportWidth = viewport.width;
    state->brushPickViewportHeight = viewport.height;
    state->brushPickClipToScene = view.getCamera().getModelMatrix() *
        inverse(view.getCamera().getProjectionMatrix());
    const std::uint64_t pickSerial = ++state->brushPickSerial;
    state->brushPickPending = true;
    if (commitBrush) {
        state->brushStatus = "Waiting for GPU pick...";
    }

    view.pick(pickX, pickY, [state, commitBrush, pickSerial](
        const View::PickingQueryResult& result) {
        if (pickSerial != state->brushPickSerial) {
            return;
        }
        state->brushPickPending = false;
        const Entity referenceEntity = state->meshRenderer.entity(0);
        const Entity heatmapEntity = state->brushHeatmap.entity();
        if (!result.renderable ||
            (result.renderable != referenceEntity && result.renderable != heatmapEntity)) {
            state->brushCursorVisible = false;
            if (commitBrush) {
                state->brushStatus = "The brush stroke did not hit the Reference surface";
            }
            return;
        }
        if (state->brushPickViewportWidth == 0 || state->brushPickViewportHeight == 0) {
            state->brushStatus = "The pick viewport was invalid";
            return;
        }

        const filament::math::double3 clipPosition{
            static_cast<double>(result.fragCoords.x) /
                    state->brushPickViewportWidth * 2.0 - 1.0,
            static_cast<double>(result.fragCoords.y) /
                    state->brushPickViewportHeight * 2.0 - 1.0,
            static_cast<double>(result.fragCoords.z) * 2.0 - 1.0};
        const filament::math::double3 scenePosition = mat4::project(
            state->brushPickClipToScene,
            clipPosition);
        const openvdb::Vec3d worldPosition{
            scenePosition.x / state->displayScale + state->referenceCenter.x,
            scenePosition.y / state->displayScale + state->referenceCenter.y,
            (scenePosition.z + 4.0) / state->displayScale + state->referenceCenter.z};
        state->brushCursorScenePosition = scenePosition;
        state->brushCursorVisible = true;

        if (!commitBrush || !shouldAppendPathCenter(*state, worldPosition)) {
            return;
        }
        state->brushLastPathCenterWorld = worldPosition;
        state->brushHasLastPathCenter = true;
        const bool firstPathCenter = state->brushStrokePathWorld.empty();
        state->brushStrokePathWorld.push_back(worldPosition);
        recordProfilePointImpl(*state, worldPosition);
        if (firstPathCenter) {
            state->brushStrokeNextFitTime =
                std::chrono::steady_clock::now() + std::chrono::milliseconds(120);
        }
        state->brushStatus = "Recording brush path: " +
            std::to_string(state->brushStrokePathWorld.size()) + " surface centers";
    });
}

} // namespace

void BrushInteractionController::beginProfileStroke(ViewerState& state) const
{
    beginProfileStrokeImpl(state);
}

void BrushInteractionController::discardProfileStroke(ViewerState& state) const
{
    discardProfileStrokeImpl(state);
}

void BrushInteractionController::recordProfilePoint(
    ViewerState& state,
    const openvdb::Vec3d& worldPosition) const
{
    recordProfilePointImpl(state, worldPosition);
}

void BrushInteractionController::appendProfileStroke(ViewerState& state) const
{
    appendProfileStrokeImpl(state);
}

void BrushInteractionController::recordProfileFit(
    ViewerState& state,
    std::size_t anchorCount,
    bool finalFit,
    double heatmapMilliseconds,
    double endToEndMilliseconds) const
{
    recordProfileFitImpl(
        state,
        anchorCount,
        finalFit,
        heatmapMilliseconds,
        endToEndMilliseconds);
}

bool BrushInteractionController::applyHeatmapSamples(
    ViewerState& state,
    Engine& engine,
    Scene& scene,
    const std::vector<volume_surface::SurfaceBrushSample>& samples,
    bool countStroke) const
{
    return applyHeatmapSamplesImpl(state, engine, scene, samples, countStroke);
}

bool BrushInteractionController::mergeResultIntoWeightField(ViewerState& state) const
{
    return mergeResultIntoWeightFieldImpl(state);
}

void BrushInteractionController::processPendingCenter(
    ViewerState& state,
    Engine& engine,
    Scene& scene) const
{
    processPendingCenterImpl(state, engine, scene);
}

void BrushInteractionController::processPendingStroke(
    ViewerState& state,
    Engine& engine,
    Scene& scene) const
{
    processPendingStrokeImpl(state, engine, scene);
}

void BrushInteractionController::runProfileReplay(
    ViewerState& state,
    Engine& engine,
    Scene& scene) const
{
    runProfileReplayImpl(state, engine, scene);
}

void BrushInteractionController::requestPick(
    const std::shared_ptr<ViewerState>& state,
    View& view) const
{
    requestPickImpl(state, view);
}

} // namespace volume_surface::viewer
