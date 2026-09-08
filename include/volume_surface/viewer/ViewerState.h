#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <math/vec3.h>
#include <openvdb/openvdb.h>

#include "volume_surface/SliceDiagnostics.h"
#include "volume_surface/SurfaceBrush.h"
#include "volume_surface/SurfaceReconstruction.h"
#include "volume_surface/SurfaceNormalField.h"
#include "volume_surface/SurfaceNormalLocalSeed.h"
#include "volume_surface/SurfaceTarget.h"
#include "volume_surface/SurfaceTargetPreview.h"
#include "volume_surface/viewer/SurfaceAwareCameraController.h"
#include "volume_surface/viewer/BrushCursorRenderer.h"
#include "volume_surface/viewer/BrushHeatmapRenderer.h"
#include "volume_surface/viewer/BrushInteractionController.h"
#include "volume_surface/viewer/CameraPickController.h"
#include "volume_surface/viewer/BrushInteractionState.h"
#include "volume_surface/viewer/BrushPaintingPanel.h"
#include "volume_surface/viewer/BrushProfileRecorder.h"
#include "volume_surface/viewer/DocumentSession.h"
#include "volume_surface/viewer/MeshRenderer.h"
#include "volume_surface/viewer/PresentationController.h"
#include "volume_surface/viewer/ReconstructionPanel.h"
#include "volume_surface/viewer/ReconstructionPick.h"
#include "volume_surface/viewer/SliceRenderer.h"
#include "volume_surface/viewer/SurfaceFitPlaneRenderer.h"
#include "volume_surface/viewer/SurfaceFitExpansionDebugRenderer.h"
#include "volume_surface/viewer/SurfaceNormalLocalSeedRenderer.h"
#include "volume_surface/viewer/SurfaceNormalLocalSeedStore.h"
#include "volume_surface/viewer/SurfaceTargetPointPicker.h"
#include "volume_surface/viewer/SurfaceNormalSeedStore.h"
#include "volume_surface/viewer/ViewerContext.h"
#include "volume_surface/viewer/WorkflowController.h"
#include "volume_surface/viewer/WorkflowPanel.h"

namespace volume_surface::viewer {

struct ViewerState : DocumentSession, BrushInteractionState {
    struct LocalNormalSeedEntry {
        std::size_t sampleIndex = static_cast<std::size_t>(-1);
        openvdb::Coord coordinate{};
        std::uint8_t targetSign = 0;
        bool accepted = false;
        bool activeFlipPoint = true;
        std::array<std::uint8_t, 4> color{255, 255, 255, 255};
    };
    ViewerContext context;
    WorkflowController workflowController;
    WorkflowPanel workflowPanel;
    ReconstructionPanel reconstructionPanel;
    BrushInteractionController brushInteractionController;
    BrushPaintingPanel brushPaintingPanel;
    std::array<MeshSlot, 4> slots{
        MeshSlot{"Reference", {}, {0.72f, 0.78f, 0.90f}, 1.0f, true},
        MeshSlot{"Result A", {}, {0.30f, 0.78f, 0.98f}, 1.0f, false},
        MeshSlot{"Result B", {}, {0.98f, 0.55f, 0.30f}, 0.65f, false},
        MeshSlot{"Result C", {}, {0.50f, 0.90f, 0.50f}, 0.65f, false}};
    MeshRenderer meshRenderer;
    CameraPickController cameraPickController;
    SurfaceAwareCameraController surfaceAwareCameraController;
    PresentationController presentationController;
    filament::math::float3 referenceCenter{};
    float displayScale = 1.0f;
    volume_surface::SurfaceNormalFitSettings normalFitSettings;
    volume_surface::SurfaceNormalField normalFitField;
    volume_surface::SurfaceNormalField primaryOrientedNormalField;
    bool normalFitReady = false;
    bool normalFitPreviewActive = false;
    std::string normalFitStatus = "Fitted normal seeds have not been built";
    volume_surface::SurfaceNormalSmoothingSettings normalSmoothingSettings;
    volume_surface::SurfaceNormalField normalField;
    bool normalFieldReady = false;
    bool normalFieldPreviewSmoothed = true;
    bool normalFieldPreviewActive = false;
    bool normalFieldPreviewDirty = false;
    std::string normalFieldStatus = "Normal field has not been built";
    volume_surface::SurfaceNormalField orientedNormalField;
    volume_surface::SurfaceNormalExpansionTrace orientedNormalExpansionTrace;
    bool orientedNormalFieldReady = false;
    std::string orientedNormalStatus = "Orientation seed has not been applied";
    volume_surface::SurfaceNormalAdjacencyStatistics normalFitAdjacencyStatistics;
    volume_surface::SurfaceNormalAdjacencyStatistics orientedNormalAdjacencyStatistics;
    volume_surface::SurfaceNormalLocalSeedSettings localNormalSeedSettings;
    std::vector<LocalNormalSeedEntry> localNormalSeeds;
    std::filesystem::path localNormalSeedPath;
    std::string localNormalSeedCacheStatus = "No saved local flip-point cache";
    volume_surface::SurfaceNormalLocalSeedPreview localNormalSeedPreview;
    SurfaceNormalLocalSeedRenderer surfaceNormalLocalSeedRenderer;
    std::size_t localNormalSeedSelected = static_cast<std::size_t>(-1);
    bool localNormalSeedPreviewVisible = true;
    bool localNormalSeedBoundaryVisible = true;
    bool localNormalSeedPickArmed = false;
    bool localNormalSeedPickRequested = false;
    int localNormalSeedPickX = 0;
    int localNormalSeedPickY = 0;
    std::string localNormalSeedStatus = "No local flip point selected";
    float wheelZoomMultiplier = 12.0f;
    float wheelHitDistanceRatio = 0.15f;
    float wheelMinimumDistanceMillimeters = 1.0f;
    float wheelSurfaceClearanceMillimeters = 0.5f;
    CameraPickHit cameraWheelHit;
    float cameraWheelProjectedDistanceMillimeters = 0.0f;
    float cameraWheelAppliedStepMillimeters = 0.0f;
    bool cameraWheelUsedAdaptiveStep = false;
    float globalLightIntensity = 25000.0f;
    float directionalLightAzimuthDegrees = 225.0f;
    float directionalLightElevationDegrees = -48.0f;
    bool showReferenceMesh = true;
    bool showSurfaceTargetMesh = true;
    bool brushPreviewEnabled = true;
    float brushCoreRadiusMillimeters = 3.0f;
    float brushFalloffRadiusMillimeters = 8.0f;
    float brushStrength = 1.0f;
    float brushPlanarityRadiusMillimeters = 2.0f;
    float brushPlanarityAngleDegrees = 15.0f;
    float brushPlaneOffsetPenalty = 16.0f;
    float brushNormalChangePenalty = 0.5f;
    float brushNormalChangeAngleDegrees = 20.0f;
    double brushComputeMilliseconds = 0.0;
    float brushAveragePlanarity = 0.0f;
    std::size_t brushStrokeCount = 0;
    volume_surface::SurfaceBrushResult brushResult;
    volume_surface::SurfaceTargetPreview surfaceTargetPreview;
    SurfaceTargetPointPicker surfaceTargetPointPicker;
    SurfaceFitPlaneRenderer surfaceFitPlaneRenderer;
    SurfaceFitExpansionDebugRenderer surfaceFitExpansionDebugRenderer;
    volume_surface::SurfaceNormalExpansionNeighborhood surfaceFitExpansionNeighborhood;
    std::int32_t surfaceFitExpansionParentDepthThreshold = 0;
    bool surfaceFitExpansionPickArmed = false;
    bool surfaceFitExpansionPickRequested = false;
    int surfaceFitExpansionPickX = 0;
    int surfaceFitExpansionPickY = 0;
    std::string surfaceFitExpansionStatus = "No debug Core point selected";
    SurfaceNormalSeed surfaceNormalSeed;
    std::filesystem::path surfaceNormalSeedPath;
    std::string surfaceNormalSeedStatus = "No saved orientation seed";
    bool orientationSeedPickArmed = false;
    bool orientationSeedPickRequested = false;
    int orientationSeedPickX = 0;
    int orientationSeedPickY = 0;
    std::string surfaceTargetStatus = "Surface target pending";
    std::string surfaceTargetCacheStatus = "No saved surface target cache";
    std::string reconstructionStatus = "Result A has not been generated";
    bool reconstructionPickArmed = false;
    bool reconstructionPickRequested = false;
    int reconstructionPickX = 0;
    int reconstructionPickY = 0;
    ReconstructionPickReport reconstructionPickReport;
    std::string reconstructionPickStatus = "No reconstruction point has been picked";
    std::string brushWeightFieldStatus = "No weight field has been saved";
    BrushHeatmapRenderer brushHeatmap;
    BrushCursorRenderer brushCursorRenderer;
    std::string brushStatus = "Ctrl + left-drag the surface to paint brush weights";
    openvdb::CoordBBox sliceBounds;
    volume_surface::SliceAxis sliceAxis = volume_surface::SliceAxis::Z;
    std::array<int, 3> sliceIndices{};
    float sliceDisplayMaximum = 255.0f;
    std::array<bool, 4> auxiliaryContours{true, true, true, true};
    bool referenceContour = true;
    bool surfaceTargetCoreSamples = true;
    bool surfaceTargetTransitionSamples = true;
    bool sliceDirty = true;
    bool mouseOverUi = false;
    bool headlessSmoke = false;
    bool replayBrushProfile = false;
    int smokeFramesRemaining = 3;
    std::filesystem::path brushSettingsPath = "brush_settings.json";
    std::filesystem::path brushProfilePath = "brush_profile.jsonl";
    BrushProfileRecorder brushProfileRecorder;
    std::vector<BrushProfileStroke> brushProfileReplayStrokes;
    std::string brushProfileStatus = "No completed stroke has been recorded";
    double brushHeatmapMilliseconds = 0.0;
    std::string brushSettingsStatus = "Built-in brush parameters";
    volume_surface::DensitySlice densitySlice;
    SliceRenderer sliceRenderer;
    std::string sliceStatus = "Slice pending";
    std::string status = "Ready";
};

} // namespace volume_surface::viewer
