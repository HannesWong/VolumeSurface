#include "volume_surface/viewer/BrushPaintingPanel.h"

#include "volume_surface/viewer/BrushProfileRecorder.h"
#include "volume_surface/viewer/BrushSettingsStore.h"
#include "volume_surface/viewer/ViewerState.h"
#include "volume_surface/viewer/WeightFieldRepository.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include <filament/Engine.h>
#include <filament/Scene.h>
#include <imgui.h>

namespace volume_surface::viewer {
namespace {

using BrushParameterId = volume_surface::viewer::BrushParameterId;
using BrushProfileSettings = volume_surface::viewer::BrushProfileSettings;
using BrushSettingsDocument = volume_surface::viewer::BrushSettingsDocument;

BrushProfileSettings captureBrushProfileSettings(const ViewerState& state)
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

void applyBrushProfileSettings(
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

bool drawBrushNumericParameter(
    ViewerState& state,
    BrushParameterId parameterId,
    const char* label,
    float* value,
    float dragSpeed,
    float minimum,
    float maximum,
    const char* format,
    const std::array<float, 3>& precisionSteps,
    const char* precisionUnit)
{
    bool changed = ImGui::DragFloat(
        label,
        value,
        dragSpeed,
        minimum,
        maximum,
        format,
        ImGuiSliderFlags_AlwaysClamp);
    const bool itemHovered = ImGui::IsItemHovered();
    const ImGuiIO& io = ImGui::GetIO();
    constexpr float precisionWidth = 78.0f;
    constexpr float precisionRowHeight = 21.0f;

    if (!state.brushParameterAdjustActive &&
        itemHovered &&
        ImGui::IsMouseClicked(ImGuiMouseButton_Middle)) {
        state.brushParameterAdjustActive = true;
        state.brushParameterAdjustField = parameterId;
        state.brushParameterAdjustInitialValue = *value;
        state.brushParameterAdjustLastX = io.MousePos.x;
        state.brushParameterAdjustAnchorX = io.MousePos.x;
        state.brushParameterAdjustAnchorY = io.MousePos.y;
        state.brushParameterAdjustPrecisionIndex = 1;
        state.brushParameterAdjustExitedPrecisionZone = false;
    }

    if (state.brushParameterAdjustActive &&
        state.brushParameterAdjustField == parameterId) {
        state.mouseOverUi = true;
        const float activePrecisionLeft =
            state.brushParameterAdjustAnchorX - precisionWidth * 0.5f;
        const float activePrecisionRight =
            state.brushParameterAdjustAnchorX + precisionWidth * 0.5f;
        const float activePrecisionTop = state.brushParameterAdjustAnchorY -
            precisionRowHeight * 1.5f;
        const float activePrecisionBottom = activePrecisionTop + precisionRowHeight * 3.0f;
        const bool precisionZone =
            io.MousePos.x >= activePrecisionLeft &&
            io.MousePos.x <= activePrecisionRight &&
            io.MousePos.y >= activePrecisionTop &&
            io.MousePos.y <= activePrecisionBottom;

        if (!ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
            state.brushParameterAdjustActive = false;
            state.brushParameterAdjustField = BrushParameterId::None;
        } else if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
            *value = state.brushParameterAdjustInitialValue;
            changed = true;
            state.brushParameterAdjustActive = false;
            state.brushParameterAdjustField = BrushParameterId::None;
        } else {
            if (!state.brushParameterAdjustExitedPrecisionZone && precisionZone) {
                const float rowPosition = io.MousePos.y - activePrecisionTop;
                if (rowPosition >= 0.0f &&
                    rowPosition < precisionRowHeight * 3.0f) {
                    state.brushParameterAdjustPrecisionIndex = std::clamp(
                        static_cast<int>(rowPosition / precisionRowHeight),
                        0,
                        2);
                }
            } else {
                if (!state.brushParameterAdjustExitedPrecisionZone) {
                    state.brushParameterAdjustExitedPrecisionZone = true;
                    state.brushParameterAdjustLastX = io.MousePos.x;
                }
                const float horizontalDelta =
                    io.MousePos.x - state.brushParameterAdjustLastX;
                if (horizontalDelta != 0.0f) {
                    *value = std::clamp(
                        *value + horizontalDelta *
                            precisionSteps[state.brushParameterAdjustPrecisionIndex] *
                            0.1f,
                        minimum,
                        maximum);
                    changed = true;
                }
                state.brushParameterAdjustLastX = io.MousePos.x;
            }

            if (!state.brushParameterAdjustExitedPrecisionZone) {
                ImDrawList* drawList = ImGui::GetForegroundDrawList();
                const ImU32 popupBackground = ImGui::GetColorU32(ImGuiCol_PopupBg);
                const ImU32 borderColor = ImGui::GetColorU32(ImGuiCol_Border);
                const ImU32 selectedColor = ImGui::GetColorU32(ImGuiCol_HeaderActive);
                const ImU32 textColor = ImGui::GetColorU32(ImGuiCol_Text);
                const ImU32 disabledTextColor = ImGui::GetColorU32(ImGuiCol_TextDisabled);
                const float popupLeft = state.brushParameterAdjustAnchorX - precisionWidth * 0.5f;
                const float popupRight = state.brushParameterAdjustAnchorX + precisionWidth * 0.5f;
                const float popupTop = state.brushParameterAdjustAnchorY -
                    precisionRowHeight * 1.5f;
                const float popupBottom = popupTop + precisionRowHeight * 3.0f;
                drawList->AddRectFilled(
                    ImVec2(popupLeft, popupTop),
                    ImVec2(popupRight, popupBottom),
                    popupBackground,
                    3.0f);
                drawList->AddRect(
                    ImVec2(popupLeft, popupTop),
                    ImVec2(popupRight, popupBottom),
                    borderColor,
                    3.0f);
                static constexpr const char* precisionNames[] = {"Coarse", "Normal", "Fine"};
                for (int index = 0; index < 3; ++index) {
                    const float rowTop = popupTop + precisionRowHeight * index;
                    if (index == state.brushParameterAdjustPrecisionIndex) {
                        drawList->AddRectFilled(
                            ImVec2(popupLeft + 1.0f, rowTop + 1.0f),
                            ImVec2(popupRight - 1.0f, rowTop + precisionRowHeight - 1.0f),
                            selectedColor,
                            2.0f);
                    }
                    char precisionLabel[48]{};
                    std::snprintf(
                        precisionLabel,
                        sizeof(precisionLabel),
                        "%s  %.3g%s",
                        precisionNames[index],
                        precisionSteps[index],
                        precisionUnit);
                    drawList->AddText(
                        ImVec2(popupLeft + 6.0f, rowTop + 3.0f),
                        index == state.brushParameterAdjustPrecisionIndex
                            ? textColor
                            : disabledTextColor,
                        precisionLabel);
                }
            }
        }
    }
    return changed;
}

void drawBrushControls(ViewerState& state, filament::Engine& engine, filament::Scene& scene)
{
    ImGui::Checkbox("Enable brush input", &state.brushPreviewEnabled);
    const bool coreChanged = drawBrushNumericParameter(
        state,
        BrushParameterId::CoreRadius,
        "Core radius",
        &state.brushCoreRadiusMillimeters,
        0.1f,
        0.0f,
        50.0f,
        "%.2f mm",
        {1.0f, 0.1f, 0.01f},
        " mm");
    const bool falloffChanged = drawBrushNumericParameter(
        state,
        BrushParameterId::FalloffRadius,
        "Falloff radius",
        &state.brushFalloffRadiusMillimeters,
        0.1f,
        0.1f,
        50.0f,
        "%.2f mm",
        {1.0f, 0.1f, 0.01f},
        " mm");
    if (coreChanged || falloffChanged) {
        state.brushFalloffRadiusMillimeters = std::max(
            state.brushFalloffRadiusMillimeters,
            state.brushCoreRadiusMillimeters + 0.1f);
        state.brushPointerDirty = true;
    }
    drawBrushNumericParameter(
        state,
        BrushParameterId::Strength,
        "Strength",
        &state.brushStrength,
        0.05f,
        0.0f,
        100.0f,
        "%.3f",
        {1.0f, 0.1f, 0.01f},
        "");

    if (ImGui::CollapsingHeader("Brush propagation settings")) {
        drawBrushNumericParameter(
            state,
            BrushParameterId::PlanarityRadius,
            "Planarity radius",
            &state.brushPlanarityRadiusMillimeters,
            0.1f,
            0.1f,
            20.0f,
            "%.2f mm",
            {1.0f, 0.1f, 0.01f},
            " mm");
        drawBrushNumericParameter(
            state,
            BrushParameterId::PlanarityAngle,
            "Planarity angle scale",
            &state.brushPlanarityAngleDegrees,
            0.5f,
            1.0f,
            90.0f,
            "%.1f deg",
            {1.0f, 0.5f, 0.1f},
            " deg");
        drawBrushNumericParameter(
            state,
            BrushParameterId::PlaneOffsetPenalty,
            "Plane offset penalty",
            &state.brushPlaneOffsetPenalty,
            0.25f,
            0.0f,
            100.0f,
            "%.2f",
            {1.0f, 0.1f, 0.01f},
            "");
        drawBrushNumericParameter(
            state,
            BrushParameterId::NormalChangePenalty,
            "Normal change penalty",
            &state.brushNormalChangePenalty,
            0.05f,
            0.0f,
            20.0f,
            "%.2f",
            {1.0f, 0.1f, 0.01f},
            "");
        drawBrushNumericParameter(
            state,
            BrushParameterId::NormalChangeAngle,
            "Normal change scale",
            &state.brushNormalChangeAngleDegrees,
            0.5f,
            1.0f,
            90.0f,
            "%.1f deg",
            {1.0f, 0.5f, 0.1f},
            " deg");
    }

    auto& panel = state.brushPaintingPanel;
    if (ImGui::Button("Clear weight field")) {
        state.brushResult = {};
        state.brushAveragePlanarity = 0.0f;
        state.brushComputeMilliseconds = 0.0;
        state.brushStrokeCount = 0;
        state.brushWeightGrid = panel.createWeightGrid(*state.grid);
        state.brushWeightFieldDirty = true;
        state.brushWeightFieldStatus = "Weight field cleared";
        panel.clearHeatmap(state, engine, scene);
        state.brushStrokePathWorld.clear();
        state.brushStrokeFittedCenterCount = 0;
        state.brushStrokeFinalizeRequested = false;
        state.brushInteractionController.discardProfileStroke(state);
        state.brushStatus = "Brush strokes cleared";
    }

    if (ImGui::CollapsingHeader(
            "Weight field library",
            ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::TextWrapped(
            "Library: %s",
            state.brushWeightFieldDirectory.string().c_str());
        std::string selectedFieldLabel = "No weight field files";
        if (state.brushWeightFieldSelection >= 0 &&
            state.brushWeightFieldSelection <
                static_cast<int>(state.brushWeightFieldFiles.size())) {
            selectedFieldLabel = state.brushWeightFieldFiles[
                static_cast<std::size_t>(state.brushWeightFieldSelection)].filename().string();
        }
        if (ImGui::BeginCombo("Available fields", selectedFieldLabel.c_str())) {
            for (std::size_t index = 0;
                 index < state.brushWeightFieldFiles.size();
                 ++index) {
                const bool selected = state.brushWeightFieldSelection ==
                    static_cast<int>(index);
                const std::string label =
                    state.brushWeightFieldFiles[index].filename().string();
                if (ImGui::Selectable(label.c_str(), selected)) {
                    state.brushWeightFieldSelection = static_cast<int>(index);
                }
                if (selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
        ImGui::InputText(
            "Field name",
            state.brushWeightFieldName.data(),
            state.brushWeightFieldName.size());

        if (ImGui::Button("Refresh fields")) {
            panel.refreshWeightFieldLibrary(state);
        }
        ImGui::SameLine();
        if (ImGui::Button("New empty")) {
            if (state.brushWeightFieldDirty) {
                state.brushWeightFieldStatus =
                    "Save or discard the current field before creating a new one";
            } else {
                state.brushWeightGrid = panel.createWeightGrid(*state.grid);
                state.brushWeightFieldPath.clear();
                panel.setWeightFieldName(state, "untitled");
                state.brushStrokeCount = 0;
                panel.clearHeatmap(state, engine, scene);
                state.brushWeightFieldStatus = "Created an empty unsaved weight field";
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Load selected")) {
            if (state.brushWeightFieldDirty) {
                state.brushWeightFieldStatus =
                    "Save or discard the current field before loading another one";
            } else if (state.brushWeightFieldSelection < 0 ||
                state.brushWeightFieldSelection >=
                    static_cast<int>(state.brushWeightFieldFiles.size())) {
                state.brushWeightFieldStatus = "Select a weight field to load";
            } else {
                panel.loadWeightField(
                    state,
                    engine,
                    scene,
                    state.brushWeightFieldFiles[
                        static_cast<std::size_t>(state.brushWeightFieldSelection)]);
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Save field")) {
            panel.saveWeightField(state, false);
        }
        ImGui::SameLine();
        if (ImGui::Button("Save as new")) {
            panel.saveWeightField(state, true);
        }
        ImGui::SameLine();
        if (ImGui::Button("Discard changes")) {
            panel.discardWeightFieldChanges(state, engine, scene);
        }

        const std::string activeFieldLabel = state.brushWeightFieldPath.empty()
            ? "Unsaved field"
            : state.brushWeightFieldPath.filename().string();
        ImGui::Text(
            "Active: %s%s / %lld active voxels",
            activeFieldLabel.c_str(),
            state.brushWeightFieldDirty ? " *" : "",
            state.brushWeightGrid
                ? static_cast<long long>(state.brushWeightGrid->activeVoxelCount())
                : 0LL);
        ImGui::TextWrapped("Weight field: %s", state.brushWeightFieldStatus.c_str());
    }

    if (ImGui::Button("Save brush preset")) {
        panel.saveSettings(state);
    }
    ImGui::SameLine();
    if (ImGui::Button("Load brush preset")) {
        panel.loadSettings(state);
        state.brushPointerDirty = true;
    }
    ImGui::TextWrapped(
        "Preset: %s",
        state.brushSettingsStatus.c_str());
    ImGui::TextWrapped(
        "Middle-drag a numeric field: choose precision around the cursor, then move horizontally outside the precision strip.");
    ImGui::TextWrapped(
        "Hold Ctrl to show the core and falloff circles; left-drag to preview a fitted path region.");
    ImGui::TextUnformatted("Falloff radius is the outer boundary, not an added width.");
    ImGui::TextWrapped(
        "Current strokes accumulate in a color-only weight overlay; the mesh geometry is not modified yet.");
    ImGui::TextWrapped("Brush: %s", state.brushStatus.c_str());
    ImGui::TextWrapped("Profile: %s", state.brushProfileStatus.c_str());
    if (state.brushHierarchy) {
        ImGui::Text(
            "Surface BVH: %zu VDB leaves, built in %.1f ms",
            state.brushHierarchy->leafCount(),
            state.brushHierarchyBuildMilliseconds);
    }
    if (state.brushStrokeCount > 0) {
        ImGui::Text(
            "%zu accumulated strokes / %zu colored vertices / %zu triangles",
            state.brushStrokeCount,
            state.brushHeatmap.stats().affectedVertexCount,
            state.brushHeatmap.stats().visibleTriangleCount);
    }
    if (!state.brushResult.empty()) {
        ImGui::Text(
            "Seed: [%d, %d, %d]",
            state.brushResult.seedCoordinate.x(),
            state.brushResult.seedCoordinate.y(),
            state.brushResult.seedCoordinate.z());
        ImGui::Text(
            "Latest stroke: %zu samples",
            state.brushResult.samples.size());
        ImGui::Text(
            "Candidates: %zu voxels / %zu leaves, average planarity: %.3f, core: %.1f ms",
            state.brushResult.candidateVoxelCount,
            state.brushResult.candidateLeafCount,
            state.brushAveragePlanarity,
            state.brushComputeMilliseconds);
        ImGui::Text(
            "BVH %.1f / block %.1f / anchors %.1f / route %.1f / sweep %.1f / heatmap %.1f ms",
            state.brushResult.timings.hierarchyQueryMilliseconds,
            state.brushResult.timings.blockMilliseconds,
            state.brushResult.timings.anchorResolveMilliseconds,
            state.brushResult.timings.centerlineRouteMilliseconds,
            state.brushResult.timings.surfaceSweepMilliseconds,
            state.brushHeatmapMilliseconds);
    }
}

} // namespace

openvdb::FloatGrid::Ptr BrushPaintingPanel::createWeightGrid(
    const openvdb::FloatGrid& referenceGrid) const
{
    return WeightFieldRepository::createEmpty(referenceGrid);
}

std::filesystem::path BrushPaintingPanel::weightFieldPathForName(
    const ViewerState& state,
    const std::string& name) const
{
    return WeightFieldRepository::pathForName(state.brushWeightFieldDirectory, name);
}

std::string BrushPaintingPanel::weightFieldName(const ViewerState& state) const
{
    return std::string(state.brushWeightFieldName.data());
}

void BrushPaintingPanel::setWeightFieldName(
    ViewerState& state,
    const std::string& name) const
{
    state.brushWeightFieldName.fill('\0');
    std::snprintf(
        state.brushWeightFieldName.data(),
        state.brushWeightFieldName.size(),
        "%s",
        name.c_str());
}

void BrushPaintingPanel::setWeightFieldNameFromPath(
    ViewerState& state,
    const std::filesystem::path& path) const
{
    setWeightFieldName(state, WeightFieldRepository::nameFromPath(path));
}

void BrushPaintingPanel::refreshWeightFieldLibrary(ViewerState& state) const
{
    const std::filesystem::path previousSelection =
        state.brushWeightFieldSelection >= 0 &&
            state.brushWeightFieldSelection <
                static_cast<int>(state.brushWeightFieldFiles.size())
        ? state.brushWeightFieldFiles[static_cast<std::size_t>(state.brushWeightFieldSelection)]
        : std::filesystem::path{};
    state.brushWeightFieldFiles.clear();
    state.brushWeightFieldSelection = -1;

    std::string error;
    state.brushWeightFieldFiles = WeightFieldRepository::list(
        state.brushWeightFieldDirectory,
        error);
    if (!error.empty()) {
        state.brushWeightFieldStatus = error;
        return;
    }
    const std::filesystem::path preferredSelection = !state.brushWeightFieldPath.empty()
        ? state.brushWeightFieldPath
        : previousSelection;
    for (std::size_t index = 0; index < state.brushWeightFieldFiles.size(); ++index) {
        if (state.brushWeightFieldFiles[index] == preferredSelection) {
            state.brushWeightFieldSelection = static_cast<int>(index);
            break;
        }
    }
    if (state.brushWeightFieldSelection < 0 && !state.brushWeightFieldFiles.empty()) {
        state.brushWeightFieldSelection = 0;
    }
}

bool BrushPaintingPanel::saveWeightField(ViewerState& state, bool saveAsNew) const
{
    if (!state.brushWeightGrid || !state.grid) {
        state.brushWeightFieldStatus = "Weight field is unavailable";
        return false;
    }
    const std::string name = weightFieldName(state);
    std::filesystem::path savedPath;
    std::string error;
    if (!WeightFieldRepository::save(
            state.brushWeightFieldDirectory,
            state.brushWeightFieldPath,
            name,
            saveAsNew,
            state.brushWeightGrid,
            *state.grid,
            state.gridName,
            savedPath,
            error)) {
        state.brushWeightFieldStatus = error;
        return false;
    }
    state.brushWeightFieldPath = savedPath;
    setWeightFieldNameFromPath(state, savedPath);
    state.brushWeightFieldDirty = false;
    refreshWeightFieldLibrary(state);
    state.brushWeightFieldStatus = "Saved " + savedPath.filename().string();
    return true;
}

void BrushPaintingPanel::saveSettings(ViewerState& state) const
{
    const BrushSettingsDocument document{
        state.brushPreviewEnabled,
        captureBrushProfileSettings(state)};
    std::string error;
    if (!BrushSettingsStore::save(state.brushSettingsPath, document, error)) {
        state.brushSettingsStatus = error;
        return;
    }
    state.brushSettingsStatus = "Brush parameters saved to " +
        state.brushSettingsPath.string();
}

void BrushPaintingPanel::loadSettings(ViewerState& state) const
{
    BrushSettingsDocument document{
        state.brushPreviewEnabled,
        captureBrushProfileSettings(state)};
    std::string error;
    if (!BrushSettingsStore::load(state.brushSettingsPath, document, error)) {
        if (!error.empty()) {
            state.brushSettingsStatus = error;
        }
        return;
    }
    state.brushPreviewEnabled = document.brushPreviewEnabled;
    applyBrushProfileSettings(state, document.settings);
    state.brushSettingsStatus = "Brush parameters loaded from " +
        state.brushSettingsPath.string();
}

void BrushPaintingPanel::clearHeatmap(
    ViewerState& state,
    filament::Engine& engine,
    filament::Scene& scene) const
{
    state.brushHeatmap.clear(engine, scene);
}

bool BrushPaintingPanel::rebuildHeatmapFromWeightField(
    ViewerState& state,
    filament::Engine& engine,
    filament::Scene& scene) const
{
    clearHeatmap(state, engine, scene);
    if (!state.brushWeightGrid || state.brushWeightGrid->activeVoxelCount() == 0) {
        return false;
    }

    std::vector<volume_surface::SurfaceBrushSample> samples;
    samples.reserve(static_cast<std::size_t>(state.brushWeightGrid->activeVoxelCount()));
    for (auto value = state.brushWeightGrid->cbeginValueOn(); value; ++value) {
        if (*value <= 0.0f) {
            continue;
        }
        volume_surface::SurfaceBrushSample sample;
        sample.coordinate = value.getCoord();
        sample.weight = *value;
        samples.push_back(sample);
    }
    return state.brushInteractionController.applyHeatmapSamples(
        state,
        engine,
        scene,
        samples,
        false);
}

bool BrushPaintingPanel::loadWeightField(
    ViewerState& state,
    filament::Engine& engine,
    filament::Scene& scene,
    const std::filesystem::path& path) const
{
    if (!state.grid) {
        state.brushWeightFieldStatus = "Weight field was not loaded: Reference density grid is unavailable";
        return false;
    }
    openvdb::FloatGrid::Ptr weightGrid;
    std::string error;
    if (!WeightFieldRepository::load(
            path,
            *state.grid,
            state.gridName,
            weightGrid,
            error)) {
        state.brushWeightFieldStatus = "Weight field was not loaded: " + error;
        return false;
    }

    state.brushWeightGrid = std::move(weightGrid);
    state.brushWeightFieldPath = path;
    setWeightFieldNameFromPath(state, path);
    state.brushWeightFieldDirty = false;
    state.brushStrokeCount = 0;
    rebuildHeatmapFromWeightField(state, engine, scene);
    refreshWeightFieldLibrary(state);
    state.brushWeightFieldStatus = "Loaded " + path.filename().string() + " (" +
        std::to_string(state.brushWeightGrid->activeVoxelCount()) + " active voxels)";
    return true;
}

void BrushPaintingPanel::discardWeightFieldChanges(
    ViewerState& state,
    filament::Engine& engine,
    filament::Scene& scene) const
{
    if (state.brushWeightFieldPath.empty()) {
        state.brushWeightGrid = createWeightGrid(*state.grid);
        state.brushWeightFieldDirty = false;
        state.brushStrokeCount = 0;
        clearHeatmap(state, engine, scene);
        state.brushWeightFieldStatus = "Discarded unsaved weight field";
        return;
    }
    loadWeightField(state, engine, scene, state.brushWeightFieldPath);
}

void BrushPaintingPanel::draw(
    ViewerState& state,
    filament::Engine& engine,
    filament::Scene& scene) const
{
    ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(390.0f, 460.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(1.0f);
    ImGui::Begin("Weight painting");
    state.mouseOverUi |= ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);
    drawBrushControls(state, engine, scene);
    ImGui::End();
}

} // namespace volume_surface::viewer
