#include "volume_surface/viewer/ReconstructionPanel.h"

#include <algorithm>
#include <cstddef>
#include <string>

#include <imgui.h>

#include "volume_surface/viewer/ViewerState.h"

namespace volume_surface::viewer {

ReconstructionPanelAction ReconstructionPanel::draw(ViewerState& state) const
{
    ReconstructionPanelAction action = ReconstructionPanelAction::None;
    ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(390.0f, 720.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(1.0f);
    ImGui::Begin("Surface Reconstruction");
    state.mouseOverUi |= ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);
    ImGui::TextWrapped(
        "Result A is a zero-offset source-topology mesh projected through the selected MLS normal field. Brush displacement is applied in a later result.");
    if (ImGui::Button(
            state.showReferenceMesh ? "Hide original mesh" : "Show original mesh")) {
        state.showReferenceMesh = !state.showReferenceMesh;
        action = ReconstructionPanelAction::ReferenceVisibilityChanged;
    }
    ImGui::SameLine();
    ImGui::TextUnformatted("Result A uses opaque depth-tested shading");
    ImGui::Separator();
    auto& settings = state.reconstructionSettings;
    float radiusMillimeters = static_cast<float>(settings.mlsRadius * 1000.0);
    if (ImGui::SliderFloat(
            "MLS radius (mm)",
            &radiusMillimeters,
            1.0f,
            20.0f,
            "%.2f mm",
            ImGuiSliderFlags_AlwaysClamp)) {
        settings.mlsRadius = std::max(0.001, static_cast<double>(radiusMillimeters) * 0.001);
    }
    int projectionIterations = static_cast<int>(std::clamp<std::size_t>(
        settings.projectionIterations,
        1,
        6));
    if (ImGui::SliderInt(
            "Projection iterations",
            &projectionIterations,
            1,
            6)) {
        settings.projectionIterations = static_cast<std::size_t>(projectionIterations);
    }
    float projectionMaximumDisplacementMillimeters = static_cast<float>(
        settings.projectionMaximumDisplacement * 1000.0);
    if (ImGui::SliderFloat(
            "Maximum projection displacement (mm)",
            &projectionMaximumDisplacementMillimeters,
            0.5f,
            5.0f,
            "%.2f mm")) {
        settings.projectionMaximumDisplacement = std::clamp(
            static_cast<double>(projectionMaximumDisplacementMillimeters) * 0.001,
            0.0005,
            0.020);
    }
    float transitionGeometryWeight = static_cast<float>(
        settings.transitionGeometryWeight);
    if (ImGui::SliderFloat(
            "Transition geometry weight",
            &transitionGeometryWeight,
            0.0f,
            0.25f,
            "%.3f")) {
        settings.transitionGeometryWeight = std::clamp(
            static_cast<double>(transitionGeometryWeight),
            0.0,
            1.0);
    }
    float minimumFogDensityFraction = static_cast<float>(
        settings.minimumFogDensityFraction);
    if (ImGui::SliderFloat(
            "Minimum fog density fraction",
            &minimumFogDensityFraction,
            0.0f,
            1.0f,
            "%.2f")) {
        settings.minimumFogDensityFraction = std::clamp(
            static_cast<double>(minimumFogDensityFraction),
            0.0,
            1.0);
    }
    const bool canGenerate = state.surfaceTargetCache &&
        !state.surfaceTargetCache->empty();
    ImGui::BeginDisabled(!canGenerate);
    const bool generate = ImGui::Button("Generate Result A");
    ImGui::EndDisabled();
    ImGui::SameLine();
    const bool clear = ImGui::Button("Clear Result A");
    const bool normalFieldReady = state.normalFieldReady &&
        state.surfaceTargetCache &&
        state.normalField.normals.size() == state.surfaceTargetCache->samples.size();
    const bool orientedNormalFieldReady = state.orientedNormalFieldReady &&
        state.surfaceTargetCache &&
        state.orientedNormalField.normals.size() == state.surfaceTargetCache->samples.size();
    const char* fitSource = state.normalFitSettings.neighborhood ==
            volume_surface::SurfaceFitNeighborhood::Grid9x9
        ? "9 x 9"
        : state.normalFitSettings.neighborhood ==
                volume_surface::SurfaceFitNeighborhood::Grid5x5
            ? "5 x 5"
            : "3 x 3";
    const char* trendSource = state.normalSmoothingSettings.neighborhood ==
            volume_surface::SurfaceNormalNeighborhood::Grid5x5
        ? "5 x 5"
        : state.normalSmoothingSettings.neighborhood ==
                volume_surface::SurfaceNormalNeighborhood::Grid3x3
            ? "3 x 3"
            : "none";
    const std::string normalSource = orientedNormalFieldReady
        ? "seed-oriented surface fit (" + std::string(fitSource) + ") + trend (" +
            std::string(trendSource) + ")"
        : normalFieldReady
        ? "surface fit (" + std::string(fitSource) + ") + trend (" +
            std::string(trendSource) + ")"
        : "raw SurfaceTarget normals";
    ImGui::Text("Normals: %s", normalSource.c_str());
    ImGui::TextWrapped("Status: %s", state.reconstructionStatus.c_str());
    ImGui::Text("Result A: %s", state.slots[1].available() ? "available" : "empty");
    ImGui::Text("Result B: %s", state.slots[2].available() ? "available" : "empty");
    ImGui::Text("Result C: %s", state.slots[3].available() ? "available" : "empty");
    if (state.reconstructionProjectionVertexCount > 0) {
        ImGui::Text(
            "Source topology: %zu vertices / %zu triangles",
            state.reconstructionProjectionVertexCount,
            state.reconstructionCrossingCellCount);
        ImGui::Text(
            "Projection rejected: %zu total / %zu density",
            state.reconstructionProjectionRejectedCount,
            state.reconstructionProjectionDensityRejectedCount);
        ImGui::Text(
            "Maximum vertex displacement: %.3f mm",
            state.reconstructionProjectionMaximumDisplacement * 1000.0);
        ImGui::Text(
            "Timing: anchors %.1f / index %.1f / field %.1f / mesh %.1f / total %.1f ms",
            state.reconstructionTimings.anchorLocalizationMilliseconds,
            state.reconstructionTimings.spatialIndexMilliseconds,
            state.reconstructionTimings.fieldSamplingMilliseconds,
            state.reconstructionTimings.meshExtractionMilliseconds,
            state.reconstructionTimings.totalMilliseconds);
    }
    ImGui::End();
    if (generate) {
        return ReconstructionPanelAction::GenerateResultA;
    }
    if (clear) {
        return ReconstructionPanelAction::ClearResultA;
    }
    return action;
}

} // namespace volume_surface::viewer
