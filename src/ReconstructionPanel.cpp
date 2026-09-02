#include "volume_surface/viewer/ReconstructionPanel.h"

#include <algorithm>
#include <cstddef>

#include <imgui.h>

#include "volume_surface/viewer/ViewerState.h"

namespace volume_surface::viewer {

ReconstructionPanelAction ReconstructionPanel::draw(ViewerState& state) const
{
    ReconstructionPanelAction action = ReconstructionPanelAction::None;
    ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(390.0f, 520.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(1.0f);
    ImGui::Begin("Surface Reconstruction");
    state.mouseOverUi |= ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);
    ImGui::TextWrapped(
        "Result A is a zero-offset MLS baseline. Brush displacement is applied in a later result.");
    if (ImGui::Button(
            state.showReferenceMesh ? "Hide original mesh" : "Show original mesh")) {
        state.showReferenceMesh = !state.showReferenceMesh;
        action = ReconstructionPanelAction::ReferenceVisibilityChanged;
    }
    ImGui::SameLine();
    ImGui::TextUnformatted("Result A uses opaque depth-tested shading");
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
    float cellSizeMillimeters = static_cast<float>(settings.targetCellSize * 1000.0);
    if (ImGui::SliderFloat(
            "Target cell (mm)",
            &cellSizeMillimeters,
            0.5f,
            2.0f,
            "%.2f")) {
        settings.targetCellSize = std::max(
            0.00025,
            static_cast<double>(cellSizeMillimeters) * 0.001);
    }
    int paddingCells = static_cast<int>(settings.candidatePaddingCells);
    if (ImGui::SliderInt("Candidate padding", &paddingCells, 1, 4)) {
        settings.candidatePaddingCells = static_cast<std::size_t>(paddingCells);
    }
    int maximumCells = static_cast<int>(std::min<std::size_t>(
        settings.maximumCellCount / 1'000'000,
        static_cast<std::size_t>(16)));
    maximumCells = std::max(maximumCells, 1);
    if (ImGui::SliderInt("Max cells (million)", &maximumCells, 1, 16)) {
        settings.maximumCellCount = static_cast<std::size_t>(maximumCells) * 1'000'000;
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
    ImGui::TextWrapped("Status: %s", state.reconstructionStatus.c_str());
    ImGui::Text("Result A: %s", state.slots[1].available() ? "available" : "empty");
    ImGui::Text("Result B: %s", state.slots[2].available() ? "available" : "empty");
    ImGui::Text("Result C: %s", state.slots[3].available() ? "available" : "empty");
    if (state.reconstructionCandidateCellCount > 0) {
        ImGui::Text(
            "Cells: %zu candidates / %zu crossings / %zu field vertices",
            state.reconstructionCandidateCellCount,
            state.reconstructionCrossingCellCount,
            state.reconstructionFieldSampleCount);
        ImGui::Text(
            "Source-density fallback: %zu",
            state.reconstructionSourceSupportFallbackCount);
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
