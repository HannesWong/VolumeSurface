#include "volume_surface/viewer/WorkflowPanel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>

#include <filament/Engine.h>
#include <imgui.h>
#include <imgui_internal.h>

#include "volume_surface/viewer/LightingController.h"
#include "volume_surface/viewer/ViewerState.h"

namespace volume_surface::viewer {

namespace {

void drawDashedArrow(
    ImDrawList* drawList,
    ImVec2 start,
    ImVec2 end,
    ImU32 color,
    float thickness)
{
    const float dx = end.x - start.x;
    const float dy = end.y - start.y;
    const float length = std::sqrt(dx * dx + dy * dy);
    if (length <= 1.0e-4f) {
        return;
    }
    const float invLength = 1.0f / length;
    constexpr float dashLength = 6.0f;
    constexpr float gapLength = 4.0f;
    for (float distance = 0.0f; distance < length; distance += dashLength + gapLength) {
        const float dashEnd = std::min(distance + dashLength, length);
        const ImVec2 dashStart{
            start.x + dx * distance * invLength,
            start.y + dy * distance * invLength};
        const ImVec2 dashStop{
            start.x + dx * dashEnd * invLength,
            start.y + dy * dashEnd * invLength};
        drawList->AddLine(dashStart, dashStop, color, thickness);
    }
    const ImVec2 direction{dx * invLength, dy * invLength};
    const ImVec2 normal{-direction.y, direction.x};
    constexpr float arrowSize = 7.0f;
    const ImVec2 arrowLeft{
        end.x - direction.x * arrowSize + normal.x * arrowSize * 0.55f,
        end.y - direction.y * arrowSize + normal.y * arrowSize * 0.55f};
    const ImVec2 arrowRight{
        end.x - direction.x * arrowSize - normal.x * arrowSize * 0.55f,
        end.y - direction.y * arrowSize - normal.y * arrowSize * 0.55f};
    drawList->AddLine(end, arrowLeft, color, thickness);
    drawList->AddLine(end, arrowRight, color, thickness);
}

void drawSolidArrow(
    ImDrawList* drawList,
    ImVec2 start,
    ImVec2 end,
    ImU32 color,
    float thickness)
{
    drawList->AddLine(start, end, color, thickness);
    const float dx = end.x - start.x;
    const float dy = end.y - start.y;
    const float length = std::sqrt(dx * dx + dy * dy);
    if (length <= 1.0e-4f) {
        return;
    }
    const float invLength = 1.0f / length;
    const ImVec2 direction{dx * invLength, dy * invLength};
    const ImVec2 normal{-direction.y, direction.x};
    constexpr float arrowSize = 7.0f;
    const ImVec2 arrowLeft{
        end.x - direction.x * arrowSize + normal.x * arrowSize * 0.55f,
        end.y - direction.y * arrowSize + normal.y * arrowSize * 0.55f};
    const ImVec2 arrowRight{
        end.x - direction.x * arrowSize - normal.x * arrowSize * 0.55f,
        end.y - direction.y * arrowSize - normal.y * arrowSize * 0.55f};
    drawList->AddLine(end, arrowLeft, color, thickness);
    drawList->AddLine(end, arrowRight, color, thickness);
}

} // namespace

filament::math::float3 WorkflowPanel::directionalLightDirection(
    const ViewerState& state)
{
    constexpr float degreesToRadians = 3.14159265358979323846f / 180.0f;
    const float azimuth = state.directionalLightAzimuthDegrees * degreesToRadians;
    const float elevation = state.directionalLightElevationDegrees * degreesToRadians;
    const float cosElevation = std::cos(elevation);
    const filament::math::float3 rayDirection{
        cosElevation * std::cos(azimuth),
        std::sin(elevation),
        cosElevation * std::sin(azimuth)};
    // Filament expects the surface-to-light vector, while the sliders describe
    // the ray direction from the virtual light toward the fixed model target.
    return filament::math::float3{
        -rayDirection.x,
        -rayDirection.y,
        -rayDirection.z};
}

std::optional<WorkflowStage> WorkflowPanel::draw(
    ViewerState& state,
    filament::Engine& engine) const
{
    ImGui::SetNextWindowPos(ImVec2(810.0f, 10.0f), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(460.0f, 335.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(1.0f);
    ImGui::Begin("Workflow");
    state.mouseOverUi |= ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows);

    ImGui::Text(
        "Active: %s",
        workflowStageName(state.workflowController.stage()));
    const ImVec2 canvasOrigin = ImGui::GetCursorScreenPos();
    constexpr float nodeWidth = 105.0f;
    constexpr float nodeHeight = 54.0f;
    constexpr float nodeGap = 8.0f;
    constexpr float rowGap = 28.0f;
    constexpr float canvasWidth = nodeWidth * 4.0f + nodeGap * 3.0f + 20.0f;
    constexpr float canvasHeight = nodeHeight * 2.0f + rowGap + 20.0f;
    ImGui::Dummy(ImVec2(canvasWidth, canvasHeight));
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    const std::array<WorkflowStage, 7> stages{
        WorkflowStage::Source,
        WorkflowStage::SurfaceTarget,
        WorkflowStage::SurfaceFit,
        WorkflowStage::NormalField,
        WorkflowStage::WeightPainting,
        WorkflowStage::Reconstruction,
        WorkflowStage::Review};
    const std::array<ImVec2, 7> positions{
        ImVec2(10.0f, 10.0f),
        ImVec2(10.0f + nodeWidth + nodeGap, 10.0f),
        ImVec2(10.0f + (nodeWidth + nodeGap) * 2.0f, 10.0f),
        ImVec2(10.0f + (nodeWidth + nodeGap) * 3.0f, 10.0f),
        ImVec2(10.0f + (nodeWidth + nodeGap) * 3.0f, 10.0f + nodeHeight + rowGap),
        ImVec2(10.0f + (nodeWidth + nodeGap) * 2.0f, 10.0f + nodeHeight + rowGap),
        ImVec2(10.0f + nodeWidth + nodeGap, 10.0f + nodeHeight + rowGap)};

    auto topLeftFor = [&](std::size_t index) {
        return ImVec2{
            canvasOrigin.x + positions[index].x,
            canvasOrigin.y + positions[index].y};
    };
    auto centerFor = [&](std::size_t index) {
        const ImVec2 topLeft = topLeftFor(index);
        return ImVec2{topLeft.x + nodeWidth * 0.5f, topLeft.y + nodeHeight * 0.5f};
    };
    const ImU32 sequenceColor = ImGui::GetColorU32(ImVec4(0.84f, 0.86f, 0.92f, 0.9f));
    const ImU32 dataColor = ImGui::GetColorU32(ImVec4(0.25f, 0.78f, 0.98f, 0.9f));
    const std::array<std::pair<std::size_t, std::size_t>, 6> sequenceEdges{
        std::pair<std::size_t, std::size_t>{0, 1},
        std::pair<std::size_t, std::size_t>{1, 2},
        std::pair<std::size_t, std::size_t>{2, 3},
        std::pair<std::size_t, std::size_t>{3, 4},
        std::pair<std::size_t, std::size_t>{4, 5},
        std::pair<std::size_t, std::size_t>{5, 6}};
    for (const auto [from, to] : sequenceEdges) {
        ImVec2 start = centerFor(from);
        ImVec2 end = centerFor(to);
        if (positions[to].x > positions[from].x) {
            start.x += nodeWidth * 0.5f;
            end.x -= nodeWidth * 0.5f;
        } else if (positions[to].x < positions[from].x) {
            start.x -= nodeWidth * 0.5f;
            end.x += nodeWidth * 0.5f;
        } else if (positions[to].y > positions[from].y) {
            start.y += nodeHeight * 0.5f;
            end.y -= nodeHeight * 0.5f;
        }
        drawSolidArrow(drawList, start, end, sequenceColor, 2.0f);
    }

    ImVec2 sourceDataStart = topLeftFor(0);
    ImVec2 targetDataEnd = topLeftFor(1);
    sourceDataStart.x += nodeWidth * 0.5f;
    sourceDataStart.y += nodeHeight;
    targetDataEnd.x += nodeWidth * 0.5f;
    targetDataEnd.y += nodeHeight;
    sourceDataStart.y += 8.0f;
    targetDataEnd.y += 8.0f;
    drawDashedArrow(drawList, sourceDataStart, targetDataEnd, dataColor, 1.5f);

    ImVec2 fitDataStart = centerFor(2);
    ImVec2 normalDataEnd = centerFor(3);
    fitDataStart.x += nodeWidth * 0.5f;
    normalDataEnd.x -= nodeWidth * 0.5f;
    drawDashedArrow(drawList, fitDataStart, normalDataEnd, dataColor, 1.5f);

    ImVec2 normalDataStart = centerFor(3);
    ImVec2 reconstructionDataEnd = centerFor(5);
    normalDataStart.y += nodeHeight * 0.5f;
    reconstructionDataEnd.y -= nodeHeight * 0.5f;
    drawDashedArrow(
        drawList,
        normalDataStart,
        reconstructionDataEnd,
        dataColor,
        1.5f);

    ImVec2 weightDataStart = centerFor(4);
    reconstructionDataEnd = centerFor(5);
    weightDataStart.x -= nodeWidth * 0.5f;
    reconstructionDataEnd.x += nodeWidth * 0.5f;
    drawDashedArrow(drawList, weightDataStart, reconstructionDataEnd, dataColor, 1.5f);

    std::optional<WorkflowStage> requestedStage;
    for (std::size_t index = 0; index < stages.size(); ++index) {
        const WorkflowStage stage = stages[index];
        const bool ready = stage == WorkflowStage::Source
            ? state.grid != nullptr
            : stage == WorkflowStage::SurfaceFit
                ? state.surfaceTargetCache && !state.surfaceTargetCache->empty()
            : stage == WorkflowStage::NormalField
                ? state.normalFitReady
                : state.slots[0].available();
        const ImVec2 topLeft = topLeftFor(index);
        ImGui::SetCursorScreenPos(topLeft);
        ImGui::PushID(static_cast<int>(index));
        ImGui::InvisibleButton("##stage", ImVec2(nodeWidth, nodeHeight));
        const bool hovered = ImGui::IsItemHovered();
        if (ImGui::IsItemClicked() && ready) {
            requestedStage = stage;
        }
        const bool active = state.workflowController.stage() == stage;
        const ImU32 fillColor = active
            ? ImGui::GetColorU32(ImVec4(0.17f, 0.38f, 0.56f, 1.0f))
            : ready
                ? ImGui::GetColorU32(ImVec4(0.12f, 0.15f, 0.20f, 1.0f))
                : ImGui::GetColorU32(ImVec4(0.08f, 0.09f, 0.11f, 1.0f));
        const ImU32 borderColor = hovered
            ? ImGui::GetColorU32(ImVec4(0.98f, 0.78f, 0.28f, 1.0f))
            : ImGui::GetColorU32(ImVec4(0.40f, 0.45f, 0.54f, 1.0f));
        drawList->AddRectFilled(
            topLeft,
            ImVec2(topLeft.x + nodeWidth, topLeft.y + nodeHeight),
            fillColor,
            5.0f);
        drawList->AddRect(
            topLeft,
            ImVec2(topLeft.x + nodeWidth, topLeft.y + nodeHeight),
            borderColor,
            5.0f,
            0,
            1.5f);
        const char* title = stage == WorkflowStage::SurfaceTarget
            ? "Surface Target"
            : stage == WorkflowStage::SurfaceFit
                ? "Surface Fit"
            : stage == WorkflowStage::NormalField
                ? "Surface Normal"
            : stage == WorkflowStage::WeightPainting
                ? "Weight Painting"
                : stage == WorkflowStage::Reconstruction
                    ? "Reconstruction"
                    : stage == WorkflowStage::Review
                            ? "Review / Export"
                            : "Source VDB";
        drawList->AddText(
            ImVec2(topLeft.x + 7.0f, topLeft.y + 8.0f),
            ready ? ImGui::GetColorU32(ImGuiCol_Text)
                  : ImGui::GetColorU32(ImGuiCol_TextDisabled),
            title);
        drawList->AddText(
            ImVec2(topLeft.x + 7.0f, topLeft.y + 31.0f),
            ImGui::GetColorU32(ImGuiCol_TextDisabled),
            ready ? (active ? "active" : "ready") : "locked");
        if (hovered) {
            ImGui::BeginTooltip();
            ImGui::TextUnformatted(workflowStageName(stage));
            if (!ready) {
                ImGui::TextUnformatted("Reference surface is not available");
            } else if (stage == WorkflowStage::SurfaceTarget) {
                ImGui::TextUnformatted("Reference surface and BVH preview");
            } else if (stage == WorkflowStage::SurfaceFit) {
                ImGui::TextUnformatted("Fit local surface trends and generate seed normals");
            } else if (stage == WorkflowStage::NormalField) {
                ImGui::TextUnformatted("Adjust and inspect the surface normal trend");
            } else if (stage == WorkflowStage::WeightPainting) {
                ImGui::TextUnformatted("Paint and save surface weight fields");
            } else if (stage == WorkflowStage::Reconstruction) {
                ImGui::TextUnformatted("Point-based smoothing and mesh generation");
            }
            ImGui::EndTooltip();
        }
        ImGui::PopID();
    }

    ImGui::SetCursorScreenPos(
        ImVec2(canvasOrigin.x, canvasOrigin.y + canvasHeight + 4.0f));
    ImGui::TextUnformatted("Solid arrow: stage order");
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.25f, 0.78f, 0.98f, 1.0f), "Dashed arrow: data dependency");
    if (ImGui::CollapsingHeader("Viewer", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderFloat(
            "Wheel zoom speed",
            &state.wheelZoomMultiplier,
            1.0f,
            30.0f,
            "%.1fx");
        ImGui::SliderFloat(
            "Wheel hit-distance ratio",
            &state.wheelHitDistanceRatio,
            0.02f,
            0.50f,
            "%.2f");
        ImGui::SliderFloat(
            "Wheel minimum step",
            &state.wheelMinimumDistanceMillimeters,
            0.1f,
            10.0f,
            "%.2f mm");
        ImGui::SliderFloat(
            "Wheel surface clearance",
            &state.wheelSurfaceClearanceMillimeters,
            0.0f,
            5.0f,
            "%.2f mm");
        if (state.cameraWheelUsedAdaptiveStep) {
            ImGui::Text(
                "Last wheel pick: slot %zu, depth %.2f mm, step %.2f mm",
                state.cameraWheelHit.slotIndex,
                state.cameraWheelProjectedDistanceMillimeters,
                state.cameraWheelAppliedStepMillimeters);
        } else {
            ImGui::TextDisabled("Last wheel pick: no adaptive hit (fallback)");
        }
        if (ImGui::SliderFloat(
                "Global light",
                &state.globalLightIntensity,
                0.0f,
                100000.0f,
                "%.0f lux")) {
            state.context.lighting.setIntensity(state.globalLightIntensity);
        }
        bool directionalLightChanged = false;
        directionalLightChanged |= ImGui::SliderFloat(
            "Light azimuth (deg)",
            &state.directionalLightAzimuthDegrees,
            0.0f,
            360.0f,
            "%.1f");
        directionalLightChanged |= ImGui::SliderFloat(
            "Light elevation (deg)",
            &state.directionalLightElevationDegrees,
            -90.0f,
            90.0f,
            "%.1f");
        if (directionalLightChanged) {
            state.context.lighting.setDirection(
                engine,
                directionalLightDirection(state));
        }
        ImGui::TextUnformatted(
            "Angles describe rays from the virtual light toward the model center.");
    }
    ImGui::End();
    return requestedStage;
}

} // namespace volume_surface::viewer
