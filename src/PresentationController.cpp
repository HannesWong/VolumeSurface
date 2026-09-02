#include "volume_surface/viewer/PresentationController.h"

#include <algorithm>

#include <filament/Scene.h>

namespace volume_surface::viewer {

void PresentationController::apply(
    filament::Scene& scene,
    MeshRenderer& meshRenderer,
    std::array<MeshSlot, 4>& slots,
    SurfaceTargetPreview& surfaceTargetPreview,
    BrushHeatmapRenderer& brushHeatmap,
    const State& state) const
{
    const bool showReferenceMesh = state.surfaceTargetStage
        ? state.showSurfaceTargetMesh
        : state.weightPaintingStage
            ? true
            : state.showReferenceMesh;

    for (std::size_t index = 0; index < slots.size(); ++index) {
        auto& slot = slots[index];
        const bool visible = index == 0
            ? showReferenceMesh
            : state.showResults && slot.available();
        slot.visible = visible;
        meshRenderer.setVisible(scene, index, visible);
    }

    surfaceTargetPreview.setVisible(scene, state.surfaceTargetStage);
    brushHeatmap.setVisible(scene, state.weightPaintingStage);
}

} // namespace volume_surface::viewer
