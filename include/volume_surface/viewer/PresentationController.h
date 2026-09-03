#pragma once

#include <array>

#include "volume_surface/SurfaceTargetPreview.h"
#include "volume_surface/viewer/BrushHeatmapRenderer.h"
#include "volume_surface/viewer/MeshRenderer.h"
#include "volume_surface/viewer/SurfaceFitPlaneRenderer.h"

namespace filament {
class Scene;
}

namespace volume_surface::viewer {

class PresentationController final {
public:
    struct State {
        bool surfaceTargetStage = false;
        bool weightPaintingStage = false;
        bool showResults = false;
        bool showReferenceMesh = true;
        bool showSurfaceTargetMesh = true;
        bool surfaceFitStage = false;
        bool surfaceFitPlaneAvailable = false;
        bool showSurfaceFitMesh = true;
    };

    void apply(
        filament::Scene& scene,
        MeshRenderer& meshRenderer,
        std::array<MeshSlot, 4>& slots,
        SurfaceTargetPreview& surfaceTargetPreview,
        SurfaceFitPlaneRenderer& surfaceFitPlaneRenderer,
        BrushHeatmapRenderer& brushHeatmap,
        const State& state) const;
};

} // namespace volume_surface::viewer
