#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "volume_surface/SurfaceBrush.h"

namespace filament {
class Engine;
class Scene;
class View;
}

namespace volume_surface::viewer {

struct ViewerState;

class BrushInteractionController final {
public:
    void beginProfileStroke(ViewerState& state) const;
    void discardProfileStroke(ViewerState& state) const;
    void recordProfilePoint(
        ViewerState& state,
        const openvdb::Vec3d& worldPosition) const;
    void appendProfileStroke(ViewerState& state) const;
    void recordProfileFit(
        ViewerState& state,
        std::size_t anchorCount,
        bool finalFit,
        double heatmapMilliseconds,
        double endToEndMilliseconds) const;

    bool applyHeatmapSamples(
        ViewerState& state,
        filament::Engine& engine,
        filament::Scene& scene,
        const std::vector<volume_surface::SurfaceBrushSample>& samples,
        bool countStroke = true) const;
    bool mergeResultIntoWeightField(ViewerState& state) const;

    void processPendingCenter(
        ViewerState& state,
        filament::Engine& engine,
        filament::Scene& scene) const;
    void processPendingStroke(
        ViewerState& state,
        filament::Engine& engine,
        filament::Scene& scene) const;
    void runProfileReplay(
        ViewerState& state,
        filament::Engine& engine,
        filament::Scene& scene) const;
    void requestPick(
        const std::shared_ptr<ViewerState>& state,
        filament::View& view) const;
};

} // namespace volume_surface::viewer
