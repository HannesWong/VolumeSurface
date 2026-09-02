#pragma once

#include <optional>

#include <math/vec3.h>

#include "volume_surface/viewer/WorkflowController.h"

namespace filament {
class Engine;
}

namespace volume_surface::viewer {

struct ViewerState;

class WorkflowPanel final {
public:
    // Draw the workflow window and return a stage requested by the user.
    std::optional<WorkflowStage> draw(
        ViewerState& state,
        filament::Engine& engine) const;

    // Convert the light sliders into Filament's surface-to-light direction.
    static filament::math::float3 directionalLightDirection(
        const ViewerState& state);
};

} // namespace volume_surface::viewer
