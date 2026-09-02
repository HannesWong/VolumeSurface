#pragma once

#include "volume_surface/SurfaceBrush.h"
#include "volume_surface/viewer/WorkflowController.h"

namespace volume_surface::viewer {

struct WeightPaintingSettings {
    float coreRadiusMillimeters = 3.0f;
    float falloffRadiusMillimeters = 8.0f;
    float strength = 1.0f;
    float planarityRadiusMillimeters = 2.0f;
    float planarityAngleDegrees = 15.0f;
    float planeOffsetPenalty = 16.0f;
    float normalChangePenalty = 0.5f;
    float normalChangeAngleDegrees = 20.0f;
};

struct WeightPaintingRequest {
    SurfaceBrushParameters brush;
    SurfacePropagationSettings propagation;
};

class WeightPaintingStage final {
public:
    [[nodiscard]] static bool isActive(WorkflowStage stage) noexcept;
    [[nodiscard]] static WeightPaintingRequest makeRequest(
        const WeightPaintingSettings& settings,
        float referenceIsoValue) noexcept;
};

} // namespace volume_surface::viewer
