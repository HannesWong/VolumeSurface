#include "volume_surface/viewer/WeightPaintingStage.h"

#include <algorithm>

namespace volume_surface::viewer {

bool WeightPaintingStage::isActive(WorkflowStage stage) noexcept
{
    return stage == WorkflowStage::WeightPainting;
}

WeightPaintingRequest WeightPaintingStage::makeRequest(
    const WeightPaintingSettings& settings,
    float referenceIsoValue) noexcept
{
    constexpr double millimetersToMeters = 0.001;
    constexpr double degreesToRadians = 0.017453292519943295;
    WeightPaintingRequest request;
    request.brush.coreRadius = settings.coreRadiusMillimeters * millimetersToMeters;
    request.brush.falloffRadius = std::max(
        settings.falloffRadiusMillimeters - settings.coreRadiusMillimeters,
        0.1f) * millimetersToMeters;
    request.brush.strength = settings.strength;
    request.propagation.isoValue = referenceIsoValue;
    request.propagation.planarityRadius =
        settings.planarityRadiusMillimeters * millimetersToMeters;
    request.propagation.planarityAngularScaleRadians =
        settings.planarityAngleDegrees * degreesToRadians;
    request.propagation.planeOffsetPenalty = settings.planeOffsetPenalty;
    request.propagation.normalChangePenalty = settings.normalChangePenalty;
    request.propagation.normalChangeScaleRadians =
        settings.normalChangeAngleDegrees * degreesToRadians;
    return request;
}

} // namespace volume_surface::viewer
