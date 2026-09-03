#pragma once

#include <cstdint>

namespace volume_surface::viewer {

enum class WorkflowStage : std::uint8_t {
    Source,
    SurfaceTarget,
    SurfaceFit,
    NormalField,
    WeightPainting,
    Reconstruction,
    Review,
};

class WorkflowController final {
public:
    WorkflowController() = default;

    [[nodiscard]] WorkflowStage stage() const noexcept { return mStage; }
    bool setStage(WorkflowStage stage) noexcept;

private:
    WorkflowStage mStage = WorkflowStage::Source;
};

[[nodiscard]] const char* workflowStageName(WorkflowStage stage) noexcept;

} // namespace volume_surface::viewer
