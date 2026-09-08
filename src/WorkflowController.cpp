#include "volume_surface/viewer/WorkflowController.h"

namespace volume_surface::viewer {

bool WorkflowController::setStage(WorkflowStage stage) noexcept
{
    if (mStage == stage) {
        return false;
    }
    mStage = stage;
    return true;
}

const char* workflowStageName(WorkflowStage stage) noexcept
{
    switch (stage) {
        case WorkflowStage::Source:
            return "Source VDB";
        case WorkflowStage::SurfaceTarget:
            return "Surface Target";
        case WorkflowStage::SurfaceFit:
            return "Surface Fit / Normal Seed";
        case WorkflowStage::NormalField:
            return "Local Flip Points";
        case WorkflowStage::WeightPainting:
            return "Weight Painting";
        case WorkflowStage::Reconstruction:
            return "Surface Reconstruction";
        case WorkflowStage::Review:
            return "Review / Export";
    }
    return "Unknown";
}

} // namespace volume_surface::viewer
