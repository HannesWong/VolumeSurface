#pragma once

namespace volume_surface::viewer {

struct ViewerState;

enum class ReconstructionPanelAction {
    None,
    ReferenceVisibilityChanged,
    GenerateResultA,
    ClearResultA};

class ReconstructionPanel final {
public:
    ReconstructionPanelAction draw(ViewerState& state) const;
};

} // namespace volume_surface::viewer
