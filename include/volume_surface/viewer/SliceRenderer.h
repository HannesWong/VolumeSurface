#pragma once

#include "volume_surface/SliceDiagnostics.h"

namespace filament {
class Engine;
class Texture;
}

namespace volume_surface::viewer {

class SliceRenderer final {
public:
    SliceRenderer() = default;
    ~SliceRenderer() = default;

    SliceRenderer(const SliceRenderer&) = delete;
    SliceRenderer& operator=(const SliceRenderer&) = delete;

    void rebuild(
        filament::Engine& engine,
        const DensitySlice& slice);

    [[nodiscard]] filament::Texture* texture() const noexcept
    {
        return mTexture;
    }

    void destroy(filament::Engine& engine);

private:
    filament::Texture* mTexture = nullptr;
};

} // namespace volume_surface::viewer
