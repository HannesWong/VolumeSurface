#pragma once

#include <cstddef>
#include <memory>

#include <math/vec3.h>
#include <utils/Entity.h>

namespace filament {
class Engine;
class Scene;
class View;
}

namespace volume_surface::viewer {

class BrushCursorRenderer final {
public:
    BrushCursorRenderer();
    BrushCursorRenderer(const BrushCursorRenderer&) = delete;
    BrushCursorRenderer& operator=(const BrushCursorRenderer&) = delete;
    BrushCursorRenderer(BrushCursorRenderer&&) = delete;
    BrushCursorRenderer& operator=(BrushCursorRenderer&&) = delete;
    ~BrushCursorRenderer();

    void create(filament::Engine& engine, filament::Scene& scene);
    void destroy(filament::Engine& engine, filament::Scene& scene);

    void update(
        filament::Engine& engine,
        filament::Scene& scene,
        filament::View& view,
        bool visible,
        const filament::math::float3& center,
        float displayScale,
        float coreRadiusMillimeters,
        float falloffRadiusMillimeters);

    utils::Entity entity() const noexcept;
    std::size_t vertexCount() const noexcept;
    bool addedToScene() const noexcept;

private:
    struct Resources;
    std::unique_ptr<Resources> mResources;
};

} // namespace volume_surface::viewer
